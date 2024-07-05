/*
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */

//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"
#include "pid.h"
#include "Chirp.h"
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_communication_task(); // code to be executed in the slow communication task
void loop_application_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS-------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

static float meas_data; // temp storage meas value (ctrl task)

float32_t duty_cycle = 0.3;
float32_t duty = 0.3;

static float32_t voltage_reference = 15; //voltage reference

/* PID coefficient for a 8.6ms step response*/

static float32_t kp = 0.000215;
static float32_t Ti = 7.5175e-5;
static float32_t Td = 0.0;
static float32_t N = 0.0;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = control_task_period * 1e-6;
static PidParams pid_params(Ts, kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pid;
static float32_t fmin_chirp = 0.0;
static float32_t fmax_chirp = 5000.0;
static float32_t duration_chirp = 80e-3;
static Chirp chirp(fmin_chirp, fmax_chirp, duration_chirp);
static float32_t chirp_time;
static float32_t chirp_value;
static float32_t duty_disturb;
static bool go_chirp;
static bool is_downloading;
static float32_t is_chirp_on;
static ScopeMimicry scope(512, 6);
//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE
};

uint8_t mode = IDLEMODE;
bool my_trig() {
	return go_chirp;
}
//--------------SETUP FUNCTIONS-------------------------------

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    // Setup the hardware first
    spin.version.setBoardVersion(SPIN_v_1_0);
    twist.setVersion(shield_TWIST_V1_3);

    /* buck voltage mode */
    twist.initAllBuck();

    data.enableTwistDefaultChannels();

    pid.init(pid_params);
	go_chirp = false;
	scope.start();
	scope.connectChannel(chirp_value, "chirp_value");
	scope.connectChannel(chirp_time, "chirp_time");
	scope.connectChannel(duty, "I_duty");
	scope.connectChannel(V1_low_value, "V1_low");
	scope.connectChannel(is_chirp_on, "is_chirp_on");
	scope.connectChannel(duty_disturb, "I_duty_disturb");
	scope.set_trigger(my_trig);
	scope.set_delay(0.0);
    // Then declare tasks
    uint32_t app_task_number = task.createBackground(loop_application_task);
    uint32_t com_task_number = task.createBackground(loop_communication_task);
    task.createCritical(loop_critical_task, 100); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(app_task_number);
    task.startBackground(com_task_number);
    task.startCritical(); // Uncomment if you use the critical task
}

//--------------LOOP FUNCTIONS--------------------------------

void loop_communication_task()
{
	received_serial_char = console_getchar();
	switch (received_serial_char)
	{
		case 'h':
			//----------SERIAL INTERFACE MENU-----------------------
			printk(" ________________________________________\n");
			printk("|     ---- MENU buck voltage mode ----   |\n");
			printk("|     press i : idle mode                |\n");
			printk("|     press p : power mode               |\n");
			printk("|     press u : voltage reference UP     |\n");
			printk("|     press d : voltage reference DOWN   |\n");
			printk("|________________________________________|\n\n");
			//------------------------------------------------------
			break;
		case 'i':
			printk("idle mode\n");
			mode = IDLEMODE;
			break;
		case 'p':
			printk("power mode\n");
			mode = POWERMODE;
			break;
		case 'u':
			voltage_reference += 0.5;
			break;
		case 'd':
			voltage_reference -= 0.5;
			break;
		case 'g':
			go_chirp = !go_chirp;
			if (!go_chirp) {
				scope.start();
				chirp_time = 0.0F;
			}
			break;
		case 'r':
			is_downloading = true;
			break;
		default:
			break;
	}
}

/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
void loop_application_task()
{
    if (mode == IDLEMODE)
    {
        spin.led.turnOff();
		if (is_downloading) {
			printk("begin record\n");
			scope.reset_dump();
			while (scope.get_dump_state() != finished) {
				printk("%s", scope.dump_datas());
				task.suspendBackgroundUs(100);
			}
			printk("end record\n");
		}
		is_downloading = false;
    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();

        printk("%f:", (double) I1_low_value);
        printk("%f:", (double) V1_low_value);
        printk("%f:", (double) I2_low_value);
        printk("%f:", (double) V2_low_value);
        printk("%f:", (double) I_high);
		printk("%d:", go_chirp);
		printk("%f:", (double) is_chirp_on);
        printk("%f\n", (double) V_high);
    }
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{
    meas_data = data.getLatest(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = data.getLatest(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;

    if (mode == IDLEMODE)
    {
        if (pwm_enable == true)
        {
            twist.stopAll();
        }
        pwm_enable = false;
    }
    else if (mode == POWERMODE)
    {
		is_chirp_on = (float32_t) (go_chirp && chirp_time < chirp.duration);
		if (is_chirp_on)
		{
			chirp_value = chirp.generate(chirp_time);
			duty_disturb = 0.1F * (2.0F * (chirp_value > 0.0F) - 1.0F);
			chirp_time += Ts;
			duty_cycle = duty + duty_disturb;
		} else {
			chirp_value = 0.0;
			duty_disturb = 0.0;
			duty = pid.calculateWithReturn(voltage_reference, V1_low_value);
			duty_cycle = duty;
		}

        twist.setAllDutyCycle(duty_cycle);
		scope.acquire();
        /* Set POWER ON */
        if (!pwm_enable)
        {
            pwm_enable = true;
            twist.startAll();
        }
    }

}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}

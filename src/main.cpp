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
 */
//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"
#include "arm_math_types.h"
#include "control_factory.h"
#include "transform.h"
#include "trigo.h" 
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"
// #include "shield_channels.h"
//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_background_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task
void application_task();
//--------------USER fARIABLES DECLARATIONS-------------------
#define HALL1 PA7
#define HALL2 PD2
#define HALL3 PC6

static const float32_t V_HIGH_MIN = 10.0;

// Hall effect sensors
static uint8_t HALL1_value;
static uint8_t HALL2_value;
static uint8_t HALL3_value;
static uint8_t angle_index;
static float32_t angle_index_f;
static float32_t hall_angle;

// LEG meas
static float32_t meas_data;
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
// static float32_t V3_low_value;
// static float32_t I3_low_value;
// dc meas
static float32_t I_high;
static float32_t V_high;


static three_phase_t Vabc;
static three_phase_t duty_abc;
static three_phase_t Iabc;
static dqo_t Vdq;
static dqo_t Idq;
static dqo_t Idq_ref;

static float32_t Va, Vb;
static float32_t duty_a, duty_b;
static float32_t Id_meas, Iq_meas;
static float32_t Id_ref, Iq_ref;
static float32_t Vd, Vq;

static float32_t AngleTab_rad[] = 
{
    -1,
     60.0 * PI / 180.0, 
    300.0 * PI / 180.0,
      0.0 * PI / 180.0,
    180.0 * PI / 180.0,
    120.0 * PI / 180.0,
    240.0 * PI / 180.0
};

static float32_t Ts = 100.e-6F;
static uint32_t control_task_period = (uint32_t) (Ts * 1.e6F);

static PllAngle pllangle = controlLibFactory.pllAngle(Ts, 10.0F, 0.05F);
static PllDatas pllDatas;
static float32_t angle_filtered;
static float32_t w_ref;
static float32_t w_ref_filtered;
static float32_t w_meas;
static float32_t manual_Iq_ref;
static float32_t manual_w_ref;
static float32_t speed_regul_Iq_ref;

static LowPassFirstOrderFilter w_ref_filter = controlLibFactory.lowpassfilter(Ts, 5.0e-3F);
static LowPassFirstOrderFilter vHigh_filter = controlLibFactory.lowpassfilter(Ts, 5.0e-3F);
static LowPassFirstOrderFilter w_mes_filter = controlLibFactory.lowpassfilter(Ts, 1.0e-3F);
static float32_t V_high_filtered;
static float32_t inverse_Vhigh;
//--- PID ---
static float32_t Kp = 30*0.035;
static float32_t Ti = 0.001029;
static float32_t Td = 0.0;
static float32_t N = 1.0;
static float32_t lower_bound = -30.0;
static float32_t upper_bound = 30.0;
static Pid pi_d = controlLibFactory.pid(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pi_q = controlLibFactory.pid(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
static float32_t Kp_speed = 0.001;
static float32_t Ti_speed = 0.10;
static float32_t lower_bound_speed = 0.0;
static float32_t upper_bound_speed = 2.0;
static Pid pi_speed = controlLibFactory.pid(Ts, Kp_speed, Ti_speed, Td, N, lower_bound_speed, upper_bound_speed);
// transform electrical pulsation to linear speed in km/h
// to_kmh = (1.0/45.0*26*0.0254)/1000.0*3600.0; 
// 45 pair of poles
// 1 wheel of 26 inches (1 inch = 2.54cm)
const float32_t to_kmh = 0.02641;
const static uint32_t decimation = 30;
static uint32_t counter_time;
uint8_t received_serial_char;
enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE = 1,
    ERRORMODE = 2
};

static const float32_t CURRENT_LIMIT = 10.0;
static bool pwm_enable;

enum regulation_mode_enum {
    TORQUE = 0,
    SPEED = 1
};

uint8_t mode = IDLEMODE;
enum regulation_mode_enum regulation_mode = TORQUE;

void init_filt_and_reg(void) {
    pllangle.reset(0.F);
    vHigh_filter.reset(V_HIGH_MIN);
    w_ref_filter.reset(0.0);
    pi_d.reset();
    pi_q.reset();
    pi_speed.reset();
}

ScopeMimicry scope(512, 11);
static bool is_downloading;
bool mytrigger() {
    return true;
}

void dump_scope_datas(ScopeMimicry &scope)  {
    uint8_t *buffer = scope.get_buffer();
    uint16_t buffer_size = scope.get_buffer_size() >> 2; // we divide by 4 (4 bytes per float data) 
    printk("begin record\n");
    printk("#");
    for (uint16_t k=0;k < scope.get_nb_channel(); k++) {
        printk("%s,", scope.get_channel_name(k));
    }
    printk("\n");
    for (uint16_t k=0;k < buffer_size; k++) {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
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
    // spin.version.setBoardVersion(TWIST_v_1_1_3);
    twist.setVersion(shield_ownverter);

    twist.initAllBuck();

    data.enableTwistDefaultChannels(); 
    data.setParameters(I1_LOW, 0.0051, -10.438);
    data.setParameters(I2_LOW, 0.0049, -9.9797);

    spin.gpio.configurePin(HALL1, INPUT);
    spin.gpio.configurePin(HALL2, INPUT);
    spin.gpio.configurePin(HALL3, INPUT);

    scope.connectChannel(I1_low_value, "ILow1");
    scope.connectChannel(I2_low_value, "ILow2");
    scope.connectChannel(Va, "Va");
    scope.connectChannel(Vq, "Vq");
    scope.connectChannel(Vd, "Vd");
    scope.connectChannel(duty_a, "duty_a");
    scope.connectChannel(angle_filtered, "angle");
    scope.connectChannel(w_meas, "w_meas");
    scope.connectChannel(Iq_ref, "Iq_ref");
    scope.connectChannel(w_ref_filtered, "w_ref");
    scope.connectChannel(Iq_meas, "Iq");

    scope.set_trigger(&mytrigger);
    scope.set_delay(0.0);
    scope.start();
    init_filt_and_reg();
    counter_time = 0;
    I1_low_value = 0.0;
    I2_low_value = 0.0;
    I_high = 0.0;
    V_high = 0.0;

    pwm_enable = false;
    mode = IDLEMODE;
    // Then declare tasks
    uint32_t background_task_number = task.createBackground(loop_background_task);
    uint32_t app_task_number = task.createBackground(application_task);
    task.createCritical(loop_critical_task, control_task_period); // Uncomment if you use the critical task

    // Finally, start tasks
    task.startBackground(background_task_number);
    task.startBackground(app_task_number);
    task.startCritical(); // Uncomment if you use the critical task
}

//--------------LOOP FUNCTIONS--------------------------------

/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
void loop_background_task()
{
    // Task content
    received_serial_char = console_getchar();
    switch (received_serial_char) {
        case 'h':
            printk("help");
        break;
        case 'p':
            printk("power asked");
            mode = POWERMODE;
            scope.start();
        break;
        case 'i':
            printk("idle asked");
            mode = IDLEMODE;
            manual_Iq_ref = 0.0;
            manual_w_ref = 0.0;
            // reset scope.
            scope.start();
        break;
        case 't':
            printk("TORQUE mode asked\n");
            regulation_mode = TORQUE;
        break;
        case 's':
            printk("SPEED mode asked\n");
            regulation_mode = SPEED;
        break;
        case 'u':
            if (regulation_mode == SPEED) {
                manual_w_ref += 10.0F;
                if (manual_w_ref > 1000.0F) manual_w_ref = 1000.0F;
            } else {
                manual_Iq_ref += 0.025F;
                if (manual_Iq_ref > 2.0F) manual_Iq_ref = 2.0F;
            }
        break;
        case 'd':
            if (regulation_mode == SPEED) {
                manual_w_ref -= 10.0F;
                if (manual_w_ref < 0.0F) manual_w_ref = 00.0F;
            }
            else {
                manual_Iq_ref -= 0.025;
                if (manual_Iq_ref < 0.0) manual_Iq_ref = 0.0;
            }
        break;
        case 'r':
            is_downloading = true;
        break;
    }

    // Pause between two runs of the task
}

    void application_task() {

    // data.retrieveParametersFromMemory()


    printk("%.2f:", I_high);
    printk("%.2f:", V_high);
    printk("%.2f:", V_high_filtered);
    printk("%d:", angle_index);
    printk("%f:", manual_Iq_ref);
    printk("%f:", manual_w_ref);
    printk("%d:", regulation_mode);
    printk("%d\n", mode);

    if (is_downloading) {
        dump_scope_datas(scope);
        is_downloading = false;
    }

    task.suspendBackgroundMs(250);
}



/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
void loop_critical_task()
{
    counter_time++;
    meas_data = data.getLatest(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != NO_VALUE) I_high = meas_data;

    meas_data = data.getLatest(V1_LOW);
    if (meas_data != NO_VALUE) V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data != NO_VALUE) V2_low_value = meas_data;


    // Mesure de Vhigh
    V_high_filtered = vHigh_filter.calculateWithReturn(V_high); 
    if (V_high_filtered < V_HIGH_MIN) {
        V_high_filtered = V_HIGH_MIN;
    }

    inverse_Vhigh = 0.0333;//1.0 / V_high_filtered;

    //reconstitution de l'index à partir de la lecture
    HALL1_value = spin.gpio.readPin(HALL1);
    HALL2_value = spin.gpio.readPin(HALL2);
    HALL3_value = spin.gpio.readPin(HALL3);


    angle_index = HALL3_value*4 + HALL2_value*2 + HALL1_value*1;

    angle_index_f = (float32_t) angle_index;
    hall_angle = ot_modulo_2pi(AngleTab_rad[angle_index] + PI * 8.0 / 12.0);
    pllDatas = pllangle.calculateWithReturn(hall_angle);
    // DEBUG
    angle_filtered = pllDatas.angle;
    if (I1_low_value > CURRENT_LIMIT || I1_low_value < -CURRENT_LIMIT || I2_low_value > CURRENT_LIMIT || I2_low_value < -CURRENT_LIMIT) {
        mode = ERRORMODE;
    }
    if (mode == IDLEMODE || mode == ERRORMODE) {
        if (pwm_enable == true) {
            twist.stopAll();
            // reset filters and pid
            init_filt_and_reg();
            pwm_enable = false;
        }
    } else if (mode == POWERMODE) {
        //--- (combinaison de 3 bits : binary to decimal) ---------------------


        if (regulation_mode == TORQUE) {
            w_ref = pllDatas.w;
            Idq_ref.q = manual_Iq_ref;
            Idq_ref.d = 0.0;
        } else if (regulation_mode == SPEED) {
            w_ref = manual_w_ref;
            Idq_ref.q = speed_regul_Iq_ref;
            Idq_ref.d = 0.0;
        } else {
        };

        w_meas = w_mes_filter.calculateWithReturn(pllDatas.w);    
        w_ref_filtered = w_ref_filter.calculateWithReturn(w_ref);
        speed_regul_Iq_ref = pi_speed.calculateWithReturn(w_ref_filtered, w_meas);

        Iabc.a = -I1_low_value; // FIXME:WHY ??
        Iabc.b = -I2_low_value; // FIXME:WHY ??
        Iabc.c = -(I1_low_value + I2_low_value);
        Idq = Transform::to_dqo(Iabc, angle_filtered);
        Id_ref = Idq_ref.d;
        Iq_ref = Idq_ref.q;
        Idq_ref.o = 0.0F;
        Id_meas = Idq.d;
        Iq_meas = Idq.q;
        Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d);
        Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q);
        Vdq.o = 0.0F;
        Vabc = Transform::to_threephase(Vdq, angle_filtered);
        Vd = Vdq.d;
        Vq = Vdq.q;
        Va = Vabc.a;
        Vb = Vabc.b;
        duty_abc.a = (Vabc.a * 0.5 * inverse_Vhigh + 0.5); // + dead_time_comp(Iabc_ref.a, comp_dt)); 
        duty_abc.b = (Vabc.b * 0.5 * inverse_Vhigh + 0.5); // + dead_time_comp(Iabc_ref.b, comp_dt));
        duty_abc.c = (Vabc.c * 0.5 * inverse_Vhigh + 0.5); // + dead_time_comp(Iabc_ref.c, comp_dt));
        duty_a = duty_abc.a;
        duty_b = duty_abc.b;
        
        twist.setLegDutyCycle(LEG1, duty_abc.a);
        twist.setLegDutyCycle(LEG2, duty_abc.b);
        twist.setLegDutyCycle(LEG3, duty_abc.c);

        if (counter_time%decimation == 0)
        {
            scope.acquire();
        }
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

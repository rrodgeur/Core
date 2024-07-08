#include "trigo.h"
#include "sogi_pll.h"
void sogi_pll_init(sogi_pll_params_t *params, float32_t Vgrid, float32_t w0, float32_t rise_time, float32_t Ts)
{
    float32_t wn = 3.0F / rise_time;
    float32_t xsi = 0.7F;
    float32_t Kp = 2 * wn * xsi / Vgrid;
    float32_t Ki = (wn  * wn) / Vgrid;
    params->Ts = Ts;
    params->w = w0;
    params->w_ref = w0;
    float32_t Kr = 500.0;
    sogi_init(&params->sogi_params, Kr, Ts);

	params->pi_params.Ts = Ts;
	params->pi_params.Td = 0.0;
	params->pi_params.N = 1;
    params->pi_params.Ti = Kp / Ki;
    params->pi_params.Kp = Kp;
    params->pi_params.lower_bound = -10.0F * w0;
    params->pi_params.upper_bound = 10.0F * w0;
	params->pi.init(params->pi_params);
	params->pi.reset();
}

void sogi_pll_calc(float32_t v_grid, sogi_pll_params_t *params)
{
    params->theta = params->next_theta;
    params->Vab = sogi_calc(v_grid, params->w, &params->sogi_params);
    params->Vdq = Transform::rotation_to_dqo(params->Vab, params->theta); 
    params->w = params->w_ref + params->pi.calculateWithReturn(0, -params->Vdq.q);
    params->next_theta = ot_modulo_2pi(params->theta + params->Ts * params->w);
}

void sogi_init(sogi_params_t *params, float32_t Kr, float32_t Ts)
{
	params->Kr = Kr;
	params->Ts = Ts;
    params->den[0] = 0.0;
    params->den[1] = 0.0;
    params->den[2] = 0.0;

    params->num[0] = 0.0;
    params->num[1] = 0.0;

}

clarke_t sogi_calc(float32_t input, float32_t w0, sogi_params_t *params)
{
	// Proportional resonant using discretization of R1h with impulse invariant. 
	// (see page p. 55 of Ph D Digital Resonant Current Controller For Voltage Source Inverter, from Gï Alejandro)
	// FIXME: how to check if series expansion is sufficient ?
	float32_t coswt = 1.0F - 0.5F*(w0*params->Ts)*(w0*params->Ts); 
	float32_t inv_sinwt = 1.0F / (w0 * params->Ts);
	clarke_t result; 
	params->num[0] = params->Kr * (input - params->den[0]);
	params->den[0] = params->Ts * (params->num[0] - coswt * params->num[1]) + 2.0F * coswt * params->den[1] - params->den[2];
	result.alpha = params->den[0];
	result.beta = - coswt * inv_sinwt * params->den[0] + inv_sinwt * params->den[1];
	result.o = 0.0;

	params->num[1] = params->num[0];
	params->den[2] = params->den[1];
	params->den[1] = params->den[0];

	return result;
}



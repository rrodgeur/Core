#include "power_ac1phase.h"

// TODO return error code
void power_ac1phase_init(power_ac1phase_params_t *params, float32_t grid_voltage, float32_t w0, float32_t Ts) {
    float32_t rise_time = 1.0F * 2.0F * PI / w0;
    params->w0 = w0;
    sogi_pll_init(&params->sogi_pll_params, grid_voltage, w0, rise_time, Ts);
    float32_t Kr = 500;
    sogi_init(&params->sogi_i_params, Kr, Ts);
}

power_ac1phase_t power_ac1phase(float32_t v, float32_t i, power_ac1phase_params_t *params) {
    dqo_t Idq;
    clarke_t Iab;
    dqo_t Vdq; 
    power_ac1phase_t power;
    sogi_pll_calc(v, &params->sogi_pll_params);
    Iab = sogi_calc(i, params->w0, &params->sogi_i_params) ;
    Idq = Transform::rotation_to_dqo(Iab, params->sogi_pll_params.theta);
    Vdq = params->sogi_pll_params.Vdq;
    power.p = Vdq.d * Idq.d + Vdq.q * Idq.q;
    power.q = Idq.d * Vdq.q - Idq.q * Vdq.d;
    return power;
}

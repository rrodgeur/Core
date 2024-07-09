#ifndef POWER_AC1PHASE_H
#define POWER_AC1PHASE_H
#include "sogi_pll.h"

typedef struct {
    float32_t p;
    float32_t q;
} power_ac1phase_t;

typedef struct {
    sogi_pll_params_t sogi_pll_params;
    sogi_params_t sogi_i_params;
    float32_t w0;
} power_ac1phase_params_t; 


void power_ac1phase_init(power_ac1phase_params_t *params, float32_t grid_voltage, float32_t w0, float32_t Ts);
power_ac1phase_t power_ac1phase(float32_t v, float32_t i, power_ac1phase_params_t *params);
#endif

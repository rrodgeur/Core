#ifndef SOGI_PLL_H
#define SOGI_PLL_H
#include "pid.h"
#ifndef __arm__ 
#include "my_types.h"
#else
#include "arm_math.h"
#endif

#include "transform.h"
typedef struct {
    float32_t num[2];
    float32_t den[3];
    float32_t Ts;
    float32_t Kr;
} sogi_params_t;

typedef struct {
    sogi_params_t sogi_params;
	PidParams pi_params;
    Pid pi;
    float32_t Ts;
    dqo_t Vdq;
    clarke_t Vab;
    float32_t theta;
    float32_t next_theta;
    float32_t w;
    float32_t w_ref;
} sogi_pll_params_t;

void sogi_init(sogi_params_t *params, float32_t Kr, float32_t Ts);
clarke_t sogi_calc(float32_t input, float32_t w0, sogi_params_t *params);
void sogi_pll_init(sogi_pll_params_t *sogi_pll_params, float32_t Vgrid, float32_t w0, float32_t rise_time, float32_t Ts);
void sogi_pll_calc(float32_t v_grid, sogi_pll_params_t *sogi_pll_params);
#endif

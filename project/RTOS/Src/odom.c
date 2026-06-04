/**
  ******************************************************************************
  * @file    odom.c
  * @brief   Wheel-odometry integrator. See odom.h for frame and units.
  *
  *  Per-tick (20 ms):
  *    delta_R = (int16_t)(motorInterrupt1 - prev_R) * ENC_R_SIGN   // wrap-safe
  *    delta_L = (int16_t)(motorInterrupt2 - prev_L) * ENC_L_SIGN
  *    ds_R   = delta_R / TICKS_PER_CM
  *    ds_L   = delta_L / TICKS_PER_CM
  *    ds     = (ds_R + ds_L) / 2
  *    dth    = (ds_R - ds_L) / WHEEL_BASE_CM
  *    a      = theta + dth / 2          // midpoint heading (RK2)
  *    x     += ds * cos(a);  y += ds * sin(a);  theta += dth
  ******************************************************************************
  */

#include "odom.h"
#include <math.h>

extern volatile uint16_t motorInterrupt1;   /* right encoder, defined in main.c */
extern volatile uint16_t motorInterrupt2;   /* left  encoder, defined in main.c */

#define PI_F        3.14159265358979f
#define TWO_PI_F    6.28318530717958f

static uint16_t prev_R_raw;
static uint16_t prev_L_raw;
static int32_t  R_ticks_total;
static int32_t  L_ticks_total;
static float    x_cm;
static float    y_cm;
static float    theta_rad;
static float    last_ds_R;       /* per-tick wheel displacement, exposed for EKF */
static float    last_ds_L;

void odom_init(void)
{
    prev_R_raw   = motorInterrupt1;
    prev_L_raw   = motorInterrupt2;
    R_ticks_total = 0;
    L_ticks_total = 0;
    x_cm     = 0.0f;
    y_cm     = 0.0f;
    theta_rad = 0.0f;
    last_ds_R = 0.0f;
    last_ds_L = 0.0f;
}

void odom_tick(void)
{
    uint16_t now_R = motorInterrupt1;
    uint16_t now_L = motorInterrupt2;
    /* int16_t cast on unsigned subtraction = wrap-safe signed delta for any
     * single-step movement smaller than 32768 ticks (always true at 20 ms). */
    int16_t dR_raw = (int16_t)(now_R - prev_R_raw);
    int16_t dL_raw = (int16_t)(now_L - prev_L_raw);
    prev_R_raw = now_R;
    prev_L_raw = now_L;

    int32_t dR = (int32_t)dR_raw * ENC_R_SIGN;
    int32_t dL = (int32_t)dL_raw * ENC_L_SIGN;
    R_ticks_total += dR;
    L_ticks_total += dL;

    float ds_R = (float)dR / TICKS_PER_CM;
    float ds_L = (float)dL / TICKS_PER_CM;
    last_ds_R  = ds_R;
    last_ds_L  = ds_L;
    float ds   = (ds_R + ds_L) * 0.5f;
    float dth  = (ds_R - ds_L) / WHEEL_BASE_CM;

    float alpha = theta_rad + dth * 0.5f;
    x_cm  += ds * cosf(alpha);
    y_cm  += ds * sinf(alpha);
    theta_rad += dth;

    /* Keep theta in (-pi, pi] so sinf/cosf stay near their best-precision range
     * over long runs (large theta values lose ULP precision). */
    if (theta_rad >  PI_F) theta_rad -= TWO_PI_F;
    if (theta_rad < -PI_F) theta_rad += TWO_PI_F;
}

void odom_get(float *x_out, float *y_out, float *theta_out)
{
    if (x_out     != 0) *x_out     = x_cm;
    if (y_out     != 0) *y_out     = y_cm;
    if (theta_out != 0) *theta_out = theta_rad;
}

void odom_reset(float x_in, float y_in, float theta_in)
{
    prev_R_raw = motorInterrupt1;
    prev_L_raw = motorInterrupt2;
    x_cm     = x_in;
    y_cm     = y_in;
    theta_rad = theta_in;
}

void odom_get_ticks(int32_t *r_out, int32_t *l_out)
{
    if (r_out != 0) *r_out = R_ticks_total;
    if (l_out != 0) *l_out = L_ticks_total;
}

void odom_get_last_delta_cm(float *ds_R_out, float *ds_L_out)
{
    if (ds_R_out != 0) *ds_R_out = last_ds_R;
    if (ds_L_out != 0) *ds_L_out = last_ds_L;
}

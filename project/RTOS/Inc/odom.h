/**
  ******************************************************************************
  * @file    odom.h
  * @brief   Wheel odometry — pose (x, y, theta) integrated from differential-
  *          drive kinematics. Reads the existing encoder counters directly.
  *
  *  Pose frame: start state = (0, 0, 0). +x = robot's initial forward,
  *  +y = robot's initial left, theta = CCW from +x in radians.
  *
  *  Encoder source: motorInterrupt1 (right) / motorInterrupt2 (left), defined
  *  in main.c and updated by the EXTI ISR. Since rotate_iterative() no longer
  *  resets them (time-based pivot — Option A), they act as natural monotonic
  *  counters; uint16_t wraparound is handled here via signed delta cast.
  ******************************************************************************
  */

#ifndef __ODOM_H
#define __ODOM_H

#include <stdint.h>

/* ===========================================================================
 *  Calibration — set via CALIB_ODOM before relying on the pose.
 *  Placeholders below are ORDER-OF-MAGNITUDE guesses and WILL be wrong.
 * =========================================================================== */
#define WHEEL_BASE_CM    15.0f     /* TODO(measure): caliper between wheel contact patches */
#define TICKS_PER_CM     20.0f     /* TODO(CALIB_ODOM=1): mean(|dR|,|dL|) / measured_cm   */

/* EXTI sign is partner-pin keyed, not forward-direction keyed. CALIB_ODOM=1
 * tells you whether right/left increment in opposite directions on a forward
 * drive; flip to -1 to align both with "forward = positive". */
#define ENC_R_SIGN       (+1)
#define ENC_L_SIGN       (+1)

/* ===========================================================================
 *  API
 * =========================================================================== */
void odom_init(void);                                       /* zero pose, snapshot encoder baseline */
void odom_tick(void);                                       /* call once per SensorTask period (20 ms) */
void odom_get(float *x_cm, float *y_cm, float *theta_rad);  /* NULL-safe per field */
void odom_reset(float x_cm, float y_cm, float theta_rad);   /* force pose without touching counters */

/* Accumulated signed ticks since last odom_init(), for CALIB_ODOM diagnostics
 * (straight-line ticks-per-cm measurement). NULL-safe per field. */
void odom_get_ticks(int32_t *r_ticks, int32_t *l_ticks);

#endif /* __ODOM_H */

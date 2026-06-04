/**
  ******************************************************************************
  * @file    ekf.h
  * @brief   Extended Kalman Filter for 2D pose (x_cm, y_cm, theta_rad).
  *
  *  Reference frame matches odom.h: start state = (0, 0, 0), +x = robot's
  *  initial forward, +y = robot's initial left, theta = CCW.
  *
  *  Predict input: per-tick wheel displacements (ds_L, ds_R) in cm — read
  *  from odom_get_last_delta_cm() inside SensorTask.
  *
  *  Update input: three ultrasonic distances (cm) and per-sensor stddev
  *  proxies (cm). Measurement model intersects each beam ray with the
  *  wall_seg_t list in map_geom.h; if n_walls == 0 the update is a no-op
  *  and EKF degenerates to predict-only.
  ******************************************************************************
  */

#ifndef __EKF_H
#define __EKF_H

#include <stdint.h>

/* ===========================================================================
 *  Sensor mount offsets in the robot frame (cm, rad).
 *  (d_x = forward of wheel axle, d_y = left of centerline, phi = beam angle
 *   with 0=front, +pi/2=left, -pi/2=right). TODO(measure) with calipers
 *   from the wheel axle midpoint. Heading bias scales with d_y mismatch.
 * =========================================================================== */
#define US_F_DX     6.0f
#define US_F_DY     0.0f
#define US_F_PHI    0.0f

#define US_L_DX     4.0f
#define US_L_DY     5.0f
#define US_L_PHI    1.5707963f         /* +pi/2 */

#define US_R_DX     4.0f
#define US_R_DY    -5.0f
#define US_R_PHI   -1.5707963f         /* -pi/2 */

/* Clip a no-hit / max-range raycast result to this distance (cm). Matches
 * the "no wall on this side" semantics already used elsewhere in the code. */
#define EKF_RANGE_MAX_CM    150.0f

/* Per-sensor noise floor (cm). Prevents R from becoming singular when the
 * integer stddev7() returns 0 for a perfectly clean reading. */
#define EKF_R_MIN_CM        1.0f

/* Mahalanobis gate: drop a sensor row from the update when its innovation
 * magnitude exceeds N * sqrt(S_ii). */
#define EKF_INNOV_GATE      3.0f

/* ===========================================================================
 *  API
 * =========================================================================== */
void ekf_init(void);
void ekf_predict(float ds_L_cm, float ds_R_cm);
void ekf_update(int dF_cm, int dL_cm, int dR_cm,
                int sF,    int sL,    int sR);
void ekf_get(float *x_cm, float *y_cm, float *theta_rad);   /* NULL-safe per field */
void ekf_get_cov_diag(float *vx, float *vy, float *vth);    /* P diagonal, NULL-safe */

#endif /* __EKF_H */

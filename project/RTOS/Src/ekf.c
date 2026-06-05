/**
  ******************************************************************************
  * @file    ekf.c
  * @brief   EKF on (x, y, theta) — see ekf.h for frame, units, API.
  *
  *  Predict: midpoint (RK2) differential-drive integration with analytical
  *  Jacobians F_k = df/dx and V_k = df/du. Q is propagated from per-wheel
  *  encoder noise: Q = V * diag(sigma_L^2, sigma_R^2) * V^T, with
  *  sigma_i = max(k_slip * |ds_i|, sigma_floor).
  *
  *  Update: sequential per-sensor (3 ultrasonics processed independently
  *  because their R is uncorrelated). Each sensor's beam is ray-cast against
  *  the wall_seg_t list in map_geom.h; the closest valid intersection
  *  becomes the predicted range z_hat. H_i is the analytical row from the
  *  closed-form z_hat formula. Joseph form keeps P symmetric PD.
  *
  *  When n_walls == 0 or no beam hits a wall, the corresponding sensor row
  *  is skipped (degenerates to predict-only).
  ******************************************************************************
  */

#include "ekf.h"
#include "map_geom.h"
#include "odom.h"        /* WHEEL_BASE_CM */
#include <math.h>

#define PI_F        3.14159265358979f
#define TWO_PI_F    6.28318530717958f

/* Process-noise tunables. Slip k applied to |ds|; floor prevents Q=0 when
 * the robot is stationary (one tick of jitter still injects some growth). */
#define EKF_SLIP_K          0.05f
#define EKF_SIGMA_FLOOR_CM  0.05f

static float xs[3];          /* state: px_cm, py_cm, theta_rad */
static float P[3][3];

/* -------- 3x3 matrix helpers (hand-rolled; sizes are fixed) --------
 * Parameters are non-const because Keil ARMCC refuses to convert
 * float (*)[3] to const float (*)[3] without an explicit cast (C does
 * not propagate const through nested pointer types). Helpers don't
 * mutate A/B in practice; the discipline is by convention. */
static void mat3_mul(float A[3][3], float B[3][3], float C[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
    }
}

static void mat3_mul_tr(float A[3][3], float B[3][3], float C[3][3])
{
    /* C = A * B^T */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += A[i][k] * B[j][k];
            C[i][j] = s;
        }
    }
}

static void mat3_add(float A[3][3], float B[3][3], float C[3][3])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i][j] = A[i][j] + B[i][j];
}

static void wrap_theta(void)
{
    if (xs[2] >  PI_F) xs[2] -= TWO_PI_F;
    if (xs[2] < -PI_F) xs[2] += TWO_PI_F;
}

/* ===========================================================================
 *  Public API
 * =========================================================================== */
void ekf_init(void)
{
    xs[0] = xs[1] = xs[2] = 0.0f;
    /* Start state is the defined reference frame; uncertainty is only the
     * placement tolerance, intentionally small. */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = 0.0f;
    P[0][0] = 0.01f;             /* 0.1 cm 1-sigma */
    P[1][1] = 0.01f;
    P[2][2] = (PI_F / 180.0f) * (PI_F / 180.0f);   /* 1 deg 1-sigma */
}

void ekf_predict(float ds_L, float ds_R)
{
    const float b   = WHEEL_BASE_CM;
    const float ds  = 0.5f * (ds_R + ds_L);
    const float dth = (ds_R - ds_L) / b;
    const float theta = xs[2];
    const float alpha = theta + 0.5f * dth;
    const float c = cosf(alpha);
    const float s = sinf(alpha);

    /* --- State propagation (midpoint / RK2) --- */
    xs[0] += ds * c;
    xs[1] += ds * s;
    xs[2] += dth;
    wrap_theta();

    /* --- F = df/dx (state Jacobian, evaluated at pre-update theta) --- */
    float F[3][3] = {
        { 1.0f, 0.0f, -ds * s },
        { 0.0f, 1.0f,  ds * c },
        { 0.0f, 0.0f,  1.0f   }
    };

    /* --- V = df/du (control Jacobian); columns are d/d(ds_L), d/d(ds_R) --- */
    const float half = 0.5f;
    const float ds_over_2b = ds / (2.0f * b);
    float V[3][2];
    V[0][0] = half * c + ds_over_2b * s;
    V[0][1] = half * c - ds_over_2b * s;
    V[1][0] = half * s - ds_over_2b * c;
    V[1][1] = half * s + ds_over_2b * c;
    V[2][0] = -1.0f / b;
    V[2][1] =  1.0f / b;

    /* --- Q_uu = diag(sigma_L^2, sigma_R^2) --- */
    float aL = ds_L < 0.0f ? -ds_L : ds_L;
    float aR = ds_R < 0.0f ? -ds_R : ds_R;
    float sigL = EKF_SLIP_K * aL; if (sigL < EKF_SIGMA_FLOOR_CM) sigL = EKF_SIGMA_FLOOR_CM;
    float sigR = EKF_SLIP_K * aR; if (sigR < EKF_SIGMA_FLOOR_CM) sigR = EKF_SIGMA_FLOOR_CM;
    const float qLL = sigL * sigL;
    const float qRR = sigR * sigR;

    /* --- Q = V * Q_uu * V^T (3x3); Q_uu diagonal so inline the product --- */
    float Q[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Q[i][j] = V[i][0] * qLL * V[j][0] + V[i][1] * qRR * V[j][1];
        }
    }

    /* --- P = F * P * F^T + Q --- */
    float FP[3][3];
    mat3_mul(F, P, FP);
    float FPFT[3][3];
    mat3_mul_tr(FP, F, FPFT);
    mat3_add(FPFT, Q, P);
}

/* -------- Per-sensor measurement helpers -------- */

/* Sensor world origin and beam-direction unit vector given robot pose and
 * mount offset (d_x_robot, d_y_robot, phi_sensor). */
static void sensor_world(float px, float py, float th,
                         float dx_r, float dy_r, float phi_s,
                         float *ox, float *oy, float *bx, float *by)
{
    const float c = cosf(th), s = sinf(th);
    *ox = px + dx_r * c - dy_r * s;
    *oy = py + dx_r * s + dy_r * c;
    const float beam = th + phi_s;
    *bx = cosf(beam);
    *by = sinf(beam);
}

/* Returns true and fills *t_hit and the hit wall's normal (nx, ny) +
 * implicit-form offset d (so n.x*x + n.y*y = d) if a wall is hit within
 * [0, EKF_RANGE_MAX_CM]. Picks the nearest valid intersection. */
static bool raycast_walls(float ox, float oy, float rx, float ry,
                          float *t_hit, float *nx_out, float *ny_out, float *d_out)
{
    float best_t = EKF_RANGE_MAX_CM;
    bool  found  = false;
    for (uint8_t i = 0; i < n_walls; i++) {
        const float x0 = walls[i].x0, y0 = walls[i].y0;
        const float ex = walls[i].x1 - x0;
        const float ey = walls[i].y1 - y0;
        const float det = ex * ry - rx * ey;
        if (det > -1e-6f && det < 1e-6f) continue;   /* parallel */
        const float inv = 1.0f / det;
        const float dx  = x0 - ox;
        const float dy  = y0 - oy;
        /* Cramer's rule on  | rx -ex | (t)   = | dx |
         *                   | ry -ey | (s)     | dy |  with det = ex*ry - rx*ey. */
        const float t   = (ex * dy - ey * dx) * inv;
        const float seg = (rx * dy - ry * dx) * inv;
        if (t < 0.0f || t >= best_t) continue;
        if (seg < 0.0f || seg > 1.0f) continue;
        best_t = t;
        /* Implicit-form normal of the wall segment. Direction sign isn't
         * load-bearing for H — chain rule keeps it consistent. */
        const float len = sqrtf(ex * ex + ey * ey);
        if (len < 1e-6f) continue;
        *nx_out = -ey / len;
        *ny_out =  ex / len;
        *d_out  = (*nx_out) * x0 + (*ny_out) * y0;
        found   = true;
    }
    if (found) *t_hit = best_t;
    return found;
}

/* Sequential 1D Kalman update for one sensor. H is 1x3; analytical row
 * derived from z_hat = (d - n.ox - n.oy) / (n . beam_dir). */
static void update_one(float z_meas, float sigma_meas,
                       float dx_r, float dy_r, float phi_s)
{
    const float px = xs[0], py = xs[1], th = xs[2];
    float ox, oy, bx, by;
    sensor_world(px, py, th, dx_r, dy_r, phi_s, &ox, &oy, &bx, &by);

    float t_hit, nx, ny, d;
    if (!raycast_walls(ox, oy, bx, by, &t_hit, &nx, &ny, &d)) return;

    const float denom = nx * bx + ny * by;
    if (denom > -1e-4f && denom < 1e-4f) return;   /* grazing — unstable H */

    /* z_hat already computed as t_hit, but recompute via the closed form so
     * H derivatives stay consistent with how z_hat is expressed. */
    const float z_hat = (d - nx * ox - ny * oy) / denom;

    /* Theta derivative: chain through both the sensor origin (ox, oy depend
     * on theta via the mount rotation) and the beam direction (bx, by depend
     * on theta via theta + phi). */
    const float cth = cosf(th), sth = sinf(th);
    const float dox_dth = -dx_r * sth - dy_r * cth;
    const float doy_dth =  dx_r * cth - dy_r * sth;
    const float beam    = th + phi_s;
    const float dbx_dth = -sinf(beam);
    const float dby_dth =  cosf(beam);
    const float dDenom_dth = nx * dbx_dth + ny * dby_dth;

    /* H = d(z_hat)/d(state) */
    const float H0 = -nx / denom;
    const float H1 = -ny / denom;
    const float H2 = (-nx * dox_dth - ny * doy_dth - z_hat * dDenom_dth) / denom;

    /* y = z - z_hat */
    const float y = z_meas - z_hat;

    /* S = H P H^T + R  (scalar) */
    float HP[3];
    HP[0] = H0 * P[0][0] + H1 * P[1][0] + H2 * P[2][0];
    HP[1] = H0 * P[0][1] + H1 * P[1][1] + H2 * P[2][1];
    HP[2] = H0 * P[0][2] + H1 * P[1][2] + H2 * P[2][2];
    const float R = sigma_meas * sigma_meas;
    const float S = HP[0] * H0 + HP[1] * H1 + HP[2] * H2 + R;
    if (S <= 0.0f) return;   /* numerical guard */

    /* Mahalanobis gating */
    const float gate = EKF_INNOV_GATE * sqrtf(S);
    if (y > gate || y < -gate) return;

    /* K = P H^T / S */
    float K[3];
    K[0] = (P[0][0] * H0 + P[0][1] * H1 + P[0][2] * H2) / S;
    K[1] = (P[1][0] * H0 + P[1][1] * H1 + P[1][2] * H2) / S;
    K[2] = (P[2][0] * H0 + P[2][1] * H1 + P[2][2] * H2) / S;

    /* x += K * y */
    xs[0] += K[0] * y;
    xs[1] += K[1] * y;
    xs[2] += K[2] * y;
    wrap_theta();

    /* Joseph form: P = (I - K H) P (I - K H)^T + K R K^T */
    float IKH[3][3] = {
        { 1.0f - K[0]*H0, -K[0]*H1, -K[0]*H2 },
        { -K[1]*H0, 1.0f - K[1]*H1, -K[1]*H2 },
        { -K[2]*H0, -K[2]*H1, 1.0f - K[2]*H2 }
    };
    float IKHP[3][3];
    mat3_mul(IKH, P, IKHP);
    float IKHPIKHT[3][3];
    mat3_mul_tr(IKHP, IKH, IKHPIKHT);
    float KRKT[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            KRKT[i][j] = K[i] * R * K[j];
    mat3_add(IKHPIKHT, KRKT, P);
}

void ekf_update(int dF_cm, int dL_cm, int dR_cm,
                int sF,    int sL,    int sR)
{
    if (n_walls == 0) return;

    float sigF = (float)sF; if (sigF < EKF_R_MIN_CM) sigF = EKF_R_MIN_CM;
    float sigL = (float)sL; if (sigL < EKF_R_MIN_CM) sigL = EKF_R_MIN_CM;
    float sigR = (float)sR; if (sigR < EKF_R_MIN_CM) sigR = EKF_R_MIN_CM;

    /* z==0 means "no echo" in this codebase; skip that sensor row. */
    if (dF_cm > 0) update_one((float)dF_cm, sigF, US_F_DX, US_F_DY, US_F_PHI);
    if (dL_cm > 0) update_one((float)dL_cm, sigL, US_L_DX, US_L_DY, US_L_PHI);
    if (dR_cm > 0) update_one((float)dR_cm, sigR, US_R_DX, US_R_DY, US_R_PHI);
}

void ekf_get(float *x_out, float *y_out, float *th_out)
{
    if (x_out  != 0) *x_out  = xs[0];
    if (y_out  != 0) *y_out  = xs[1];
    if (th_out != 0) *th_out = xs[2];
}

void ekf_get_cov_diag(float *vx, float *vy, float *vth)
{
    if (vx  != 0) *vx  = P[0][0];
    if (vy  != 0) *vy  = P[1][1];
    if (vth != 0) *vth = P[2][2];
}

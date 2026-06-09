/**
  ******************************************************************************
  * @file    main.c
  * @brief   Baseline — peripheral init + RTOS tasks per README spec.
  *
  *  Tasks (README "Functions"):
  *    UltraSonicTask  : median7 / stddev7 / SensorTask
  *    IR_Task         : IR_Task
  *    ControlTask     : Motor_Drive / Motor_Stop / switchTracking /
  *                      angleAdjusting / angleCalculate /
  *                      canProgressDirection / isEmergency /
  *                      emergencyResolved / ControlTask
  ******************************************************************************
  */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "cmsis_os.h"
#include "odom.h"
#include "ekf.h"
#include "map_geom.h"

/* ===========================================================================
 *  Calibration / tunables  (tune everything here)
 * =========================================================================== */
/* Sensing */
#define SAMPLE_N          7        /* sliding window length per signal */
#define US_TICKS_PER_CM   58       /* TIM3 IC diff -> cm divisor       */

/* Task timing (ms) */
#define CTRL_PERIOD_MS    20
#define SENS_PERIOD_MS    20
#define IR_PERIOD_MS      20
#define DBG_PERIOD_MS     100       /* DebugTask tick — LED refresh rate */
#define TASK_WARMUP_MS    200
#define CTRL_WARMUP_MS    300

/* Distance thresholds (cm) */
#define D_TARGET          8        /* wall-follow target distance */
#define D_MIN             4        /* lower safety bound          */
#define D_OPEN            150      /* > D_OPEN => "no wall on this side" */
#define EMG_FRONT         8        /* emergency front threshold (cm) */
#define EMG_FRONT_HYST    2        /* +cm margin to clear EMERGENCY */
#define IR_BUMPER_THRESH  2100     /* raw ADC; ir_left/right ABOVE this => bumper triggered. */
#define EMERG_IR_DEG      30       /* rotation angle when IR bumper triggers EMERGENCY */
#define EMERG_US_DEG      90       /* rotation angle when front ultrasonic triggers EMERGENCY */

/* Motor PWM */
#define PWM_PERIOD          20000
#define V_CRUISE            17000   /* duty for all forward drive (SEEK / ALIGN / NON_ALIGN) */
#define V_TRIM_L            500     /* extra duty on LEFT wheel for straight-line drift trim
                                     * (motors/wheels asymmetric -- same duty != same speed). */
#define V_TURN              20000   /* full duty for max pivot torque */

/* Iterative pivot — the only rotation primitive.
 * 90° = PIVOT_SUBSTEPS_90_x × PIVOT_SUBSTEP_TICKS encoder ticks total.
 * L/R split because the two wheels have slightly different effective
 * ticks-per-degree (motor/encoder asymmetry). Tune by 4×90° round-trip
 * calibration (CALIB_PIVOT mode); keep tick count fixed.
 * Note: each micro-pivot uses a start-snapshot of the encoder rather than
 * resetting it, so motorInterrupt1/2 remain monotonic for odometry. */
#define PIVOT_SUBSTEP_TICKS       30   /* encoder ticks per micro-pivot (~3°) */
#define PIVOT_SUBSTEPS_90_L       24   /* micro-pivots for 90° LEFT  (calib 2026-05-27 final) */
#define PIVOT_SUBSTEPS_90_R       24   /* micro-pivots for 90° RIGHT (calib 2026-05-27 final) */
#define PIVOT_PAUSE_MS            10   /* brief stop between micro-pivots */
#define PIVOT_SUBSTEP_TIMEOUT_MS  200  /* per-substep safety */
#define POST_TURN_SETTLE_MS       300  /* let SensorTask median refresh after pivot */

/* HC-SR04 trigger */
#define TRIG_PULSE        2

/* ADC */
#define ADC_POLL_TIMEOUT  0xFF

/* ALIGN_PROGRESS side-wall avoidance: when a side reads < D_MIN cm, stop and
 * pivot away by this many degrees (replaces the previous half-speed veer-off). */
#define ROTATE_VEER_DEG   10

/* After a small pivot (VEER or IR-triggered EMERGENCY), drive forward this
 * long so the next FSM tick isn't stuck in the same trigger zone -- in-place
 * pivots don't change position, so the trigger condition recurs without a
 * forward step. Split per trigger because their geometry differs:
 *   - VEER fires on side ultrasonic (further from front), needs longer escape
 *   - IR fires near the front corners, shorter forward step is enough        */
#define ESCAPE_FORWARD_MS_VEER 700
#define ESCAPE_FORWARD_MS_IR   500
#define VEER_COOLDOWN_TICKS    50   /* 50 * CTRL_PERIOD_MS = 1s lockout after each VEER */

/* Corner-turn (fixed-side wall follower): when the tracked-side ultrasonic
 * jumps from a tracking value (<= D_TARGET) to a much larger one in a single
 * tick, the wall just ended at a corridor opening on that side. Pivot 90°
 * INTO that opening so the wall stays on the tracked side. CORNER_RATE_CM
 * is the minimum increase between consecutive samples to qualify as a real
 * corner (rather than a transient miss); CORNER_COOLDOWN_TICKS prevents
 * re-firing while still mid-escape. */
#define CORNER_RATE_CM         15   /* dR - dR_prev (or 0/D_OPEN) to flag corner */
#define CORNER_TURN_DEG        90   /* pivot magnitude toward the opening */
#define CORNER_COOLDOWN_TICKS  50   /* 1s lockout after a corner pivot */
#define CORNER_ESCAPE_MS       700  /* forward drive after corner so we settle into the new wall */
#define CORNER_LEAD_MS         500  /* forward drive BEFORE corner pivot so the back end clears
                                     * the wall edge that just ended (side US sits at the flank,
                                     * so by the time the pivot starts the corner is at the side
                                     * — driving past it first gives the rear swing room). */

/* Pivot calibration mode — when 1, ControlTask skips the FSM and runs a
 * 4x90° round-trip (right then left) so PIVOT_SUBSTEPS_90 can be measured
 * against floor markings. Set to 0 for normal driving. */
#define CALIB_PIVOT       0

/* IR calibration mode — when 1, ControlTask skips the FSM, motors stay
 * stopped, and IR readings stream to UART at 100ms cadence so a human
 * can wave obstacles in front of each sensor to find IR_BUMPER_THRESH. */
#define CALIB_IR          0

/* Odometry calibration:
 *   1 = straight-line: drive V_CRUISE for 3 s, three reps; print enc deltas
 *       so the operator measures floor distance and computes TICKS_PER_CM.
 *       Also reveals ENC_R_SIGN / ENC_L_SIGN (flip in odom.h if dR/dL come
 *       out with opposite signs on a forward drive).
 *   2 = square test: drive a 1 m × 1 m square (forward 100 cm, rotate 90°
 *       right, repeat 4×) and stream the integrated (x, y, theta). Expected
 *       end pose ≈ (0, 0, 0); use the residual to gauge WHEEL_BASE_CM and
 *       PIVOT_SUBSTEPS_90_R quality. Run after CALIB_ODOM=1 values are in odom.h. */
#define CALIB_ODOM        0

/* ===========================================================================
 *  Peripherals
 * =========================================================================== */
TIM_HandleTypeDef    TimHandle1, TimHandle2, TimHandle3, TimHandle4;
TIM_IC_InitTypeDef   sICConfig;
TIM_OC_InitTypeDef   sConfig1, sConfig2, sConfig3;

uint32_t uwPrescalerValue = 0;
volatile uint16_t motorInterrupt1 = 0;     /* Right encoder */
volatile uint16_t motorInterrupt2 = 0;     /* Left  encoder */
uint8_t  encoder_right  = READY;
uint8_t  encoder_left   = READY;

/* Ultrasonic input-capture raw diffs (TIM3 CH2/3/4) */
uint32_t uwIC2Value1 = 0, uwIC2Value2 = 0, uwDiffCapture1 = 0;  /* Right */
uint32_t uwIC2Value3 = 0, uwIC2Value4 = 0, uwDiffCapture2 = 0;  /* Front */
uint32_t uwIC2Value5 = 0, uwIC2Value6 = 0, uwDiffCapture3 = 0;  /* Left  */
uint32_t uwFrequency = 0;

ADC_HandleTypeDef       AdcHandle1, AdcHandle2, AdcHandle3;
ADC_ChannelConfTypeDef  adcConfig1, adcConfig2, adcConfig3;

__IO uint32_t uhADCxRight;
__IO uint32_t uhADCxForward;
__IO uint32_t uhADCxLeft;

extern UART_HandleTypeDef UartHandle1, UartHandle2;

/* ===========================================================================
 *  Shared sensing state
 * =========================================================================== */
static int us_buf_F[SAMPLE_N], us_buf_L[SAMPLE_N], us_buf_R[SAMPLE_N];
static int us_idx = 0;

int dF = 0, dL = 0, dR = 0;            /* filtered distances (cm)        */
int sF = 0, sL = 0, sR = 0;            /* per-window stddev (confidence) */
int dF_prev = 0, dL_prev = 0, dR_prev = 0;

int ir_floor = 0, ir_left = 0, ir_right = 0;

DriveState state = INIT;

/* Discrete cardinal heading, updated on every committed 90° pivot
 * (CORNER and EMERGENCY-US). Boot orientation is taken as NORTH; the
 * EMERGENCY front-blocked branch uses this to bias its pivot direction
 * toward whichever of the two 90° outcomes lands closer to NORTH. Small
 * pivots (VEER 10°, EMERGENCY IR 30°) intentionally do NOT update this
 * because they aren't cardinal-aligned; the model accepts that drift. */
typedef enum {
    HEAD_N = 0,
    HEAD_E = 1,
    HEAD_S = 2,
    HEAD_W = 3
} CardHead;
static CardHead heading = HEAD_N;

/* EMERGENCY commit state: lock turn direction until clear for a while */
static bool emerg_committed  = false;
static bool emerg_turn_left  = false;
static int  emerg_turn_deg   = EMERG_US_DEG;
static int  seek_clear_ticks = 0;
#define EMERG_RELEASE_TICKS  25

/* VEER LED display */
static int  veer_show_ticks = 0;
static bool veer_show_left  = false;
#define VEER_LED_SHOW_TICKS  20

/* CORNER / VEER cooldowns */
static int  veer_cooldown_ticks   = 0;
static int  corner_cooldown_ticks = 0;

/* ===========================================================================
 *  Forward declarations
 * =========================================================================== */
static void SystemClock_Config(void);
static void EXTILine_Config(void);
static void Error_Handler(void);

/* Sensing */
int  median7(int *a);
int  stddev7(int *a);
void SensorTask(void *arg);
void IR_Task(void *arg);

/* Debug */
void DebugTask(void *arg);

/* Control */
void    Motor_Drive(int v_left, int v_right);
void    Motor_Stop(void);
void    rotate_iterative(int degrees, bool left);
uint8_t bestTurnDirection(void);
bool    isEmergency(void);
bool    emergencyResolved(void);
void    ControlTask(void *arg);

/* ===========================================================================
 *  printf -> UART
 * =========================================================================== */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/**
 * @brief  Redirects single-character stdio output to UART1 so printf works.
 * @param  ch  Character to emit (passed by stdio runtime).
 * @return The same character, as expected by stdio.
 */
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/* ===========================================================================
 *  Sensing Layer — UltraSonicTask
 *  7-sample window per signal -> filtered value (median) + confidence (stddev).
 *  Rate of change is derived from (d* - d*_prev).
 * =========================================================================== */

/**
 * @brief  Returns the median of SAMPLE_N (=7) integers via insertion sort.
 * @param  a  Pointer to the sliding-window buffer (SAMPLE_N entries).
 * @return The middle element after sorting a local copy (input is untouched).
 */
int median7(int *a)
{
    int tmp[SAMPLE_N];
    for (int i = 0; i < SAMPLE_N; i++) tmp[i] = a[i];
    for (int i = 1; i < SAMPLE_N; i++) {
        int v = tmp[i], j = i - 1;
        while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = v;
    }
    return tmp[SAMPLE_N / 2];
}

/**
 * @brief  Integer-only standard deviation of SAMPLE_N (=7) samples.
 * @param  a  Pointer to the sliding-window buffer.
 * @return floor(sqrt(variance)); used as a per-sensor noise/confidence proxy.
 */
int stddev7(int *a)
{
    long sum = 0;
    for (int i = 0; i < SAMPLE_N; i++) sum += a[i];
    int mean = (int)(sum / SAMPLE_N);
    long var = 0;
    for (int i = 0; i < SAMPLE_N; i++) {
        long d = a[i] - mean;
        var += d * d;
    }
    var /= SAMPLE_N;
    int s = 0;
    while ((long)(s + 1) * (s + 1) <= var) s++;
    return s;
}


/**
 * @brief  RTOS task — samples HC-SR04 trio and runs odometry + EKF.
 * @param  arg  Unused (FreeRTOS task signature).
 */
void SensorTask(void *arg)
{
    (void)arg;
    osDelay(TASK_WARMUP_MS);
    odom_init();
    ekf_init();
    for (;;) {
        us_buf_F[us_idx] = (int)(uwDiffCapture2 / US_TICKS_PER_CM);
        us_buf_L[us_idx] = (int)(uwDiffCapture3 / US_TICKS_PER_CM);
        us_buf_R[us_idx] = (int)(uwDiffCapture1 / US_TICKS_PER_CM);
        us_idx = (us_idx + 1) % SAMPLE_N;

        dF_prev = dF; dL_prev = dL; dR_prev = dR;
        dF = median7(us_buf_F);
        dL = median7(us_buf_L);
        dR = median7(us_buf_R);
        sF = stddev7(us_buf_F);
        sL = stddev7(us_buf_L);
        sR = stddev7(us_buf_R);

        odom_tick();
        float ds_R, ds_L;
        odom_get_last_delta_cm(&ds_R, &ds_L);
        ekf_predict(ds_L, ds_R);
        ekf_update(dF, dL, dR, sF, sL, sR);

        osDelay(SENS_PERIOD_MS);
    }
}

/* ===========================================================================
 *  Sensing Layer — IR_Task
 * =========================================================================== */
/**
 * @brief  RTOS task — polls the three IR ADC channels (left, right, floor).
 * @param  arg  Unused (FreeRTOS task signature).
 * @note   Period = IR_PERIOD_MS. Stores raw 12-bit ADC values into
 *         ir_left/ir_right/ir_floor for ControlTask diagnostics.
 */
void IR_Task(void *arg)
{
    (void)arg;
    osDelay(TASK_WARMUP_MS);
    for (;;) {
        HAL_ADC_Start(&AdcHandle1);
        HAL_ADC_PollForConversion(&AdcHandle1, ADC_POLL_TIMEOUT);
        ir_left  = (int)HAL_ADC_GetValue(&AdcHandle1);

        HAL_ADC_Start(&AdcHandle2);
        HAL_ADC_PollForConversion(&AdcHandle2, ADC_POLL_TIMEOUT);
        ir_right = (int)HAL_ADC_GetValue(&AdcHandle2);

        HAL_ADC_Start(&AdcHandle3);
        HAL_ADC_PollForConversion(&AdcHandle3, ADC_POLL_TIMEOUT);
        ir_floor = (int)HAL_ADC_GetValue(&AdcHandle3);

        osDelay(IR_PERIOD_MS);
    }
}

/* ===========================================================================
 *  Debug Layer — DebugTask
 *  Three outputs, refreshed every DBG_PERIOD_MS:
 *    1) LED visualization, priority high->low:
 *         a) VEER overlay (~2s after each VEER fires, even while ALIGN_PROG):
 *              LED1 = R-side US trigger,  LED4 = L-side US trigger
 *         b) EMERGENCY state, trigger source (committed in ControlTask):
 *              Front US : LED1+LED2+LED3+LED4 blink all (~5Hz)
 *              IR left  : LED2 solid
 *              IR right : LED3 solid
 *         c) Default: LED1..LED3 = state code binary, LED4 = side (R=0 / L=1)
 *  All UART output is now rotation-event driven (CORNER/VEER/EMERG/ANG-CORR)
 *  in ControlTask. DebugTask only manages LEDs.
 *  Lowest priority so it never starves Sensor/IR/Control.
 * =========================================================================== */
void DebugTask(void *arg)
{
    (void)arg;
    uint32_t tick = 0;

    osDelay(TASK_WARMUP_MS);
    for (;;) {
        /* --- LED priority cascade ------------------------------------- */
        if (veer_show_ticks > 0) {
            veer_show_ticks--;
            BSP_LED_Off(LED1);
            BSP_LED_Off(LED2);
            BSP_LED_Off(LED3);
            BSP_LED_Off(LED4);
            if (veer_show_left) BSP_LED_On(LED4);
            else                BSP_LED_On(LED1);
        } else if (state == EMERGENCY) {
            /* Trigger source committed in ControlTask EMERGENCY branch:
             *   emerg_turn_deg == EMERG_US_DEG  => front US (blink all)
             *   else IR; emerg_turn_left == true  => right bumper (LED3)
             *                          false => left  bumper (LED2)        */
            BSP_LED_Off(LED1);
            BSP_LED_Off(LED2);
            BSP_LED_Off(LED3);
            BSP_LED_Off(LED4);
            if (emerg_turn_deg == EMERG_US_DEG) {
                if ((tick & 1) == 0) {
                    BSP_LED_On(LED1);
                    BSP_LED_On(LED2);
                    BSP_LED_On(LED3);
                    BSP_LED_On(LED4);
                }
            } else if (emerg_turn_left) {
                BSP_LED_On(LED3);   /* R bumper -> turn L */
            } else {
                BSP_LED_On(LED2);   /* L bumper -> turn R */
            }
        } else {
            /* LED1..LED3 = state binary (INIT=000, DRIVE=001, EMERGENCY=010)
             * LED4 = cardinal heading (N=off, E=slow blink, S=on, W=fast blink) */
            uint8_t s = (uint8_t)state;
            (s & 0x1) ? BSP_LED_On(LED1) : BSP_LED_Off(LED1);
            (s & 0x2) ? BSP_LED_On(LED2) : BSP_LED_Off(LED2);
            (s & 0x4) ? BSP_LED_On(LED3) : BSP_LED_Off(LED3);
            /* LED4: N=off, S=on, E/W=toggle with tick parity */
            if      (heading == HEAD_N) BSP_LED_Off(LED4);
            else if (heading == HEAD_S) BSP_LED_On(LED4);
            else { if ((tick & 1) == 0) BSP_LED_On(LED4); else BSP_LED_Off(LED4); }
        }

        /* Rotation-only output policy: all diagnostic printf is in the
         * ControlTask rotation branches (CORNER, VEER, EMERG, ANG-CORR,
         * ROT-ITER). DebugTask only drives the LEDs. */

        tick++;
        osDelay(DBG_PERIOD_MS);
    }
}

/* ===========================================================================
 *  Motor Layer
 *  v_left / v_right ∈ [-PWM_PERIOD, +PWM_PERIOD].  Sign = direction.
 *  TIM8 drives right motor (CH1=fwd, CH2=rev); TIM4 drives left (CH2=fwd, CH1=rev).
 * =========================================================================== */

/**
 * @brief  Sets PWM duty on both motors; sign encodes direction.
 * @param  v_left   Signed duty for left  wheel; positive = forward.
 * @param  v_right  Signed duty for right wheel; positive = forward.
 * @note   Magnitudes are clamped to PWM_PERIOD. Idle channel gets CCR=0 (PWM1
 *         mode outputs LOW), the active channel gets |v|. Channels are pre-
 *         started in main(); this function only updates compare registers.
 */
void Motor_Drive(int v_left, int v_right)
{
    /* PWM channels are pre-started in main(); we just gate by CCR.
     * CCR = 0 with PWM1 mode -> output always LOW (channel idle).
     * TIM8: CH1=right fwd, CH2=right rev
     * TIM4: CH2=left  fwd, CH1=left  rev
     */
    int aL = v_left  < 0 ? -v_left  : v_left;
    int aR = v_right < 0 ? -v_right : v_right;
    if (aL > PWM_PERIOD) aL = PWM_PERIOD;
    if (aR > PWM_PERIOD) aR = PWM_PERIOD;

    if (v_right >= 0) { TIM8->CCR1 = aR; TIM8->CCR2 = 0;  }
    else              { TIM8->CCR1 = 0;  TIM8->CCR2 = aR; }
    if (v_left  >= 0) { TIM4->CCR2 = aL; TIM4->CCR1 = 0;  }
    else              { TIM4->CCR2 = 0;  TIM4->CCR1 = aL; }
}

/**
 * @brief  Coasts the robot — both motors get zero duty without disabling PWM.
 * @note   Uses Motor_Drive(0, 0) so the PWM channels stay enabled and the
 *         next non-zero call takes effect immediately. HAL_TIM_PWM_Stop()
 *         would have to be matched by a Start() and is intentionally avoided.
 */
void Motor_Stop(void)
{
    Motor_Drive(0, 0);
}

/**
 * @brief  Closed-loop in-place pivot driven by encoder ticks, in micro-steps.
 * @param  degrees  Magnitude of rotation in degrees (positive).
 * @param  left     true => pivot CCW (left turn); false => pivot CW (right).
 * @note   Decomposes the turn into `PIVOT_SUBSTEPS_90 * degrees / 90`
 *         micro-pivots, each consuming PIVOT_SUBSTEP_TICKS ticks on the wheel
 *         that goes forward for this direction. Stops between micro-steps to
 *         null out inertia. Uses a start-snapshot of the encoder per substep
 *         instead of resetting it, so motorInterrupt1/2 stay monotonic for
 *         the odometry layer (uint16_t wrap handled in odom.c). Settles
 *         POST_TURN_SETTLE_MS so the median filter refreshes.
 */
void rotate_iterative(int degrees, bool left)
{
    /* Hardware wiring inverts our software convention -- callers say "left"
     * but the original wheel mapping rotates the opposite way. Flip here so
     * every call site (VEER, EMERGENCY, CALIB_PIVOT) is corrected in one place. */
    left = !left;
    int subs90 = left ? PIVOT_SUBSTEPS_90_L : PIVOT_SUBSTEPS_90_R;
    int substeps = (subs90 * degrees) / 90;
    if (substeps <= 0) return;

    /* pivot LEFT  (CCW): right wheel forward -> use motorInterrupt1
     * pivot RIGHT (CW):  left  wheel forward -> use motorInterrupt2 */
    volatile uint16_t *enc = left ? &motorInterrupt1 : &motorInterrupt2;

    int total_ms = 0;
    for (int i = 0; i < substeps; i++) {
        Motor_Stop();
        osDelay(PIVOT_PAUSE_MS);
        /* Snapshot encoder at substep start; compare deltas against it
         * (int16_t cast = wrap-safe signed delta) so the global counter
         * stays monotonic for odometry. */
        int16_t start = (int16_t)*enc;
        if (left) Motor_Drive(-V_TURN,  V_TURN);
        else      Motor_Drive( V_TURN, -V_TURN);
        int t = 0;
        while (t < PIVOT_SUBSTEP_TIMEOUT_MS) {
            int16_t s = (int16_t)((int16_t)*enc - start);
            if (s < 0) s = -s;
            if (s >= PIVOT_SUBSTEP_TICKS) break;
            osDelay(1);
            t++;
        }
        Motor_Stop();
        total_ms += t + PIVOT_PAUSE_MS;
    }
    printf("\r\n>> ROT-ITER %s deg=%d substeps=%d ms=%d",
           left ? "L" : "R", degrees, substeps, total_ms);
    osDelay(POST_TURN_SETTLE_MS);
}

/* ===========================================================================
 *  Control Layer — helpers
 * =========================================================================== */

/**
 * @brief  Pick the best 90° pivot direction given N > E/W > S priority.
 * @return DIR_LEFT or DIR_RIGHT.
 * @note   Priority: heading that results in N wins. If both or neither
 *         would give N, prefer the side with more clearance (wider dL/dR).
 *         E and W are tied in heading score — break tie by sensor width.
 */
uint8_t bestTurnDirection(void)
{
    CardHead after_left  = (CardHead)((heading + 3u) % 4u);
    CardHead after_right = (CardHead)((heading + 1u) % 4u);

    /* Score: N=0, E/W=1, S=2 */
    int sl = (after_left  == HEAD_N) ? 0 : (after_left  == HEAD_S) ? 2 : 1;
    int sr = (after_right == HEAD_N) ? 0 : (after_right == HEAD_S) ? 2 : 1;

    if (sl != sr) return (sl < sr) ? DIR_LEFT : DIR_RIGHT;

    /* Tie: pick the wider/clearer side */
    int eff_L = (dL == 0) ? 9999 : dL;
    int eff_R = (dR == 0) ? 9999 : dR;
    return (eff_L >= eff_R) ? DIR_LEFT : DIR_RIGHT;
}

/**
 * @brief  Advances the cardinal-heading model by one 90° pivot.
 * @param  left  Caller-convention left (matches rotate_iterative arg).
 * @note   90° CCW (left)  : N -> W -> S -> E -> N.
 *         90° CW  (right) : N -> E -> S -> W -> N.
 */
static void heading_apply_turn(bool left)
{
    if (left) heading = (CardHead)((heading + 3u) % 4u);
    else      heading = (CardHead)((heading + 1u) % 4u);
}

/**
 * @brief  Returns true if a left 90° pivot would orient closer to N than
 *         a right 90° pivot, given the current cardinal heading.
 * @note   Score: N=0 (best), E or W=1, S=2 (worst). Tie -> left, matching
 *         the EMERGENCY left-preference default.
 *         Use case: front-blocked EMERGENCY picks turn direction so the
 *         post-pivot heading is as N-ward as physical clearance allows.
 */
static bool prefer_left_for_north(void)
{
    CardHead after_left  = (CardHead)((heading + 3u) % 4u);
    CardHead after_right = (CardHead)((heading + 1u) % 4u);
    int score_left  = (after_left  == HEAD_N) ? 0 : (after_left  == HEAD_S) ? 2 : 1;
    int score_right = (after_right == HEAD_N) ? 0 : (after_right == HEAD_S) ? 2 : 1;
    return score_left <= score_right;
}

/**
 * @brief  True when a forward obstacle is close enough to require evasion.
 * @return dF in (0, EMG_FRONT] OR a side IR bumper reads above IR_BUMPER_THRESH
 *         (direct sensor: higher ADC = closer).
 */
bool isEmergency(void)
{
    return (dF > 0 && dF <= EMG_FRONT) ||
           (ir_left  > IR_BUMPER_THRESH) ||
           (ir_right > IR_BUMPER_THRESH);
}

/**
 * @brief  True when the front has cleared past the hysteresis threshold.
 * @return dF > EMG_FRONT + EMG_FRONT_HYST. Used to release EMERGENCY commit
 *         once the robot is solidly back into safe territory.
 */
bool emergencyResolved(void)
{
    return (dF > EMG_FRONT + EMG_FRONT_HYST);
}

/* ===========================================================================
 *  Control Layer — FSM  (3-state: INIT / DRIVE / EMERGENCY)
 *  Navigation policy: heading priority N > E/W > S, no wall-following.
 *  DRIVE state:  straight cruise + CORNER (opening detected) + VEER (too close).
 *  EMERGENCY:    front blocked or IR bumper -> pivot to best heading direction.
 * =========================================================================== */
void ControlTask(void *arg)
{
    (void)arg;
    uint32_t tick = 0;

    osDelay(CTRL_WARMUP_MS);

#if CALIB_PIVOT
    osDelay(1000);
    for (int i = 0; i < 4; i++) {
        printf("\r\n>> CALIB L %d/4", i + 1);
        rotate_iterative(90, true);
        osDelay(1000);
    }
    printf("\r\n>> CALIB DONE");
    Motor_Stop();
    for (;;) osDelay(1000);
#endif

#if CALIB_IR
    Motor_Stop();
    printf("\r\n>> CALIB IR START");
    for (;;) {
        printf("\r\n>> IR L=%d R=%d F=%d", ir_left, ir_right, ir_floor);
        osDelay(100);
    }
#endif

#if CALIB_ODOM == 1
    osDelay(2000);
    for (int rep = 1; rep <= 3; rep++) {
        printf("\r\n>> CALIB_ODOM straight %d/3 starts in 2 s", rep);
        osDelay(2000);
        odom_init();
        Motor_Drive(V_CRUISE + V_TRIM_L, V_CRUISE);
        osDelay(3000);
        Motor_Stop();
        osDelay(500);
        int32_t dRt, dLt;
        odom_get_ticks(&dRt, &dLt);
        long aR = dRt < 0 ? -dRt : dRt;
        long aL = dLt < 0 ? -dLt : dLt;
        printf("\r\n>> ODOM rep=%d dR=%ld dL=%ld mean_abs=%ld", rep, (long)dRt, (long)dLt, (aR + aL) / 2);
        printf("\r\n>> measure floor distance now (5 s)...");
        osDelay(5000);
    }
    printf("\r\n>> CALIB_ODOM=1 DONE");
    Motor_Stop();
    for (;;) osDelay(1000);
#endif

#if CALIB_ODOM == 2
    osDelay(2000);
    odom_init();
    printf("\r\n>> CALIB_ODOM square start");
    for (int sq = 0; sq < 4; sq++) {
        float x0, y0, th0;
        odom_get(&x0, &y0, &th0);
        Motor_Drive(V_CRUISE + V_TRIM_L, V_CRUISE);
        for (int t = 0; t < 200; t++) {
            osDelay(50);
            float x, y, th;
            odom_get(&x, &y, &th);
            float dx = x - x0, dy = y - y0;
            float disp = sqrtf(dx * dx + dy * dy);
            printf("\r\n>> SQ sq=%d t=%d x=%d y=%d th=%d disp=%d",
                   sq, t, (int)x, (int)y, (int)(th * 57.2957795f), (int)disp);
            if (disp >= 100.0f) break;
        }
        Motor_Stop();
        osDelay(500);
        rotate_iterative(90, false);
        osDelay(500);
    }
    Motor_Stop();
    { float x, y, th; odom_get(&x, &y, &th);
      printf("\r\n>> CALIB_ODOM square END x=%d y=%d th_deg=%d",
             (int)x, (int)y, (int)(th * 57.2957795f)); }
    for (;;) osDelay(1000);
#endif

    for (;;) {
        if (isEmergency()) state = EMERGENCY;

        switch (state) {
            /* ---- INIT: wait for valid sensor readings ---- */
            case INIT:
                if (dF > 0) state = DRIVE;
                break;

            /* ---- DRIVE: straight cruise + CORNER + VEER ---- */
            case DRIVE: {
                /* CORNER: side opening detected (either side jumps open).
                 * heading==N -> skip pivot (just keep going N).
                 * heading!=N -> 500ms lead forward then pivot toward best N heading. */
                if (corner_cooldown_ticks > 0) corner_cooldown_ticks--;
                if (veer_cooldown_ticks   > 0) veer_cooldown_ticks--;

                bool r_had  = (dR_prev > 0 && dR_prev <= D_TARGET);
                bool l_had  = (dL_prev > 0 && dL_prev <= D_TARGET);
                bool r_open = (dR == 0) || (dR > D_TARGET && dR - dR_prev >= CORNER_RATE_CM);
                bool l_open = (dL == 0) || (dL > D_TARGET && dL - dL_prev >= CORNER_RATE_CM);
                bool corner = (corner_cooldown_ticks == 0) && ((r_had && r_open) || (l_had && l_open));

                if (corner) {
                    if (heading == HEAD_N) {
                        printf("\r\n>> CORNER-SKIP (head=N) dR=%d->%d dL=%d->%d",
                               dR_prev, dR, dL_prev, dL);
                    } else {
                        bool turn_left = (bestTurnDirection() == DIR_LEFT);
                        printf("\r\n>> CORNER %s dR=%d->%d dL=%d->%d",
                               turn_left ? "L" : "R", dR_prev, dR, dL_prev, dL);
                        Motor_Drive(V_CRUISE + V_TRIM_L, V_CRUISE);
                        osDelay(CORNER_LEAD_MS);
                        Motor_Stop();
                        osDelay(50);
                        rotate_iterative(CORNER_TURN_DEG, turn_left);
                        if (CORNER_TURN_DEG == 90) heading_apply_turn(turn_left);
                        Motor_Drive(V_CRUISE + V_TRIM_L, V_CRUISE);
                        osDelay(CORNER_ESCAPE_MS);
                    }
                    corner_cooldown_ticks = CORNER_COOLDOWN_TICKS;
                    break;
                }

                /* VEER: side wall too close -> pivot away */
                if (veer_cooldown_ticks == 0 && dR > 0 && dR < D_MIN) {
                    printf("\r\n>> VEER L deg=%d dR=%d", ROTATE_VEER_DEG, dR);
                    veer_show_left = false;
                    veer_show_ticks = VEER_LED_SHOW_TICKS;
                    Motor_Stop(); osDelay(50);
                    rotate_iterative(ROTATE_VEER_DEG, true);
                    Motor_Drive(V_CRUISE + V_TRIM_L, V_CRUISE);
                    osDelay(ESCAPE_FORWARD_MS_VEER);
                    veer_cooldown_ticks = VEER_COOLDOWN_TICKS;
                } else if (veer_cooldown_ticks == 0 && dL > 0 && dL < D_MIN) {
                    printf("\r\n>> VEER R deg=%d dL=%d", ROTATE_VEER_DEG, dL);
                    veer_show_left = true;
                    veer_show_ticks = VEER_LED_SHOW_TICKS;
                    Motor_Stop(); osDelay(50);
                    rotate_iterative(ROTATE_VEER_DEG, false);
                    Motor_Drive(V_CRUISE + V_TRIM_L, V_CRUISE);
                    osDelay(ESCAPE_FORWARD_MS_VEER);
                    veer_cooldown_ticks = VEER_COOLDOWN_TICKS;
                } else {
                    Motor_Drive(V_CRUISE + V_TRIM_L, V_CRUISE);
                }

                if (++seek_clear_ticks >= EMERG_RELEASE_TICKS) emerg_committed = false;
            } break;

            /* ---- EMERGENCY: front/IR blocked -> priority pivot ---- */
            case EMERGENCY: {
                bool first = !emerg_committed;
                if (first) {
                    bool front_emerg = (dF > 0 && dF <= EMG_FRONT);
                    bool ir_l_hit    = (ir_left  > IR_BUMPER_THRESH);
                    bool ir_r_hit    = (ir_right > IR_BUMPER_THRESH);

                    if (front_emerg) {
                        emerg_turn_deg = EMERG_US_DEG;
                        /* bestTurnDirection picks N > E/W(wider) > S;
                         * but clearance check overrides: never pivot into a close wall. */
                        bool want_left   = (bestTurnDirection() == DIR_LEFT);
                        bool left_clear  = (dL == 0) || (dL > EMG_FRONT);
                        bool right_clear = (dR == 0) || (dR > EMG_FRONT);
                        if (want_left && left_clear)        emerg_turn_left = true;
                        else if (!want_left && right_clear) emerg_turn_left = false;
                        else if (left_clear)                emerg_turn_left = true;
                        else if (right_clear)               emerg_turn_left = false;
                        else                                emerg_turn_left = true;
                    } else if (ir_l_hit || ir_r_hit) {
                        emerg_turn_deg = EMERG_IR_DEG;
                        if (ir_l_hit && ir_r_hit) emerg_turn_left = (ir_right > ir_left);
                        else if (ir_l_hit)        emerg_turn_left = false;
                        else                      emerg_turn_left = true;
                    }
                    emerg_committed = true;
                }
                seek_clear_ticks = 0;
                printf("\r\n>> EMERG %s deg=%d commit=%s dF=%d dL=%d dR=%d",
                       emerg_turn_left ? "L" : "R", emerg_turn_deg,
                       first ? "NEW" : "KEEP", dF, dL, dR);
                Motor_Stop();
                osDelay(50);
                rotate_iterative(emerg_turn_deg, emerg_turn_left);
                if (emerg_turn_deg == EMERG_US_DEG) heading_apply_turn(emerg_turn_left);
                if (emerg_turn_deg == EMERG_IR_DEG) {
                    Motor_Drive(V_CRUISE + V_TRIM_L, V_CRUISE);
                    osDelay(ESCAPE_FORWARD_MS_IR);
                }
                state = DRIVE;
            } break;
        }
        tick++;
        osDelay(CTRL_PERIOD_MS);
    }
}

/* ===========================================================================
 *  main
 * =========================================================================== */
/**
 * @brief  System entry point: brings up peripherals, then launches the RTOS.
 * @return Never returns in normal operation. If vTaskStartScheduler() falls
 *         through, LED2 lights solid as a fault indicator and the loop spins.
 * @note   Init order: HAL_Init -> SystemClock_Config -> UART/LED -> motor PWM
 *         (TIM8/TIM4) -> EXTI (encoders) -> ultrasonic IC (TIM3) + trigger
 *         (TIM10) -> three ADCs for IR -> task creation -> scheduler start.
 */
int main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_Init();
    SystemClock_Config();
    BSP_COM1_Init();
    /* LED1..LED4 driven by DebugTask as a 4-bit FSM-state display
     * (LED1..LED3 = state binary, LED4 = side, all-blink on emerg commit). */
    BSP_LED_Init(LED1);
    BSP_LED_Init(LED2);
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);

    /* -------- Motor PWM (TIM8 right, TIM4 left) -------- */
    uwPrescalerValue = (SystemCoreClock / 2) / 1000000;

    __GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   /* MC_EN */

    sConfig1.OCMode     = TIM_OCMODE_PWM1;
    sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig1.Pulse      = 0;                    /* CCR set by Motor_Drive at runtime */

    TimHandle1.Instance               = TIM8;
    TimHandle1.Init.Prescaler         = uwPrescalerValue;
    TimHandle1.Init.Period            = PWM_PERIOD;
    TimHandle1.Init.ClockDivision     = 0;
    TimHandle1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_2);

    sConfig2 = sConfig1;
    TimHandle2.Instance               = TIM4;
    TimHandle2.Init.Prescaler         = uwPrescalerValue;
    TimHandle2.Init.Period            = PWM_PERIOD;
    TimHandle2.Init.ClockDivision     = 0;
    TimHandle2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle2);
    HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_2);

    /* Start all 4 motor PWM channels once; Motor_Drive only updates CCR */
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
    TIM8->CCR1 = 0; TIM8->CCR2 = 0;
    TIM4->CCR1 = 0; TIM4->CCR2 = 0;

    EXTILine_Config();

    /* -------- Ultrasonic input capture (TIM3 CH2/3/4) -------- */
    uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;
    TimHandle3.Instance               = TIM3;
    TimHandle3.Init.Period            = 0xFFFF;
    TimHandle3.Init.Prescaler         = uwPrescalerValue;
    TimHandle3.Init.ClockDivision     = 0;
    TimHandle3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    if (HAL_TIM_IC_Init(&TimHandle3) != HAL_OK) Error_Handler();

    sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
    sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
    sICConfig.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_3);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_4);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_4);

    /* -------- Ultrasonic trigger (TIM10 CH1) -------- */
    uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;
    TimHandle4.Instance               = TIM10;
    TimHandle4.Init.Prescaler         = uwPrescalerValue;
    TimHandle4.Init.Period            = 0xFFFF;
    TimHandle4.Init.ClockDivision     = 0;
    TimHandle4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle4);

    sConfig3.OCMode     = TIM_OCMODE_PWM1;
    sConfig3.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig3.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig3.Pulse      = TRIG_PULSE;
    HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);

    /* -------- IR ADC1/2/3 -------- */
    AdcHandle1.Instance                   = ADC3;
    AdcHandle1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle1.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle1.Init.ScanConvMode          = DISABLE;
    AdcHandle1.Init.ContinuousConvMode    = DISABLE;
    AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle1.Init.NbrOfDiscConversion   = 0;
    AdcHandle1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle1.Init.NbrOfConversion       = 1;
    AdcHandle1.Init.DMAContinuousRequests = DISABLE;
    AdcHandle1.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&AdcHandle1);
    adcConfig1.Channel      = ADC_CHANNEL_11;
    adcConfig1.Rank         = 1;
    adcConfig1.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig1.Offset       = 0;
    HAL_ADC_ConfigChannel(&AdcHandle1, &adcConfig1);

    AdcHandle2.Instance                   = ADC2;
    AdcHandle2.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle2.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle2.Init.ScanConvMode          = DISABLE;
    AdcHandle2.Init.ContinuousConvMode    = DISABLE;
    AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle2.Init.NbrOfDiscConversion   = 0;
    AdcHandle2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle2.Init.NbrOfConversion       = 1;
    AdcHandle2.Init.DMAContinuousRequests = DISABLE;
    AdcHandle2.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&AdcHandle2);
    adcConfig2.Channel      = ADC_CHANNEL_14;
    adcConfig2.Rank         = 1;
    adcConfig2.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig2.Offset       = 0;
    HAL_ADC_ConfigChannel(&AdcHandle2, &adcConfig2);

    AdcHandle3.Instance                   = ADC1;
    AdcHandle3.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle3.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle3.Init.ScanConvMode          = DISABLE;
    AdcHandle3.Init.ContinuousConvMode    = DISABLE;
    AdcHandle3.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle3.Init.NbrOfDiscConversion   = 0;
    AdcHandle3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle3.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle3.Init.NbrOfConversion       = 1;
    AdcHandle3.Init.DMAContinuousRequests = DISABLE;
    AdcHandle3.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&AdcHandle3);
    adcConfig3.Channel      = ADC_CHANNEL_15;
    adcConfig3.Rank         = 1;
    adcConfig3.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig3.Offset       = 0;
    HAL_ADC_ConfigChannel(&AdcHandle3, &adcConfig3);

    /* -------- RTOS tasks (sizes match main_good) -------- */
    xTaskCreate(SensorTask,  "sensor",  1024, NULL, 3, NULL);   /* +EKF stack frame */
    xTaskCreate(IR_Task,     "ir",       512, NULL, 3, NULL);
    xTaskCreate(ControlTask, "control", 1024, NULL, 2, NULL);
    xTaskCreate(DebugTask,   "debug",    512, NULL, 1, NULL);
    vTaskStartScheduler();

    /* Only reached if scheduler failed */
    while (1) { }
}

/* ===========================================================================
 *  System clock
 * =========================================================================== */
/**
 * @brief  Configures HSE+PLL clock tree to 180MHz with overdrive.
 * @note   PLL: HSE / 25 * 360 / 2 = 180MHz SYSCLK. AHB=180, APB1=45, APB2=90.
 *         Flash latency set to 5 wait states for 180MHz operation.
 */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    __PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 360;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    HAL_PWREx_ActivateOverDrive();

    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                        RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  HAL assert hook used when USE_FULL_ASSERT is defined.
 * @param  file  Source file where the assert tripped (unused).
 * @param  line  Line number of the failing assert (unused).
 * @note   Currently traps in an infinite loop without logging.
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
    while (1) { }
}
#endif

/* ===========================================================================
 *  ISR callbacks
 * =========================================================================== */
/**
 * @brief  EXTI ISR — increments encoder counters from quadrature edges.
 * @param  GPIO_Pin  Pin that triggered the EXTI (PA15 = right, PB4 = left).
 * @note   Reads the partner pin (PB3 / PB5) to decide direction. Counter
 *         increments on logic-0 partner, decrements on logic-1. `READY` is
 *         the sentinel between edges so transient reads don't double-count.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_15:
            encoder_right = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
            if      (encoder_right == 0) { motorInterrupt1++; encoder_right = READY; }
            else if (encoder_right == 1) { motorInterrupt1--; encoder_right = READY; }
            break;
        case GPIO_PIN_4:
            encoder_left = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
            if      (encoder_left == 0) { motorInterrupt2++; encoder_left = READY; }
            else if (encoder_left == 1) { motorInterrupt2--; encoder_left = READY; }
            break;
    }
}

/**
 * @brief  TIM input-capture ISR — measures HC-SR04 echo pulse widths on TIM3.
 * @param  htim  HAL timer handle; only TIM3 is handled here.
 * @note   Each of CH2 (right), CH3 (front), CH4 (left) toggles capture
 *         polarity on every edge. Pulse width = falling - rising, with
 *         16-bit wraparound handled inline. Result feeds uwDiffCapture1/2/3
 *         consumed by SensorTask (divided by US_TICKS_PER_CM for cm).
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        if ((TIM3->CCER & TIM_CCER_CC2P) == 0) {
            uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            TIM3->CCER |= TIM_CCER_CC2P;
        } else {
            uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            uwDiffCapture1 = (uwIC2Value2 > uwIC2Value1)
                ? (uwIC2Value2 - uwIC2Value1)
                : ((0xFFFF - uwIC2Value1) + uwIC2Value2);
            TIM3->CCER &= ~TIM_CCER_CC2P;
        }
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        if ((TIM3->CCER & TIM_CCER_CC3P) == 0) {
            uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            TIM3->CCER |= TIM_CCER_CC3P;
        } else {
            uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            uwDiffCapture2 = (uwIC2Value4 > uwIC2Value3)
                ? (uwIC2Value4 - uwIC2Value3)
                : ((0xFFFF - uwIC2Value3) + uwIC2Value4);
            TIM3->CCER &= ~TIM_CCER_CC3P;
        }
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        if ((TIM3->CCER & TIM_CCER_CC4P) == 0) {
            uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            TIM3->CCER |= TIM_CCER_CC4P;
        } else {
            uwIC2Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            uwDiffCapture3 = (uwIC2Value6 > uwIC2Value5)
                ? (uwIC2Value6 - uwIC2Value5)
                : ((0xFFFF - uwIC2Value5) + uwIC2Value6);
            TIM3->CCER &= ~TIM_CCER_CC4P;
        }
    }
}

/**
 * @brief  Fatal-error trap — lights LED3 and stops forward progress.
 * @note   Called when an HAL init routine returns HAL_ERROR. No recovery.
 */
static void Error_Handler(void)
{
    BSP_LED_On(LED3);
    while (1) { }
}

/**
 * @brief  Wires up the encoder EXTI lines (PA15, PB3/4/5) and enables NVIC.
 * @note   PA15 -> EXTI15_10 (right encoder edge). PB4 -> EXTI4 (left encoder
 *         edge). PB3/PB5 are partner pins read inside the EXTI callback to
 *         determine rotation direction. All set to rising-edge interrupts.
 */
static void EXTILine_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    __GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    __GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/************************ (C) BaseLine ************************/
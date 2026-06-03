## Premise
- ~~Right Tracking~~ $\rightarrow$ Dynamic switching Tracking
-  초음파 3: 전(dF) · 좌(dL) · 우(dR) — 거리 측정 2 ~ 40cm 각도 넓은 편
-  IR 3: 바닥 · 좌 · 우 (코앞 접촉 직전 범퍼) 3~15 cm 각도 좁은 편
-  모터 2: 좌/우 차동 구동
-  LED 4 (LED1~LED4): DebugTask 상태/이벤트 시각화
- 직진시엔 벽과 parallel하게 직진한다고 가정.
#### Drive Scenario
<img width="1472" height="2710" alt="image" src="https://github.com/user-attachments/assets/29473391-1861-4f0a-98c7-ccb84dcbc591" />


ISLAND 없다고 가정시 T-Stem(5), OPEN(6)은 없는걸로 볼 수 있음.

## Finite State Machine
<img width="1472" height="1030" alt="image" src="https://github.com/user-attachments/assets/cc1a77b3-5687-4951-8b78-e307a2785b40" />


#### Sensing Layer
We maintain a 7-sample window per signal, giving us both the filtered value and its confidence measure. The rate of change is also available.
#### Motor Layer
The robot supports multiple drive modes. We maintain straight-line motion based on the rate of change between sensor readings, while compensating for drift using adaptive coordinate systems that scale with operating time.
#### Control Layer
An FSM handles state transitions and condition checks. Wall-switching logic is also required for dynamic tracking selection.
#### Debug Layer
A low-priority task mirrors FSM state and event triggers onto the 4 onboard LEDs and prints a periodic snapshot to UART. LED encoding prioritizes transient events (VEER, EMERGENCY) over the steady state-code so a glance at the board tells you what just fired.

## Island Premised
- Dynamic Tracking Premised
- Parallel with wall Straight 
- Visited Like -Sparse event log
#### Sensing Layer
#### Motor Layer
#### Control Layer

#### ++Coorder Layer
---
## Functions
#### UltraSonicTask
###### Subfunction
- `int  median7(int *a)` — 7-샘플 슬라이딩 윈도우 중앙값 (insertion sort)
- `int  stddev7(int *a)` — 7-샘플 표준편차 (정수 floor-sqrt, 신뢰도 지표)
- `void angleHistoryFlush(void)` — 각도 보정 ring buffer 초기화 (회전 후/추적측 변경 시)
###### Mainfunction
- `void SensorTask(void *arg)` — 20ms 주기, 초음파 trio 샘플링 + 필터링 + ring buffer push

#### IR_Task
###### Mainfunction
- `void IR_Task(void *arg)` — 20ms 주기, IR ADC 3채널 (좌/우/바닥) 폴링

#### ControlTask
###### Subfunction (Motor Layer)
- `void Motor_Drive(int v_left, int v_right)` — 좌/우 PWM 듀티 설정 (부호 = 방향)
- `void Motor_Stop(void)` — 양쪽 듀티 0으로 coast (PWM 채널은 유지)
- `void rotate_iterative(int degrees, bool left)` — 엔코더 기반 micro-pivot 회전 (rotate primitive)

###### Subfunction (Control Helpers)
- `bool switchTracking(void)` — Dynamic tracking sticky 정책, 추적측 끊기면 swap
- `int  angleCalculate(void)` — arctan 기반 heading 편차 계산 (signed degrees, 소스 자동 선택)
- `void angleAdjusting(void)` — `|θ| >= ANGLE_CORRECT_DEG` 시 이벤트성 회전 보정
- `uint8_t canProgressDirection(void)` — 더 넓은 측면 선택 → DIR_LEFT 또는 DIR_RIGHT (dL vs dR 비교, 0=무한대 처리). EMG_FRONT 제외.
- `bool isEmergency(void)` — 정면 충돌 임박 판정 (`0 < dF <= EMG_FRONT`)
- `bool emergencyResolved(void)` — 정면 hysteresis 해제 (`dF > EMG_FRONT + EMG_FRONT_HYST`)

###### Mainfunction
- `void ControlTask(void *arg)` — 20ms 주기, 5상태 FSM 실행

#### DebugTask
###### Mainfunction
- `void DebugTask(void *arg)` — `DBG_PERIOD_MS` 주기, LED 시각화 + state/side edge 로그 + `DBG_PRINT_DIVIDER` tick마다 UART snapshot (거리/표준편차/IR/EMERGENCY commit). 최저 우선순위 — Sensor/IR/Control을 굶기지 않음.

---
## LED Display (DebugTask)

LED1~LED4를 우선순위 기반으로 갱신해 FSM 상태와 이벤트 트리거를 한눈에 확인.
우선순위 높음 → 낮음:

| 우선순위 | 조건 | LED 패턴 | 의미 |
|---|---|---|---|
| 1 | VEER 오버레이 (~2s, `VEER_LED_SHOW_TICKS`) | **LED1** solid | R-side US 트리거 (R벽 근접 → 좌 피벗) |
| 1 | VEER 오버레이 (~2s) | **LED4** solid | L-side US 트리거 (L벽 근접 → 우 피벗) |
| 2 | `state == EMERGENCY` (전면 US) | **LED1+LED2+LED3+LED4** 동시 점멸 (~5Hz) | `emerg_turn_deg == EMERG_US_DEG` |
| 2 | `state == EMERGENCY` (IR 좌 범퍼) | **LED2** solid | `ir_left > IR_BUMPER_THRESH` |
| 2 | `state == EMERGENCY` (IR 우 범퍼) | **LED3** solid | `ir_right > IR_BUMPER_THRESH` |
| 3 | 그 외 모든 state (기본) | LED1..LED3 = state binary, LED4 = side | INIT=000 / SEEK=001 / ALIGN_PROG=010 / NON_ALIGN=011, LED4: R=0 / L=1 |

> VEER 오버레이는 ALIGN_PROGRESS 중 일시적으로 기본 인코딩을 덮어씀 (사이드월 회피가 더 의미 있는 시각 정보). EMERGENCY LED는 `emerg_turn_deg` / `emerg_turn_left` (commit된 트리거 소스)에 기반하므로 회전 중 센서가 풀려도 패턴이 유지됨.

---
## Constants & Defines

### `main.h` — 공유 타입
```c
#define READY  3                          /* encoder ISR sentinel */

typedef enum {
    INIT = 0, SEEK, ALIGN_PROGRESS,
    NON_ALIGN_PROGRESS, EMERGENCY
} DriveState;

typedef enum { TRACK_LEFT = 0, TRACK_RIGHT } TrackingSide;

#define DIR_LEFT     0x01                 /* canProgressDirection() bits */
#define DIR_FORWARD  0x02
#define DIR_RIGHT    0x04
```

### Sensing
| 상수 | 값 | 의미 |
|---|---|---|
| `SAMPLE_N` | 7 | 슬라이딩 윈도우 길이 |
| `US_TICKS_PER_CM` | 58 | TIM3 IC diff → cm 변환 분모 |

### Task Timing (ms)
| 상수 | 값 | 의미 |
|---|---|---|
| `CTRL_PERIOD_MS` | 20 | ControlTask 주기 |
| `SENS_PERIOD_MS` | 20 | SensorTask 주기 |
| `IR_PERIOD_MS` | 20 | IR_Task 주기 |
| `DBG_PERIOD_MS` | 100 | DebugTask tick — LED 갱신 단위 |
| `DBG_PRINT_DIVIDER` | 5 | printf snapshot은 N tick마다 발생 (= 500ms) |
| `TASK_WARMUP_MS` | 200 | Sensor/IR Task 시작 지연 |
| `CTRL_WARMUP_MS` | 300 | ControlTask 시작 지연 |

### Distance Thresholds (cm)
| 상수 | 값 | 역할 |
|---|---|---|
| `D_TARGET` | 8 | 추종 가능 벽 범위 상한 (switchTracking, has_track) |
| `D_MIN` | 4 | 회피 트리거 (SEEK veer-off, ALIGN VEER) |
| `D_OPEN` | 150 | "벽 없음" 임계 (예약, 향후 사용) |
| `EMG_FRONT` | 8 | 정면 EMERGENCY 진입 임계 (cm). **EMERGENCY 진입 분기 우선** — front_emerg=true 면 IR 무시하고 US 분기 (90°) |
| `EMG_FRONT_HYST` | 2 | EMERGENCY 해제용 +cm 마진 |
| `IR_BUMPER_THRESH` | 2100 | IR 범퍼 raw ADC 임계 — `ir_left/right > 이 값` 이면 EMERGENCY (단 front 정상일 때만 IR 분기) |
| `EMERG_IR_DEG` | 30 | EMERGENCY 회전 각도 (IR 범퍼 트리거 시) |
| `EMERG_US_DEG` | 90 | EMERGENCY 회전 각도 (초음파 정면 벽 트리거 시 — 코드 정렬) |
| `ROTATE_VEER_DEG` | 10 | ALIGN_PROGRESS 측면 회피용 회전 각도 (`dR<D_MIN` 또는 `dL<D_MIN` 시) |
| `ESCAPE_FORWARD_MS_VEER` | 700 | VEER 회전 후 강제 직진 시간 (측면 ultrasonic 트리거; 더 멀리 빠져나가야 함) |
| `ESCAPE_FORWARD_MS_IR` | 500 | IR-triggered EMERGENCY 회전 후 강제 직진 시간 (전방 코너 근접) |
| `VEER_COOLDOWN_TICKS` | 50 | VEER 발동 후 잠금 ticks (50 × 20ms = 1초). 떨림떨림 방지 |

### Motor PWM
| 상수 | 값 | 의미 |
|---|---|---|
| `PWM_PERIOD` | 20000 | TIM4/TIM8 Auto-Reload (= 100% duty) |
| `V_CRUISE` | 18000 | 모든 직진 듀티 (SEEK/ALIGN/NON_ALIGN 공통) — 우바퀴에 적용 |
| `V_TRIM_L` | 1000 | 좌바퀴 직진 보정 (좌/우 모터 비대칭 보상). 직진 시 `Motor_Drive(V_CRUISE + V_TRIM_L, V_CRUISE)` |
| `V_TURN` | 20000 | 피벗 회전 듀티 (full torque) |

### Iterative Pivot (회전 캘리브)
| 상수 | 값 | 의미 |
|---|---|---|
| `PIVOT_SUBSTEP_TICKS` | 30 | 마이크로 피벗 1회당 엔코더 틱 (~3°) |
| `PIVOT_SUBSTEPS_90_L` | 24 | 90° 좌회전 마이크로 피벗 횟수 (캘리브 2026-05-27 final) |
| `PIVOT_SUBSTEPS_90_R` | 25 | 90° 우회전 마이크로 피벗 횟수 (캘리브 2026-05-27 final) |
| `PIVOT_PAUSE_MS` | 10 | 마이크로 피벗 사이 정지 시간 |
| `PIVOT_SUBSTEP_TIMEOUT_MS` | 200 | 마이크로 피벗당 안전 timeout |
| `POST_TURN_SETTLE_MS` | 300 | 회전 후 median filter 갱신 대기 |

> L/R 분리: 좌/우 바퀴의 effective ticks-per-degree 가 미세하게 달라서 (모터·인코더 비대칭) 분리 상수가 필요. 캘리브는 `CALIB_PIVOT` 모드에서 4×90° 라운드트립으로 측정 → 시작/끝 방향 일치하도록 각 상수만 조정 (틱 수는 고정).

### Angle Correction (arctan 기반)
| 상수 | 값 | 의미 |
|---|---|---|
| `ANGLE_HISTORY_N` | 10 | Ring buffer 길이 (10 × 20ms = 200ms 누적) |
| `ANGLE_CORRECT_DEG` | 10 | 보정 발동 임계 (이하 무시) |
| `MIN_DF_DELTA_CM` | 2 | `|ΔdF|` 이하면 분모 신뢰 불가 → skip |

### EMERGENCY / VEER Display
| 상수 | 값 | 의미 |
|---|---|---|
| `EMERG_RELEASE_TICKS` | 25 | 회전 방향 commit 유지 시간 (~500ms) |
| `VEER_LED_SHOW_TICKS` | 20 | VEER 발화 후 LED 오버레이 유지 (20 × DBG_PERIOD_MS = 2s) |

### Hardware
| 상수 | 값 | 의미 |
|---|---|---|
| `TRIG_PULSE` | 2 | HC-SR04 트리거 펄스 폭 |
| `ADC_POLL_TIMEOUT` | 0xFF | IR ADC 폴링 timeout |

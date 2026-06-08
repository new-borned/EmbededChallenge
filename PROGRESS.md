# Progress Report — Localization + Mapping Stack

STM32F429 (STM324x9I-EVAL) 기반 벽 추종 로봇에 EKF localization, occupancy grid mapping, A* 경로 계획을 단계적으로 통합하는 작업의 현재 상태.

작업 브랜치: `My_version---Sensor-Task-And-Control-Task-with-some-Algorithm`

| 단계 | 모듈 | 상태 |
|---|---|---|
| Phase 1 | Wheel Odometry | 통합 완료 (캘리브레이션 1회 통과) |
| Phase 2 | EKF Localization | 통합 완료 (1개 벽 시드로 smoke test 통과) |
| Phase 3 | Occupancy Grid + Wall Extraction | 통합 완료 (실주행 검증 대기) |
| Phase 4 | A* 경로 계획 | **미시작** |

---

## 시스템 좌표계

기준 프레임은 부팅 시점의 로봇 위치. 모든 cm/rad.

$$
\mathbf{x} = \begin{bmatrix} p_x \\ p_y \\ \theta \end{bmatrix}, \quad
\mathbf{x}_0 = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}
$$

- $+x$: 로봇이 부팅 시 바라보던 방향
- $+y$: 그 시점의 좌측 (CCW 양수 컨벤션)
- $\theta$: $+x$축 기준 헤딩, $(-\pi, \pi]$ wrap

---

## Phase 1 — Wheel Odometry

### 데이터 소스
EXTI 인코더 ISR이 매 quadrature edge마다 `motorInterrupt1` (우 휠), `motorInterrupt2` (좌 휠)를 증감. `uint16_t`라 wrap 발생 가능 — odometry layer에서 signed delta cast로 보정.

```
delta_R = (int16_t)(motorInterrupt1 - prev_R)
delta_L = (int16_t)(motorInterrupt2 - prev_L)
```

위 캐스트로 한 tick에 $|\Delta| < 2^{15}$ ticks 범위까지 wrap-safe.

### 거리 / 헤딩 적분
캘리브레이션 상수:
- $K = $ `TICKS_PER_CM` (틱당 cm의 역수)
- $b = $ `WHEEL_BASE_CM` (effective kinematic wheelbase)

각 휠 변위:

$$
\Delta s_R = \frac{\Delta_R}{K}, \qquad \Delta s_L = \frac{\Delta_L}{K}
$$

본체 변위와 회전:

$$
\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \qquad
\Delta \theta = \frac{\Delta s_R - \Delta s_L}{b}
$$

### 미드포인트 (RK2) 통합
한 tick 동안 헤딩이 선형 변화한다고 가정하면 평균 헤딩 $\alpha = \theta + \Delta \theta / 2$를 사용해 1차 Euler보다 한 차수 정확:

$$
\begin{aligned}
\alpha &= \theta + \tfrac{\Delta \theta}{2} \\
p_x &\leftarrow p_x + \Delta s \cos \alpha \\
p_y &\leftarrow p_y + \Delta s \sin \alpha \\
\theta &\leftarrow \theta + \Delta \theta
\end{aligned}
$$

`SENS_PERIOD_MS = 20` 주기로 SensorTask가 호출.

### 캘리브레이션 결과 (2026-06-04 측정)
| 상수 | 값 | 출처 |
|---|---|---|
| `TICKS_PER_CM` | $50.8$ | `CALIB_ODOM=1` 직진 3초 × 3회, sub-1% 분산 |
| `WHEEL_BASE_CM` | $22.0$ | 캘리퍼 직접 측정 (휠 접지점간) |
| `ENC_R_SIGN` | $+1$ | `CALIB_ODOM=1` 부호 확인 |
| `ENC_L_SIGN` | $+1$ | 동일 |
| `PIVOT_SUBSTEPS_90_L` | $24$ | 기존 `CALIB_PIVOT` (origin 보존) |
| `PIVOT_SUBSTEPS_90_R` | $25$ | 동일 |

### Rotation Primitive
`rotate_iterative()`는 micro-pivot 폐루프 (각 substep = 30 ticks). **start-snapshot** 방식으로 변경 — `*enc = 0` reset 제거, 시작 시점 인코더 값을 캡처해 누적 변위만 비교. 결과적으로 `motorInterrupt1/2`가 단조 증감해서 odometry에 영향 안 줌.

$$
\text{substep done} \iff \big| \text{(int16)}(\text{enc} - \text{enc}_{\text{start}}) \big| \geq 30
$$

---

## Phase 2 — Extended Kalman Filter

### 상태와 공분산
$$
\mathbf{x} = [p_x, p_y, \theta]^\top \in \mathbb{R}^3, \qquad
\mathbf{P} \in \mathbb{R}^{3 \times 3}
$$

초기화: $\mathbf{x}_0 = \mathbf{0}$, $\mathbf{P}_0 = \mathrm{diag}(0.01, 0.01, (\pi/180)^2)$ (시작 위치 측정 불확실성 1cm·1° 수준).

### Predict
입력 $\mathbf{u} = [\Delta s_L, \Delta s_R]^\top$. 비선형 운동 모델:

$$
f(\mathbf{x}, \mathbf{u}) =
\begin{bmatrix}
p_x + \Delta s \cos(\theta + \tfrac{\Delta \theta}{2}) \\
p_y + \Delta s \sin(\theta + \tfrac{\Delta \theta}{2}) \\
\theta + \Delta \theta
\end{bmatrix}
$$

여기서 $\Delta s = (\Delta s_R + \Delta s_L)/2$, $\Delta \theta = (\Delta s_R - \Delta s_L)/b$.

### 상태 야코비안 $\mathbf{F}_k$
$\partial f / \partial \mathbf{x}$를 $\alpha = \theta + \Delta\theta/2$에서 평가:

$$
\mathbf{F}_k = \begin{bmatrix}
1 & 0 & -\Delta s \sin\alpha \\
0 & 1 & \;\;\;\Delta s \cos\alpha \\
0 & 0 & 1
\end{bmatrix}
$$

### 제어 야코비안 $\mathbf{V}_k$
$\partial f / \partial \mathbf{u}$ — 2열 (좌·우 휠 변위):

$$
\mathbf{V}_k = \begin{bmatrix}
\tfrac{1}{2}\cos\alpha + \tfrac{\Delta s}{2b}\sin\alpha & \tfrac{1}{2}\cos\alpha - \tfrac{\Delta s}{2b}\sin\alpha \\
\tfrac{1}{2}\sin\alpha - \tfrac{\Delta s}{2b}\cos\alpha & \tfrac{1}{2}\sin\alpha + \tfrac{\Delta s}{2b}\cos\alpha \\
-1/b & +1/b
\end{bmatrix}
$$

### 과정 노이즈 $\mathbf{Q}$
휠 슬립을 변위 비례로 모델링. $\sigma_i = \max(k_{\text{slip}} |\Delta s_i|, \sigma_{\text{floor}})$, $k_{\text{slip}} = 0.05$, $\sigma_{\text{floor}} = 0.05\,\mathrm{cm}$:

$$
\mathbf{Q}_{uu} = \mathrm{diag}(\sigma_L^2, \sigma_R^2), \qquad
\mathbf{Q} = \mathbf{V}_k \, \mathbf{Q}_{uu} \, \mathbf{V}_k^\top
$$

이렇게 propagate해야 $(x, y, \theta)$ 상관관계가 보존됨. 하드코딩 diagonal $\mathbf{Q}$는 거짓말.

### 공분산 전파
$$
\mathbf{P} \leftarrow \mathbf{F}_k \, \mathbf{P} \, \mathbf{F}_k^\top + \mathbf{Q}
$$

### Update — Line-Segment Wall List
측정값: 초음파 3개의 거리 $(z_F, z_L, z_R)$ in cm. 모델은 격자 ray-cast 대신 `wall_seg_t` 리스트와 빔의 해석적 교차를 사용 — H 야코비안이 닫힌 형태로 구해져서 finite-diff보다 정확·저렴.

#### 빔 기하
센서 $i$의 마운트 오프셋 $(d_x^i, d_y^i, \phi^i)$ from `ekf.h`:

| 센서 | $d_x$ (cm) | $d_y$ (cm) | $\phi$ (rad) |
|---|---|---|---|
| 전 (F) | $+6$ | $0$ | $0$ |
| 좌 (L) | $+4$ | $+5$ | $+\pi/2$ |
| 우 (R) | $+4$ | $-5$ | $-\pi/2$ |

월드 좌표계 빔 원점:

$$
\mathbf{o}_i = \begin{bmatrix} p_x \\ p_y \end{bmatrix} +
\begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} d_x^i \\ d_y^i \end{bmatrix}
$$

빔 방향: $\beta_i = \theta + \phi^i$, $\hat{\mathbf{b}}_i = (\cos\beta_i, \sin\beta_i)$.

#### Ray–Segment 교차
각 벽 $j$를 $(\mathbf{p}_0^j, \mathbf{p}_1^j)$의 선분으로 두고 $\mathbf{e}^j = \mathbf{p}_1^j - \mathbf{p}_0^j$. 빔 파라미터 $t$와 선분 파라미터 $s$를 동시에 풂:

$$
\mathbf{o}_i + t \, \hat{\mathbf{b}}_i = \mathbf{p}_0^j + s \, \mathbf{e}^j
$$

행렬 형태로:

$$
\begin{bmatrix} b_x & -e_x \\ b_y & -e_y \end{bmatrix}
\begin{bmatrix} t \\ s \end{bmatrix} =
\begin{bmatrix} p_{0x} - o_x \\ p_{0y} - o_y \end{bmatrix}
$$

determinant $\det = e_x b_y - e_y b_x$가 0에 가까우면 (parallel) skip. Cramer's rule로 풀이 후 $t \geq 0$ 그리고 $0 \leq s \leq 1$인 해 중 가장 가까운 $t$가 $\hat{z}_i$.

#### 예측 측정값 $\hat{z}_i$ — 닫힌 형태
벽 법선 $\mathbf{n}^j$와 implicit offset $d^j = \mathbf{n}^j \cdot \mathbf{p}_0^j$로 표기하면:

$$
\hat{z}_i = \frac{d^j - \mathbf{n}^j \cdot \mathbf{o}_i}{\mathbf{n}^j \cdot \hat{\mathbf{b}}_i}
$$

분모가 grazing (≈ 0)이면 H 불안정 → skip.

#### 측정 야코비안 $\mathbf{H}_i$ (1×3 행)
$\partial \hat{z}_i / \partial \mathbf{x}$를 chain rule로:

$$
\frac{\partial \hat{z}_i}{\partial p_x} = -\frac{n_x^j}{\mathbf{n}^j \cdot \hat{\mathbf{b}}_i}, \quad
\frac{\partial \hat{z}_i}{\partial p_y} = -\frac{n_y^j}{\mathbf{n}^j \cdot \hat{\mathbf{b}}_i}
$$

$\theta$ 미분은 빔 방향과 마운트 회전 두 경로를 모두 통과:

$$
\frac{\partial \hat{z}_i}{\partial \theta} =
\frac{-\mathbf{n}^j \cdot \frac{\partial \mathbf{o}_i}{\partial \theta} - \hat{z}_i \cdot \mathbf{n}^j \cdot \frac{\partial \hat{\mathbf{b}}_i}{\partial \theta}}{\mathbf{n}^j \cdot \hat{\mathbf{b}}_i}
$$

여기서
$$
\frac{\partial \mathbf{o}_i}{\partial \theta} = \begin{bmatrix} -d_x^i \sin\theta - d_y^i \cos\theta \\ \;\;\;d_x^i \cos\theta - d_y^i \sin\theta \end{bmatrix}, \quad
\frac{\partial \hat{\mathbf{b}}_i}{\partial \theta} = \begin{bmatrix} -\sin\beta_i \\ \;\;\;\cos\beta_i \end{bmatrix}
$$

### Update — Sequential per-Sensor
$\mathbf{R}$이 diagonal이라 3개 센서를 동시 처리할 필요 없이 순차 1D Kalman update로 처리. 매트릭스 inverse 회피.

각 센서 $i$:

$$
\begin{aligned}
y &= z_i - \hat{z}_i \\
S &= \mathbf{H}_i \mathbf{P} \mathbf{H}_i^\top + \sigma_i^2 \\
\mathbf{K} &= \mathbf{P} \mathbf{H}_i^\top / S
\end{aligned}
$$

#### Innovation Gating
$|y| > 3\sqrt{S}$이면 그 센서 row 제외 — 잘못 추출된 벽으로 인한 EKF 폭주 방지.

#### Joseph form 업데이트
대칭 양정치(PD) 보존:

$$
\mathbf{x} \leftarrow \mathbf{x} + \mathbf{K} y, \quad
\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H}_i) \mathbf{P} (\mathbf{I} - \mathbf{K}\mathbf{H}_i)^\top + \mathbf{K} R \mathbf{K}^\top
$$

### 측정 노이즈 $\mathbf{R}$
기존 `stddev7()`이 반환하는 정수 cm를 1-sigma proxy로:

$$
\sigma_i = \max(\text{stddev7}_i, 1\,\mathrm{cm})
$$

floor 없으면 깨끗한 측정에서 $\sigma = 0$ → $\mathbf{R}$ singular.

---

## Phase 3 — Occupancy Grid + Wall Extraction

### 격자 구조
크기·해상도·시작 셀이 모두 5개 매크로에서 자동 도출:

```
MAP_W_CM = 500, MAP_H_CM = 1000, CELL_CM = 20
START_FRAC_X = 1/2, START_FRAC_Y = 1/10
→ GRID_W = 25, GRID_H = 50, START_CX = 12, START_CY = 5
```

저장: `int8_t grid[GRID_H][GRID_W]` ≈ 1.25 KB BSS. **float 0개** — log-odds saturating integer arithmetic만.

### Log-Odds 표현
셀 값 $\ell \in [-100, +100]$. 베이지안 update:

$$
\ell_{\text{post}} = \mathrm{clip}\big(\ell_{\text{prior}} + \Delta \ell, \, L_{\min}, \, L_{\max}\big)
$$

| 이벤트 | $\Delta \ell$ |
|---|---|
| Free cell (빔이 통과만) | $-2$ |
| Occupied cell (빔 종점이 hit) | $+4$ |

비대칭(점유가 free의 2배 weight) — 드문 진짜 장애물이 잦은 노이즈 free reading으로 묻히지 않도록.

임계값:
- $L_{\text{free}} = -20$: A*에서 "확실히 free"
- $L_{\text{occ}} = +40$: A* 불가, walls_extract 후보

### 월드 → 셀 변환

$\delta$ = `CELL_CM`, $(s_x, s_y)$ = `(START_CX, START_CY)`라 두면:

$$
c_x = \lfloor x / \delta \rfloor + s_x, \qquad
c_y = \lfloor y / \delta \rfloor + s_y
$$

격자 밖은 silently reject (격자 손상 방지).

### Ray Update (Bresenham)
매 SensorTask tick, 각 초음파에 대해:

1. EKF post-fusion pose로 빔 원점·방향 계산
2. 종점·범위:

$$
\mathbf{e}_i = \mathbf{o}_i + r \, \hat{\mathbf{b}}_i, \qquad
r = \min(z_{\text{meas}}, R_{\max}), \qquad
\text{hit} \iff 0 < z_{\text{meas}} < R_{\max}
$$

3. Bresenham 알고리즘으로 원점 셀 → 종점 셀 라인 따라가며:
   - 중간 셀: $\ell \leftarrow \ell - 2$ (saturating)
   - 종점 셀: hit이면 $\ell \leftarrow \ell + 4$, 아니면 $\ell \leftarrow \ell - 2$

신뢰도 게이팅: $\sigma_i > 3$ cm이면 그 센서 update skip.

비용: ~25 µs/tick @ 180 MHz, 무시 가능.

### Wall Extraction
5초 (250 tick) 주기로 SensorTask 안에서 `walls_extract_from_grid()` 호출:

1. `walls[]` clear
2. 각 row $y$ 스캔: 같은 행에 $\ell \geq L_{\text{occ}}$인 셀이 $\geq 3$ 연속이면 그 구간을 가로 wall segment로 push
3. 각 column $x$ 스캔: 같은 열에 동일 조건이면 세로 wall segment로 push
4. 셀 중심 좌표를 cm으로 변환 ($s_x$ = `START_CX`, $\delta$ = `CELL_CM`):

$$
x_{\text{world}}(c) = (c - s_x + 0.5) \cdot \delta
$$

이렇게 추출된 walls가 EKF measurement update의 ray-cast 타겟이 됨 — 자동 chicken-and-egg 해소.

#### 한계
- **WALL_MIN_RUN = 3** → 60cm 미만 벽 무시
- **격자 정렬 가로/세로만** → 대각선 벽은 여러 짧은 조각으로 깨져 MIN_RUN 컷에 걸림
- **OCC_RAY_MAX_CM = 150** → 150cm 너머 벽은 hit으로 안 잡혀 격자에 안 누적

---

## 통합 흐름 (SensorTask, 20ms 주기)

```
1.  ultrasonic IC raw  →  median7  →  stddev7  →  (dF, dL, dR, sF, sL, sR)
2.  odom_tick()           // raw differential-drive integration
3.  ekf_predict(ds_L, ds_R)   // 매트릭스 전파
4.  ekf_update(dF, dL, dR, sF, sL, sR)   // n_walls > 0이면 활성
5.  occ_update(x_ekf, y_ekf, θ_ekf, ...)  // Bresenham 격자 누적
6.  매 250 tick: walls_extract_from_grid()  // EKF 타겟 갱신
```

DebugTask (100ms 주기):
- LED 우선순위 cascade (VEER > EMERGENCY > 기본 state 인코딩)
- 500ms마다 UART snapshot (odom + ekf pose 포함)
- 30s마다 occupancy grid ASCII dump

---

## 알려진 이슈

### 1. 인코더 reverse decoding 의심 (HIGH)
EMERGENCY 90° pivot 후 odom $\theta$가 1°밖에 안 적분되고 $y$는 +4cm 이동으로 적분됨 — 마치 직진한 것처럼. 가설: 한쪽 휠이 후진할 때 EXTI ISR의 partner-pin 디코드가 decrement 못 하고 increment를 누적해서 $\Delta s_R \approx \Delta s_L$ (같은 부호) → $\Delta s > 0$, $\Delta \theta \approx 0$.

검증 방법: CALIB_ODOM=1을 후진(`-V_CRUISE`)으로 한번 돌려서 `dR`/`dL` 부호 확인. 양수 그대로면 reverse decoding 깨진 게 confirm.

영향: EKF 입력이 잘못된 prediction을 받으니 측정 update가 따라잡기 어려움. Phase 4 A* 결과도 odom 신뢰도에 의존.

### 2. 모터 좌우 비대칭 큼
이 보드의 좌우 모터 출력 격차가 일반 디바이스보다 큼. `V_TRIM_L=1000`이 정적 보정용이지만 12초 직진 같은 긴 open-loop에서 비대칭이 누적. CALIB_ODOM=2 정사각 테스트 결과가 run-to-run 분산 큼:

| run | (x, y, θ°) | 비고 |
|---|---|---|
| 1 (b=22) | (73, 94, 108) | 배터리 낮음 의심 |
| 2 (b=?) | (34, 64, -175) | 배터리 풀충전 |
| 3 (b=18.04) | (9, 0, 103) | 위치 닫힘 ✓, θ만 어긋남 |

θ 어긋남 일부는 #1, 일부는 wheelbase 캘리브 미세 차이.

### 3. IMU 부재
이 보드(STM32F429II_HUINS)는 자이로/가속도계 BSP driver 없음. 자이로가 있으면 $\Delta \theta$를 인코더 적분 대신 자이로 적분에서 직접 얻어 위 두 이슈 모두 우회 가능. 현재로선 EKF의 측정 update에 의존.

### 4. Bootstrap chicken-and-egg
초기 5초는 walls[] 비어있어 EKF가 predict-only. 그 동안 격자 누적은 raw odom pose 기준이라 부정확할 수 있음. 5초 이후 첫 walls_extract부터 EKF 측정 update가 동작하면서 점진 안정화.

---

## 캘리브레이션 위치 종합

| 파일 | 매크로 | 현재 값 | 의미 |
|---|---|---|---|
| `odom.h` | `WHEEL_BASE_CM` | `22.0f` | effective kinematic wheelbase |
| `odom.h` | `TICKS_PER_CM` | `50.8f` | 인코더 1 tick의 cm 환산 (역수) |
| `odom.h` | `ENC_R_SIGN`, `ENC_L_SIGN` | `+1`, `+1` | EXTI partner-pin 부호 보정 |
| `main.c` | `PIVOT_SUBSTEPS_90_L` | `24` | 90° 좌회전 substep 수 |
| `main.c` | `PIVOT_SUBSTEPS_90_R` | `25` | 90° 우회전 substep 수 |
| `main.c` | `D_TARGET`, `EMG_FRONT` | 8 cm | wall-following 임계 |
| `main.c` | `V_CRUISE`, `V_TRIM_L`, `V_TURN` | 16k, 1k, 20k | PWM duty |
| `ekf.h` | `US_F/L/R_DX/DY/PHI` | placeholder | 센서 마운트 (TODO 실측) |
| `ekf.h` | `EKF_INNOV_GATE` | `3.0` | $\lvert y \rvert > N\sqrt{S}$ Mahalanobis |
| `occgrid.h` | `MAP_W_CM`, `MAP_H_CM` | 500, 1000 | 월드 크기 |
| `occgrid.h` | `CELL_CM` | 20 | 격자 해상도 |
| `occgrid.h` | `START_FRAC_X`, `_Y` | 1/2, 1/10 | 시작 셀 위치 |
| `occgrid.h` | `L_OCC_THRESH`, `L_FREE_THRESH` | +40, -20 | 분류 임계 |

---

## Phase 4 — A* (미구현, 계획 단계)

격자 위에서 8-connected 경로 계획. MISRA-C 친화:
- malloc 금지: open list (binary heap), closed set (bitset), g-score, came_from 모두 BSS 고정 배열
- 정수 산술: $D_1 = 10$ (cardinal), $D_2 = 14$ (diagonal). Octile heuristic
$$
h(c) = D_1 (d_x + d_y) + (D_2 - 2 D_1) \min(d_x, d_y) = 10 (d_x + d_y) - 6 \min(d_x, d_y)
$$
- 셀 cost: log-odds로부터
$$
\text{cost}(c) = \begin{cases} \infty & \ell(c) \geq L_{\text{occ}} \\ 10 + \max(0, \ell(c) - L_{\text{free}}) & \text{else} \end{cases}
$$
- 메모리: ~8 KB BSS (heap 2.5 KB + closed 0.16 KB + g 2.5 KB + came_from 2.5 KB)

통합 후 ControlTask에 `PATH_FOLLOW` state 추가해 경로 추종까지가 최종 형태.

---

## 커밋 히스토리 (이번 작업분)

```
02aa7eb Auto-dump occupancy grid every 30 s in DebugTask
96abce4 Add occupancy grid + wall extraction (Phase 3)
c3acf67 Seed a single wall ahead of start for EKF measurement-model smoke test
2d85139 Stream odom/ekf pose in DebugTask snapshot; drop IR_Task spam
eb18e07 Drop const from EKF mat3 helpers; remove unused mat3_copy
371dee9 Add EKF localization layer (predict + wall-anchored update)
bb71f6d Apply CALIB_ODOM measurements: TICKS_PER_CM=50.8, WHEEL_BASE_CM=22
3fb7edf Restore DebugTask LED display per README spec
4efe162 Restore tick-based rotate_iterative; keep monotonic via start-snapshot
9e3f42a Document DebugTask LED encoding and tunables table
7d9fd79 Add wheel odometry layer; switch rotate to time-based pivot
```

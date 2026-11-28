# Step 07. 도립진자 수학 모델링 및 1kHz LQR / PD 밸런싱 제어기 구현

> **관련 Linear 이슈**: [CH-11](https://linear.app/ch-cho/issue/CH-11)
> **전체 회로도**: [`docs/images/inverted_pendulum_full_schematic.png`](images/inverted_pendulum_full_schematic.png) | [`docs/images/inverted_pendulum_real_hardware_pinmap.png`](images/inverted_pendulum_real_hardware_pinmap.png)
> **제어 목표**: STM32G474RE **1kHz (`TIM6`) 실시간 루프**에서 상태 벡터 $\mathbf{x} = [x,\ \dot{x},\ \theta,\ \dot{\theta}]^T$를 추정하고, $u = -\mathbf{K}\mathbf{x}$ 제어로 진자를 12시 수직 상태로 세운 채 카트를 레일 중앙(`0 mm`)에 안정화

---

## 1. 개요 및 학습 목표

Step 05(20kHz H-Bridge 구동)와 Step 06(`TIM3` 직교 엔코더 + `ADC1` 진자 각도계 + Soft Limit + 가이드 호밍/캘리브레이션)으로 **액추에이터와 4대 상태 센서**가 모두 완성되었습니다.

이번 마지막 단계에서는 제어 공학의 꽃인 **도립진자(Inverted Pendulum on a Cart) 밸런싱**을 처음부터 끝까지 구현합니다.

1. **라그랑지안(Lagrangian) 동역학 모델링**과 12시 평형점($\theta = 0$) 주변 **선형화(Linearization)**
2. **실물 로그 기반 시스템 식별**: 자유 진동 주기($T_n \approx 1.02\text{ s}$)에서 유효 길이($l_{\text{eff}} \approx 0.26\text{ m}$) 추출
3. **Python (`scipy`) 기반 LQR 게인 행렬 $\mathbf{K}$ 계산** 및 **4-상태 LQR vs 각도 전용 PD 제어 비교**
4. **1kHz `TIM6` 실시간 제어 루프**: 10ms 슬라이딩 링버퍼 미분, 정지 마찰력(Deadzone) 보상, 자동 무장(`|θ| < 8°`), 낙하 차단(`|θ| > 28°`), 실시간 UART 게인 튜닝

---

## 2. 카트-도립진자 수학 모델링 & 선형화

### 2.1 좌표계 통일 (제어 부호의 생명)

제어기에서 부호 하나만 반대로 들어가면 수렴 대신 폭주(Positive Feedback)가 일어납니다. 본 프로젝트는 다음과 같이 모든 좌표 부호를 오른쪽 기준으로 통일합니다.

```text
           +θ (오른쪽 기울임, θ=0은 12시 수직)
             /
            /  l_eff ≈ 0.26m
           ●  (진자 질량 중심)
     +----/----+
  <--|  Cart   |-->  +x (오른쪽 이동, f 명령, PB0=1, +u)
     +---------+
```

- **$x$ (m)**: 레일 중앙 `0`, 오른쪽 `+`, 왼쪽 `-`
- **$\theta$ (rad)**: 12시 수직 `0`, **오른쪽 기울임 `+`**, 왼쪽 기울임 `-`
  *(실물 WDD35D4-5K는 CCW 회전 시 ADC가 증가하므로 `DEFAULT_ANGLE_SIGN = -1`로 정렬하며, `p` 키로 실시간 전환 가능)*
- **$u \in [-1, +1]$**: 정규화 모터 PWM 명령 (`+u` = 오른쪽 가속, `-u` = 왼쪽 가속)

### 2.2 라그랑지안 동역학과 선형화

카트 질량 $M$, 진자 질량 $m$, 피벗~질량중심 거리 $l$, 관성모멘트 $I$, 유효 길이 $l_{\text{eff}} = \frac{I + m l^2}{m l}$ 에 대해 진자의 운동 에너지와 위치 에너지로부터 오일러-라그랑주 방정식을 세우면:

$$(I + m l^2)\ddot{\theta} + m l \cos\theta\,\ddot{x} - m g l \sin\theta = 0$$

직립 평형점 $\theta \approx 0$ 근방에서 $\sin\theta \approx \theta$, $\cos\theta \approx 1$ 로 선형화하고 양변을 $m l$ 로 나누면 **구동기 기반 카트-폴 핵심 식**을 얻습니다.

$$\ddot{\theta} = \frac{g}{l_{\text{eff}}}\theta - \frac{1}{l_{\text{eff}}}\ddot{x}$$

여기에 Step 06 실물 로그에서 관찰된 DC 모터+타이밍벨트 동역학($\ddot{x} \approx -b_v \dot{x} + K_{\text{act}} u$, 50% 듀티 시 $\dot{x} \approx 0.23\text{ m/s}$ 도달 특성에서 $b_v \approx 8.0$, $K_{\text{act}} \approx 4.0$)과 자유 진동 주기 $T_n = 2\pi\sqrt{l_{\text{eff}}/g} \approx 1.02\text{ s} \implies l_{\text{eff}} \approx 0.26\text{ m}$ 를 대입하면 상태공간방정식 $\dot{\mathbf{x}} = A\mathbf{x} + B u$ 가 완성됩니다.

$$\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}, \quad
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -b_v & 0 & 0 \\
0 & 0 & 0 & 1 \\
0 & \frac{b_v}{l_{\text{eff}}} & \frac{g}{l_{\text{eff}}} & 0
\end{bmatrix}, \quad
B = \begin{bmatrix} 0 \\ K_{\text{act}} \\ 0 \\ -\frac{K_{\text{act}}}{l_{\text{eff}}} \end{bmatrix}$$

---

## 3. Python 기반 LQR 게인 행렬 $\mathbf{K}$ 도출 & PD 비교

선형 2차 조절기(LQR)는 아래 비용 함수 $J$를 최소화하는 최적 상태 피드백 게인 $u = -\mathbf{K}\mathbf{x}$ 를 계산합니다.

$$J = \int_0^\infty \left(\mathbf{x}^T Q \mathbf{x} + R u^2\right)\,dt$$

```python
import numpy as np
from scipy.linalg import solve_continuous_are

g = 9.81
l_eff = 0.26  # Step 06 자유 진동 로그(Tn ≈ 1.02s) 기반 실측 유효 길이
b_v = 8.0     # 벨트/모터 속도 감쇠 상수
K_act = 4.0   # 정규화 입력 u -> 카트 가속도 이득

A = np.array([
    [0.0, 1.0,       0.0,     0.0],
    [0.0, -b_v,      0.0,     0.0],
    [0.0, 0.0,       0.0,     1.0],
    [0.0, b_v/l_eff, g/l_eff, 0.0],
])
B = np.array([[0.0], [K_act], [0.0], [-K_act / l_eff]])

Q = np.diag([15.0, 1.0, 80.0, 2.0])  # 위치(x)와 각도(theta) 집중 안정화
R = np.array([[1.0]])

P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P
print("K =", np.round(K[0], 3))
print("Poles =", np.round(np.linalg.eigvals(A - B @ K), 3))
```

실행 결과 (`uv run --with numpy --with scipy python3 ...`):
- **LQR 게인**: $\mathbf{K} = [-3.873,\ -6.411,\ -20.759,\ -3.362]$
- **폐루프 극점**: $[-24.157,\ -6.324,\ -1.797 \pm 0.773j]$ (전부 좌반평면 안정!)

> [!TIP]
> **왜 PD 모드(`m` 키)와 LQR 모드를 둘 다 넣었을까?**  
> 각도만 제어하는 **PD 모드**($u = -(k_\theta\theta + k_{\dot{\theta}}\dot{\theta})$)는 진자는 세우지만 카트가 레일 끝으로 조금씩 밀려 결국 Soft Limit에 닿게 됩니다. 반면 **LQR 모드**는 $k_x, k_{\dot{x}}$ 항이 비직관적이지만 영리한 보상(카트가 오른쪽으로 흐르면 일부러 카트를 오른쪽으로 살짝 더 밀어 진자를 왼쪽으로 기울인 뒤, 다시 왼쪽으로 데려와 중앙 `x=0`에 복귀!)을 수행합니다. `m` 키로 두 모드의 차이를 직접 체감할 수 있습니다.

---

## 4. 실물 밸런싱을 위한 임베디드 제어 기법

1. **1kHz 단일 소유자 센서 + 10ms 슬라이딩 링버퍼 미분 (`TIM6`)**:
   - `TIM6` 1ms 인터럽트 내부에서만 `TIM3_CNT`와 `ADC1_IN2`(4회 평균 $\approx 15\,\mu\text{s}$)를 읽어 하드웨어 경합을 원천 차단했습니다.
   - `1 ms` 차분 대신 최근 **10개 샘플(`10 ms`) 원형 버퍼 차분 + EMA LPF ($\alpha = 0.15$)** 를 사용하여 속도/각속도 해상도를 10배 높이면서도 위상 지연은 ~5ms 수준으로 최소화했습니다.
2. **정지 마찰력(Deadzone / Stiction) 보상**:
   - Step 05에서 `25%` 이하 듀티로는 IP570 벨트가 꿈쩍도 하지 않음을 확인했습니다. 제어 출력 $|u| \ge 0.015$ 일 때 기본 데드존 보상 듀티(`g_deadzone_pct = 26%`)를 더해 매핑(`26% ~ 95%`)하므로 미소 각도 오차에도 즉각 반응합니다.
3. **안전한 자동 무장(`ARMED`) 및 낙하/레일 차단**:
   - **`c` 키**를 눌러 무장(`CTRL_ARMED`)하면 진자가 내려가 있는 동안에는 모터가 **완전 정지**합니다.
   - 손으로 진자를 들어 **12시 ±8° 이내(`|x| < 100 mm`)에 `100 ms` 유지**하면 자동으로 `CTRL_BALANCING`이 시작되고, **`|θ| > 28°` 낙하 또는 Soft Limit (`±150 mm`) 도달 시 즉시 차단** 후 `ARMED`로 복귀합니다.

---

## 5. 전체 소스 코드 (`firmware/G474RE_Blink/Src/main.c`)

```c
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-11 / Step 07] 도립진자 1kHz 실시간 LQR & PD 밸런싱 제어기
 *                   + 가이드 위치 초기화(h), 진자 2점 교정(a, d), 데드존 보상 & 실시간 튜닝
 *                   - PA0 (A0) / PA5 : TIM2_CH1 20kHz PWM (DRV8876 EN / LD2 LED)
 *                   - PB0 (A3)       : GPIO Output 모터 방향 (PH: 1=오른쪽 +, 0=왼쪽 -)
 *                   - PC6 (CN10-4)   : TIM3_CH1 (AF2, 5V FT) 엔코더 A상
 *                   - PC7 (D9)       : TIM3_CH2 (AF2, 5V FT) 엔코더 B상
 *                   - PA1 (A1)       : ADC1_IN2 진자 포텐쇼미터(WDD35D4-5K) 각도
 *                   - USART2 (VCP)   : 115200 bps 텔레메트리 및 실시간 게인 튜닝
 *                   - TIM6           : 1ms(1kHz) 결정론적 LQR/PD 제어 + Soft Limit 루프
 * @repository     : https://github.com/CH-Cho/InvertedPendulum
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

/* ============================================================================
 * 1. STM32G474RE 베어메탈 레지스터 정의 (RM0440 기준)
 * ========================================================================== */
#define SCB_CPACR           (*(volatile uint32_t *)0xE000ED88U)

// RCC
#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB1ENR1        (*(volatile uint32_t *)(RCC_BASE + 0x58U))

// GPIO A, B, C
#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_PUPDR         (*(volatile uint32_t *)(GPIOA_BASE + 0x0CU))
#define GPIOA_AFRL          (*(volatile uint32_t *)(GPIOA_BASE + 0x20U))

#define GPIOB_BASE          0x48000400U
#define GPIOB_MODER         (*(volatile uint32_t *)(GPIOB_BASE + 0x00U))
#define GPIOB_ODR           (*(volatile uint32_t *)(GPIOB_BASE + 0x14U))

#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))
#define GPIOC_PUPDR         (*(volatile uint32_t *)(GPIOC_BASE + 0x0CU))
#define GPIOC_AFRL          (*(volatile uint32_t *)(GPIOC_BASE + 0x20U))

// NVIC
#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104U))

// USART2 (ST-LINK VCP: PA2=TX, PA3=RX @ 16MHz HSI)
#define USART2_BASE         0x40004400U
#define USART2_CR1          (*(volatile uint32_t *)(USART2_BASE + 0x00U))
#define USART2_BRR          (*(volatile uint32_t *)(USART2_BASE + 0x0CU))
#define USART2_ISR          (*(volatile uint32_t *)(USART2_BASE + 0x1CU))
#define USART2_ICR          (*(volatile uint32_t *)(USART2_BASE + 0x20U))
#define USART2_RDR          (*(volatile uint32_t *)(USART2_BASE + 0x24U))
#define USART2_TDR          (*(volatile uint32_t *)(USART2_BASE + 0x28U))

// TIM2 (20kHz PWM: PA0 & PA5)
#define TIM2_BASE           0x40000000U
#define TIM2_CR1            (*(volatile uint32_t *)(TIM2_BASE + 0x00U))
#define TIM2_CCMR1          (*(volatile uint32_t *)(TIM2_BASE + 0x18U))
#define TIM2_CCER           (*(volatile uint32_t *)(TIM2_BASE + 0x20U))
#define TIM2_PSC            (*(volatile uint32_t *)(TIM2_BASE + 0x28U))
#define TIM2_ARR            (*(volatile uint32_t *)(TIM2_BASE + 0x2CU))
#define TIM2_CCR1           (*(volatile uint32_t *)(TIM2_BASE + 0x34U))

// TIM3 (4체배 직교 엔코더: PC6=CH1, PC7=CH2)
#define TIM3_BASE           0x40000400U
#define TIM3_CR1            (*(volatile uint32_t *)(TIM3_BASE + 0x00U))
#define TIM3_SMCR           (*(volatile uint32_t *)(TIM3_BASE + 0x08U))
#define TIM3_CCMR1          (*(volatile uint32_t *)(TIM3_BASE + 0x18U))
#define TIM3_CCER           (*(volatile uint32_t *)(TIM3_BASE + 0x20U))
#define TIM3_CNT            (*(volatile uint32_t *)(TIM3_BASE + 0x24U))
#define TIM3_PSC            (*(volatile uint32_t *)(TIM3_BASE + 0x28U))
#define TIM3_ARR            (*(volatile uint32_t *)(TIM3_BASE + 0x2CU))

// TIM6 (1ms = 1kHz 제어 루프 타임베이스)
#define TIM6_BASE           0x40001000U
#define TIM6_CR1            (*(volatile uint32_t *)(TIM6_BASE + 0x00U))
#define TIM6_DIER           (*(volatile uint32_t *)(TIM6_BASE + 0x0CU))
#define TIM6_SR             (*(volatile uint32_t *)(TIM6_BASE + 0x10U))
#define TIM6_PSC            (*(volatile uint32_t *)(TIM6_BASE + 0x28U))
#define TIM6_ARR            (*(volatile uint32_t *)(TIM6_BASE + 0x2CU))

// ADC1 & ADC12 Common (PA1 = ADC1_IN2)
#define ADC1_BASE           0x50000000U
#define ADC1_ISR            (*(volatile uint32_t *)(ADC1_BASE + 0x00U))
#define ADC1_CR             (*(volatile uint32_t *)(ADC1_BASE + 0x08U))
#define ADC1_CFGR           (*(volatile uint32_t *)(ADC1_BASE + 0x0CU))
#define ADC1_SMPR1          (*(volatile uint32_t *)(ADC1_BASE + 0x14U))
#define ADC1_SQR1           (*(volatile uint32_t *)(ADC1_BASE + 0x30U))
#define ADC1_DR             (*(volatile uint32_t *)(ADC1_BASE + 0x40U))
#define ADC12_CCR           (*(volatile uint32_t *)0x50000308U)

/* ============================================================================
 * 2. 하드웨어 상수, 제어 파라미터 및 전역 상태
 * ========================================================================== */
#define PI_F                    3.14159265f
#define RAD_TO_DEG_F            57.2957795f
#define DEG_TO_RAD_F            0.01745329f

#define PWM_ARR_MAX             799U    // 16MHz / (0+1) / (799+1) = 20,000 Hz (20kHz)
#define PWM_MAX_PCT             95.0f

// IP570 물리 기구부 캘리브레이션 상수
#define RAIL_TRAVEL_M           0.385f
#define RAIL_TRAVEL_MM          385
#define DEFAULT_TOTAL_COUNTS    6279
#define SOFT_LIMIT_MM           150.0f
#define HOME_CENTER_BAND_CNT    50
#define HOME_CENTER_TIMEOUT_MS  4000U

// WDD35D4-5K 각도 센서 상수 (관성 로그 기반: CCW=ADC증가 -> 오른쪽 기울임=+ 기준 ANGLE_SIGN=-1)
#define POT_DEFAULT_DOWN        1024
#define POT_DEFAULT_UPRIGHT     3140
#define POT_DEFAULT_180_COUNTS  2116
#define DEFAULT_ANGLE_SIGN      (-1)

// 제어기 안전 임계값 (12시 수직 근처에서 자동 무장, 낙하 시 즉시 차단)
#define CTRL_ARM_ANGLE_DEG      8.0f    // |theta| < 8.0 deg 진입 시 밸런싱 시작
#define CTRL_ARM_POS_MM         100.0f  // |x| < 100.0 mm 안에서 시작
#define CTRL_ARM_HOLD_MS        100U    // 수직 유지 100ms 확인 후 작동
#define CTRL_FALL_ANGLE_DEG     28.0f   // |theta| > 28.0 deg 시 낙하 차단
#define DIFF_WIN_SIZE           10U     // 1kHz 루프 내 10ms 슬라이딩 차분 윈도우

typedef enum {
    HOME_IDLE = 0,
    HOME_WAIT_RIGHT,
    HOME_WAIT_LEFT,
    HOME_AUTO_CENTER
} CartHomeState;

typedef enum {
    ACAL_IDLE = 0,
    ACAL_WAIT_DOWN,
    ACAL_WAIT_UP
} AngleCalState;

typedef enum {
    CTRL_IDLE = 0,      // 수동 대기 (키보드 킥/초기화 가능)
    CTRL_ARMED,         // 무장 대기 (12시 ±8 deg 세우면 자동 밸런싱 시작)
    CTRL_BALANCING      // 1kHz 능동 밸런싱 동작 중
} ControlState;

typedef enum {
    MODE_LQR = 0,       // 4-상태 LQR: u = -(kx*x + kdx*dx + kth*th + kdth*dth)
    MODE_PD             // 비교 학습용 각도 PD: u = -(kth*th + kdth*dth)
} ControlMode;

typedef enum {
    TUNE_KX = 0,
    TUNE_KDX,
    TUNE_KTH,
    TUNE_KDTH,
    TUNE_DEADZONE,
    TUNE_COUNT
} TuneParamIndex;

// 시스템 틱 및 UART 명령
static volatile uint32_t g_system_millis      = 0;
static volatile uint8_t  g_rx_cmd             = 0;

// 모터 상태 및 수동 킥 제어
static volatile uint32_t g_motor_stop_at      = 0;
static volatile uint8_t  g_kick_active        = 0;
static volatile uint8_t  g_test_duty_pct      = 50;
static volatile uint8_t  g_motor_dir          = 1;
static volatile uint8_t  g_soft_limit_on      = 1;
static volatile uint8_t  g_soft_limit_trip    = 0;

// 카트 엔코더(TIM3) 상태 및 10ms 슬라이딩 미분 버퍼
static volatile int8_t   g_enc_sign           = 1;
static volatile int32_t  g_enc_total_counts   = DEFAULT_TOTAL_COUNTS;
static volatile int32_t  g_cart_counts        = 0;
static volatile uint16_t s_prev_cnt_1ms       = 0;
static volatile float    g_pos_m              = 0.0f;
static volatile float    g_vel_mps            = 0.0f;
static float             s_pos_hist[DIFF_WIN_SIZE];

// 진자 각도계(ADC1) 상태 및 10ms 슬라이딩 미분 버퍼
static volatile int8_t   g_angle_sign         = DEFAULT_ANGLE_SIGN;
static volatile uint16_t g_angle_down_adc     = POT_DEFAULT_DOWN;
static volatile int32_t  g_angle_zero_adc     = POT_DEFAULT_UPRIGHT;
static volatile int32_t  g_pot_180_counts     = POT_DEFAULT_180_COUNTS;
static volatile int8_t   g_pot_up_dir_sign    = 1;
static volatile uint16_t g_angle_adc_raw      = POT_DEFAULT_UPRIGHT;
static volatile float    g_theta_rad          = 0.0f;
static volatile float    g_dtheta_rps         = 0.0f;
static float             s_th_hist[DIFF_WIN_SIZE];
static uint8_t           s_hist_idx           = 0;
static volatile uint8_t  s_hist_reset_req     = 1;

// LQR / PD 제어 상태 및 튜닝 가능한 게인 (IP570 Python LQR 도출 초기값)
// u = -(kx*x + kdx*dx + kth*th + kdth*dth),  x=[m, m/s, rad, rad/s], u in [-1, +1]
static volatile ControlState   g_ctrl_state   = CTRL_IDLE;
static volatile ControlMode    g_ctrl_mode    = MODE_LQR;
static volatile TuneParamIndex g_tune_idx     = TUNE_KTH;
static volatile float          g_k_x          = -3.873f;
static volatile float          g_k_dx         = -6.411f;
static volatile float          g_k_th         = -20.759f;
static volatile float          g_k_dth        = -3.362f;
static volatile float          g_deadzone_pct = 26.0f;
static volatile float          g_last_u_cmd   = 0.0f;
static volatile uint16_t       g_arm_hold_ms  = 0;
static volatile uint8_t        g_fall_trip    = 0;

// 텔레메트리 스냅샷 (1kHz ISR -> main 정수 출력)
static volatile int32_t  g_telem_pos_mm_x10   = 0;
static volatile int32_t  g_telem_vel_mms_x10  = 0;
static volatile int32_t  g_telem_ang_cdeg     = 0;
static volatile int32_t  g_telem_dth_x10      = 0;
static volatile int32_t  g_telem_u_pct        = 0;

// 호밍 & 각도 캘리브레이션 마법사 상태
static volatile CartHomeState g_home_state    = HOME_IDLE;
static int32_t           g_home_right_cnt     = 0;
static int32_t           g_home_left_cnt      = 0;
static uint32_t          g_home_center_start  = 0;
static AngleCalState     g_acal_state         = ACAL_IDLE;

/* ============================================================================
 * 3. 유틸리티 및 드라이버 초기화 함수
 * ========================================================================== */
static void fpu_enable(void)
{
    // CubeIDE fpv4-sp-d16 설정에서 1kHz ISR 및 main의 float 연산 HardFault 방지
    SCB_CPACR |= (0xFU << 20);
    __asm volatile ("dsb");
    __asm volatile ("isb");
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_system_millis;
    while ((g_system_millis - start) < ms) {
    }
}

static void uart2_write_char(uint8_t ch)
{
    while (!(USART2_ISR & (1U << 7)));
    USART2_TDR = ch;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') {
            uart2_write_char('\r');
        }
        uart2_write_char((uint8_t)ptr[i]);
    }
    return len;
}

static void print_fixed_1(int32_t val_x10)
{
    char sign = (val_x10 < 0) ? '-' : '+';
    int32_t abs_v = (val_x10 < 0) ? -val_x10 : val_x10;
    printf("%c%ld.%01ld", sign, (long)(abs_v / 10), (long)(abs_v % 10));
}

static void print_fixed_2(int32_t val_x100)
{
    char sign = (val_x100 < 0) ? '-' : '+';
    int32_t abs_v = (val_x100 < 0) ? -val_x100 : val_x100;
    printf("%c%ld.%02ld", sign, (long)(abs_v / 100), (long)(abs_v % 100));
}

static void print_fixed_3(float val)
{
    int32_t scaled = (int32_t)(val * 1000.0f + ((val >= 0.0f) ? 0.5f : -0.5f));
    char sign = (scaled < 0) ? '-' : '+';
    int32_t abs_v = (scaled < 0) ? -scaled : scaled;
    printf("%c%ld.%03ld", sign, (long)(abs_v / 1000), (long)(abs_v % 1000));
}

static void uart2_init(void)
{
    RCC_AHB2ENR  |= (1U << 0);  // GPIOAEN
    RCC_APB1ENR1 |= (1U << 17); // USART2EN

    GPIOA_MODER &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA_MODER |=  ((2U << (2 * 2)) | (2U << (3 * 2)));
    GPIOA_AFRL  &= ~((0xFU << (2 * 4)) | (0xFU << (3 * 4)));
    GPIOA_AFRL  |=  ((0x7U << (2 * 4)) | (0x7U << (3 * 4)));

    USART2_BRR = 139U; // 115200 bps @ 16MHz
    USART2_CR1 = (1U << 0) | (1U << 2) | (1U << 3) | (1U << 5);

    NVIC_ISER1 = (1U << (38 - 32)); // IRQ 38 (USART2)
}

static void tim6_init(void)
{
    RCC_APB1ENR1 |= (1U << 4); // TIM6EN
    TIM6_PSC = 15U;            // 16MHz / 16 = 1MHz
    TIM6_ARR = 999U;           // 1ms = 1kHz 제어 루프
    TIM6_DIER |= (1U << 0);
    NVIC_ISER1 = (1U << (54 - 32)); // IRQ 54 (TIM6)
    TIM6_CR1 |= (1U << 0);
}

static void motor_driver_init(void)
{
    RCC_AHB2ENR  |= (1U << 0) | (1U << 1); // GPIOAEN, GPIOBEN
    RCC_APB1ENR1 |= (1U << 0);             // TIM2EN

    // PA0 (A0 -> DRV8876 EN) & PA5 (LD2 LED) -> AF1 (TIM2_CH1)
    GPIOA_MODER &= ~((3U << (0 * 2)) | (3U << (5 * 2)));
    GPIOA_MODER |=  ((2U << (0 * 2)) | (2U << (5 * 2)));
    GPIOA_AFRL  &= ~((0xFU << (0 * 4)) | (0xFU << (5 * 4)));
    GPIOA_AFRL  |=  ((0x1U << (0 * 4)) | (0x1U << (5 * 4)));

    // PB0 (A3 -> DRV8876 PH) -> GPIO Output
    GPIOB_MODER &= ~(3U << (0 * 2));
    GPIOB_MODER |=  (1U << (0 * 2));
    GPIOB_ODR   |=  (1U << 0);

    // TIM2 20kHz PWM
    TIM2_PSC   = 0U;
    TIM2_ARR   = PWM_ARR_MAX;
    TIM2_CCR1  = 0U;
    TIM2_CCMR1 = (0x6U << 4) | (1U << 3);
    TIM2_CCER  = (1U << 0);
    TIM2_CR1   = (1U << 0);
}

static void motor_stop(void)
{
    TIM2_CCR1 = 0U;
    g_kick_active = 0U;
    g_last_u_cmd  = 0.0f;
}

static void motor_drive_pct(uint8_t dir, float duty_pct)
{
    if (duty_pct <= 0.0f) {
        TIM2_CCR1 = 0U;
        return;
    }
    if (duty_pct > PWM_MAX_PCT) duty_pct = PWM_MAX_PCT;

    g_motor_dir = dir ? 1U : 0U;
    if (g_motor_dir) {
        GPIOB_ODR |= (1U << 0);
    } else {
        GPIOB_ODR &= ~(1U << 0);
    }

    uint32_t ccr = (uint32_t)(((float)(PWM_ARR_MAX + 1U) * duty_pct) / 100.0f + 0.5f);
    if (ccr > PWM_ARR_MAX) ccr = PWM_ARR_MAX;
    TIM2_CCR1 = ccr;
}

// 정규화 제어 입력 u in [-1.0, +1.0] -> 정지 마찰력(Deadzone) 보상 PWM 매핑
static void motor_apply_control(float u)
{
    if (u > 1.0f)  u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    g_last_u_cmd = u;

    float abs_u = (u >= 0.0f) ? u : -u;
    if (abs_u < 0.015f) {
        TIM2_CCR1 = 0U;
        g_telem_u_pct = 0;
        return;
    }

    uint8_t dir = (u >= 0.0f) ? 1U : 0U;
    float dz = g_deadzone_pct;
    float duty = dz + abs_u * (PWM_MAX_PCT - dz);
    motor_drive_pct(dir, duty);

    int32_t signed_duty = (int32_t)(duty + 0.5f);
    g_telem_u_pct = dir ? signed_duty : -signed_duty;
}

static uint8_t is_direction_blocked_by_limit(uint8_t dir, float pos_mm)
{
    if (!g_soft_limit_on) return 0U;
    if (dir  && (pos_mm >= SOFT_LIMIT_MM))  return 1U;
    if (!dir && (pos_mm <= -SOFT_LIMIT_MM)) return 1U;
    return 0U;
}

static void motor_kick(uint8_t dir, uint8_t duty_pct, uint32_t duration_ms)
{
    uint8_t target_dir = dir ? 1U : 0U;
    float pos_mm = g_pos_m * 1000.0f;
    if (is_direction_blocked_by_limit(target_dir, pos_mm)) {
        motor_stop();
        g_soft_limit_trip = 1U;
        printf("\r\n>>> [SOFT LIMIT] Blocked %s! Pos=", target_dir ? "RIGHT" : "LEFT");
        print_fixed_1(g_telem_pos_mm_x10);
        printf(" mm (Limit +/-%d mm)\r\n", (int)SOFT_LIMIT_MM);
        return;
    }

    motor_drive_pct(target_dir, (float)duty_pct);
    g_motor_stop_at = g_system_millis + duration_ms;
    g_kick_active   = 1U;
}

static void encoder_tim3_init(void)
{
    RCC_AHB2ENR  |= (1U << 2); // GPIOCEN
    RCC_APB1ENR1 |= (1U << 1); // TIM3EN

    GPIOC_MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOC_MODER |=  ((2U << (6 * 2)) | (2U << (7 * 2)));
    GPIOC_PUPDR &= ~((3U << (6 * 2)) | (3U << (7 * 2))); // 00: No Pull-Up/Down (5V FT 필수)
    GPIOC_AFRL  &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOC_AFRL  |=  ((0x2U << (6 * 4)) | (0x2U << (7 * 4)));

    TIM3_CR1   = 0U;
    TIM3_PSC   = 0U;
    TIM3_ARR   = 0xFFFFU;
    TIM3_CCMR1 = (1U << 0) | (6U << 4) | (1U << 8) | (6U << 12);
    TIM3_CCER  = (1U << 0) | (1U << 1) | (1U << 4); // CC1P=1: 오른쪽(f) = +
    TIM3_SMCR  = 3U;                                // 4체배 직교 인코더
    TIM3_CNT   = 0U;
    TIM3_CR1   = (1U << 0);
}

static void encoder_set_counts_atomic(int32_t new_counts, int32_t new_total_counts, int8_t new_sign)
{
    __asm volatile ("cpsid i");
    if (new_total_counts > 0) {
        g_enc_total_counts = new_total_counts;
    }
    g_enc_sign        = new_sign;
    g_cart_counts     = new_counts;
    s_prev_cnt_1ms    = (uint16_t)TIM3_CNT;
    g_pos_m           = ((float)g_cart_counts * RAIL_TRAVEL_M) / (float)g_enc_total_counts;
    g_vel_mps         = 0.0f;
    g_telem_pos_mm_x10 = (int32_t)(g_pos_m * 10000.0f);
    g_telem_vel_mms_x10 = 0;
    s_hist_reset_req  = 1U;
    g_soft_limit_trip = 0U;
    __asm volatile ("cpsie i");
}

static void adc1_pa1_init(void)
{
    RCC_AHB2ENR |= (1U << 0) | (1U << 13); // GPIOAEN, ADC12EN

    GPIOA_MODER |=  (3U << (1 * 2));
    GPIOA_PUPDR &= ~(3U << (1 * 2));

    ADC12_CCR &= ~(3U << 16);
    ADC12_CCR |=  (1U << 16); // HCLK/1 (16MHz)

    ADC1_CR &= ~(1U << 29); // DEEPPWD = 0
    ADC1_CR |=  (1U << 28); // ADVREGEN = 1
    delay_ms(2);

    ADC1_CR &= ~(1U << 30); // ADCALDIF = 0
    ADC1_CR |=  (1U << 31); // ADCAL = 1
    while (ADC1_CR & (1U << 31));

    ADC1_ISR = (1U << 0);
    ADC1_CR |= (1U << 0);   // ADEN = 1
    while (!(ADC1_ISR & (1U << 0)));

    ADC1_CFGR  = 0U;
    ADC1_SMPR1 &= ~(7U << 6);
    ADC1_SMPR1 |=  (4U << 6); // SMP2 = 47.5 cycles (~3us)
    ADC1_SQR1  =  (2U << 6); // SQ1 = CH2 (PA1)
}

static uint16_t adc1_read_pa1_avg4(void)
{
    uint32_t sum = 0U;
    for (uint8_t i = 0U; i < 4U; i++) {
        uint32_t timeout = 2000U;
        ADC1_CR |= (1U << 2); // ADSTART
        while (!(ADC1_ISR & (1U << 2)) && (--timeout > 0U));
        sum += (uint16_t)(ADC1_DR & 0x0FFFU);
    }
    return (uint16_t)(sum >> 2);
}

/* ============================================================================
 * 4. 1kHz 실시간 센서 추정 & LQR/PD 밸런싱 제어 인터럽트
 * ========================================================================== */
void TIM6_DACUNDER_IRQHandler(void)
{
    if (!(TIM6_SR & (1U << 0))) return;
    TIM6_SR &= ~(1U << 0);
    g_system_millis++;

    // (1) 카트 위치(m) & 진자 각도(rad) 1kHz 동시 샘플링
    uint16_t curr_cnt = (uint16_t)TIM3_CNT;
    int16_t  delta_1ms = (int16_t)(curr_cnt - s_prev_cnt_1ms);
    s_prev_cnt_1ms = curr_cnt;
    g_cart_counts += (int32_t)g_enc_sign * (int32_t)delta_1ms;

    float pos_m = ((float)g_cart_counts * RAIL_TRAVEL_M) / (float)g_enc_total_counts;
    float pos_mm = pos_m * 1000.0f;

    uint16_t adc_raw = adc1_read_pa1_avg4();
    g_angle_adc_raw = adc_raw;
    float theta_rad = (float)g_angle_sign * ((float)((int32_t)adc_raw - g_angle_zero_adc) * PI_F) / (float)g_pot_180_counts;
    float theta_deg = theta_rad * RAD_TO_DEG_F;

    // (2) 10ms 슬라이딩 윈도우 미분 + EMA LPF (alpha=0.15 @ 1kHz, fc ~ 28 Hz)
    if (s_hist_reset_req) {
        s_hist_reset_req = 0U;
        for (uint8_t i = 0U; i < DIFF_WIN_SIZE; i++) {
            s_pos_hist[i] = pos_m;
            s_th_hist[i]  = theta_rad;
        }
        g_vel_mps  = 0.0f;
        g_dtheta_rps = 0.0f;
    } else {
        float vel_raw = (pos_m - s_pos_hist[s_hist_idx]) * 100.0f;     // / 0.010s
        float dth_raw = (theta_rad - s_th_hist[s_hist_idx]) * 100.0f;  // / 0.010s
        s_pos_hist[s_hist_idx] = pos_m;
        s_th_hist[s_hist_idx]  = theta_rad;
        s_hist_idx = (uint8_t)((s_hist_idx + 1U) % DIFF_WIN_SIZE);

        g_vel_mps    = 0.15f * vel_raw + 0.85f * g_vel_mps;
        g_dtheta_rps = 0.15f * dth_raw + 0.85f * g_dtheta_rps;
    }
    g_pos_m     = pos_m;
    g_theta_rad = theta_rad;

    // 텔레메트리용 정수 스냅샷 업데이트
    g_telem_pos_mm_x10  = (int32_t)(pos_mm * 10.0f);
    g_telem_vel_mms_x10 = (int32_t)(g_vel_mps * 10000.0f);
    g_telem_ang_cdeg    = (int32_t)(theta_deg * 100.0f);
    g_telem_dth_x10     = (int32_t)(g_dtheta_rps * RAD_TO_DEG_F * 10.0f);

    // (3) 제어 상태 머신: BALANCING -> ARMED -> IDLE / AUTO_CENTER
    float abs_th_deg = (theta_deg >= 0.0f) ? theta_deg : -theta_deg;
    float abs_pos_mm = (pos_mm >= 0.0f) ? pos_mm : -pos_mm;

    if (g_ctrl_state == CTRL_BALANCING) {
        // 낙하 또는 Soft Limit 초과 시 즉시 모터 차단 후 ARMED 복귀
        if (abs_th_deg > CTRL_FALL_ANGLE_DEG) {
            motor_stop();
            g_telem_u_pct = 0;
            g_ctrl_state  = CTRL_ARMED;
            g_arm_hold_ms = 0U;
            g_fall_trip   = 1U;
            return;
        }
        if (g_soft_limit_on && (abs_pos_mm >= SOFT_LIMIT_MM)) {
            motor_stop();
            g_telem_u_pct     = 0;
            g_ctrl_state      = CTRL_ARMED;
            g_arm_hold_ms     = 0U;
            g_soft_limit_trip = 1U;
            return;
        }

        // u = -(k_th * theta + k_dth * dtheta [+ k_x * x + k_dx * dx])
        float u = -(g_k_th * theta_rad + g_k_dth * g_dtheta_rps);
        if (g_ctrl_mode == MODE_LQR) {
            u -= (g_k_x * pos_m + g_k_dx * g_vel_mps);
        }
        motor_apply_control(u);
    } else if (g_ctrl_state == CTRL_ARMED) {
        motor_stop();
        g_telem_u_pct = 0;
        if ((abs_th_deg < CTRL_ARM_ANGLE_DEG) && (abs_pos_mm < CTRL_ARM_POS_MM)) {
            g_arm_hold_ms++;
            if (g_arm_hold_ms >= CTRL_ARM_HOLD_MS) {
                g_ctrl_state  = CTRL_BALANCING;
                g_arm_hold_ms = 0U;
            }
        } else {
            g_arm_hold_ms = 0U;
        }
    } else if (g_home_state == HOME_AUTO_CENTER) {
        int32_t err = -g_cart_counts;
        int32_t abs_err = (err >= 0) ? err : -err;
        if (abs_err <= HOME_CENTER_BAND_CNT) {
            motor_stop();
            g_telem_u_pct   = 0;
            g_home_state    = HOME_IDLE;
            g_soft_limit_on = 1U;
        } else if ((g_system_millis - g_home_center_start) >= HOME_CENTER_TIMEOUT_MS) {
            motor_stop();
            g_telem_u_pct   = 0;
            g_home_state    = HOME_IDLE;
            g_soft_limit_on = 1U;
        } else {
            int32_t half_span = g_enc_total_counts / 2;
            if (half_span < 1) half_span = 1;
            float duty = 40.0f + ((float)(abs_err * 12) / (float)half_span);
            if (duty > 52.0f) duty = 52.0f;
            uint8_t dir = (err > 0) ? 1U : 0U;
            motor_drive_pct(dir, duty);
            g_telem_u_pct = dir ? (int32_t)duty : -(int32_t)duty;
        }
    } else if (g_kick_active) {
        if (is_direction_blocked_by_limit(g_motor_dir, pos_mm)) {
            motor_stop();
            g_telem_u_pct     = 0;
            g_soft_limit_trip = 1U;
        } else if ((int32_t)(g_system_millis - g_motor_stop_at) >= 0) {
            motor_stop();
            g_telem_u_pct = 0;
        }
    } else {
        g_telem_u_pct = 0;
    }
}

void USART2_IRQHandler(void)
{
    if (USART2_ISR & (1U << 3)) {
        USART2_ICR = (1U << 3);
    }
    if (USART2_ISR & (1U << 5)) {
        g_rx_cmd = (uint8_t)USART2_RDR;
    }
}

/* ============================================================================
 * 5. 메뉴, 호밍/캘리브레이션, 실시간 게인 튜닝
 * ========================================================================== */
static void print_menu(void)
{
    printf("\r\n==================================================================\r\n");
    printf("  STM32G474RE Inverted Pendulum Project                           \r\n");
    printf("  [Step 07 / CH-11] 1kHz LQR / PD Balancing Controller            \r\n");
    printf("  Rate    : 1kHz Control (TIM6) | 20kHz PWM (TIM2) | 5V FT (TIM3) \r\n");
    printf("------------------------------------------------------------------\r\n");
    printf("  Balance : c = Arm/Disarm Balance (Lift <8deg to Auto-Start!)    \r\n");
    printf("            m = Mode LQR <-> PD  | p = Flip Angle Sign (+/-)      \r\n");
    printf("            s / Space = EMERGENCY STOP (Always Immediate Cut)     \r\n");
    printf("  Tuning  : g = Select Gain (Kx/Kdx/Kth/Kdth/DZ) | +/- = Adjust   \r\n");
    printf("  Homing  : h = Cart Homing (R->L->Center) | a = 2-Pt Angle Cal   \r\n");
    printf("            d = 6h Zero | u = 12h Zero | z = Cart Zero | l = Lim  \r\n");
    printf("  Manual  : f/b = 250ms Kick | 1~6 = Kick Duty | ? = Show Menu    \r\n");
    printf("==================================================================\r\n");
    printf("  Current K = [kx:");
    print_fixed_3(g_k_x);
    printf(", kdx:");
    print_fixed_3(g_k_dx);
    printf(", kth:");
    print_fixed_3(g_k_th);
    printf(", kdth:");
    print_fixed_3(g_k_dth);
    printf("] | DZ:%d%% | Sign:%+d\r\n\r\n", (int)g_deadzone_pct, (int)g_angle_sign);
}

static void handle_cart_homing_key(void)
{
    g_ctrl_state = CTRL_IDLE;
    if (g_home_state == HOME_IDLE) {
        motor_stop();
        g_soft_limit_on = 0U;
        g_home_state = HOME_WAIT_RIGHT;
        printf("\r\n[CART HOMING 1/2] 카트를 손으로 [오른쪽 끝]까지 밀고 'h'를 누르세요. (취소: s)\r\n");
    } else if (g_home_state == HOME_WAIT_RIGHT) {
        g_home_right_cnt = g_cart_counts;
        g_home_state = HOME_WAIT_LEFT;
        printf("\r\n>>> [1/2 완료] 오른쪽 끝 기록: %ld\r\n", (long)g_home_right_cnt);
        printf("[CART HOMING 2/2] 카트를 손으로 [왼쪽 끝]까지 밀고 'h'를 누르세요. (취소: s)\r\n");
    } else if (g_home_state == HOME_WAIT_LEFT) {
        g_home_left_cnt = g_cart_counts;
        int32_t diff = g_home_right_cnt - g_home_left_cnt;
        int8_t  new_sign = g_enc_sign;
        int32_t span = diff;

        if (diff < 0) {
            new_sign = (int8_t)(-g_enc_sign);
            span = -diff;
        }
        if (span < 1000) {
            g_home_state = HOME_IDLE;
            g_soft_limit_on = 1U;
            printf("\r\n>>> [HOMING 실패] 간격(%ld)이 너무 작습니다.\r\n\r\n", (long)span);
            return;
        }

        encoder_set_counts_atomic(-(span / 2), span, new_sign);
        printf("\r\n>>> [2/2 완료] 스트로크=%ld counts (~%d mm), Sign=%+d\r\n",
               (long)span, RAIL_TRAVEL_MM, (int)new_sign);
        printf(">>> [AUTO CENTER] 중앙(0.0 mm)으로 이동합니다...\r\n");
        g_home_center_start = g_system_millis;
        g_home_state = HOME_AUTO_CENTER;
    }
}

static void handle_angle_cal_key(void)
{
    if (g_acal_state == ACAL_IDLE) {
        g_acal_state = ACAL_WAIT_DOWN;
        printf("\r\n[ANGLE CAL 1/2] 진자가 아래(6시) 정지 후 'a'를 누르세요. (취소: s)\r\n");
    } else if (g_acal_state == ACAL_WAIT_DOWN) {
        g_angle_down_adc = g_angle_adc_raw;
        g_acal_state = ACAL_WAIT_UP;
        printf("\r\n>>> [1/2 완료] 6시 ADC=%u\r\n", (unsigned int)g_angle_down_adc);
        printf("[ANGLE CAL 2/2] 진자를 위(12시) 수직에 맞추고 'a'를 누르세요. (취소: s)\r\n");
    } else if (g_acal_state == ACAL_WAIT_UP) {
        uint16_t up_adc = g_angle_adc_raw;
        int32_t diff = (int32_t)up_adc - (int32_t)g_angle_down_adc;
        int32_t span = (diff >= 0) ? diff : -diff;

        if (span < 1200 || span > 2800) {
            g_acal_state = ACAL_IDLE;
            printf("\r\n>>> [ANGLE CAL 실패] 6시(%u)/12시(%u) 차이(%ld) 비정상.\r\n\r\n",
                   (unsigned int)g_angle_down_adc, (unsigned int)up_adc, (long)span);
            return;
        }

        g_pot_up_dir_sign = (diff >= 0) ? 1 : -1;
        g_pot_180_counts  = span;
        g_angle_zero_adc  = (int32_t)up_adc;
        s_hist_reset_req  = 1U;
        g_acal_state      = ACAL_IDLE;
        printf("\r\n>>> [ANGLE CAL 완료] 12시=%ld, 실측 180deg=%ld counts\r\n\r\n",
               (long)g_angle_zero_adc, (long)g_pot_180_counts);
    }
}

static void adjust_selected_gain(int8_t dir)
{
    float scale = (dir > 0) ? 1.10f : 0.90909f;
    const char *names[TUNE_COUNT] = {"Kx", "Kdx", "Kth", "Kdth", "Deadzone%"};
    float new_val = 0.0f;

    switch (g_tune_idx) {
    case TUNE_KX:
        g_k_x *= scale;
        new_val = g_k_x;
        break;
    case TUNE_KDX:
        g_k_dx *= scale;
        new_val = g_k_dx;
        break;
    case TUNE_KTH:
        g_k_th *= scale;
        new_val = g_k_th;
        break;
    case TUNE_KDTH:
        g_k_dth *= scale;
        new_val = g_k_dth;
        break;
    case TUNE_DEADZONE:
        g_deadzone_pct += (dir > 0) ? 2.0f : -2.0f;
        if (g_deadzone_pct < 0.0f)  g_deadzone_pct = 0.0f;
        if (g_deadzone_pct > 50.0f) g_deadzone_pct = 50.0f;
        new_val = g_deadzone_pct;
        break;
    default:
        break;
    }

    printf("\r\n>>> [TUNE %s] = ", names[g_tune_idx]);
    print_fixed_3(new_val);
    printf("\r\n");
}

/* ============================================================================
 * 6. 메인 루프 (명령 처리, 상태 전환 알림, 5Hz 텔레메트리)
 * ========================================================================== */
int main(void)
{
    fpu_enable();
    tim6_init();
    uart2_init();
    motor_driver_init();
    encoder_tim3_init();
    adc1_pa1_init();
    delay_ms(5); // 1kHz ISR이 초기 ADC 샘플 및 링버퍼 채울 시간 대기

    print_menu();

    uint32_t last_print = 0U;
    ControlState prev_ctrl_state = g_ctrl_state;
    CartHomeState prev_home_state = g_home_state;

    while (1)
    {
        // (A) 인터럽트 안전/상태 전환 이벤트 출력
        if (g_fall_trip) {
            g_fall_trip = 0U;
            printf("\r\n>>> [FALL CUTOFF] |Angle| > %d deg! Motor stopped -> ARMED.\r\n",
                   (int)CTRL_FALL_ANGLE_DEG);
        }
        if (g_soft_limit_trip) {
            g_soft_limit_trip = 0U;
            printf("\r\n>>> [SOFT LIMIT] Motor cut at ");
            print_fixed_1(g_telem_pos_mm_x10);
            printf(" mm!\r\n");
        }
        if (g_ctrl_state != prev_ctrl_state) {
            if (g_ctrl_state == CTRL_BALANCING) {
                printf("\r\n>>> [BALANCING START] %s 1kHz Active!\r\n",
                       (g_ctrl_mode == MODE_LQR) ? "LQR" : "PD");
            }
            prev_ctrl_state = g_ctrl_state;
        }
        if (prev_home_state == HOME_AUTO_CENTER && g_home_state == HOME_IDLE) {
            printf("\r\n>>> [HOMING 완료] 중앙 위치 정지! Pos=");
            print_fixed_1(g_telem_pos_mm_x10);
            printf(" mm (Soft Limit ON)\r\n\r\n");
        }
        prev_home_state = g_home_state;

        // (B) USART2 키보드 명령 처리
        if (g_rx_cmd != 0U) {
            uint8_t cmd = g_rx_cmd;
            g_rx_cmd = 0U;

            if (cmd == 's' || cmd == 'S' || cmd == ' ') {
                motor_stop();
                g_ctrl_state = CTRL_IDLE;
                if (g_home_state != HOME_IDLE || g_acal_state != ACAL_IDLE) {
                    g_home_state    = HOME_IDLE;
                    g_acal_state    = ACAL_IDLE;
                    g_soft_limit_on = 1U;
                    printf("\r\n>>> [ABORT] 비상 정지 및 마법사 취소\r\n\r\n");
                } else {
                    printf("\r\n>>> [STOP] IDLE 모드 (CCR1=0)\r\n");
                }
            } else if (cmd == '?') {
                print_menu();
            } else if (cmd == 'c' || cmd == 'C') {
                if (g_ctrl_state == CTRL_IDLE) {
                    g_home_state  = HOME_IDLE;
                    g_acal_state  = ACAL_IDLE;
                    g_arm_hold_ms = 0U;
                    g_ctrl_state  = CTRL_ARMED;
                    printf("\r\n>>> [ARMED] 진자를 12시(+/-8 deg)로 세우면 %s 밸런싱이 시작됩니다!\r\n",
                           (g_ctrl_mode == MODE_LQR) ? "LQR" : "PD");
                } else {
                    motor_stop();
                    g_ctrl_state = CTRL_IDLE;
                    printf("\r\n>>> [DISARMED] IDLE 모드 복귀\r\n");
                }
            } else if (cmd == 'm' || cmd == 'M') {
                g_ctrl_mode = (g_ctrl_mode == MODE_LQR) ? MODE_PD : MODE_LQR;
                printf("\r\n>>> [MODE] %s\r\n",
                       (g_ctrl_mode == MODE_LQR) ? "LQR (4-State Position+Angle)" : "PD (Angle-Only)");
            } else if (cmd == 'p' || cmd == 'P') {
                g_angle_sign = (int8_t)(-g_angle_sign);
                s_hist_reset_req = 1U;
                printf("\r\n>>> [ANGLE SIGN] %+d (오른쪽 기울임 = + 가 되도록 맞추세요)\r\n",
                       (int)g_angle_sign);
            } else if (cmd == 'g' || cmd == 'G') {
                const char *names[TUNE_COUNT] = {"Kx", "Kdx", "Kth", "Kdth", "Deadzone%"};
                g_tune_idx = (TuneParamIndex)((g_tune_idx + 1U) % TUNE_COUNT);
                printf("\r\n>>> [TUNE SELECT] %s (조절: '+' 또는 '-')\r\n", names[g_tune_idx]);
            } else if (cmd == '+' || cmd == '=') {
                adjust_selected_gain(1);
            } else if (cmd == '-' || cmd == '_') {
                adjust_selected_gain(-1);
            } else if (cmd == 'h' || cmd == 'H') {
                handle_cart_homing_key();
            } else if (cmd == 'a' || cmd == 'A') {
                handle_angle_cal_key();
            } else if (g_home_state != HOME_IDLE || g_acal_state != ACAL_IDLE) {
                if (cmd == '\r' || cmd == '\n') {
                    if (g_home_state == HOME_WAIT_RIGHT || g_home_state == HOME_WAIT_LEFT) {
                        handle_cart_homing_key();
                    } else if (g_acal_state == ACAL_WAIT_DOWN || g_acal_state == ACAL_WAIT_UP) {
                        handle_angle_cal_key();
                    }
                }
            } else if (cmd == 'z' || cmd == 'Z') {
                encoder_set_counts_atomic(0, g_enc_total_counts, g_enc_sign);
                printf("\r\n>>> [ZERO CART] Pos = +0.0 mm\r\n");
            } else if (cmd == 'd' || cmd == 'D') {
                g_angle_down_adc = g_angle_adc_raw;
                g_angle_zero_adc = (int32_t)g_angle_down_adc + (g_pot_up_dir_sign * g_pot_180_counts);
                s_hist_reset_req = 1U;
                printf("\r\n>>> [6h ZERO] 6시 ADC=%u -> 12시 영점(%ld) 자동 산출\r\n",
                       (unsigned int)g_angle_down_adc, (long)g_angle_zero_adc);
            } else if (cmd == 'u' || cmd == 'U') {
                g_angle_zero_adc = (int32_t)g_angle_adc_raw;
                s_hist_reset_req = 1U;
                printf("\r\n>>> [12h ZERO] 12시 영점 ADC=%ld -> +0.00 deg\r\n",
                       (long)g_angle_zero_adc);
            } else if (cmd == 'l' || cmd == 'L') {
                g_soft_limit_on = g_soft_limit_on ? 0U : 1U;
                printf("\r\n>>> [SOFT LIMIT] %s\r\n", g_soft_limit_on ? "ON" : "OFF");
            } else if (g_ctrl_state == CTRL_IDLE) {
                if (cmd == 'f' || cmd == 'F') {
                    printf("\r\n>>> [KICK RIGHT] Duty=%u%%\r\n", g_test_duty_pct);
                    motor_kick(1U, g_test_duty_pct, 250U);
                } else if (cmd == 'b' || cmd == 'B') {
                    printf("\r\n>>> [KICK LEFT] Duty=%u%%\r\n", g_test_duty_pct);
                    motor_kick(0U, g_test_duty_pct, 250U);
                } else if (cmd >= '1' && cmd <= '6') {
                    const uint8_t duties[6] = {15, 25, 35, 50, 65, 80};
                    g_test_duty_pct = duties[cmd - '1'];
                    printf("\r\n>>> [DUTY SET] %u%%\r\n", g_test_duty_pct);
                }
            }
        }

        // (C) 200ms (5Hz) 텔레메트리 출력
        if ((g_home_state == HOME_IDLE || g_home_state == HOME_AUTO_CENTER) &&
            (g_acal_state == ACAL_IDLE) &&
            ((g_system_millis - last_print) >= 200U)) {
            last_print = g_system_millis;
            const char *st = (g_ctrl_state == CTRL_BALANCING) ? "BAL" :
                             ((g_ctrl_state == CTRL_ARMED) ? "ARM" : "IDL");

            printf("[%s|%s] x:", st, (g_ctrl_mode == MODE_LQR) ? "LQR" : "PD ");
            print_fixed_1(g_telem_pos_mm_x10);
            printf("mm dx:");
            print_fixed_1(g_telem_vel_mms_x10);
            printf(" | th:");
            print_fixed_2(g_telem_ang_cdeg);
            printf("deg dth:");
            print_fixed_1(g_telem_dth_x10);
            printf(" | PWM:%+3ld%%\r\n", (long)g_telem_u_pct);
        }
    }
}
```

---

## 6. 실물 밸런싱 테스트 및 현장 튜닝 순서

1. **초기화 (`h`, `a`/`d`)**:
   - **`h`** 로 카트 양 끝을 찍어 중앙(`0.0 mm`) 자동 정렬을 마칩니다.
   - **`a`** (최초 1회) 또는 **`d`** (6시 간편 영점)로 진자 각도 영점을 맞춥니다.
2. **부호 최종 확인 (`IDL` 상태)**:
   - 손으로 진자를 12시 근처로 올려 **오른쪽(`f` 방향)으로 살짝 기울였을 때 `th`가 `+` 로 나오는지** 확인합니다. 만약 `-`가 나온다면 **`p`** 키를 한 번 눌러 부호를 반전시키세요!
3. **밸런싱 시작 (`c` 키)**:
   - **`c`** 를 눌러 `ARMED` 상태로 들어간 뒤, 진자를 손으로 12시(`±8°` 이내)에 0.1초간 세우면 `[BALANCING START] LQR 1kHz Active!` 와 함께 제어가 시작됩니다.
   - 비상 시 언제든 **`s` 또는 `Space`** 로 즉시 정지할 수 있습니다.
4. **현장 미세 튜닝 (`g` + `+` / `-`)**:
   - **제자리에서 부르르 떨거나 진동함**: `Deadzone%`(`g`로 선택 후 `-`) 또는 `Kdth`를 10%씩 낮춥니다.
   - **반응이 굼뜨고 한쪽으로 눕는 경향**: `Kth` 또는 `Deadzone%`를 `+`로 한두 단계 올립니다.
   - **진자는 서는데 카트가 천천히 중앙에서 벗어남**: `Kx`, `Kdx`를 `+`로 키웁니다.

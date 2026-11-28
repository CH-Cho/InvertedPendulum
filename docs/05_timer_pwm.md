# 05. TIM 하드웨어 PWM 및 듀티비 제어 (Timer PWM)

## 1. 개요
도립진자(Inverted Pendulum) 카트의 DC/BLDC 구동 모터에 정밀한 전기적 힘(토크)을 인가하기 위한 **하드웨어 타이머 PWM(Pulse Width Modulation, 펄스 폭 변조)**의 동작 원리를 이해하고 구현합니다.

본 튜토리얼은 두 단계로 구성되어 있으며, **Step 05-1 (기초 PWM 및 버튼 제어)**과 **Step 05-2 (실전 Pololu #4037 DRV8876 H-Bridge 구동, 20kHz 비가청 PWM 및 UART 원격 조종)**를 모두 포함합니다.
- **Step 05-1**: TIM2 하드웨어 PWM (1 kHz) + 온보드 LED(`PA5`) 밝기 6단계 조절 + 스코프 관측 핀(`PA0`) 동시 출력 + TIM6 기반 200ms 논블로킹 디바운싱 버튼 제어 + 정수형 mV 시리얼 텔레메트리
- **Step 05-2**: `20kHz` 비가청 대역 PWM 업그레이드 + Pololu #4037 (`DRV8876`) PH/EN 모드 정/역회전 제어(`PA0`, `PB0`) + `USART2 RX` 인터럽트 키보드 조종 & `250ms` 안전 펄스 킥(Pulse Kick) + IP570 하드웨어 통합 검증

---

## 2. 하드웨어 스펙 및 핀 매핑

| 기능 | 핀 번호 | 대체 기능 (AF) | 물리적 연결 및 용도 |
| :--- | :---: | :---: | :--- |
| **PWM 출력 1 (시각 확인)** | `PA5` | `AF1` (`TIM2_CH1`) | NUCLEO 온보드 녹색 LED (LD2, 아두이노 D13 핀) $\rightarrow$ **LED 밝기(Dimming) 확인** |
| **PWM 출력 2 (계측기 관측)** | `PA0` | `AF1` (`TIM2_CH1`) | 아두이노 헤더 `A0` 핀 $\rightarrow$ **오실로스코프 / 로직 아날라이저 관측** |
| **유저 버튼 (B1)** | `PC13` | - | 누를 때마다 듀티비 +20% 증가 (Falling Edge EXTI13 인터럽트) |
| **VCP 시리얼 송신 (TX)** | `PA2` | `AF7` (`USART2_TX`) | 115200 bps 텔레메트리 출력 (PC 터미널로 실시간 상태 전송) |
| **VCP 시리얼 수신 (RX)** | `PA3` | `AF7` (`USART2_RX`) | ST-LINK VCP 수신 핀 (Step 05-2 키보드 입력용) |

> [!TIP]
> **왜 PA5와 PA0을 동시에 묶었을까?**  
> `PA5`(LED)와 `PA0`(A0 헤더)은 STM32G474 내부에서 동일한 타이머 채널인 **`TIM2_CH1` (AF1)**을 공유할 수 있습니다.  
> 따라서 한 번의 레지스터 설정으로 **눈앞의 LED 밝기가 변하는 동시에, 오실로스코프/로직 아날라이저 프로브를 A0 핀과 인접 GND 핀에 물려 완벽히 동기화된 사각파 펄스를 계측**할 수 있습니다.

---

## 3. 핵심 동작 원리 및 수식 분석

### 3.1 PWM(펄스 폭 변조)의 개념
PWM은 전압의 크기를 직접 바꾸는 것이 아니라, 전압을 빠른 속도로 켰다 껐다(ON/OFF) 반복하면서 **켜져 있는 시간($T_{\text{on}}$)의 비율(듀티비, Duty Cycle)**을 바꾸어 부하(모터, LED)에 인가되는 **평균 전압($V_{\text{avg}}$)**을 제어하는 기법입니다.

$$\text{Duty Cycle } D = \frac{T_{\text{on}}}{T} \times 100\%$$

$$V_{\text{avg}} = V_{\text{DD}} \times D = 3.3\text{V} \times \frac{\text{CCR1}}{\text{ARR}}$$

---

### 3.2 하드웨어 타이머의 PWM 생성 메커니즘 (RM0440 Section 30.4.11)

STM32 내부의 타이머 카운터(`CNT`)는 클럭 펄스가 들어올 때마다 0부터 1씩 계속 증가하는 계수기입니다. PWM은 다음 3개의 레지스터 상호작용으로 생성됩니다:

```text
 전압 / 카운트
  ARR (1000) ┌ - - - - - - - - - - - - - - - - - - - - - ┐ (오버플로우 -> 0으로 자동 리셋)
             │                                   /│      /│
             │                                  / │     / │
  CCR1 (400) ├ - - - - - - - - - - - - - - - - / -│- - / -│- - (하드웨어 비교 문턱값)
             │                     /│         /   │   /   │
             │                    / │        /    │  /    │
   CNT (0)   └───────────────────/──┴───────/─────┴─/─────┴───> 시간 (t)
                                 |  |       |     |
출력 핀 (PA5/PA0)                |  |       |     |
  High (3.3V)┌───────────────────┐  │       ┌─────┐
             │                   │  │       │     │
   Low (0V)  └───-───────────────┴──┴───────┴─────┴───────> 시간 (t)
             |<---- Ton (400us) ->| |
             |<--------- 1 주기 T (1000us = 1ms) --------->|
             |                   (40% Duty)                |
```

1. **`PSC` (Prescaler)**: 클럭 분주기 $\rightarrow$ 카운터 속도를 **$1\,\mu\text{s}$ 단위로 정규화**
2. **`ARR` (Auto-Reload Register)**: 카운터의 천장 $\rightarrow$ **PWM 주기 및 주파수 결정**
3. **`CCR1` (Capture/Compare Register 1)**: 전압 반전 기준선 $\rightarrow$ **PWM 듀티비 결정**

---

### 3.3 PWM Mode 1 동작 규칙
본 프로젝트에서는 **PWM Mode 1 (`OC1M = 0110`)**을 사용합니다:

$$\text{출력 신호} = 
\begin{cases} 
\mathbf{High \, (3.3V)}, & \text{if } \text{CNT} < \text{CCR1} \\
\mathbf{Low \, (0V)}, & \text{if } \text{CNT} \ge \text{CCR1}
\end{cases}$$

- `CNT < CCR1`: 하드웨어가 자동으로 핀을 **High (3.3V)**로 유지합니다.
- `CNT >= CCR1`: 하드웨어가 자동으로 핀을 **Low (0V)**로 떨어뜨립니다.
- `CNT == ARR`: 천장에 도달하면 카운터가 0으로 리셋되면서 다시 High로 상승합니다.
- 따라서 `CCR1` 값을 키우면 펄스 폭($T_{\text{on}}$)이 정밀하게 늘어납니다.

---

### 3.4 레지스터 수치 계산식

#### [Step 1] 프리스케일러 (`PSC = 15`)
시스템 클럭 $f_{\text{CLK}} = 16\,\text{MHz}$ ($16,000,000\,\text{Hz}$):
$$f_{\text{CNT}} = \frac{f_{\text{CLK}}}{\text{PSC} + 1} = \frac{16,000,000}{15 + 1} = 1,000,000\,\text{Hz} = \mathbf{1\,\text{MHz}}$$
- 타이머 카운터(`CNT`)가 정확히 **$1\,\mu\text{s}$ (1마이크로초)마다 1씩 증가**하도록 세팅됩니다.

#### [Step 2] 자동 리로드 (`ARR = 1000`)
목표 주파수 $1\,\text{kHz}$ (주기 $1\,\text{ms} = 1,000\,\mu\text{s}$):
$$f_{\text{PWM}} = \frac{f_{\text{CNT}}}{\text{ARR}} = \frac{1,000,000\,\text{Hz}}{1000} = \mathbf{1,000\,\text{Hz} \, (1\,\text{kHz})}$$
- 천장을 1000으로 설정함으로써 **분해능이 1,000단계(0.1% 단위)**가 되어 `CCR1` 값과 듀티비(%)의 1:1 환산이 매우 직관적입니다.

#### [Step 3] 듀티비 스텝 매핑표 (버튼 입력에 따른 6단계 변화)
$$\text{Duty Cycle (\%)} = \frac{\text{CCR1}}{\text{ARR}} \times 100\% = \frac{\text{CCR1}}{1000} \times 100\%$$

| 단계 (`g_duty_step`) | `CCR1` 값 | 듀티비 (%) | High 시간 ($T_{\text{on}}$) | 평균 환산 전압 ($V_{\text{avg}}$) | LED 밝기 |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **0** | **`0`** | **$0\%$** | $0\,\mu\text{s}$ | **$0.00\,\text{V}$** | 완전 소등 (OFF) |
| **1** | **`200`** | **$20\%$** | $200\,\mu\text{s}$ | **$0.66\,\text{V}$** | 은은하게 켜짐 |
| **2** | **`400`** | **$40\%$** | $400\,\mu\text{s}$ | **$1.32\,\text{V}$** | 중간 밝기 |
| **3** | **`600`** | **$60\%$** | $600\,\mu\text{s}$ | **$1.98\,\text{V}$** | 밝음 |
| **4** | **`800`** | **$80\%$** | $800\,\mu\text{s}$ | **$2.64\,\text{V}$** | 매우 밝음 |
| **5** | **`1000`** | **$100\%$** | $1,000\,\mu\text{s}$ | **$3.30\,\text{V}$** | 최대 밝기 (상시 High) |

---

### 3.5 글리치 방지: Preload Register (`OC1PE = 1`)
런타임에 CPU가 버튼을 눌러 `CCR1` 값을 바꿀 때, 카운터가 동작하는 도중 즉시 반영되면 원치 않는 가짜 펄스(Glitch)가 발생할 수 있습니다.  
`TIM2_CCMR1 |= (1U << 3);` (OC1PE = 1)을 설정하면, CPU가 `CCR1`에 새 값을 써도 즉시 바뀌지 않고 **그림자 레지스터(Shadow Register)**에 머물다가 **현재 1ms 주기가 완전히 끝나는 시점에 안전하게 반영**됩니다.

---

## 4. 모터 제어 관점 심층 분석: 왜 1 kHz인가?

### 4.1 모터의 본질: 인덕터($L$)에 의한 연속 전류(Continuous Conduction)
모터는 단순한 저항이 아니라 구리선을 수백 번 감아놓은 거대한 코일(인덕턴스 $L$)입니다.
- 인덕터는 전류의 급격한 변화를 방해하는 성질($V = L \frac{di}{dt}$)이 있어, 전압이 0V로 떨어지는 오프(OFF) 시간에도 자기장이 전류를 계속 밀어줍니다.
- 따라서 $1\,\text{kHz}$ 이상의 주파수에서는 전류가 끊기지 않고 **부드러운 연속 직류(Continuous Current)**로 유지되어 모터가 진동 없이 매끄럽게 회전합니다.
- 만약 주파수가 너무 낮으면($10 \sim 50\,\text{Hz}$) 전류가 0으로 뚝뚝 떨어지며 모터가 덜컥거리는 토크 리플(Torque Ripple)이 심해집니다.

### 4.2 1 kHz의 장단점 및 20 kHz 무소음 확장

| 주파수 대역 | 모터 동작 특성 | 소음 특성 | 발열/효율 | 비고 |
| :---: | :--- | :--- | :---: | :--- |
| **$1\,\text{kHz}$ (현재)** | 부드럽고 안정적으로 회전 | **"삐---" 고주파 가청 소음 발생** | **발열 거의 없음 (매우 안전)** | 초보자 실습, 저가형 드라이버(L298N) 호환 |
| **$20\,\text{kHz}$ (실전)** | 완벽한 연속 전류, 고응답 | **완벽한 무소음 (가청대역 밖)** | 고속 스위칭 발열 관리 필요 | **도립진자, 산업용 서보, 드론** |

> [!NOTE]
> 인간의 가청 주파수는 $20\,\text{Hz} \sim 20,000\,\text{Hz} (20\,\text{kHz})$입니다. $1\,\text{kHz}$는 사람의 귀가 가장 민감하게 반응하는 대역이므로 모터 코일에서 고주파 비명소리(Whining Noise)가 발생합니다.  
> 실제 상용 도립진자 로봇에서는 모터 소음을 없애기 위해 추후 $20\,\text{kHz}$ 초음파 대역으로 주파수를 상향하게 됩니다.

---

## 5. 4중 하드웨어 통합 아키텍처

본 예제는 임베디드 실전 제어기의 핵심인 4대 주변장치가 유기적으로 결합된 구조입니다:

```text
[하드웨어 주변장치 계층]
  ├── TIM2 (1kHz PWM)  ────> PA5(LED) & PA0(Scope)로 하드웨어 자율 사각파 출력 (CPU 무관)
  ├── TIM6 (1kHz Tick) ────> 1ms마다 g_system_millis 증가 (시간의 심장)
  ├── EXTI13 (PC13 버튼) ───> TIM6 틱 기반 200ms 논블로킹 디바운싱 -> 듀티비 스텝(CCR1) 변경
  └── USART2 (115200 bps) ──> 변경된 듀티비, 펄스폭, 환산 전압을 정수형(mV)으로 실시간 전송
```

### 안티패턴(Anti-pattern) 제거: 타이머 기반 논블로킹 디바운싱
Step 02에서 사용했던 `for` 루프 지연 함수(`delay_debounce`)를 인터럽트(ISR) 내부에서 돌리는 것은 **CPU의 모든 제어 루프를 멈춰 세우는 치명적인 안티패턴**입니다.
- 본 예제에서는 Step 04의 `TIM6` 시스템 틱(`g_system_millis`)을 활용하여, **"마지막으로 버튼이 눌린 시간과의 차이가 200ms 이상일 때만 유효 입력으로 인정"**하는 **논블로킹 타임스탬프 디바운싱**을 적용하여 스위치 채터링을 100% 방어했습니다.

### newlib-nano `_printf_float` 에러 원천 차단
임베디드 GCC 라이브러리(`newlib-nano`)는 기본적으로 Flash 메모리 절약을 위해 `printf`의 `%f` 포맷팅을 차단합니다.  
본 코드에서는 부동소수점 라이브러리를 강제로 링크하는 대신, **밀리볼트(mV) 정수 연산**(`v_int = mv / 1000`, `v_dec = (mv % 1000) / 10`)으로 변환하여 **추가 플래그 없이도 가볍고 안전하게 전압을 출력**합니다.

---

## 6. 전체 소스 코드 (`Src/main.c`)

```c
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-9 / Step 05-1] TIM2 PWM (1kHz) & TIM6 1ms TimeBase & Button EXTI
 *                   PA5 (LED) & PA0 (Arduino A0 / Scope) -> TIM2_CH1 Hardware PWM
 *                   TIM6 -> 1ms System Tick (Non-blocking Debounce TimeBase)
 *                   Button (PC13) -> 0% -> 20% -> 40% -> 60% -> 80% -> 100% -> 0%
 * @repository     : https://github.com/CH-Cho/InvertedPendulum
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

/* 레지스터 주소 매핑 (STM32G474 RM0440 기준) */
// 1. RCC (Reset and Clock Control)
#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB1ENR1        (*(volatile uint32_t *)(RCC_BASE + 0x58U))
#define RCC_APB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x60U))

// 2. GPIO Port A (PA0: Scope/A0, PA2: TX, PA3: RX, PA5: LED)
#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_AFRL          (*(volatile uint32_t *)(GPIOA_BASE + 0x20U))

// 3. GPIO Port C (PC13: Blue User Button)
#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))

// 4. System Configuration (SYSCFG) & EXTI
#define SYSCFG_BASE         0x40010000U
#define SYSCFG_EXTICR4      (*(volatile uint32_t *)(SYSCFG_BASE + 0x14U))
#define EXTI_BASE           0x40010400U
#define EXTI_IMR1           (*(volatile uint32_t *)(EXTI_BASE + 0x00U))
#define EXTI_FTSR1          (*(volatile uint32_t *)(EXTI_BASE + 0x0CU))
#define EXTI_PR1            (*(volatile uint32_t *)(EXTI_BASE + 0x14U))
#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104U))

// 5. USART2 Peripheral (115200 bps VCP)
#define USART2_BASE         0x40004400U
#define USART2_CR1          (*(volatile uint32_t *)(USART2_BASE + 0x00U))
#define USART2_BRR          (*(volatile uint32_t *)(USART2_BASE + 0x0CU))
#define USART2_ISR          (*(volatile uint32_t *)(USART2_BASE + 0x1CU))
#define USART2_TDR          (*(volatile uint32_t *)(USART2_BASE + 0x28U))

// 6. TIM6 Basic Timer (1ms System Heartbeat TimeBase)
#define TIM6_BASE           0x40001000U
#define TIM6_CR1            (*(volatile uint32_t *)(TIM6_BASE + 0x00U))
#define TIM6_DIER           (*(volatile uint32_t *)(TIM6_BASE + 0x0CU))
#define TIM6_SR             (*(volatile uint32_t *)(TIM6_BASE + 0x10U))
#define TIM6_PSC            (*(volatile uint32_t *)(TIM6_BASE + 0x28U))
#define TIM6_ARR            (*(volatile uint32_t *)(TIM6_BASE + 0x2CU))

// 7. TIM2 General Purpose Timer (1kHz PWM Hardware Generator)
#define TIM2_BASE           0x40000000U
#define TIM2_CR1            (*(volatile uint32_t *)(TIM2_BASE + 0x00U))
#define TIM2_CCMR1          (*(volatile uint32_t *)(TIM2_BASE + 0x18U))
#define TIM2_CCER           (*(volatile uint32_t *)(TIM2_BASE + 0x20U))
#define TIM2_PSC            (*(volatile uint32_t *)(TIM2_BASE + 0x28U))
#define TIM2_ARR            (*(volatile uint32_t *)(TIM2_BASE + 0x2CU))
#define TIM2_CCR1           (*(volatile uint32_t *)(TIM2_BASE + 0x34U))

/* 시스템 전역 변수 */
static volatile uint32_t g_system_millis = 0;      // 1ms 정주기 시스템 틱
static volatile uint32_t g_last_button_time = 0;   // 하드웨어 타이머 기반 디바운싱 기준 시간
static volatile uint8_t  g_duty_step = 0;          // 0~5 단계 (0%, 20%, 40%, 60%, 80%, 100%)
static volatile uint8_t  g_button_event = 0;       // 버튼 이벤트 플래그

/**
 * @brief USART2 1바이트 송신
 */
static void uart2_write_char(uint8_t ch)
{
    while (!(USART2_ISR & (1U << 7)));
    USART2_TDR = ch;
}

/**
 * @brief printf 리디렉션 (_write)
 */
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

/**
 * @brief USART2 초기화 (115200 bps, PA2=TX, PA3=RX)
 */
static void uart2_init(void)
{
    RCC_AHB2ENR  |= (1U << 0);  // GPIOAEN
    RCC_APB1ENR1 |= (1U << 17); // USART2EN

    // PA2, PA3 -> Alternate Function (10)
    GPIOA_MODER &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA_MODER |=  ((2U << (2 * 2)) | (2U << (3 * 2)));

    // PA2, PA3 -> AF7 (USART2)
    GPIOA_AFRL &= ~((0xFU << (2 * 4)) | (0xFU << (3 * 4)));
    GPIOA_AFRL |=  ((0x7U << (2 * 4)) | (0x7U << (3 * 4)));

    USART2_BRR = 139U; // 16MHz / 115200 = 138.88 -> 139 (OVER8 = 0)
    USART2_CR1 = (1U << 0) | (1U << 3); // UE, TE
}

/**
 * @brief TIM6 기본 타이머 초기화 (정확히 1ms마다 인터럽트 발생)
 */
static void tim6_init(void)
{
    RCC_APB1ENR1 |= (1U << 4); // TIM6EN

    TIM6_PSC = 15U;  // 16MHz -> 1MHz (1us 주기)
    TIM6_ARR = 999U; // 1000us = 1ms (1000Hz)

    TIM6_DIER |= (1U << 0);          // Update Interrupt 허용
    NVIC_ISER1 |= (1U << (54 - 32)); // NVIC IRQ 54 (TIM6) 활성화
    TIM6_CR1 |= (1U << 0);           // 타이머 시작
}

/**
 * @brief TIM2 하드웨어 PWM 초기화 (1kHz 주파수, 0~1000 분해능)
 *        PA5(LED)와 PA0(아두이노 A0 핀 / 스코프) 동시 출력
 */
static void tim2_pwm_init(void)
{
    RCC_APB1ENR1 |= (1U << 0); // TIM2EN
    RCC_AHB2ENR  |= (1U << 0); // GPIOAEN

    // 1. PA0(A0핀)과 PA5(온보드 녹색 LED)를 Alternate Function 모드(10)로 설정
    GPIOA_MODER &= ~((3U << (0 * 2)) | (3U << (5 * 2)));
    GPIOA_MODER |=  ((2U << (0 * 2)) | (2U << (5 * 2)));

    // 2. PA0과 PA5를 AF1 (TIM2_CH1)으로 라우팅
    GPIOA_AFRL &= ~((0xFU << (0 * 4)) | (0xFU << (5 * 4)));
    GPIOA_AFRL |=  ((0x1U << (0 * 4)) | (0x1U << (5 * 4)));

    // 3. 1 kHz PWM 주파수 설정 (16MHz / 16 / 1000 = 1000Hz)
    TIM2_PSC = 15U;
    TIM2_ARR = 1000U;

    // 4. 초기 듀티비 0% (소등 상태)
    TIM2_CCR1 = 0U;

    // 5. PWM Mode 1 설정 (OC1M = 0110), Preload 활성화 (OC1PE = 1)
    TIM2_CCMR1 &= ~((0x7U << 4) | (1U << 16) | (3U << 0));
    TIM2_CCMR1 |=  ((0x6U << 4) | (1U << 3));

    // 6. 채널 1 출력 활성화 (CC1E = 1) 및 카운터 시작 (CEN = 1)
    TIM2_CCER |= (1U << 0);
    TIM2_CR1  |= (1U << 0);
}

/**
 * @brief 파란색 유저 버튼(PC13) EXTI13 외부 인터럽트 초기화
 */
static void button_exti_init(void)
{
    RCC_AHB2ENR |= (1U << 2); // GPIOCEN
    RCC_APB2ENR |= (1U << 0); // SYSCFGEN

    GPIOC_MODER &= ~(3U << (13 * 2)); // PC13 Input

    SYSCFG_EXTICR4 &= ~(0xFU << 4);
    SYSCFG_EXTICR4 |=  (0x2U << 4);  // EXTI13 -> Port C 매핑

    EXTI_FTSR1 |= (1U << 13);        // Falling Edge 감지 (버튼 눌림)
    EXTI_IMR1  |= (1U << 13);        // 인터럽트 언마스크
    NVIC_ISER1 |= (1U << (40 - 32)); // NVIC IRQ 40 (EXTI15_10) 활성화
}

/**
 * @brief TIM6 정주기 인터럽트 서비스 루틴 (1ms마다 호출)
 *        순수하게 시스템 밀리초 시계(g_system_millis)만 증가시킵니다.
 */
void TIM6_DACUNDER_IRQHandler(void)
{
    if (TIM6_SR & (1U << 0)) {
        TIM6_SR &= ~(1U << 0); // 플래그 클리어
        g_system_millis++;
    }
}

/**
 * @brief 버튼(PC13) 외부 인터럽트 서비스 루틴 (EXTI15_10)
 *        TIM6 시스템 틱 기반 200ms 논블로킹 디바운싱 적용
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_PR1 & (1U << 13)) {
        EXTI_PR1 = (1U << 13); // 인터럽트 펜딩 플래그 클리어

        // 200ms 하드웨어 타이머 기반 소프트웨어 디바운싱 (채터링 완벽 차단)
        if (g_system_millis - g_last_button_time > 200) {
            g_last_button_time = g_system_millis;

            // 0% -> 20% -> 40% -> 60% -> 80% -> 100% -> 0% 순환
            g_duty_step = (g_duty_step + 1) % 6;
            TIM2_CCR1 = g_duty_step * 200U; // CCR1: 0, 200, 400, 600, 800, 1000

            g_button_event = 1; // 메인 루프에 알림
        }
    }
}

int main(void)
{
    // 1. 하드웨어 주변장치 초기화
    uart2_init();
    tim6_init();
    tim2_pwm_init();
    button_exti_init();

    // 2. 시작 배너 출력
    printf("\r\n");
    printf("====================================================\r\n");
    printf("  STM32G474RE Inverted Pendulum Project             \r\n");
    printf("  [Step 05-1] TIM2 Hardware PWM (1 kHz) Ready!      \r\n");
    printf("  Outputs : PA5 (LED) & PA0 (Arduino A0 / Scope)    \r\n");
    printf("  Control : Press Blue Button (B1) for +20%% Step    \r\n");
    printf("====================================================\r\n\r\n");
    printf("[STATUS] Initial Duty: 0%% (LED OFF | CCR1: 0/1000 | 0.00 V)\r\n");

    // 3. 메인 제어 루프
    while (1)
    {
        if (g_button_event) {
            g_button_event = 0; // 플래그 초기화

            uint32_t duty_percent = g_duty_step * 20;
            // float 대신 밀리볼트(mV) 정수 연산으로 소수점 안전 출력 (Flash 절약 및 float 에러 방지)
            uint32_t millivolts = (3300 * duty_percent) / 100;
            uint32_t v_int = millivolts / 1000;
            uint32_t v_dec = (millivolts % 1000) / 10; // 소수점 둘째 자리

            printf(">>> [PWM EVENT] Duty: %3lu%% | CCR1: %4u/1000 | Pulse: %3lu us | Avg Voltage: %lu.%02lu V\r\n",
                   duty_percent,
                   (unsigned int)TIM2_CCR1,
                   duty_percent * 10,
                   v_int,
                   v_dec);
        }
    }
}
```

---

## 7. 실습 및 검증 결과

1. **온보드 녹색 LED (`PA5`) Dimming**:
   - 파란색 버튼(B1)을 누를 때마다 LED가 **0% (소등) $\rightarrow$ 20% $\rightarrow$ 40% $\rightarrow$ 60% $\rightarrow$ 80% $\rightarrow$ 100% (최대 밝기)**로 순환하며 부드럽게 밝기가 조절됨을 확인.
2. **시리얼 터미널 텔레메트리 (`screen /dev/cu.usbmodem1403 115200`)**:
   ```text
   ====================================================
     STM32G474RE Inverted Pendulum Project             
     [Step 05-1] TIM2 Hardware PWM (1 kHz) Ready!      
     Outputs : PA5 (LED) & PA0 (Arduino A0 / Scope)    
     Control : Press Blue Button (B1) for +20% Step    
   ====================================================

   [STATUS] Initial Duty: 0% (LED OFF | CCR1: 0/1000 | 0.00 V)
   >>> [PWM EVENT] Duty:  20% | CCR1:  200/1000 | Pulse: 200 us | Avg Voltage: 0.66 V
   >>> [PWM EVENT] Duty:  40% | CCR1:  400/1000 | Pulse: 400 us | Avg Voltage: 1.32 V
   >>> [PWM EVENT] Duty:  60% | CCR1:  600/1000 | Pulse: 600 us | Avg Voltage: 1.98 V
   >>> [PWM EVENT] Duty:  80% | CCR1:  800/1000 | Pulse: 800 us | Avg Voltage: 2.64 V
   >>> [PWM EVENT] Duty: 100% | CCR1: 1000/1000 | Pulse: 1000 us | Avg Voltage: 3.30 V
   >>> [PWM EVENT] Duty:   0% | CCR1:    0/1000 | Pulse:   0 us | Avg Voltage: 0.00 V
   ```
3. **오실로스코프 / 테스터기 관측 핀 (`PA0`, 아두이노 `A0`)**:
   - `PA0` 핀과 인접 `GND` 핀 사이의 전압을 측정했을 때, 이론치 계산값($0.00\text{V} \rightarrow 0.66\text{V} \rightarrow 1.32\text{V} \dots \rightarrow 3.30\text{V}$)과 정확히 일치하는 직류 환산 평균 전압 및 $1\text{kHz}$ 사각파 펄스 폭 확인.

---

## 8. [Step 05-2] 실전 H-Bridge 모터 구동 (`DRV8876`), 20kHz PWM & 키보드 원격 조종

### 8.1 왜 20 kHz 비가청(Ultrasonic) PWM을 사용하는가?
- 사람의 가청 주파수 대역은 **20 Hz ~ 20 kHz**입니다.
- Step 05-1에서 사용한 `1 kHz` PWM으로 실제 DC 모터를 구동하면 모터 코일과 철심이 $1\,\text{kHz}$로 진동하면서 날카로운 **"삐~" 하는 전자음(Switching Whine)**이 발생합니다.
- 따라서 실전 모터 제어에서는 PWM 주파수를 가청 대역 경계 밖인 **$20\,\text{kHz}$ 이상**(주기 $50\,\mu\text{s}$)으로 설정해 소음을 제거하고 전류 리플을 줄입니다.

$$\text{PSC} = 0, \quad \text{ARR} = 799 \quad \Longrightarrow \quad f_{\text{PWM}} = \frac{16,000,000\text{ Hz}}{(0 + 1) \times (799 + 1)} = \mathbf{20,000\text{ Hz} \, (20\text{ kHz})}$$

---

### 8.2 Pololu #4037 (`DRV8876`) H-Bridge `PH/EN` 제어 및 실물 회로도

실제 제작한 만능기판 쉴드의 전체 핀맵과 회로도는 다음 문서를 참조합니다:
- **전체 핀맵 정밀 회로도**: [inverted_pendulum_full_schematic.png](images/inverted_pendulum_full_schematic.png)
- **실물 사진 대조 핀맵**: [inverted_pendulum_real_hardware_pinmap.png](images/inverted_pendulum_real_hardware_pinmap.png)

| #4037 핀 (`PH/EN` 모드) | 연결 대상 | 기능 및 상태 |
| :--- | :--- | :--- |
| **L5 (`PMODE`) $\leftrightarrow$ L2 (`GND■`)** | 보드 바닥면 납땜 쇼트 | **`PH/EN` 2선 제어 모드** 하드웨어 고정 |
| **L6 (`nSLEEP`)** | NUCLEO `3V3` (`CN7-16`) | 내부 `10kΩ` 풀다운 해제 (상시 Wakeup) |
| **L3 (`EN/IN1`)** | NUCLEO `PA0` (`CN7-28 / A0`) | `TIM2_CH1` **20 kHz PWM** 속도 입력 |
| **L4 (`PH/IN2`)** | NUCLEO `PB0` (`CN7-34 / A3`) | `GPIO Output` 회전 방향 제어 (`1`=오른쪽 `f`, `0`=왼쪽 `b`) |
| **R1 (`VIN`) / R2 (`GND■`)** | 외부 12V 전원 (`J1` + `100uF C1`) | 모터 구동 주전원 인입 (역기전력 필터링) |
| **R3 (`OUT1`) / R4 (`OUT2`)** | IP570 모터 6P (`1번 +`, `6번 -`) | 양방향 H-Bridge 모터 구동 출력 |

#### `PH/EN` 모드 (`PMODE = LOW`) 진리표
| `EN (PA0)` | `PH (PB0)` | `OUT1` | `OUT2` | 모터 동작 (IP570 실측 기준) |
| :---: | :---: | :---: | :---: | :--- |
| `0` | `X` | `LOW` | `LOW` | **Brake Low (즉시 전기적 제동 정지)** |
| `PWM` | `1` | `PWM (H/L)` | `LOW` | **정회전 (`f`) $\rightarrow$ 카트 오른쪽 이동** |
| `PWM` | `0` | `LOW` | `PWM (H/L)` | **역회전 (`b`) $\rightarrow$ 카트 왼쪽 이동** |

---

### 8.3 STM32G474RE 데이터시트 검증 핵심 (`5V` 내성: `FT` vs `TT_a`)
NUCLEO-G474RE(`DS12288`) 핀 정의표 검증 결과, G474 시리즈는 내장 OPAMP/DAC 때문에 `PA6(D12)`, `PA7(D11)`을 포함한 18개 헤더 핀이 **`TT_a` (3.3V 전용, 최대 3.6V)**입니다.
따라서 `5V` 홀 엔코더를 직결하는 `TIM3` 핀은 **5V 내성(`FT`)이 보장되면서 동일한 `TIM3` 하드웨어 엔코더를 지원하는 `PC6 (CN10-4)` 및 `PC7 (D9 / CN10-19)`**을 사용합니다.

---

### 8.4 USART2 RX 인터럽트 (`IRQ 38`) 및 안전 펄스 킥 구현 핵심

1. **`USART2_RX` 인터럽트 & Overrun(ORE) 방어**:
   ```c
   void USART2_IRQHandler(void)
   {
       if (USART2_ISR & (1U << 3)) {
           USART2_ICR = (1U << 3); // ORE(Overrun Error) 클리어 -> 수신 멈춤 방지
       }
       if (USART2_ISR & (1U << 5)) { // RXNE
           g_rx_cmd = (uint8_t)USART2_RDR;
       }
   }
   ```
2. **레일 충돌 방지용 `250ms` 자동 정지 안전 킥**:
   - `TIM6` 1ms ISR에서 `g_motor_running` 상태를 감시하다가 `250ms`가 경과하면 자동으로 `TIM2_CCR1 = 0`을 인가해 정지시킵니다.
   - 키보드 `f` / `b`로 짧은 펄스 구동 테스트, `1`~`6`으로 듀티(`15% ~ 80%`) 조절, `s`/`Space`로 즉시 비상 정지를 수행합니다.

---

### 8.5 Step 05-2 및 전체 하드웨어 통합 검증 결과

1. **모터 구동 검증 (`DRV8876` + IP570)**:
   - `25%` 듀티에서는 기구물 정지 마찰력(Stiction)이 커서 정지 상태 유지 $\rightarrow$ `50%` 듀티(`4`번 키) 상향 시 안정적 기동 확인.
   - **`f` (`PB0 = 1`) $\rightarrow$ 카트 오른쪽 이동**, **`b` (`PB0 = 0`) $\rightarrow$ 카트 왼쪽 이동** 확인 완료.
2. **동시 계측된 센서 응답 확인**:
   - **카트 엔코더 (`PC6/PC7`, `TIM3` 4체배)**:
     - 가장 오른쪽(`-7439`) ↔ 가장 왼쪽(`-1160`) = **총 `6,279` 카운트 / `385 mm` ($\approx 16.3\text{ counts/mm}$)**, 정지 상태 노이즈 `0` 확인.
     - 참고: 현재 배선 극성 기준 오른쪽 이동 시 카운트 감소(`-`)하므로 Step 06에서 `TIM3_CCER` 극성 반전으로 `오른쪽 = +방향`으로 통일 예정.
   - **진자 각도 센서 (`WDD35D4-5K` on `PA1`, `ADC1_IN2`)**:
     - 수직 아래(6시): `~1024 (0.82V)`, 수직 위(12시): `~3140 (2.53V)` ($\approx 11.75\text{ counts/deg}$) 확인.
     - 수직 위(`3140`) 부근 ±50° 이상이 15° 불감대와 떨어져 있어 균형 제어에 적합한 선형 구간임을 확인 완료.

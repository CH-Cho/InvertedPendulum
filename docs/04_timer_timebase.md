# 04. TIM6 정주기 인터럽트 및 동적 파라미터 제어 (TimeBase)

## 1. 개요
도립진자(Inverted Pendulum) 제어의 핵심인 **1ms (1,000 Hz) 정밀 하드웨어 타이머(TIM6)**를 구축하고, 앞서 학습한 **EXTI 버튼 외부 인터럽트**와 **USART2 시리얼 통신**을 유기적으로 결합하여 **런타임 동적 주기 튜닝(Dynamic Parameter Tuning)** 시스템을 구현합니다.

---

## 2. TIM6 타이머 주파수 분주 계산식 (1ms TimeBase)

STM32G474RE의 기본 타이머(Basic Timer)인 TIM6는 부가 기능(PWM, 캡처) 없이 순수하게 정밀 시간을 계측하고 인터럽트를 발생시키는 전용 타이머입니다.

- **시스템 클럭 ($f_{\text{CLK}}$)**: HSI16 = 16 MHz ($16,000,000 \text{ Hz}$)
- **목표 주기 ($T$)**: 1ms (목표 주파수 $f = 1,000 \text{ Hz}$)

### 타이머 분주 공식 (RM0440)
$$f_{\text{Interrupt}} = \frac{f_{\text{CLK}}}{(\text{PSC} + 1) \times (\text{ARR} + 1)}$$

1. **프리스케일러 (`PSC = 15`)**:
   - $16\,\text{MHz} \div (15 + 1) = 1\,\text{MHz}$
   - 타이머 카운터(`CNT`)가 정확히 **$1\,\mu\text{s}$ (1마이크로초)마다 1씩 증가**합니다.
2. **자동 리로드 (`ARR = 999`)**:
   - 0부터 999까지 **1,000번 카운트**하면 딱 **$1,000\,\mu\text{s} = 1\,\text{ms}$**가 됩니다.
   - 999에 도달하는 순간 오버플로우가 발생하며 `TIM6_DACUNDER_IRQHandler()` 인터럽트가 실행됩니다.

$$\frac{16,000,000}{(15 + 1) \times (999 + 1)} = \frac{16,000,000}{16,000} = \mathbf{1,000 \text{ Hz (오차 0.00%)!}}$$

---

## 3. 심층 분석: 다양한 주파수 생성 및 고속 클럭(170MHz) 확장

TIM6는 기본 주파수에 고정되어 있지 않으며, 레지스터 값 또는 시스템 클럭을 변경하여 자유자재로 설정할 수 있습니다.

### 3.1 출력 주파수 가변 제어표 (16MHz 기준)
| 목표 주파수 / 주기 | PSC 값 | ARR 값 | 주요 활용 분야 |
| :---: | :---: | :---: | :--- |
| **10 kHz (0.1ms)** | `15` | `99` | 고성능 모터 전류 제어 루프 (FOC 벡터 제어) |
| **1 kHz (1ms)** | `15` | `999` | **도립진자 밸런싱 자세 제어 루프 (PID/LQR 표준)** |
| **100 Hz (10ms)** | `159` | `999` | 자이로/가속도 센서 필터링 (칼만 필터) |
| **10 Hz (100ms)** | `1599` | `999` | 배터리 잔량 확인, 상태 LED 점멸 |
| **1 Hz (1초)** | `15999` | `999` | 1초 시계 (RTC 대용) |

### 3.2 PLL 170MHz 고속 클럭 연계 계산
나중에 시스템 클럭을 170MHz로 오버클럭할 경우, 분주식만 다음과 같이 변경하면 동일하게 1ms 인터럽트를 유지할 수 있습니다:
$$f = \frac{170,000,000}{(169 + 1) \times (999 + 1)} = \mathbf{1,000 \text{ Hz}}$$

---

## 4. 3중 통합 아키텍처: TIM6 + EXTI + USART2 런타임 튜닝 구조

사용자가 직접 구현한 코드는 임베디드 실시간 제어의 핵심인 **"비동기 이벤트 기반 런타임 파라미터 변경"** 구조를 완벽하게 보여줍니다.

```text
[하드웨어 인터럽트 계층]
  ├── TIM6 (1kHz) ────> g_system_millis 1씩 증가 (시간의 심장)
  └── EXTI13 (버튼) ───> g_button_pressed = 1 플래그 세팅

[메인 제어 루프 계층]
  ├── [주기 검사]: (g_system_millis - last_print_time >= g_led_toggle_time)
  │      └── 참(True)이면: 현재 경과 시간 및 주기 시리얼 출력 (USART2)
  │
  └── [버튼 이벤트 감지]: if (g_button_pressed == 1)
         ├── g_led_toggle_time += 500ms (500 -> 1000 -> 1500 -> 2000 -> 500ms 순환)
         └── 변경된 주기값을 USART2로 즉시 텔레메트리 피드백!
```

### 도립진자 프로젝트에서의 실전 의의
이 구조는 실제 도립진자 로봇에서 **PC나 버튼을 통해 실시간으로 LQR 게인($K_p, K_i, K_d$)이나 목표 균형 각도($\theta_{\text{ref}}$)를 동적으로 변경하면서 로봇의 반응성을 테스트하는 튜닝 인터페이스의 근간**이 됩니다.

---

## 5. 전체 소스 코드 (`Src/main.c`)

```c
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-8] TIM6 1ms (1kHz) TimeBase + EXTI Button + USART2 Printf
 *                   Dynamic Period Tuning (500ms -> 1000ms -> 1500ms -> 2000ms)
 * @repository     : https://github.com/CH-Cho/InvertedPendulum
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB1ENR1        (*(volatile uint32_t *)(RCC_BASE + 0x58U))
#define RCC_APB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x60U))

#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_ODR           (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))
#define GPIOA_AFRL          (*(volatile uint32_t *)(GPIOA_BASE + 0x20U))

#define USART2_BASE         0x40004400U
#define USART2_CR1          (*(volatile uint32_t *)(USART2_BASE + 0x00U))
#define USART2_BRR          (*(volatile uint32_t *)(USART2_BASE + 0x0CU))
#define USART2_ISR          (*(volatile uint32_t *)(USART2_BASE + 0x1CU))
#define USART2_TDR          (*(volatile uint32_t *)(USART2_BASE + 0x28U))

#define TIM6_BASE           0x40001000U
#define TIM6_CR1            (*(volatile uint32_t *)(TIM6_BASE + 0x00U))
#define TIM6_DIER           (*(volatile uint32_t *)(TIM6_BASE + 0x0CU))
#define TIM6_SR             (*(volatile uint32_t *)(TIM6_BASE + 0x10U))
#define TIM6_PSC            (*(volatile uint32_t *)(TIM6_BASE + 0x28U))
#define TIM6_ARR            (*(volatile uint32_t *)(TIM6_BASE + 0x2CU))

#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))

#define SYSCFG_BASE         0x40010000U
#define SYSCFG_EXTICR4      (*(volatile uint32_t *)(SYSCFG_BASE + 0x14U))
#define EXTI_BASE           0x40010400U
#define EXTI_IMR1           (*(volatile uint32_t *)(EXTI_BASE + 0x00U))
#define EXTI_FTSR1          (*(volatile uint32_t *)(EXTI_BASE + 0x0CU))
#define EXTI_PR1            (*(volatile uint32_t *)(EXTI_BASE + 0x14U))
#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104U))

static volatile uint32_t g_system_millis = 0;
static volatile uint32_t g_button_pressed = 0;
static volatile uint32_t g_led_toggle_time = 500;

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

static void uart2_init(void)
{
    RCC_AHB2ENR  |= (1U << 0);
    RCC_APB1ENR1 |= (1U << 17);

    GPIOA_MODER &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA_MODER |=  ((2U << (2 * 2)) | (2U << (3 * 2)));

    GPIOA_AFRL &= ~((0xFU << (2 * 4)) | (0xFU << (3 * 4)));
    GPIOA_AFRL |=  ((0x7U << (2 * 4)) | (0x7U << (3 * 4)));

    USART2_BRR = 139U;
    USART2_CR1 = (1U << 0) | (1U << 3) | (1U << 2);
}

static void tim6_init(void)
{
    RCC_APB1ENR1 |= (1U << 4);

    TIM6_PSC = 15U; // 16MHz -> 1MHz (1us)
    TIM6_ARR = 999U; // 1000us -> 1ms

    TIM6_DIER |= (1U << 0);          // Update Interrupt 허용
    NVIC_ISER1 = (1U << (54 - 32));   // NVIC IRQ 54 활성화
    TIM6_CR1 |= (1U << 0);           // 카운터 시작
}

void TIM6_DACUNDER_IRQHandler(void)
{
    if (TIM6_SR & (1U << 0))
    {
        TIM6_SR &= ~(1U << 0);
        g_system_millis++;

        if (g_system_millis % g_led_toggle_time == 0) {
            GPIOA_ODR ^= (1U << 5);
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_PR1 & (1U << 13)) {
        EXTI_PR1 = (1U << 13);
        GPIOA_ODR ^= (1U << 5);
        g_button_pressed = 1;
    }
}

int main(void)
{
    uart2_init();
    tim6_init();

    RCC_AHB2ENR |= (1U << 2);
    RCC_APB2ENR |= (1U << 0);
    GPIOA_MODER &= ~(3U << (5 * 2));
    GPIOA_MODER |=  (1U << (5 * 2));

    GPIOC_MODER &= ~(3U << (13 * 2));

    SYSCFG_EXTICR4 &= ~(0xFU << 4);
    SYSCFG_EXTICR4 |=  (0x2U << 4);

    EXTI_FTSR1 |= (1U << 13);
    EXTI_IMR1  |= (1U << 13);
    NVIC_ISER1 |= (1U << (40 - 32));

    printf("\r\n");
    printf("====================================================\r\n");
    printf("  STM32G474RE Inverted Pendulum Project             \r\n");
    printf("  [CH-8] TIM6 1ms (1kHz) Hardware TimeBase Running! \r\n");
    printf("====================================================\r\n\r\n");

    uint32_t last_print_time = 0;

    while (1)
    {
        if (g_system_millis - last_print_time >= g_led_toggle_time)
        {
            last_print_time = g_system_millis;
            printf("[Real-Time Clock] Elapsed: %lu ms (Current Period: %lu ms)\r\n", g_system_millis, g_led_toggle_time);
        }

        if (g_button_pressed == 1)
        {
            g_led_toggle_time += 500;
            printf(">>> [BUTTON EVENT] Cycle Changed -> Interrupt Period: %lu ms\r\n", g_led_toggle_time);

            if (g_led_toggle_time > 2001) {
                g_led_toggle_time = 500;
            }
            g_button_pressed = 0;
        }
    }
}
```

# 02. GPIO EXTI (하드웨어 외부 인터럽트)

## 1. 개요
CPU가 무한 루프에서 버튼 상태를 계속 감시(폴링, Polling)하는 비효율적인 방식 대신, 버튼이 눌리는 순간 **하드웨어 인터럽트(EXTI)**가 CPU를 깨워 즉시 LED를 토글하는 이벤트 구동(Event-driven) 저전력 방식을 구현합니다.

---

## 2. 하드웨어 스펙
- **User Button (B1, Blue)**: `PC13`
  - 평상시: 내부/외부 Pull-Up (High, 3.3V)
  - 눌렀을 때: GND 연결 (Low, 0V) $\rightarrow$ **Falling Edge** 신호 발생
- **User LED (LD2, Green)**: `PA5`

---

## 3. 인터럽트 신호 전달 경로

```text
[PC13 Button 누름] 
       │ (Falling Edge: 3.3V -> 0V)
       ▼
  [SYSCFG] (EXTI13 라인에 Port C 연결)
       │
       ▼
   [EXTI] (13번 라인 트리거 감지 & EXTI_PR1 펜딩 플래그 세팅)
       │
       ▼
   [NVIC] (Cortex-M4 인터럽트 컨트롤러: IRQ 40 EXTI15_10 허용)
       │
       ▼
 [CPU ISR 자동 호출] void EXTI15_10_IRQHandler(void)
       │
       ▼
 [LED 토글 & 펜딩 플래그 클리어 후 다시 Sleep(wfi) 복귀]
```

---

## 4. 핵심 제어 원리 및 레지스터 분석

### Step 1. SYSCFG를 통한 핀 매핑
STM32는 핀 번호별로 EXTI 라인을 공유합니다 (PA13, PB13, PC13은 모두 EXTI13 라인을 공유). 따라서 **SYSCFG(System Configuration)**의 멀티플렉서 레지스터에서 EXTI13을 어떤 포트와 연결할지 지정해야 합니다.
- `SYSCFG_EXTICR4`의 비트 `[7:4]`에 `0x2` (Port C) 기록

### Step 2. EXTI 트리거 및 마스크 설정
- **트리거 감지**: 버튼을 누르는 순간을 포착하기 위해 **Falling Trigger (`EXTI_FTSR1`)**의 13번 비트를 활성화합니다.
- **인터럽트 언마스크**: 인터럽트 신호가 NVIC로 전달되도록 **Interrupt Mask Register (`EXTI_IMR1`)**의 13번 비트를 활성화합니다.

### Step 3. Cortex-M4 NVIC 인터럽트 활성화
Cortex-M4 코어 내부의 NVIC에 `EXTI15_10` (IRQ 번호 40) 인터럽트를 활성화합니다.
- IRQ 40번은 `NVIC_ISER1`의 8번째 비트 (`40 - 32 = 8`)에 해당합니다.

### Step 4. 기계식 스위치 채터링 방지 (Debouncing)
버튼을 손으로 딸깍 누르는 순간, 물리적 금속 접점이 수 마이크로초 단위로 미세하게 수십 번 튕기며(Bounce) 인터럽트가 연속으로 발생하는 현상이 일어납니다. 이를 방지하기 위해 ISR 내부에서 토글 후 짧은 지연 루프(`delay_debounce`)를 수행합니다.

### Step 5. 펜딩 플래그 클리어 (`EXTI_PR1`)의 치명적 중요성 ⭐
인터럽트가 발생하면 하드웨어가 자동으로 `EXTI_PR1`의 해당 비트를 1로 설정합니다.
- ISR 내부에서는 **반드시** 발생한 비트에 **`1`을 써서 클리어(RC_W1)**해야 합니다.
- **데이터시트 주의사항**: 만약 이 플래그를 지우지 않으면, 인터럽트 함수가 끝난 뒤 CPU가 다시 `wfi` 또는 Stop 0 절전 모드로 진입하려 해도 **"처리되지 않은 인터럽트가 남아있다"고 판단하여 절전 모드 진입이 통째로 무시(Ignored)되고 CPU가 계속 돌게 됩니다.**

---

## 5. 전체 소스 코드 (`Src/main.c`)

```c
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-6] GPIO EXTI Tutorial
 *                   Button (PC13) -> EXTI13 -> NVIC -> LED (PA5) Toggle
 * @repository     : https://github.com/CH-Cho/InvertedPendulum
 ******************************************************************************
 */

#include <stdint.h>

/* 레지스터 주소 매핑 (STM32G474 레퍼런스 매뉴얼 RM0440 기준) */
// 1. Reset and Clock Control (RCC)
#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x60U))

// 2. GPIO Port A (PA5 - Green LED)
#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_ODR           (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))

// 3. GPIO Port C (PC13 - Blue Push Button)
#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))

// 4. System Configuration (SYSCFG - EXTI 라우팅 제어)
#define SYSCFG_BASE         0x40010000U
#define SYSCFG_EXTICR4      (*(volatile uint32_t *)(SYSCFG_BASE + 0x14U))

// 5. Extended Interrupt and Event Controller (EXTI)
#define EXTI_BASE           0x40010400U
#define EXTI_IMR1           (*(volatile uint32_t *)(EXTI_BASE + 0x00U))
#define EXTI_FTSR1          (*(volatile uint32_t *)(EXTI_BASE + 0x0CU))
#define EXTI_PR1            (*(volatile uint32_t *)(EXTI_BASE + 0x14U))

// 6. Cortex-M4 NVIC (Nested Vectored Interrupt Controller)
#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104U))

/* 소프트웨어 디바운싱용 간단한 딜레이 함수 */
static void delay_debounce(void)
{
    for (volatile uint32_t i = 0; i < 150000; i++) {
        __asm__("nop");
    }
}

/**
 * @brief EXTI Line 15~10 하드웨어 인터럽트 서비스 루틴 (ISR)
 *        버튼(PC13)을 누르는 순간 하드웨어가 자동으로 이 함수를 호출합니다.
 */
void EXTI15_10_IRQHandler(void)
{
    // 13번 라인에서 발생한 인터럽트인지 확인
    if (EXTI_PR1 & (1U << 13))
    {
        // [중요] 인터럽트 펜딩 플래그 클리어 (해당 비트에 1을 써야 지워짐)
        EXTI_PR1 = (1U << 13);

        // 녹색 LED (PA5) 상태 반전 (토글)
        GPIOA_ODR ^= (1U << 5);

        // 기계식 스위치 채터링 방지 딜레이
        delay_debounce();
    }
}

int main(void)
{
    // [1단계] 클럭 활성화
    // GPIOA(bit 0), GPIOC(bit 2) 클럭 인가
    RCC_AHB2ENR |= (1U << 0) | (1U << 2);
    // SYSCFG 클럭 인가 (EXTI 핀 라우팅을 위해 필수)
    RCC_APB2ENR |= (1U << 0);

    // [2단계] PA5 (LED LD2)를 범용 출력(Output, 01) 모드로 설정
    GPIOA_MODER &= ~(3U << (5 * 2));
    GPIOA_MODER |=  (1U << (5 * 2));

    // [3단계] PC13 (파란색 버튼 B1)을 입력(Input, 00) 모드로 설정
    GPIOC_MODER &= ~(3U << (13 * 2));

    // [4단계] SYSCFG에서 EXTI13 라인을 Port C(0x2)로 라우팅
    // EXTICR4의 비트 [7:4]에 0x2 기록 (0x0:PA, 0x1:PB, 0x2:PC)
    SYSCFG_EXTICR4 &= ~(0xFU << 4);
    SYSCFG_EXTICR4 |=  (0x2U << 4);

    // [5단계] EXTI 13번 라인 설정
    // 버튼을 누르는 순간(High -> Low 전이)을 포착하기 위해 Falling Trigger 활성화
    EXTI_FTSR1 |= (1U << 13);
    // 13번 라인의 인터럽트 마스크 해제 (인터럽트 허용)
    EXTI_IMR1  |= (1U << 13);

    // [6단계] Cortex-M4 NVIC 인터럽트 활성화
    // EXTI15_10의 IRQ 번호는 40번 -> ISER[1]의 8번 비트 (40 - 32 = 8)
    NVIC_ISER1 = (1U << (40 - 32));

    // [7단계] 메인 루프 - CPU는 대기(Sleep) 상태 유지
    // 버튼이 눌리면 하드웨어가 CPU를 깨워 EXTI15_10_IRQHandler를 실행합니다.
    while (1)
    {
        __asm__("wfi"); // Wait For Interrupt: 전력 절약 대기 모드
    }
}
```

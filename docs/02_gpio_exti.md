# 02. GPIO EXTI (하드웨어 외부 인터럽트)

## 1. 개요
CPU가 무한 루프에서 버튼 상태를 계속 감시(폴링)하는 비효율적인 방식 대신, 버튼이 눌리는 순간 **하드웨어 인터럽트(EXTI)**가 CPU를 깨워 즉시 LED를 토글하는 이벤트 구동(Event-driven) 방식을 구현합니다.

---

## 2. 하드웨어 스펙
- **User Button (B1, Blue)**: `PC13`
  - 평상시: Pull-Up (High, 3.3V)
  - 눌렀을 때: GND 연결 (Low, 0V) $\rightarrow$ **Falling Edge** 발생
- **User LED (LD2, Green)**: `PA5`

---

## 3. 인터럽트 신호 전달 경로

```text
[PC13 Button] 
      │ (Falling Edge)
      ▼
 [SYSCFG] (EXTI13 라인에 Port C 연결)
      │
      ▼
   [EXTI] (13번 라인 트리거 감지 & 펜딩 플래그 설정)
      │
      ▼
   [NVIC] (IRQ 번호 40: EXTI15_10 인터럽트 허용)
      │
      ▼
 [CPU ISR 실행] void EXTI15_10_IRQHandler(void)
```

---

## 4. 핵심 레지스터 제어 단계

### Step 1. SYSCFG를 통한 핀 매핑
STM32의 EXTI 라인은 포트별로 공유됩니다 (예: PA13, PB13, PC13은 모두 EXTI13 공유). 따라서 `SYSCFG_EXTICR4` 레지스터에서 EXTI13을 어떤 포트와 연결할지 지정해야 합니다.

- `SYSCFG_EXTICR4`의 비트 `[7:4]`에 `0x2` (Port C) 기록

### Step 2. EXTI 트리거 및 마스크 설정
- **트리거 감지**: 버튼을 누르는 순간을 포착하기 위해 **Falling Trigger (`EXTI_FTSR1`)**의 13번 비트를 켭니다.
- **인터럽트 언마스크**: 인터럽트 신호가 통과할 수 있도록 **Interrupt Mask Register (`EXTI_IMR1`)**의 13번 비트를 켭니다.

### Step 3. Cortex-M4 NVIC 인터럽트 활성화
Cortex-M 코어 내부의 NVIC에 해당 인터럽트(EXTI15_10, IRQ 번호 40)를 활성화합니다.
- IRQ 40번은 `NVIC_ISER1`의 8번 비트 (`40 - 32 = 8`)에 해당합니다.

### Step 4. 인터럽트 서비스 루틴 (ISR)
인터럽트가 발생하면 하드웨어가 자동으로 `EXTI15_10_IRQHandler()`를 호출합니다.
ISR 내부에서는 **반드시** 발생한 인터럽트 펜딩 플래그(`EXTI_PR1`의 13번 비트)에 `1`을 써서 클리어해야 합니다 (클리어하지 않으면 함수 종료 즉시 다시 호출됨).

---

## 5. 전체 코드 (`Src/main.c`)

```c
#include <stdint.h>

#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x60U))

#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_ODR           (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))

#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))

#define SYSCFG_BASE         0x40010000U
#define SYSCFG_EXTICR4      (*(volatile uint32_t *)(SYSCFG_BASE + 0x14U))

#define EXTI_BASE           0x40010400U
#define EXTI_IMR1           (*(volatile uint32_t *)(EXTI_BASE + 0x00U))
#define EXTI_FTSR1          (*(volatile uint32_t *)(EXTI_BASE + 0x0CU))
#define EXTI_PR1            (*(volatile uint32_t *)(EXTI_BASE + 0x14U))

#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104U))

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_PR1 & (1U << 13))
    {
        EXTI_PR1 = (1U << 13); // 펜딩 플래그 클리어
        GPIOA_ODR ^= (1U << 5); // LED 토글
    }
}

int main(void)
{
    // 1. GPIOA, GPIOC, SYSCFG 클럭 켜기
    RCC_AHB2ENR |= (1U << 0) | (1U << 2);
    RCC_APB2ENR |= (1U << 0);

    // 2. PA5(LED) 출력 설정
    GPIOA_MODER &= ~(3U << (5 * 2));
    GPIOA_MODER |=  (1U << (5 * 2));

    // 3. PC13(Button) 입력 설정
    GPIOC_MODER &= ~(3U << (13 * 2));

    // 4. EXTI13에 Port C 매핑
    SYSCFG_EXTICR4 &= ~(0xFU << 4);
    SYSCFG_EXTICR4 |=  (0x2U << 4);

    // 5. Falling edge 트리거 및 인터럽트 활성화
    EXTI_FTSR1 |= (1U << 13);
    EXTI_IMR1  |= (1U << 13);

    // 6. NVIC에서 EXTI15_10 인터럽트 허용 (IRQ 40)
    NVIC_ISER1 = (1U << (40 - 32));

    // 7. 슬립 대기
    while (1)
    {
        __asm__("wfi");
    }
}
```

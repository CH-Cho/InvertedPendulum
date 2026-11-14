# 01. Bare-metal LED Blink (GPIO Output)

## 1. 개요
외부 라이브러리(HAL/LL)에 의존하지 않고, ARM Cortex-M 및 STM32 하드웨어 레지스터를 직접 조작하여 온보드 녹색 LED(LD2, `PA5`)를 점멸시킵니다.

---

## 2. 하드웨어 스펙 및 핀 맵
- **User LED (LD2, Green)**: `PA5` (Port A, Pin 5)
- **Active Level**: High (1일 때 LED 점등, 0일 때 소등)

---

## 3. 핵심 제어 원리 (3-Step)

### Step 1. 주변장치 클럭 활성화 (RCC)
STM32는 전력 절약을 위해 부팅 직후 거의 모든 주변장치의 클럭이 꺼져 있습니다. GPIOA를 사용하려면 먼저 **RCC(Reset and Clock Control)** 레지스터에서 Port A 클럭을 켜야 합니다.

- **RCC Base**: `0x40021000`
- **AHB2ENR Offset**: `0x4C` (주소: `0x4002104C`)
- **비트 0 (GPIOAEN)**: `1`로 설정하여 GPIOA 클럭 인가

```c
#define RCC_AHB2ENR (*(volatile uint32_t *)(0x40021000U + 0x4CU))
RCC_AHB2ENR |= (1U << 0); // GPIOA 클럭 활성화
```

### Step 2. GPIO 포트 모드 설정 (MODER)
GPIO 핀은 입력, 출력, 대체 기능(Alternate Function), 아날로그 중 하나의 모드로 동작합니다. `MODER` 레지스터는 각 핀마다 2비트를 할당합니다.

- **GPIOA_MODER 주소**: `0x48000000`
- **PA5 비트 위치**: `[11:10]`
- **모드 값**:
  - `00`: Input
  - `01`: General purpose output mode (목표 설정값)
  - `10`: Alternate function mode
  - `11`: Analog mode

```c
#define GPIOA_MODER (*(volatile uint32_t *)(0x48000000U + 0x00U))

// 5번 핀(비트 11:10) 클리어 후 01(출력) 설정
GPIOA_MODER &= ~(3U << (5 * 2));
GPIOA_MODER |=  (1U << (5 * 2));
```

### Step 3. 출력 데이터 제어 (ODR)
`ODR (Output Data Register)`의 해당 핀 비트를 1로 쓰면 3.3V(High), 0으로 쓰면 0V(Low)가 출력됩니다. XOR(`^`) 연산자를 이용하면 간단히 토글할 수 있습니다.

- **GPIOA_ODR 주소**: `0x48000014`
- **PA5 비트 위치**: `비트 5`

```c
#define GPIOA_ODR (*(volatile uint32_t *)(0x48000000U + 0x14U))

// PA5 출력 반전
GPIOA_ODR ^= (1U << 5);
```

---

## 4. 전체 코드 (`Src/main.c`)

```c
#include <stdint.h>

#define RCC_BASE        0x40021000U
#define RCC_AHB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x4CU))

#define GPIOA_BASE      0x48000000U
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_ODR       (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))

static void delay(volatile uint32_t count)
{
    while (count--) {
        __asm__("nop");
    }
}

int main(void)
{
    // 1. GPIOA 클럭 활성화
    RCC_AHB2ENR |= (1U << 0);

    // 2. PA5 출력 모드 설정
    GPIOA_MODER &= ~(3U << (5 * 2));
    GPIOA_MODER |=  (1U << (5 * 2));

    // 3. LED 무한 점멸 루프
    while (1)
    {
        GPIOA_ODR ^= (1U << 5);
        delay(1000000); // 약 0.5초 대기 (기본 HSI 16MHz)
    }
}
```

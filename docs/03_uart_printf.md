# 03. USART2 VCP 시리얼 통신 및 printf 리디렉션

## 1. 개요
NUCLEO-G474RE 보드의 내장 ST-LINK/V3E 가상 COM 포트(Virtual COM Port, VCP)를 통해 STM32와 PC 사이에 **115200 bps 시리얼 통신**을 연결하고, 표준 C 라이브러리의 **`printf()` 함수를 리디렉션**하여 센서 값과 상태 로그를 PC 화면으로 실시간 출력합니다.

---

## 2. 하드웨어 회로도 1:1 매핑 (MB1367 Schematic)

NUCLEO-G474RE 공식 회로도(`mb1367-g474re-c04_schematic.pdf`)와 코드의 하드웨어 연결 구조입니다:

```text
[STM32G474RE MCU]                     [ST-LINK/V3E 칩]               [PC / Mac]
  PA2 (Pin 16) ────[ SB13 / 직결 ]────> STLK_VCP_RX (Pin 21) ──USB──> /dev/cu.usbmodem1403
  [USART2_TX]                           (ST-LINK가 수신하여 USB 전송)    (터미널 screen)

  PA3 (Pin 17) <───[ SB14 / 직결 ]──── STLK_VCP_TX (Pin 22) <──USB── 키보드 입력
  [USART2_RX]                           (PC가 보낸 데이터를 MCU로 전달)

  PA5 (Pin 21) ────[ R31 (510Ω) ]─────[▶| 녹색 LED (LD2) ]──┸ GND (Active-High)
  
  PC13 (Pin 2) ────┬───[ R32 (풀업 100kΩ) ]─── +3.3V (VDD)
                   ├───[ 푸시 스위치 B1 ]─────┸ GND (누르면 Falling Edge 발생)
                   └───[ C29 (노이즈 필터 100nF) ]──┸ GND
```

1. **USART2 핀**: `PA2`가 MCU 송신(TX)선으로 ST-LINK의 수신단(STLK_VCP_RX)에 직결되어 있습니다.
2. **User LED (LD2)**: 음극이 GND에 접지된 **Active-High** 구조로, `PA5`에 3.3V(High)를 밀어주어야 510$\Omega$ 저항을 거쳐 전류가 흐르며 켜집니다.
3. **User Button (B1)**: 평상시 풀업 저항(R32)으로 3.3V를 유지하다가, 누르는 순간 GND로 단락되어 **3.3V $\rightarrow$ 0V (Falling Edge)** 전압 강하가 발생합니다. 병렬 커패시터(C29)가 1차 하드웨어 채터링 방지 역할을 수행합니다.

---

## 3. 핵심 레지스터 제어 및 주파수 계산

### Step 1. 클럭 활성화 (RCC)
- **GPIOA 클럭**: `RCC_AHB2ENR` 비트 0 (`GPIOAEN`)
- **USART2 클럭**: `RCC_APB1ENR1` 비트 17 (`USART2EN`)

### Step 2. GPIO 대체 기능(Alternate Function) 매핑
- `MODER`: PA2와 PA3를 **대체 기능 모드(`10`)**로 설정
- `AFRL (Alternate Function Low Register)`:
  - 데이터시트(DS12288)에 명시된 대로 PA2/PA3의 USART2 기능 번호인 **AF7 (`0x7`)**을 기록
  - PA2: 비트 `[11:8]`에 `0x7`
  - PA3: 비트 `[15:12]`에 `0x7`

### Step 3. 115200 bps 보레이트(Baud Rate) 계산
기본 부팅 클럭(HSI16 = 16 MHz)을 기준으로 115200 bps를 만들기 위한 분주비:
$$\text{BRR} = \frac{f_{\text{CK}}}{\text{BaudRate}} = \frac{16\,000\,000}{115\,200} \approx 138.88 \approx 139 \;(\text{0x8B})$$
실제 보레이트: $\frac{16\,000\,000}{139} \approx 115,108 \text{ bps}$ (오차율 약 $0.08\%$로 완벽함)
- `USART2_BRR = 139;`

### Step 4. 송수신 및 모듈 활성화 (`USART2_CR1`)
- 비트 0 (`UE`): USART 모듈 전체 전원 인가
- 비트 3 (`TE`): 송신기(Transmitter) 활성화
- 비트 2 (`RE`): 수신기(Receiver) 활성화

---

## 4. `printf()`에서 `_write()`까지의 내부 호출 체인 및 시스템 콜 오버라이딩

### 4.1 C 표준 라이브러리(Newlib-nano) 호출 흐름
```text
1. printf("Tick: %d\n", 42);
      │
      ▼
2. vfprintf(stdout, format, ap);        <-- 서식(%d) 파싱 및 문자열 변환
      │
      ▼
3. __sputc_r() / fputc()               <-- 변환된 문자들을 stdout 버퍼에 저장
      │
      ▼  ('\n'을 만나거나 버퍼가 가득 차면 Flush 발생!)
4. fflush() / __sflush_r()             <-- 버퍼를 비우기 위해 하위 계층 호출
      │
      ▼
5. __swrite()                          <-- Newlib 라이브러리의 저수준 I/O 어댑터
      │
      ▼
6. _write(1, ptr, len);                <-- ⭐ [우리가 작성한 오버라이딩 함수 도착!]
      │
      ▼
7. USART2_TDR = ch;                    <-- STM32 하드웨어 송신 레지스터로 전송
```

### 4.2 오버라이딩의 비밀: `__attribute__((weak))` 심볼
- 프로젝트 내 [`Src/syscalls.c`](../firmware/G474RE_Blink/Src/syscalls.c)에 `__attribute__((weak)) int _write(...)`로 선언된 임시 껍데기 함수가 들어 있습니다.
- 개발자가 `main.c`에 `weak`가 없는 일반 함수 `int _write(...)`를 작성하면, **링커(Linker)가 껍데기 함수를 버리고 개발자의 함수를 1순위로 강제 연결(Overriding)**합니다.

### 4.3 `_write()` 매개변수 구조
- `int file`: 파일 디스크립터 (`1` = stdout, `2` = stderr)
- `char *ptr`: 출력할 완성된 문자열 메모리 버퍼의 시작 포인터
- `int len`: 출력할 문자들의 총 바이트 수

### 4.4 `\n` 뒤에 `\r`을 붙이는 이유 (타자기 원리)
시리얼 터미널은 타자기 원리로 동작합니다:
- `\n` (Line Feed): 커서를 아래 줄로만 내림 (열 위치 유지)
- `\r` (Carriage Return): 커서를 줄의 맨 왼쪽 끝으로 당김
따라서 `\n`만 보내면 계단식으로 출력되므로, `_write()` 내부에서 `\n` 감지 시 자동으로 `\r`을 먼저 전송하도록 처리합니다.

---

## 5. `delay_ms`에서 2000 루프의 수학적 계산 근거

```c
for (volatile uint32_t i = 0; i < 2000; i++) {
    __asm__("nop");
}
```

1. **1ms 동안 CPU가 소비해야 하는 총 클럭 사이클**:
   - 시스템 클럭 = 16 MHz ($16,000,000 \text{ cycles/sec}$)
   - $1\text{ ms} (0.001\text{초}) \times 16,000,000 \text{ cycles/sec} = \mathbf{16,000 \text{ cycles}}$
2. **루프 1회당 소모되는 클럭 사이클**:
   - `volatile` 변수 `i`의 메모리 로드(`LDR`), 증가(`ADDS`), 메모리 쓰기(`STR`), 비교(`CMP`), 조건 분기(`BLT`)에 **약 8 사이클** 소모
3. **최종 계산**:
   $$\text{필요 루프 수} = \frac{16,000 \text{ cycles}}{8 \text{ cycles/loop}} = \mathbf{2,000 \text{ 회}}$$

---

## 6. 전체 소스 코드 (`Src/main.c`)

```c
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-7] USART2 VCP Serial Communication & printf Redirection
 *                   PA2 (TX), PA3 (RX) -> ST-LINK VCP -> PC Terminal (115200 bps)
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

#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))

#define SYSCFG_BASE         0x40010000U
#define SYSCFG_EXTICR4      (*(volatile uint32_t *)(SYSCFG_BASE + 0x14U))
#define EXTI_BASE           0x40010400U
#define EXTI_IMR1           (*(volatile uint32_t *)(EXTI_BASE + 0x00U))
#define EXTI_FTSR1          (*(volatile uint32_t *)(EXTI_BASE + 0x0CU))
#define EXTI_PR1            (*(volatile uint32_t *)(EXTI_BASE + 0x14U))
#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104U))

#define USART2_BASE         0x40004400U
#define USART2_CR1          (*(volatile uint32_t *)(USART2_BASE + 0x00U))
#define USART2_BRR          (*(volatile uint32_t *)(USART2_BASE + 0x0CU))
#define USART2_ISR          (*(volatile uint32_t *)(USART2_BASE + 0x1CU))
#define USART2_TDR          (*(volatile uint32_t *)(USART2_BASE + 0x28U))

static volatile uint8_t g_button_pressed = 0;

static void delay_ms(volatile uint32_t ms)
{
    while (ms--) {
        for (volatile uint32_t i = 0; i < 2000; i++) {
            __asm__("nop");
        }
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

    RCC_AHB2ENR |= (1U << 2);
    RCC_APB2ENR |= (1U << 0);

    GPIOA_MODER &= ~(3U << (5 * 2));
    GPIOA_MODER |=  (1U << (5 * 2));

    GPIOC_MODER &= ~(3U << (13 * 2));

    SYSCFG_EXTICR4 &= ~(0xFU << 4);
    SYSCFG_EXTICR4 |=  (0x2U << 4);

    EXTI_FTSR1 |= (1U << 13);
    EXTI_IMR1  |= (1U << 13);
    NVIC_ISER1 = (1U << (40 - 32));

    printf("\r\n");
    printf("==================================================\r\n");
    printf("  STM32G474RE Inverted Pendulum Project           \r\n");
    printf("  [CH-7] USART2 VCP Serial & printf Ready!        \r\n");
    printf("  Baud Rate: 115200 bps | SysClock: 16 MHz (HSI)  \r\n");
    printf("==================================================\r\n\r\n");

    uint32_t tick_count = 0;

    while (1)
    {
        printf("[Tick %lu] System running smoothly...\r\n", tick_count++);

        if (g_button_pressed) {
            g_button_pressed = 0;
            printf(">>> [EVENT] Blue User Button (B1) Pressed! LED Toggled!\r\n");
        }

        delay_ms(1000);
    }
}
```

---

## 7. Mac 터미널 확인 방법

```bash
# 터미널 연결 (115200 bps)
screen /dev/cu.usbmodem1403 115200
```
- **종료 단축키**: `Ctrl + A` $\rightarrow$ `K` $\rightarrow$ `y`

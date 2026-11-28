# Step 06. TIM3 직교 엔코더(Quadrature Encoder) & ADC1 진자 각도계 통합

> **관련 Linear 이슈**: [CH-10](https://linear.app/ch-cho/issue/CH-10)
> **전체 회로도**: [`docs/images/inverted_pendulum_full_schematic.png`](images/inverted_pendulum_full_schematic.png) | [`docs/images/inverted_pendulum_real_hardware_pinmap.png`](images/inverted_pendulum_real_hardware_pinmap.png)
> **참고 매뉴얼**: RM0440 (STM32G4 Reference Manual — TIM2/TIM3/TIM4/TIM5 Quadrature Encoder Interface & ADC1), DS12288 (STM32G474RE Datasheet — FT vs TT_a I/O Structure)

---

## 1. 개요 및 학습 목표

Step 05-2에서 `TIM2` 20kHz PWM과 `DRV8876` H-Bridge를 통해 카트를 원하는 방향으로 구동하는 데 성공했습니다. 이제 Step 07(PID/LQR 도립진자 제어기)로 넘어가기 전에, 시스템의 **4대 상태 변수($x, \dot{x}, \theta, \dot{\theta}$)**를 실시간으로 정확하게 측정해야 합니다.

이번 단계에서는 다음 핵심 임베디드 기술을 베어메탈 레지스터 수준에서 구현합니다.

1. **5V 내성(FT) 핀 설계 원칙**: `5V` 홀 엔코더 출력을 안전하게 받기 위한 `PC6 / PC7` (`TIM3_CH1/CH2`, `AF2`, No Pull-Up/Down) 선택 이유 이해
2. **하드웨어 4체배 직교 인코더(Quadrature Encoder Mode 3)**: CPU 개입 0%로 `TIM3`가 A/B상 모든 에지를 카운트하고, `CC1P` 극성 반전으로 **모터 구동(`f`: 오른쪽) 방향과 엔코더 양수(`+`) 방향을 통일**
3. **16-bit 카운터 오버플로우/언더플로우 확장**: `TIM3_CNT`(`0 ~ 65535`)의 랩어라운드를 2의 보수 차분 `(int16_t)(curr - prev)`으로 `int32_t` 무한 카운트로 확장
4. **물리 단위 변환 & EMA LPF 속도 추정**: 실측 캘리브레이션(`6,279 counts / 385 mm`, `2,116 counts / 180 deg`) 적용 및 수치 미분 노이즈를 억제하는 1차 저역통과필터(EMA LPF) 구현
5. **STM32G4 ADC1 시퀀스 & Soft Limit 레일 보호**: `WDD35D4-5K` 포텐쇼미터 각도 측정(`PA1`, `ADC1_IN2`)과 물리 리미트 스위치가 없는 레일을 보호하는 **1ms 고속 방향성 Soft Limit(`±150 mm`)** 구축
6. **대화형 가이드 호밍(`h`) & 2점 각도 캘리브레이션(`a`, `d`)**: 양 끝단(오른쪽/왼쪽) 엔코더 실측 후 자동 중앙 복귀, 진자 6시·12시 2점 교정 및 6시 간편 영점 복원 구현

---

## 2. 실물 하드웨어 핀 매핑 (DS12288 검증 완료)

### 2.1 카트 엔코더 (`PC6`, `PC7` — 반드시 5V FT 핀 사용!)

IP570 선형 도립진자의 6핀 모터 엔코더는 내부 홀 센서가 **`5V` 전원으로 동작하며 약 `5V` 레벨의 구형파를 출력**합니다. NUCLEO-G474RE의 흔한 `TIM3` 핀인 `PA6(D12)` / `PA7(D11)`은 아날로그 겸용 핀(`TT_a` 구조)이라 **절대 정격 최대 전압이 `VDD + 0.3V ≈ 3.6V`로 제한**됩니다. 5V 신호를 직결하면 칩 내부 ESD 보호 다이오드가 파손될 수 있습니다.

따라서 동일한 `TIM3` 하드웨어 인코더 기능을 제공하면서도 **5V 내성(`FT`: Five-volt Tolerant)**이 보장되는 **`PC6` / `PC7`**을 사용합니다.

| 센서 / 신호 | IP570 핀 | NUCLEO-G474RE 핀 | 헤더 위치 | 대체 기능 | GPIO I/O 구조 | 주의사항 |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **Enc 5V** | 5번 핀 | `5V` | `CN7-18` | 전원 | - | 엔코더 전원 공급 |
| **Enc GND** | 2번 핀 | `GND` | `CN7-20` 등 | 기준 접지 | - | 공통 접지 |
| **Enc A상** | 4번 핀 | **`PC6`** | **`CN10-4`** | **`TIM3_CH1` (`AF2`)** | **`FT`** (5V 내성) | **`PUPDR = 00` (풀업/다운 금지!)** |
| **Enc B상** | 3번 핀 | **`PC7`** | **`CN10-19` (`D9`)** | **`TIM3_CH2` (`AF2`)** | **`FT`** (5V 내성) | **`PUPDR = 00` (풀업/다운 금지!)** |

> [!CAUTION]
> **5V FT 핀에서도 내부 Pull-Up(`PUPDR = 01`)은 절대 금지입니다!**  
> 내부 Pull-Up 저항은 `3.3V` 레일에 연결되므로, `5V` 입력 신호가 들어오면 내부 다이오드 경로가 열려 `FT` 핀이더라도 전류가 역류할 수 있습니다. 반드시 `GPIOC_PUPDR = 00` (No Pull-Up/No Pull-Down, Floating Input)으로 설정해야 5V 내성이 유지됩니다.

### 2.2 진자 각도 센서 (`PA1` — `ADC1_IN2`)

| 센서 / 신호 | WDD35D4-5K 단자 | NUCLEO-G474RE 핀 | 헤더 위치 | 기능 |
| :--- | :---: | :---: | :---: | :--- |
| **3.3V 전원** | `3` (빨강) | `3V3` | `CN7-16` | 포텐쇼미터 기준 전압 |
| **와이퍼 출력** | `2` (흰색) | **`PA1`** (`A1`) | `CN7-30` | **`ADC1_IN2` (Analog Mode `11`)** |
| **GND** | `1` (검정) | `GND` | `CN7-20` 등 | 기준 접지 |

---

## 3. TIM3 직교 엔코더(4체배) 원리와 방향 일치

### 3.1 4체배(Encoder Mode 3) 디코딩 원리

엔코더의 A상(TI1)과 B상(TI2)은 서로 90° 위상차를 갖습니다. `TIM3_SMCR`의 `SMS = 011` (Encoder Mode 3)을 설정하면, 타이머가 내부 클록이 아닌 **TI1과 TI2의 모든 상승/하강 에지(1주기당 4번)**마다 현재 상대 핀의 논리 상태를 판별하여 `TIM3_CNT`를 하드웨어적으로 자동 증감합니다.

```text
오른쪽 이동 (A상이 B상보다 90° 앞섬):
  TI1 (A상):  ____|‾‾‾‾‾‾‾‾|________|‾‾‾‾‾‾‾‾|____
  TI2 (B상):  ________|‾‾‾‾‾‾‾‾|________|‾‾‾‾‾‾‾‾|
  Count    :      +1   +2   +3   +4  +5   +6   +7 ... (1주기당 4카운트!)
```

브러시 DC 모터나 긴 점퍼 와이어에서 유입되는 수백 나노초의 글리치(Glitch)를 제거하기 위해 `TIM3_CCMR1`의 디지털 입력 필터 `IC1F = 6`, `IC2F = 6`을 활성화하여 연속 샘플링으로 검증된 신호만 카운트합니다.

### 3.2 `CC1P` 극성 반전으로 제어 좌표계 통일

실물 IP570 장비를 테스트한 결과, 다음과 같은 방향 관계가 관측되었습니다.
- 모터 드라이버에 `f` 명령(`PB0 = 1`, 오른쪽)을 주면 카트가 **오른쪽**으로 이동하지만, 기본 극성에서는 `TIM3_CNT`가 **감소**(`-1160 -> -7439`)했습니다.

소프트웨어 곳곳에 `-` 부호를 붙이는 대신, **`TIM3_CCER`의 `CC1P` 비트(Bit 1)를 `1`로 설정**하여 TI1 입력 극성을 하드웨어에서 반전시키면 A상과 B상의 상대 위상이 뒤집혀 **오른쪽(`f`) 이동 시 `TIM3_CNT`가 자연스럽게 증가(`+`)**하도록 좌표계를 깔끔하게 통일할 수 있습니다.

### 3.3 16-bit `TIM3_CNT` 오버플로우/언더플로우 무결성 확장

`TIM3`은 16-bit 타이머(`0 ~ 65535`)이므로, 영점 `0`에서 왼쪽으로 1틱만 움직여도 `65535`로 언더플로우(Underflow)가 발생합니다. 이를 안전하게 32-bit 부호 있는 위치(`int32_t`)로 확장하는 고전적이면서 완벽한 기법이 **부호 없는 차분의 `int16_t` 캐스팅**입니다.

```c
uint16_t curr_cnt = (uint16_t)TIM3_CNT;
int16_t  delta    = (int16_t)(curr_cnt - s_prev_cnt); // 2의 보수 모듈로 연산
s_prev_cnt = curr_cnt;
g_cart_counts += (int32_t)delta;
```

- 예: `s_prev_cnt = 1` 에서 왼쪽으로 3틱 이동 → `curr_cnt = 65534 (0xFFFE)`
- `(uint16_t)(65534 - 1)` = `65533 (0xFFFD)` → `(int16_t)0xFFFD` = **`-3`** (정확!)

1ms마다 한 번씩만 이 차분을 누적하면(1ms 동안 32,768틱 이상 이동하지 않는 한) 카트가 어느 방향으로 몇 바퀴를 회전하든 단 1카운트도 잃지 않습니다.

---

## 4. 물리 단위 변환, EMA LPF 속도 필터 & Soft Limit

### 4.1 실측 캘리브레이션 상수

사용 중인 실제 IP570 하드웨어 실측 데이터를 코드 상수로 반영했습니다.

- **카트 선형 위치 (`mm`)**: 레일 전 구간 `385 mm` 이동 시 엔코더 총 변화량 `6,279 counts` (`-1160 ~ -7439`)
  $$\text{Scale}_x = \frac{385\text{ mm}}{6279\text{ counts}} \approx 0.061315\text{ mm/count} \quad (\approx 16.309\text{ counts/mm})$$
- **진자 회전 각도 (`deg`)**: `WDD35D4-5K` 12-bit ADC 기준 6시(수직 아래) `~1024` ↔ 12시(수직 위) `~3140` (`180°` 회전 시 `2,116 counts`)
  $$\text{Scale}_\theta = \frac{180^\circ}{2116\text{ counts}} \approx 0.085066^\circ/\text{count} \quad (\approx 11.756\text{ counts/deg})$$

### 4.2 수치 미분 노이즈와 EMA 저역통과필터(LPF)

`dt = 10 ms` (`100 Hz`) 주기로 $\Delta x / \Delta t$를 구하면, 엔코더가 1카운트 변할 때마다 속도가 `0`에서 갑자기 `6.13 mm/s`로 튀는 계단형 양자화 노이즈가 생깁니다. 진자 각도는 1카운트(`~0.085°`)만 흔들려도 각속도가 `8.5 deg/s`씩 튑니다.

이를 부드럽게 만들기 위해 **지수 이동 평균(EMA, 1차 IIR 저역통과필터)**을 적용합니다.

$$y[k] = \alpha \cdot x[k] + (1 - \alpha) \cdot y[k-1], \quad (\alpha = 0.25)$$

100Hz 샘플링에서 $\alpha = 0.25$의 차단 주파수(Cutoff Frequency)는 약 **$f_c \approx \frac{\alpha f_s}{2\pi (1-\alpha)} \approx 5.3\text{ Hz}$** 로, 역진자의 물리적 움직임(1~2Hz)은 충실히 통과시키면서 고주파 미분 노이즈만 효과적으로 걸러냅니다.

> [!NOTE]
> **고정소수점(`int32_t` / Q4) 연산 및 `SCB_CPACR` FPU 주의사항**  
> CubeIDE 기본 프로젝트(`.cproject`)는 `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` 옵션으로 컴파일되지만, 베어메탈 `main.c`에서 FPU 코프로세서(`SCB_CPACR` bits `[23:20]`)를 켜지 않고 `float`를 사용하면 `HardFault`가 발생합니다. 또한 `newlib-nano`는 기본적으로 `printf("%f")`가 비활성화되어 있습니다.  
> 이번 코드에서는 부팅 직후 `fpu_enable()`로 FPU를 안전하게 활성화하는 동시에, 실제 100Hz 연산과 UART 출력은 **오차가 없고 매우 빠른 고정소수점 정수 연산(Q4 EMA 버퍼 + `0.1 mm` / `0.01 deg` 포맷팅)**으로 구현했습니다.

### 4.3 물리 리미트 스위치가 없는 레일을 위한 방향성 Soft Limit

IP570 레일에는 하드웨어 리미트 스위치가 없으므로, `TIM6`의 **1ms 인터럽트** 내부에서 매 밀리초마다 카트 위치를 감시합니다.
- 중앙 영점(`z` 명령) 기준으로 **`±150.0 mm`**를 초과하면 즉시 `TIM2_CCR1 = 0`으로 모터를 차단합니다(전체 스트로크 절반 `±192.5 mm` 대비 40mm 이상 여유).
- **단방향 차단 로직**: `+150.0 mm` 우측 한계에 걸리면 **오른쪽(`f`) 구동만 차단**하고, **왼쪽(`b`)으로 복귀하는 명령은 허용**하므로 레일 끝에서 잠기지 않고 안전하게 중앙으로 되돌릴 수 있습니다.

---

## 5. STM32G4 ADC1 초기화 시퀀스와 WDD35D4-5K 캘리브레이션

STM32G4 시리즈의 12-bit SAR ADC는 부팅 직후 초절전 상태(`DEEPPWD = 1`)이므로 반드시 아래 순서를 지켜야 정상 동작합니다.

1. **`CKMODE = 01` (`ADC12_CCR`)**: `HCLK / 1` 동기 클록(`16 MHz`) 선택 (반드시 `ADEN = 0` 상태에서 설정)
2. **`DEEPPWD = 0` → `ADVREGEN = 1` (`ADC1_CR`)**: Deep-Power-Down 해제 후 내부 아날로그 레귤레이터를 켜고 최소 `20 µs` 이상(코드에서는 `2 ms`) 안정화 대기
3. **`ADCAL = 1` (`ADC1_CR`)**: 단일 종단(`ADCALDIF = 0`) 내부 오프셋 자동 캘리브레이션 수행 및 완료(`ADCAL == 0`) 대기
4. **`ADEN = 1` (`ADC1_CR`)**: `ADC1_ISR`의 `ADRDY` 비트 확인
5. **`SMP2 = 100` (`ADC1_SMPR1`)**: 5kΩ 포텐쇼미터 출력 임피던스에서도 충전 전압이 떨어지지 않도록 충분한 샘플링 시간(`47.5 cycles ≈ 3 µs`) 부여
6. **8회 오버샘플링 평균 (`adc1_read_pa1_avg`) + 런타임 영점 교정**: 매 10ms마다 8회 연속 변환 평균을 취해 ADC LSB 떨림을 줄입니다.

---

## 6. 대화형 위치 초기화(`h`) 및 진자 각도 캘리브레이션(`a`, `d`)

실제 도립진자 실험을 시작할 때마다 매번 자로 레일 중앙을 재거나 진자 각도를 눈대중으로 맞추기는 번거롭습니다. Step 06에서는 UART 터미널 안내에 따라 동작하는 두 가지 초기화 기능을 제공합니다.

### 6.1 카트 가이드 호밍 & 자동 중앙 복귀 (`h`)

1. **`h` 1회 입력 (`HOME_WAIT_RIGHT`)**: 모터를 정지하고 Soft Limit을 일시 해제한 뒤, 카트를 손으로 **오른쪽 끝**까지 밀도록 안내합니다.
2. **`h` 2회 입력 (`HOME_WAIT_LEFT`)**: 오른쪽 끝 엔코더 값을 기록하고, **왼쪽 끝**까지 밀도록 안내합니다.
3. **`h` 3회 입력 (`HOME_AUTO_CENTER`)**:
   - 양 끝단 차이 `span = |right - left|`로 **실측 전체 스트로크(`g_enc_total_counts`)와 방향 부호(`g_enc_sign`)를 자동 보정**합니다. (만약 오른쪽이 왼쪽보다 작았더라도 자동으로 부호를 뒤집어 정규화)
   - 현재 카트는 왼쪽 끝에 있으므로 좌표를 `-(span / 2)`로 재설정하고, 정지 마찰력을 극복하는 비례 듀티(`40% ~ 52%`)로 **중앙(`0.0 mm ± 3 mm`)까지 자동 이동**한 뒤 Soft Limit을 다시 켭니다.

### 6.2 진자 각도 초기화: 왜 6시(아래)와 12시(위)를 둘 다 저장할까? (`a` vs `d`)

- **왜 6시(자연 정지)와 12시(수직)를 함께 교정하나요? (`a` 명령어)**  
  `WDD35D4-5K` 같은 전도성 플라스틱 포텐쇼미터는 유효 회전각이 약 `345°`(~15° 데드존)이고 저항 소자 공차(±1~2%)가 있어, 이론값(`2116 counts`)만 하드코딩하면 실제 `180°`와 수십 카운트(1~2° 오차) 차이가 날 수 있습니다.  
  **6시 정지 상태(`adc_down`)와 12시 수직 상태(`adc_up`)를 둘 다 측정(`a` 마법사)** 하면:
  1. `adc_up`이 곧바로 정확한 `12시 = 0.00°` 영점이 되고,
  2. `|adc_up - adc_down|` 값으로 **내 보드·센서 조합의 실제 180° 스케일(`g_pot_180_counts`)** 까지 동시에 확정됩니다!
- **평상시 1점 간편 영점 복원 (`d` 명령어)**:  
  한 번 `a`로 실측 `180°` 간격을 학습하고 나면, 이후에는 진자를 손으로 세울 필요 없이 **아래(6시)에 자연스럽게 매달려 정지한 상태에서 `d`만 눌러도** 중력 기준 6시 값에 학습된 `180°` 카운트를 더해 `12시 = 0.00°` 기준점을 자동으로 복원해 줍니다. *(미세 조정이 필요하면 12시에서 `u`로 즉시 덮어쓸 수도 있습니다.)*

---

## 7. 전체 소스 코드 (`firmware/G474RE_Blink/Src/main.c`)

```c
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-10 / Step 06] TIM3 4체배 직교 엔코더 & ADC1 진자 각도계
 *                   + 가이드 위치 초기화(오른쪽/왼쪽 끝 -> 자동 중앙 복귀)
 *                   + 진자 2점 각도 캘리브레이션(6시/12시) 및 6시 간편 영점
 *                   - PA0 (A0) / PA5 : TIM2_CH1 20kHz PWM (DRV8876 EN / LD2 LED)
 *                   - PB0 (A3)       : GPIO Output 모터 방향 (PH: 1=오른쪽 f, 0=왼쪽 b)
 *                   - PC6 (CN10-4)   : TIM3_CH1 (AF2, 5V FT) 엔코더 A상
 *                   - PC7 (D9)       : TIM3_CH2 (AF2, 5V FT) 엔코더 B상
 *                   - PA1 (A1)       : ADC1_IN2 진자 포텐쇼미터(WDD35D4-5K) 각도
 *                   - USART2 (VCP)   : 115200 bps 텔레메트리 및 키보드 조종
 *                   - TIM6           : 1ms 타임베이스 (Soft Limit & 100Hz 샘플링)
 * @repository     : https://github.com/CH-Cho/InvertedPendulum
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

/* ============================================================================
 * 1. STM32G474RE 베어메탈 레지스터 정의 (RM0440 기준)
 * ========================================================================== */
// Cortex-M4 Core (FPU 접근 제어)
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

// TIM6 (1ms System Tick)
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
 * 2. 하드웨어 캘리브레이션 상수 및 전역 상태 변수
 * ========================================================================== */
#define PWM_ARR_MAX             799U    // 16MHz / (0+1) / (799+1) = 20,000 Hz (20kHz)

// IP570 카트 엔코더 기본 실측 교정치 (호밍 마법사 'h' 실행 시 자동 갱신됨)
#define RAIL_TRAVEL_MM          385
#define DEFAULT_TOTAL_COUNTS    6279
#define SOFT_LIMIT_MM           150     // 중앙 영점 기준 ±150.0 mm 소프트웨어 리미트
#define HOME_CENTER_BAND_CNT    50      // 자동 중앙 복귀 허용 오차 (±50 counts ≈ ±3.0 mm)
#define HOME_CENTER_TIMEOUT_MS  4000U   // 자동 중앙 복귀 안전 타임아웃 (4초)

// WDD35D4-5K 포텐쇼미터 기본 실측 교정치 (각도 마법사 'a' 실행 시 자동 갱신됨)
#define POT_DEFAULT_DOWN        1024
#define POT_DEFAULT_UPRIGHT     3140
#define POT_DEFAULT_180_COUNTS  2116
#define ANGLE_SIGN              1       // 오른쪽 기울임 시 부호 반전이 필요하면 -1로 변경

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

static volatile uint32_t g_system_millis      = 0;
static volatile uint8_t  g_tick_10ms          = 0;

// 모터 상태 및 안전 제어 변수
static volatile uint32_t g_motor_stop_at      = 0;
static volatile uint8_t  g_motor_running      = 0;
static volatile uint8_t  g_test_duty_pct      = 50; // IP570 기동 마찰력 극복 테스트용 기본 50%
static volatile uint8_t  g_motor_dir          = 1;  // 1: RIGHT(FWD), 0: LEFT(REV)
static volatile uint8_t  g_soft_limit_on      = 1;  // 1: ±150mm Soft Limit 활성화
static volatile uint8_t  g_soft_limit_trip    = 0;
static volatile uint8_t  g_rx_cmd             = 0;

// 카트 엔코더(TIM3) 확장 상태 및 동적 캘리브레이션 변수
static volatile int8_t   g_enc_sign           = 1;
static volatile int32_t  g_enc_total_counts   = DEFAULT_TOTAL_COUNTS;
static volatile int32_t  g_cart_counts        = 0;  // 중앙 0 기준 32-bit 확장 카운트
static volatile int32_t  g_cart_pos_mm_x10    = 0;  // 0.1 mm 단위 위치
static volatile int32_t  g_cart_vel_mms_x10   = 0;  // 0.1 mm/s 단위 LPF 필터링 속도
static volatile uint16_t s_prev_cnt_1ms       = 0;
static volatile int32_t  s_prev_counts_10ms   = 0;
static volatile int32_t  s_cart_vel_q4        = 0;

// 카트 호밍 마법사 상태 변수
static CartHomeState     g_home_state         = HOME_IDLE;
static int32_t           g_home_right_cnt     = 0;
static int32_t           g_home_left_cnt      = 0;
static uint32_t          g_home_center_start  = 0;

// 진자 각도계(ADC1) 상태 및 2점 캘리브레이션 변수
static AngleCalState     g_acal_state         = ACAL_IDLE;
static uint16_t          g_angle_down_adc     = POT_DEFAULT_DOWN;
static int32_t           g_angle_zero_adc     = POT_DEFAULT_UPRIGHT;
static int32_t           g_pot_180_counts     = POT_DEFAULT_180_COUNTS;
static int8_t            g_pot_up_dir_sign    = 1;  // +1: 12시가 6시보다 ADC 큼, -1: 작음
static uint16_t          g_angle_adc_raw      = POT_DEFAULT_UPRIGHT;
static int32_t           g_angle_cdeg         = 0;  // 0.01 deg (12시 = 0.00 deg)
static int32_t           g_angle_vel_x10      = 0;  // 0.1 deg/s LPF 각속도
static int32_t           g_prev_angle_cdeg    = 0;
static int32_t           g_angle_vel_q4       = 0;

/* ============================================================================
 * 3. 유틸리티 및 드라이버 초기화 함수
 * ========================================================================== */
static void fpu_enable(void)
{
    // CubeIDE 기본 fpv4-sp-d16 설정에서 부동소수점 명령 실행 시 HardFault 방지
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

static void uart2_init(void)
{
    RCC_AHB2ENR  |= (1U << 0);  // GPIOAEN
    RCC_APB1ENR1 |= (1U << 17); // USART2EN

    GPIOA_MODER &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA_MODER |=  ((2U << (2 * 2)) | (2U << (3 * 2)));
    GPIOA_AFRL  &= ~((0xFU << (2 * 4)) | (0xFU << (3 * 4)));
    GPIOA_AFRL  |=  ((0x7U << (2 * 4)) | (0x7U << (3 * 4)));

    USART2_BRR = 139U; // 115200 bps @ 16MHz
    USART2_CR1 = (1U << 0) | (1U << 2) | (1U << 3) | (1U << 5); // UE | RE | TE | RXNEIE

    NVIC_ISER1 = (1U << (38 - 32)); // IRQ 38 (USART2) 활성화
}

static void tim6_init(void)
{
    RCC_APB1ENR1 |= (1U << 4); // TIM6EN
    TIM6_PSC = 15U;            // 16MHz / 16 = 1MHz (1us 주기)
    TIM6_ARR = 999U;           // 1ms 주기 인터럽트 (1kHz)
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
    GPIOB_ODR   |=  (1U << 0); // 초기 방향 RIGHT (FWD)

    // TIM2 20kHz PWM 설정 (16MHz / 1 / 800 = 20,000 Hz)
    TIM2_PSC   = 0U;
    TIM2_ARR   = PWM_ARR_MAX;
    TIM2_CCR1  = 0U; // 부팅 시 완전 정지!
    TIM2_CCMR1 = (0x6U << 4) | (1U << 3); // PWM Mode 1 + Preload
    TIM2_CCER  = (1U << 0);               // CH1 출력 Enable
    TIM2_CR1   = (1U << 0);               // TIM2 시작
}

static void motor_drive(uint8_t dir, uint8_t duty_pct)
{
    g_motor_dir = dir ? 1U : 0U;
    if (g_motor_dir) {
        GPIOB_ODR |= (1U << 0);
    } else {
        GPIOB_ODR &= ~(1U << 0);
    }

    uint32_t ccr = ((PWM_ARR_MAX + 1U) * duty_pct) / 100U;
    if (ccr > PWM_ARR_MAX) ccr = PWM_ARR_MAX;
    TIM2_CCR1 = ccr;
}

static void motor_stop(void)
{
    TIM2_CCR1 = 0U;
    g_motor_running = 0U;
}

static uint8_t is_direction_blocked_by_limit(uint8_t dir)
{
    if (!g_soft_limit_on) return 0U;
    int32_t limit_x10 = SOFT_LIMIT_MM * 10;
    if (dir && (g_cart_pos_mm_x10 >= limit_x10))  return 1U;
    if (!dir && (g_cart_pos_mm_x10 <= -limit_x10)) return 1U;
    return 0U;
}

static void motor_kick(uint8_t dir, uint8_t duty_pct, uint32_t duration_ms)
{
    uint8_t target_dir = dir ? 1U : 0U;
    if (is_direction_blocked_by_limit(target_dir)) {
        motor_stop();
        g_soft_limit_trip = 1U;
        printf("\r\n>>> [SOFT LIMIT] Blocked %s! Pos=",
               target_dir ? "RIGHT" : "LEFT");
        print_fixed_1(g_cart_pos_mm_x10);
        printf(" mm (Limit +/-%d mm)\r\n", SOFT_LIMIT_MM);
        return;
    }

    motor_drive(target_dir, duty_pct);
    g_motor_stop_at = g_system_millis + duration_ms;
    g_motor_running = 1U;
}

static void encoder_tim3_init(void)
{
    RCC_AHB2ENR  |= (1U << 2); // GPIOCEN
    RCC_APB1ENR1 |= (1U << 1); // TIM3EN

    // PC6 (CN10-4, TIM3_CH1) & PC7 (D9, TIM3_CH2) -> AF2, Pull-Up/Down 없음 (5V FT 보호)
    GPIOC_MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOC_MODER |=  ((2U << (6 * 2)) | (2U << (7 * 2)));
    GPIOC_PUPDR &= ~((3U << (6 * 2)) | (3U << (7 * 2))); // 00: No Pull-Up/Down (5V FT 필수)
    GPIOC_AFRL  &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOC_AFRL  |=  ((0x2U << (6 * 4)) | (0x2U << (7 * 4)));

    TIM3_CR1   = 0U;
    TIM3_PSC   = 0U;
    TIM3_ARR   = 0xFFFFU; // 16-bit 전체 범위 자유 회전 카운트

    // TI1/TI2 입력 매핑(CC1S=01, CC2S=01) + 디지털 노이즈 필터(IC1F=6, IC2F=6)
    TIM3_CCMR1 = (1U << 0) | (6U << 4) | (1U << 8) | (6U << 12);

    // CC1P=1: TI1 극성 반전 -> 오른쪽(f, PH=1) 이동 시 카운트 양수(+) 증가 기준
    TIM3_CCER  = (1U << 0) | (1U << 1) | (1U << 4);

    // SMS=011 (Mode 3): 4체배 직교 인코더 모드
    TIM3_SMCR  = 3U;

    TIM3_CNT   = 0U;
    TIM3_CR1   = (1U << 0);
}

static void encoder_set_counts_atomic(int32_t new_counts, int32_t new_total_counts, int8_t new_sign)
{
    __asm volatile ("cpsid i");
    if (new_total_counts > 0) {
        g_enc_total_counts = new_total_counts;
    }
    g_enc_sign         = new_sign;
    g_cart_counts      = new_counts;
    s_prev_cnt_1ms     = (uint16_t)TIM3_CNT;
    s_prev_counts_10ms = new_counts;
    g_cart_pos_mm_x10  = (g_cart_counts * (RAIL_TRAVEL_MM * 10)) / g_enc_total_counts;
    s_cart_vel_q4      = 0;
    g_cart_vel_mms_x10 = 0;
    g_soft_limit_trip  = 0U;
    __asm volatile ("cpsie i");
}

static void encoder_zero(void)
{
    encoder_set_counts_atomic(0, g_enc_total_counts, g_enc_sign);
}

static void adc1_pa1_init(void)
{
    RCC_AHB2ENR |= (1U << 0) | (1U << 13); // GPIOAEN, ADC12EN

    GPIOA_MODER |=  (3U << (1 * 2));
    GPIOA_PUPDR &= ~(3U << (1 * 2));

    ADC12_CCR &= ~(3U << 16);
    ADC12_CCR |=  (1U << 16); // HCLK/1 동기 클록 (16MHz)

    ADC1_CR &= ~(1U << 29); // DEEPPWD = 0
    ADC1_CR |=  (1U << 28); // ADVREGEN = 1
    delay_ms(2);

    ADC1_CR &= ~(1U << 30); // ADCALDIF = 0
    ADC1_CR |=  (1U << 31); // ADCAL = 1
    while (ADC1_CR & (1U << 31));

    ADC1_ISR = (1U << 0);
    ADC1_CR |= (1U << 0); // ADEN = 1
    while (!(ADC1_ISR & (1U << 0)));

    ADC1_CFGR  = 0U;
    ADC1_SMPR1 &= ~(7U << 6);
    ADC1_SMPR1 |=  (4U << 6); // SMP2 = 100 (47.5 cycles)
    ADC1_SQR1  =  (2U << 6); // SQ1 = CH2 (PA1)
}

static uint16_t adc1_read_pa1_single(void)
{
    uint32_t timeout = 10000U;
    ADC1_CR |= (1U << 2);
    while (!(ADC1_ISR & (1U << 2)) && (--timeout > 0U));
    return (uint16_t)(ADC1_DR & 0x0FFFU);
}

static uint16_t adc1_read_pa1_avg(void)
{
    uint32_t sum = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        sum += adc1_read_pa1_single();
    }
    return (uint16_t)(sum >> 3);
}

static int32_t angle_adc_to_cdeg(uint16_t adc_raw)
{
    return ANGLE_SIGN * (((int32_t)adc_raw - g_angle_zero_adc) * 18000) / g_pot_180_counts;
}

static void angle_reseed_filter(uint16_t current_adc)
{
    g_angle_adc_raw   = current_adc;
    g_angle_cdeg      = angle_adc_to_cdeg(current_adc);
    g_prev_angle_cdeg = g_angle_cdeg;
    g_angle_vel_q4    = 0;
    g_angle_vel_x10   = 0;
}

/* ============================================================================
 * 4. 인터럽트 핸들러
 * ========================================================================== */
void TIM6_DACUNDER_IRQHandler(void)
{
    static uint8_t s_div_10ms = 0;

    if (TIM6_SR & (1U << 0)) {
        TIM6_SR &= ~(1U << 0);
        g_system_millis++;

        // (1) 1ms 단위 32-bit 확장 엔코더 위치 갱신 (부호 보정 + 오버플로우 안전)
        uint16_t curr_cnt = (uint16_t)TIM3_CNT;
        int16_t delta_1ms = (int16_t)(curr_cnt - s_prev_cnt_1ms);
        s_prev_cnt_1ms = curr_cnt;
        g_cart_counts += (int32_t)g_enc_sign * (int32_t)delta_1ms;
        g_cart_pos_mm_x10 = (g_cart_counts * (RAIL_TRAVEL_MM * 10)) / g_enc_total_counts;

        // (2) 1ms 고속 Soft Limit 레일 보호 & 250ms 펄스 킥 자동 정지
        if (g_motor_running) {
            if (is_direction_blocked_by_limit(g_motor_dir)) {
                motor_stop();
                g_soft_limit_trip = 1U;
            } else if ((int32_t)(g_system_millis - g_motor_stop_at) >= 0) {
                motor_stop();
            }
        }

        // (3) 10ms(100Hz) 속도 추정(EMA LPF alpha=0.25) 및 ADC 샘플링 트리거
        s_div_10ms++;
        if (s_div_10ms >= 10U) {
            s_div_10ms = 0U;

            int32_t delta_10ms = g_cart_counts - s_prev_counts_10ms;
            s_prev_counts_10ms = g_cart_counts;

            int32_t raw_vel_x10 = (delta_10ms * (RAIL_TRAVEL_MM * 1000)) / g_enc_total_counts;
            s_cart_vel_q4 = ((raw_vel_x10 << 4) + 3 * s_cart_vel_q4) >> 2;
            g_cart_vel_mms_x10 = (s_cart_vel_q4 + ((s_cart_vel_q4 >= 0) ? 8 : -8)) >> 4;

            g_tick_10ms = 1U;
        }
    }
}

void USART2_IRQHandler(void)
{
    if (USART2_ISR & (1U << 3)) {
        USART2_ICR = (1U << 3); // ORE 방어
    }
    if (USART2_ISR & (1U << 5)) { // RXNE
        g_rx_cmd = (uint8_t)USART2_RDR;
    }
}

/* ============================================================================
 * 5. 가이드 캘리브레이션 & 자동 중앙 복귀 핸들러
 * ========================================================================== */
static void handle_cart_homing_key(void)
{
    if (g_home_state == HOME_IDLE) {
        motor_stop();
        g_soft_limit_on = 0U; // 끝단 수동 이동 중 리미트 간섭 방지
        g_home_state = HOME_WAIT_RIGHT;
        printf("\r\n==================================================================\r\n");
        printf("  [CART HOMING 1/2] 카트를 손으로 [오른쪽 끝]까지 밀고\r\n");
        printf("                    터미널에서 'h' 를 한 번 더 누르세요. (취소: s)\r\n");
        printf("==================================================================\r\n");
    } else if (g_home_state == HOME_WAIT_RIGHT) {
        g_home_right_cnt = g_cart_counts;
        g_home_state = HOME_WAIT_LEFT;
        printf("\r\n>>> [1/2 완료] 오른쪽 끝 엔코더 기록: %ld\r\n", (long)g_home_right_cnt);
        printf("  [CART HOMING 2/2] 이제 카트를 손으로 [왼쪽 끝]까지 밀고\r\n");
        printf("                    터미널에서 'h' 를 다시 누르세요. (취소: s)\r\n");
    } else if (g_home_state == HOME_WAIT_LEFT) {
        g_home_left_cnt = g_cart_counts;
        int32_t diff = g_home_right_cnt - g_home_left_cnt;
        int8_t  new_sign = g_enc_sign;
        int32_t span = diff;

        if (diff < 0) {
            // 오른쪽이 왼쪽보다 작았다면 부호 반전으로 자동 정규화
            new_sign = (int8_t)(-g_enc_sign);
            span = -diff;
        }

        if (span < 1000) {
            g_home_state = HOME_IDLE;
            g_soft_limit_on = 1U;
            printf("\r\n>>> [HOMING 실패] 양 끝단 간격(%ld counts)이 너무 작습니다. 다시 시도하세요.\r\n",
                   (long)span);
            return;
        }

        // 현재 카트는 왼쪽 끝에 있으므로 중앙(0) 기준 위치 = -(span / 2)로 설정!
        int32_t left_origin_cnt = -(span / 2);
        encoder_set_counts_atomic(left_origin_cnt, span, new_sign);

        printf("\r\n>>> [2/2 완료] 실측 스트로크 = %ld counts (~%d mm), Sign = %+d\r\n",
               (long)span, RAIL_TRAVEL_MM, (int)new_sign);
        printf(">>> [AUTO CENTER] 중앙(Pos = 0.0 mm)으로 자동 이동합니다... (비상 정지: s)\r\n\r\n");

        g_home_center_start = g_system_millis;
        g_home_state = HOME_AUTO_CENTER;
    }
}

static void cart_auto_center_step(void)
{
    if (g_home_state != HOME_AUTO_CENTER) return;

    int32_t err = -g_cart_counts; // 목표 0 - 현재 위치
    int32_t abs_err = (err >= 0) ? err : -err;

    if (abs_err <= HOME_CENTER_BAND_CNT) {
        motor_stop();
        g_home_state = HOME_IDLE;
        g_soft_limit_on = 1U;
        printf("\r\n>>> [HOMING 완료] 중앙 위치 초기화 성공! Pos = ");
        print_fixed_1(g_cart_pos_mm_x10);
        printf(" mm (Soft Limit ON)\r\n\r\n");
        return;
    }

    if ((g_system_millis - g_home_center_start) >= HOME_CENTER_TIMEOUT_MS) {
        motor_stop();
        g_home_state = HOME_IDLE;
        g_soft_limit_on = 1U;
        printf("\r\n>>> [HOMING 타임아웃] %ums 경과로 안전 정지! Pos = ", HOME_CENTER_TIMEOUT_MS);
        print_fixed_1(g_cart_pos_mm_x10);
        printf(" mm\r\n\r\n");
        return;
    }

    // 비례 + 정지 마찰력(Stiction) 보상 듀티: 40% ~ 52%
    int32_t half_span = g_enc_total_counts / 2;
    if (half_span < 1) half_span = 1;
    uint8_t duty = (uint8_t)(40 + ((abs_err * 12) / half_span));
    if (duty > 52U) duty = 52U;

    uint8_t dir = (err > 0) ? 1U : 0U;
    motor_drive(dir, duty);
}

static void handle_angle_cal_key(void)
{
    if (g_acal_state == ACAL_IDLE) {
        g_acal_state = ACAL_WAIT_DOWN;
        printf("\r\n==================================================================\r\n");
        printf("  [ANGLE CAL 1/2] 진자가 아래(6시)에서 자연스럽게 정지하면\r\n");
        printf("                  터미널에서 'a' 를 한 번 더 누르세요. (취소: s)\r\n");
        printf("==================================================================\r\n");
    } else if (g_acal_state == ACAL_WAIT_DOWN) {
        g_angle_down_adc = adc1_read_pa1_avg();
        g_acal_state = ACAL_WAIT_UP;
        printf("\r\n>>> [1/2 완료] 6시 정지 ADC 저장: %u\r\n", (unsigned int)g_angle_down_adc);
        printf("  [ANGLE CAL 2/2] 이제 진자를 손으로 위(12시) 수직에 맞춘 뒤\r\n");
        printf("                  터미널에서 'a' 를 다시 누르세요. (취소: s)\r\n");
    } else if (g_acal_state == ACAL_WAIT_UP) {
        uint16_t up_adc = adc1_read_pa1_avg();
        int32_t  diff   = (int32_t)up_adc - (int32_t)g_angle_down_adc;
        int32_t  span   = (diff >= 0) ? diff : -diff;

        if (span < 1200 || span > 2800) {
            g_acal_state = ACAL_IDLE;
            printf("\r\n>>> [ANGLE CAL 실패] 6시(%u)와 12시(%u) 차이(%ld)가 비정상입니다.\r\n",
                   (unsigned int)g_angle_down_adc, (unsigned int)up_adc, (long)span);
            printf("                     데드존 통과 여부 확인 후 다시 시도하세요.\r\n\r\n");
            return;
        }

        g_pot_up_dir_sign = (diff >= 0) ? 1 : -1;
        g_pot_180_counts  = span;
        g_angle_zero_adc  = (int32_t)up_adc;
        g_acal_state      = ACAL_IDLE;
        angle_reseed_filter(up_adc);

        printf("\r\n>>> [ANGLE CAL 완료] 6시=%u | 12시=%ld -> 실측 180deg = %ld counts\r\n",
               (unsigned int)g_angle_down_adc, (long)g_angle_zero_adc, (long)g_pot_180_counts);
        printf("                     (이후 6시 정지 상태에서 'd'만 눌러도 자동 12시 영점 복원!)\r\n\r\n");
    }
}

/* ============================================================================
 * 6. 메인 루프 (100Hz 각도 추정 + 5Hz 텔레메트리 + 키보드 조종/초기화)
 * ========================================================================== */
int main(void)
{
    fpu_enable();
    tim6_init();
    uart2_init();
    motor_driver_init();
    encoder_tim3_init();
    adc1_pa1_init();

    angle_reseed_filter(adc1_read_pa1_avg());

    printf("\r\n==================================================================\r\n");
    printf("  STM32G474RE Inverted Pendulum Project                           \r\n");
    printf("  [Step 06 / CH-10] TIM3 4x Encoder + ADC1 Angle + Homing Wizard  \r\n");
    printf("  Sensors : PC6/PC7 (TIM3 5V FT) | PA1 (ADC1_IN2 WDD35D4-5K)      \r\n");
    printf("  Safety  : +/-%d mm Rail Soft Limit (Auto Block Outward Motion)  \r\n", SOFT_LIMIT_MM);
    printf("------------------------------------------------------------------\r\n");
    printf("  Drive   : f/b = Right/Left 250ms | s/Space = STOP | 1~6 = Duty  \r\n");
    printf("  Homing  : h = Cart Guided Homing (Right -> Left -> Auto Center) \r\n");
    printf("            a = Pendulum 2-Point Cal (6h Down -> 12h Up)          \r\n");
    printf("            d = Quick 6h Zero | u = Quick 12h Zero | z = Zero Pos \r\n");
    printf("            l = Toggle Soft Limit (Currently ON)                \r\n");
    printf("==================================================================\r\n\r\n");

    uint32_t last_print = 0U;

    while (1)
    {
        // (A) Soft Limit 긴급 차단 이벤트 즉시 알림
        if (g_soft_limit_trip) {
            g_soft_limit_trip = 0U;
            printf("\r\n>>> [SOFT LIMIT TRIGGERED] Motor cut at ");
            print_fixed_1(g_cart_pos_mm_x10);
            printf(" mm! Move opposite dir or press 'z'.\r\n");
        }

        // (B) 10ms (100Hz) 진자 각도·각속도 갱신 및 카트 자동 중앙 복귀 제어
        if (g_tick_10ms) {
            g_tick_10ms = 0U;

            g_angle_adc_raw = adc1_read_pa1_avg();
            g_angle_cdeg = angle_adc_to_cdeg(g_angle_adc_raw);

            int32_t raw_dtheta_x10 = (g_angle_cdeg - g_prev_angle_cdeg) * 10;
            g_prev_angle_cdeg = g_angle_cdeg;

            g_angle_vel_q4 = ((raw_dtheta_x10 << 4) + 3 * g_angle_vel_q4) >> 2;
            g_angle_vel_x10 = (g_angle_vel_q4 + ((g_angle_vel_q4 >= 0) ? 8 : -8)) >> 4;

            cart_auto_center_step();
        }

        // (C) USART2 키보드 명령 처리
        if (g_rx_cmd != 0U) {
            uint8_t cmd = g_rx_cmd;
            g_rx_cmd = 0U;

            if (cmd == 's' || cmd == 'S' || cmd == ' ') {
                motor_stop();
                if (g_home_state != HOME_IDLE || g_acal_state != ACAL_IDLE) {
                    g_home_state = HOME_IDLE;
                    g_acal_state = ACAL_IDLE;
                    g_soft_limit_on = 1U;
                    printf("\r\n>>> [ABORT] 비상 정지 및 캘리브레이션 마법사 취소 (Soft Limit ON)\r\n\r\n");
                } else {
                    printf("\r\n>>> [STOP] Emergency Stop! (CCR1=0)\r\n");
                }
            } else if (cmd == 'h' || cmd == 'H') {
                handle_cart_homing_key();
            } else if (cmd == 'a' || cmd == 'A') {
                handle_angle_cal_key();
            } else if (g_home_state != HOME_IDLE || g_acal_state != ACAL_IDLE) {
                // 마법사 진행 중 오조작 방지 (ENTER 키도 현재 단계 진행으로 허용)
                if (cmd == '\r' || cmd == '\n') {
                    if (g_home_state == HOME_WAIT_RIGHT || g_home_state == HOME_WAIT_LEFT) {
                        handle_cart_homing_key();
                    } else if (g_acal_state == ACAL_WAIT_DOWN || g_acal_state == ACAL_WAIT_UP) {
                        handle_angle_cal_key();
                    }
                }
            } else if (cmd == 'f' || cmd == 'F') {
                printf("\r\n>>> [KICK RIGHT] Dir=1 (PH=1), Duty=%u%% (250ms)\r\n", g_test_duty_pct);
                motor_kick(1U, g_test_duty_pct, 250U);
            } else if (cmd == 'b' || cmd == 'B') {
                printf("\r\n>>> [KICK LEFT]  Dir=0 (PH=0), Duty=%u%% (250ms)\r\n", g_test_duty_pct);
                motor_kick(0U, g_test_duty_pct, 250U);
            } else if (cmd >= '1' && cmd <= '6') {
                const uint8_t duties[6] = {15, 25, 35, 50, 65, 80};
                g_test_duty_pct = duties[cmd - '1'];
                printf("\r\n>>> [DUTY SET] Test Duty = %u%% (CCR1=%u/800)\r\n",
                       g_test_duty_pct,
                       (unsigned int)(((PWM_ARR_MAX + 1U) * g_test_duty_pct) / 100U));
            } else if (cmd == 'z' || cmd == 'Z') {
                encoder_zero();
                printf("\r\n>>> [ZERO CART] TIM3 Encoder Reset -> Pos = +0.0 mm\r\n");
            } else if (cmd == 'd' || cmd == 'D') {
                g_angle_down_adc = adc1_read_pa1_avg();
                g_angle_zero_adc = (int32_t)g_angle_down_adc + (g_pot_up_dir_sign * g_pot_180_counts);
                angle_reseed_filter(g_angle_down_adc);
                printf("\r\n>>> [6h ZERO] 6시 정지 ADC=%u 저장 -> 12시 영점(%ld) 자동 산출 (현재 Angle=",
                       (unsigned int)g_angle_down_adc, (long)g_angle_zero_adc);
                print_fixed_2(g_angle_cdeg);
                printf(" deg)\r\n");
            } else if (cmd == 'u' || cmd == 'U') {
                uint16_t up_adc = adc1_read_pa1_avg();
                g_angle_zero_adc = (int32_t)up_adc;
                angle_reseed_filter(up_adc);
                printf("\r\n>>> [12h ZERO] Upright Reference Calibrated: ADC = %ld -> +0.00 deg\r\n",
                       (long)g_angle_zero_adc);
            } else if (cmd == 'l' || cmd == 'L') {
                g_soft_limit_on = g_soft_limit_on ? 0U : 1U;
                printf("\r\n>>> [SOFT LIMIT] Protection %s (+/-%d mm)\r\n",
                       g_soft_limit_on ? "ENABLED" : "DISABLED (Caution!)",
                       SOFT_LIMIT_MM);
            }
        }

        // (D) 200ms (5Hz) 텔레메트리 출력 (안내 프롬프트 대기 중에는 화면 가독성을 위해 일시 정지)
        if ((g_home_state == HOME_IDLE || g_home_state == HOME_AUTO_CENTER) &&
            (g_acal_state == ACAL_IDLE) &&
            ((g_system_millis - last_print) >= 200U)) {
            last_print = g_system_millis;

            printf("Pos: ");
            print_fixed_1(g_cart_pos_mm_x10);
            printf(" mm (CNT:%+6ld) | Vel: ", (long)g_cart_counts);
            print_fixed_1(g_cart_vel_mms_x10);
            printf(" mm/s | Angle: ");
            print_fixed_2(g_angle_cdeg);
            printf(" deg (ADC:%4u) | dTh: ", (unsigned int)g_angle_adc_raw);
            print_fixed_1(g_angle_vel_x10);
            printf(" deg/s | Lim:%s\r\n", g_soft_limit_on ? "ON " : "OFF");
        }
    }
}
```

---

## 8. 실물 하드웨어 검증 체크리스트

보드를 플래싱하고 ST-LINK VCP 시리얼 모니터(`115200 bps`)를 연 뒤 아래 순서대로 동작을 검증합니다.

1. **카트 가이드 위치 초기화 (`h`)**:
   - 터미널에서 **`h`** 를 누른 뒤 안내에 따라 **오른쪽 끝(`h`) → 왼쪽 끝(`h`)** 을 입력합니다.
   - 양 끝단 실측 카운트(`~6,279 counts`)와 방향 부호(`Sign`)가 자동 산출된 뒤 카트가 중앙(`Pos: +0.0 mm` 부근)으로 자동 이동해 정지하는지 확인합니다.
2. **진자 2점 각도 캘리브레이션 (`a`) & 6시 간편 영점 (`d`)**:
   - **`a`** 를 누르고 **아래(6시) 자연 정지 시 `a` → 위(12시) 수직 시 `a`** 를 입력해 실측 `180°` 카운트가 자동 등록되는지 확인합니다.
   - 이후 진자를 6시에 자연스럽게 내려놓고 **`d`** 를 눌렀을 때 다시 12시에 세우면 `Angle: ~0.00 deg`가 나오는지 확인합니다.
3. **모터 방향 vs 엔코더 부호 일치 & 기울임 부호 검증**:
   - **`f`** 키(오른쪽 킥) 시 `Pos`·`CNT`가 양수(`+`)로 증가하고 **`b`** 키 시 감소하는지 확인합니다.
   - 진자를 **오른쪽(카트 `+` 방향)** 으로 살짝 기울였을 때 `Angle` 부호(`+`/`-`)를 확인합니다. (Step 07 제어기 적용 전 `ANGLE_SIGN` 정렬 확인용)
4. **Soft Limit (`±150 mm`) 안전 차단 검증**:
   - `f` 키를 반복해 카트가 `+150.0 mm`에 도달할 때 모터를 즉시 정지시키고 `[SOFT LIMIT TRIGGERED]` 경고 메시지가 뜨며 추가 `f` 명령이 차단되고, `b`로만 정상 복귀되는지 확인합니다.

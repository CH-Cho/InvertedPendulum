# [Deep Dive] STM32G4 Low-Power Stop 0 Mode & EXTI 상호작용

STM32G474RE 데이터시트(DS12288)에 명시된 **Stop 0 모드의 하드웨어 아키텍처**와 **인터럽트 펜딩 플래그(EXTI_PR1) 클리어 규칙**에 대한 심층 분석 기록입니다.

---

## 1. 데이터시트 원문 분석: Stop 0 Mode

> **Stop 0 mode**  
> *The Stop 0 mode is based on the Cortex®-M4 with FPU deepsleep mode combined with the peripheral clock gating. The voltage regulator is configured in main regulator mode. In Stop 0 mode, all clocks in the VCORE domain are stopped; the PLL, the HSI16 and the HSE oscillators are disabled. Some peripherals with the wakeup capability (I2Cx (x=1,2,3,4), U(S)ARTx(x=1,2...5) and LPUART) can switch on the HSI16 to receive a frame, and switch off the HSI16 after receiving the frame if it is not a wakeup frame. In this case, the HSI16 clock is propagated only to the peripheral requesting it.  
> SRAM1, SRAM2, CCM SRAM and register contents are preserved.  
> The BOR is always available in Stop 0 mode. The consumption is increased when thresholds higher than VBOR0 are used.*

### 핵심 메커니즘 해설

1. **Cortex-M4 Deep-sleep + Peripheral Clock Gating**:
   - 단순히 CPU만 정지하는 Sleep 모드(`WFI`)와 달리, ARM 코어의 `SLEEPDEEP` 비트를 켜서 모든 주변장치로 공급되는 클럭 트리 자체를 하드웨어적으로 차단(Clock Gating)합니다.
2. **Main Regulator (MR) 유지**:
   - Stop 1 모드는 전압 레귤레이터까지 저전력(LPR)으로 낮추지만, **Stop 0 모드는 메인 레귤레이터(1.2V 코어 전압)를 정상 유지**합니다.
   - 따라서 외부 인터럽트가 들어왔을 때 레귤레이터 안정화 대기 시간 없이 **약 $3\sim5\,\mu\text{s}$ 만에 초고속 복귀(Instant Wakeup)**가 가능합니다.
3. **자율 주변장치 (Autonomous Peripherals) 동작 ⭐**:
   - CPU와 메인 클럭이 완전히 잠들어 있는 상태에서 UART/I2C 통신 신호가 들어오면, **주변장치가 스스로 내부 16MHz HSI 오실레이터를 잠깐 켭니다.**
   - 프레임을 수신한 뒤 미리 약속된 "Wakeup 조건"에 부합하지 않는 일반 노이즈나 무관한 데이터면 **HSI 클럭을 다시 끄고 조용히 수면을 유지**합니다.
   - 올바른 깨우기 명령일 때만 비로소 CPU를 깨워 불필요한 전력 낭비를 차단합니다.
4. **SRAM 및 레지스터 100% 보존**:
   - Standby/Shutdown 모드와 달리 SRAM1, SRAM2, CCM SRAM 및 모든 주변장치 설정 레지스터가 그대로 유지됩니다.
   - 복귀 시 리셋 재부팅 없이 **잠들었던 C 코드 라인 다음 줄부터 즉시 실행**됩니다.
5. **BOR (Brown-Out Reset) 상시 감시**:
   - 전원 전압($V_{DD}$)이 기준치 이하로 떨어지면 칩이 비정상 동작을 하기 전에 하드웨어가 스스로 안전 리셋을 수행합니다.

---

## 2. 저전력 모드별 특성 비교

| 모드 | 전력 소모 | 복귀 시간 (Wakeup) | RAM/레지스터 | 특징 |
| :---: | :---: | :---: | :---: | :--- |
| **Run** | 수십 $\text{mA}$ | - | 유지 | 170MHz 풀가동 (도립진자 제어 루프 연산) |
| **Sleep** | 수 $\text{mA}$ | 즉시 ($< 1\,\mu\text{s}$) | 유지 | CPU만 정지, 타이머/PWM/ADC 등 주변장치는 계속 동작 |
| **Stop 0** | **수백 $\mu\text{A}$** | **매우 빠름 ($3\sim5\,\mu\text{s}$)** | **100% 유지** | **클럭 차단 + 전압 레귤레이터 유지 + 초고속 복귀** |
| **Stop 1** | 수십 $\mu\text{A}$ | 약 $10\,\mu\text{s}$ | 100% 유지 | 레귤레이터까지 저전력(LPR)으로 전환하여 전력 극소화 |
| **Standby** | 수 $\mu\text{A}$ | 리셋 재부팅 | 일부 손실 | 전원 재인가 수준의 극단적 절전 |

---

## 3. 데이터시트 핵심 주의사항(Note) 심층 분석

> **Note**: *To enter Stop 0 mode, all EXTI Line pending bits (in Pending register 1 (EXTI_PR1)), and the peripheral flags generating wakeup interrupts must be cleared. Otherwise, the Stop 0 mode entry procedure is ignored and program execution continues.*

### 왜 미처리 플래그가 있으면 Stop 0 진입이 무시(Ignored)되는가?

ARM Cortex-M 코어는 `WFI` (Wait For Interrupt) 명령어를 만났을 때 하드웨어 수준에서 안전 점검을 수행합니다:

1. **데드락 방지 안전장치**:
   - 만약 이미 발생한 인터럽트 펜딩 플래그(`EXTI_PR1` = 1)가 지워지지 않고 남아있는데 CPU가 잠들어버리면, **이 인터럽트를 처리하지 못한 채 칩이 영원히 깨어나지 못하는 치명적 락업(Lock-up)**에 걸릴 수 있습니다.
2. **하드웨어의 자동 회피**:
   - 하드웨어는 "처리해야 할 인터럽트가 아직 남아있으니 잠들지 마라!" 하고 판단하여 **Stop 0 진입 시퀀스를 통째로 무시(Ignored)하고 즉시 다음 C 코드 라인으로 넘어가 버립니다.**
   - 결과적으로 코드는 절전 모드에 들어가지 못하고 100% 전력을 소모하며 무한 실행되는 버그가 발생합니다.

---

## 4. 실전 대응 코드 패턴 (C Implementation)

저전력 모드에 안전하게 진입하기 위해서는 **잠들기 직전에 잔여 펜딩 플래그를 강제로 청소(Flush)**하는 루틴을 작성해야 합니다:

```c
#include "stm32g4xx.h"

void Enter_Safe_Stop0_Mode(void)
{
    // 1. 모든 EXTI 라인의 잔여 펜딩 플래그를 강제 클리어 (1을 쓰면 클리어됨: RC_W1)
    EXTI->PR1 = 0xFFFFFFFFU;

    // 2. Wakeup에 사용되는 주변장치의 플래그 클리어 (예: UART Wakeup 플래그)
    // USART1->ICR = USART_ICR_WUCF;

    // 3. 전력 제어 레지스터에서 Stop 0 모드 선택 (LPMS = 00)
    PWR->CR1 &= ~PWR_CR1_LPMS;

    // 4. Cortex-M4 코어의 Deep-sleep 모드 활성화
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // 5. 대기 진입 (모든 방해요소가 제거되었으므로 100% 확실하게 Stop 0 진입)
    __asm__("wfi");

    // ========================================================
    // 6. [Wakeup 이후] 외부 인터럽트 발생 시 즉시 여기서부터 실행 재개!
    // ========================================================
    // 깨어난 후 시스템 메인 클럭(PLL 170MHz)을 재구성하는 코드를 호출합니다.
    SystemClock_Config();
}
```

---

## 5. 도립진자(Inverted Pendulum) 시스템에서의 응용

1. **대기(Standby) 및 안전 모드**:
   - 진자가 쓰러졌거나 사용자가 모터를 껐을 때 고속 Run 모드를 유지할 필요가 없습니다.
   - Stop 0 모드로 진입시켜 배터리를 보존하다가, **유저 버튼(EXTI)을 누르거나 PC에서 UART 명령이 들어오는 순간 $3\sim5\,\mu\text{s}$ 만에 즉각 기상**하여 밸런싱을 시작합니다.
2. **신뢰성 보장**:
   - 펜딩 플래그 클리어 원칙을 준수함으로써, 버튼 누름 신호가 씹히거나 절전 모드 진입이 누락되는 현상을 원천 차단합니다.

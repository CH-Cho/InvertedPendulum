# Inverted Pendulum with STM32G474RE 🚀

NUCLEO-G474RE 보드를 기반으로 하드웨어 저수준 제어부터 도립진자(Inverted Pendulum) 실시간 밸런싱 제어 시스템까지 단계별로 학습하고 구현하는 프로젝트입니다.

---

## 🎯 Target Hardware Specifications
- **Board**: STMicroelectronics NUCLEO-G474RE (MB1367)
- **MCU**: STM32G474RET6 (Arm® 32-bit Cortex®-M4 with FPU)
  - **Clock Speed**: up to 170 MHz
  - **Memory**: 512 KB Flash, 128 KB SRAM
  - **Accelerators**: CORDIC (수학 가속기), FMAC (디지털 필터 가속기)
  - **Timers**: HRTIM (High-Resolution Timer), TIM1/TIM8 (모터 제어 고급 타이머)
- **On-board Components**:
  - Green LED (LD2): `PA5`
  - Blue Push Button (B1): `PC13`
  - ST-LINK/V3E Debugger (Virtual COM Port on `USART2`: `PA2` TX, `PA3` RX)

---

## 📚 Curriculum & Learning Roadmap

모든 학습 단계는 원리 분석, 하드웨어 레지스터/드라이버 구현, 실습 검증 가이드가 `docs/` 디렉토리에 마크다운으로 상세히 기록됩니다.

| 단계 | 토픽 | 핵심 내용 | 문서 링크 | Linear Issue | 상태 |
| :---: | :--- | :--- | :--- | :---: | :---: |
| **00** | **Environment Setup** | STM32CubeIDE 2.0 설치 및 ST-LINK 연결/트러블슈팅 | [docs/00_environment_setup.md](docs/00_environment_setup.md) | [CH-5](https://linear.app/ch-cho/issue/CH-5) | ✅ 완료 |
| **01** | **Bare-metal LED Blink** | RCC 클럭 제어, GPIO MODER/ODR 레지스터 직접 제어 | [docs/01_baremetal_blink.md](docs/01_baremetal_blink.md) | [CH-5](https://linear.app/ch-cho/issue/CH-5) | ✅ 완료 |
| **02** | **GPIO EXTI** | NVIC 및 EXTI 컨트롤러를 활용한 하드웨어 외부 인터럽트 (버튼 입력) | [docs/02_gpio_exti.md](docs/02_gpio_exti.md) | [CH-6](https://linear.app/ch-cho/issue/CH-6) | ✅ 완료 |
| **03** | **UART Printf** | USART2 VCP 시리얼 통신 및 `printf` 리디렉션을 통한 모니터링 | [docs/03_uart_printf.md](docs/03_uart_printf.md) | [CH-7](https://linear.app/ch-cho/issue/CH-7) | ✅ 완료 |
| **04** | **Timer TimeBase** | 1ms(1kHz) 고정 주기 타이머 인터럽트를 통한 실시간 제어 루프 | [docs/04_timer_timebase.md](docs/04_timer_timebase.md) | [CH-8](https://linear.app/ch-cho/issue/CH-8) | ✅ 완료 |
| **05** | **Timer PWM** | DC/BLDC 모터 제어용 하드웨어 PWM 파형 생성 | [docs/05_timer_pwm.md](docs/05_timer_pwm.md) | [CH-9](https://linear.app/ch-cho/issue/CH-9) | 🔄 진행중 |
| **06** | **Timer Encoder** | 진자 각도 및 카트 위치 측정을 위한 직교 엔코더(Quadrature) 디코딩 | [docs/06_timer_encoder.md](docs/06_timer_encoder.md) | [CH-10](https://linear.app/ch-cho/issue/CH-10) | ⏳ 예정 |
| **07** | **Pendulum Control** | 도립진자 비선형 모델링, 선형화, PID 및 LQR 상태 피드백 제어 | [docs/07_pendulum_control.md](docs/07_pendulum_control.md) | [CH-11](https://linear.app/ch-cho/issue/CH-11) | ⏳ 예정 |

---

## 🔬 Deep Dive & Technical Notes
데이터시트 및 하드웨어 아키텍처 심층 분석 문서 목록입니다.
- **[Stop 0 Mode & EXTI 상호작용 심층 분석](docs/deep_dive_stop0_low_power.md)**: Cortex-M4 Deep-sleep, Main Regulator, 자율 주변장치(Autonomous Peripheral) 동작 원리 및 펜딩 플래그(`EXTI_PR1`) 클리어 규칙

---

## 🛠 Project Management
이 프로젝트의 이슈 추적 및 마일스톤 관리는 **[Linear](https://linear.app)**를 통해 체계적으로 관리됩니다.
- **Linear Project**: [Inverted Pendulum (도립진자)](https://linear.app/ch-cho/project/inverted-pendulum-도립진자-6b3441480def)
- **Team**: `CH Cho (CH)`

---

## 📂 Repository Structure
```text
InvertedPendulum/
├── README.md                   # 전체 프로젝트 로드맵 및 개요
├── .gitignore                  # 빌드 산출물 및 임베디드 아티팩트 제외
├── docs/                       # 단계별 튜토리얼 및 이론/트러블슈팅 기록
│   ├── 00_environment_setup.md
│   ├── 01_baremetal_blink.md
│   └── 02_gpio_exti.md
└── firmware/                   # STM32 펌웨어 소스 코드
    └── G474RE_Blink/           # STM32CubeIDE 연동 프로젝트
```

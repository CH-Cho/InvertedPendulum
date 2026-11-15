# 03. USART2 VCP 시리얼 통신 및 printf 리디렉션

## 1. 개요
NUCLEO 보드의 내장 ST-LINK/V3E 가상 COM 포트(Virtual COM Port, VCP)를 통해 STM32와 PC 사이에 **115200 bps 시리얼 통신**을 연결하고, 표준 C 라이브러리의 **`printf()` 함수를 리디렉션**하여 센서 값과 상태 로그를 PC 화면으로 실시간 출력합니다.

---

## 2. 하드웨어 스펙 및 핀 매핑 (UM2505 & DS12288)
NUCLEO-G474RE 보드는 온보드 ST-LINK와 MCU의 USART2가 하드웨어적으로 직결되어 있습니다:
- **TX (송신, MCU $\rightarrow$ PC)**: `PA2` (Alternate Function 7, `AF7`)
- **RX (수신, PC $\rightarrow$ MCU)**: `PA3` (Alternate Function 7, `AF7`)
- **보레이트 (Baud Rate)**: `115200 bps` (8-N-1: 데이터 8비트, 패리티 없음, 정지 비트 1)

---

## 3. 핵심 레지스터 제어 단계

### Step 1. 클럭 활성화 (RCC)
- **GPIOA 클럭**: `RCC_AHB2ENR` 비트 0 (`GPIOAEN`)
- **USART2 클럭**: `RCC_APB1ENR1` 비트 17 (`USART2EN`)

### Step 2. GPIO 포트 대체 기능 (Alternate Function) 설정
- `MODER`: PA2와 PA3를 **대체 기능 모드(`10`)**로 설정
- `AFRL (Alternate Function Low Register)`:
  - 핀 1개당 4비트 할당
  - PA2: 비트 `[11:8]`에 `0x7` (AF7)
  - PA3: 비트 `[15:12]`에 `0x7` (AF7)

### Step 3. 보레이트 계산 및 설정 (`USART2_BRR`)
기본 부팅 클럭(HSI16 = 16 MHz)을 기준으로 115200 bps를 만들기 위한 분주비:
$$\text{BRR} = \frac{f_{\text{CK}}}{\text{BaudRate}} = \frac{16\,000\,000}{115\,200} \approx 138.88 \approx 139 \;(\text{0x8B})$$
실제 보레이트: $\frac{16\,000\,000}{139} \approx 115108 \text{ bps}$ (오차율 $0.08\%$)
- `USART2_BRR = 139;`

### Step 4. 송신/수신 및 모듈 활성화 (`USART2_CR1`)
- 비트 0 (`UE`): USART 모듈 전체 활성화
- 비트 3 (`TE`): 송신기(Transmitter) 활성화
- 비트 2 (`RE`): 수신기(Receiver) 활성화

### Step 5. `printf()` 리디렉션 (`_write` 시스템 콜)
표준 C `printf()`는 내부적으로 저수준 시스템 콜인 `_write()`를 호출합니다. 이 함수를 오버라이딩하여 들어오는 문자열 바이트를 `USART2_TDR` 레지스터로 1바이트씩 전송합니다.

---

## 4. Mac 터미널에서 시리얼 모니터링 방법

Mac 터미널에서 내장 `screen` 명령어를 사용하여 실시간 로그를 확인합니다:

```bash
# 시리얼 포트 확인
ls /dev/cu.usbmodem*

# 터미널 연결 (115200 bps)
screen /dev/cu.usbmodem103 115200
```

> **Screen 종료 단축키**: `Ctrl + A`를 누른 뒤 바로 `K`를 누르고, `y`를 누르면 세션이 종료됩니다.

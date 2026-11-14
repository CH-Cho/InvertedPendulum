# 00. 개발 환경 설정 및 트러블슈팅 가이드

## 1. 개발 환경 구성
- **OS**: macOS (Apple Silicon / Intel)
- **IDE**: STM32CubeIDE 2.0.0
- **Target Board**: NUCLEO-G474RE
- **On-board Debugger**: ST-LINK/V3E (Firmware V3J16M9)

---

## 2. 하드웨어 연결 확인
NUCLEO 보드를 Mac의 USB 포트에 연결하면 다음 장치들이 인식됩니다:
- **Mass Storage (USB 드라이브)**: `/Volumes/NOD_G474RE`
- **Virtual COM Port (VCP)**: `/dev/cu.usbmodem103` (또는 유사 포트 번호)

터미널 확인 명령어:
```bash
ls -la /Volumes/NOD_G474RE
ls -la /dev/cu.usbmodem*
```

---

## 3. 핵심 트러블슈팅: ST-LINK GDB Server 통신 오류

### 증상 (Error Log)
```text
libusb: warning [darwin_transfer_status] transfer error: device not responding
libusb: error [submit_bulk_transfer] bulk transfer failed: pipe is stalled
Target is not responding, retrying...
```

### 원인
ST-LINK 디버거의 기본 리셋 모드가 `Software system reset`으로 되어 있을 때, 타겟 MCU가 초기 상태이거나 저전력 모드/코드 락업 상태일 경우 소프트웨어 리셋 패킷에 반응하지 못해 USB 파이프가 중단됩니다.

### 해결책 1: Reset Mode 변경 (IDE 표준 해결법)
1. 상단 메뉴 **Run > Run Configurations...** 접속
2. 좌측 **STM32 C/C++ Application > [프로젝트명] Debug** 선택
3. 우측 **Debugger** 탭 선택
4. 아래쪽 **Reset Behaviour** (또는 Reset Mode)를 **`Connect under reset`** (또는 `Hardware reset`)으로 변경
5. **Apply** 후 **Run** 클릭

> **Tip**: 만약 연결이 여전히 실패한다면, 보드의 검은색 리셋 버튼(B2/RESET)을 누른 상태에서 Run을 클릭하고, 콘솔에 `Debugger connected`가 뜨는 순간 손을 떼면 하드웨어 강제 리셋으로 정상 진입합니다.

### 해결책 2: NUCLEO Mass Storage 드래그 앤 드롭 플래싱 (가장 안정적)
NUCLEO 보드는 온보드 ST-LINK가 USB 가상 드라이브(`/Volumes/NOD_G474RE`)를 제공합니다.  
빌드된 `.bin` 파일을 이 드라이브에 복사하기만 하면, GDB 서버 연결 오류와 상관없이 ST-LINK가 하드웨어 레벨에서 플래시 메모리에 기록하고 리셋합니다.

```bash
# elf 파일에서 bin 파일 추출
arm-none-eabi-objcopy -O binary Debug/firmware.elf Debug/firmware.bin

# NUCLEO 볼륨으로 복사 (자동 플래싱)
cp Debug/firmware.bin /Volumes/NOD_G474RE/
```
복사가 완료되면 보드 상단 LD1(빨강/초록) LED가 깜빡인 후 보드가 자동으로 재부팅되어 코드가 실행됩니다.

---

## 4. 공식 기술 문서 및 레퍼런스 가이드 (Documentation)

STM32G474RE 펌웨어 및 제어기 개발에 반드시 필요한 ST 공식 기술 문서입니다.  
ST 웹사이트의 보안 방화벽 정책으로 인해 아래 링크를 클릭하여 브라우저에서 직접 다운로드하시는 것을 권장합니다.

| 문서 번호 | 문서명 | 크기 | 핵심 역할 및 참조 내용 | 공식 다운로드 링크 |
| :---: | :--- | :---: | :--- | :---: |
| **RM0440** | **Reference Manual** | ~18 MB | **[필수] 전체 레지스터 맵 및 주변장치 하드웨어 제어 원리 총망라**<br>• 메모리 맵(Memory Map), RCC 클럭 트리, GPIO 레지스터 구조<br>• EXTI 인터럽트 라우팅, 범용 타이머(TIM), 고해상도 타이머(HRTIM)<br>• USART 시리얼 통신, 12-bit ADC, CORDIC/FMAC 가속기 등 | [📥 RM0440 PDF 다운로드](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) |
| **UM2505** | **User Manual** | ~2.5 MB | **[필수] NUCLEO-G474RE (MB1367) 보드 하드웨어 구성 가이드**<br>• 보드 온보드 LED(LD2: `PA5`), 유저 버튼(B1: `PC13`) 연결 회로<br>• ST-LINK/V3E 가상 COM 포트(USART2: `PA2`/`PA3`) 점퍼 연결<br>• 전원 공급 모드(5V_STLK, USB_PWR) 및 CN4 SWD 디버그 점퍼 설정 | [📥 UM2505 PDF 다운로드](https://www.st.com/resource/en/user_manual/um2505-stm32g4-nucleo64-boards-mb1367-stmicroelectronics.pdf) |
| **DS12288** | **Datasheet** | ~5 MB | **STM32G474xB/xC/xE MCU 전기적 사양서**<br>• LQFP64 패키지 핀아웃(Pinout) 다이어그램<br>• 핀별 대체 기능(Alternate Function, AF0~AF15) 매핑 테이블<br>• 전기적 특성(Operating Voltage 1.71~3.6V, 최대 클럭 170MHz, 타이머 주파수) | [📥 DS12288 PDF 다운로드](https://www.st.com/resource/en/datasheet/stm32g474re.pdf) |
| **PM0214** | **Programming Manual** | ~2 MB | **STM32 Cortex®-M4 프로그래밍 가이드**<br>• Armv7-M 아키텍처 및 Thumb-2 명령어 셋<br>• NVIC(Nested Vectored Interrupt Controller) 인터럽트 우선순위 및 제어<br>• SysTick 시스템 타이머, FPU(Floating Point Unit) 레지스터 구조 | [📥 PM0214 PDF 다운로드](https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf) |

### 💡 로컬 보관 안내
다운로드하신 PDF 파일들은 프로젝트 내 [`docs/manuals/`](docs/manuals/) 폴더에 보관하시면 학습 및 개발 중에 편리하게 열어보실 수 있습니다.

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

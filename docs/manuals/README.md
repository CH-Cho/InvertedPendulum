# STM32G474RE & NUCLEO-64 공식 기술 문서 모음

ST 공식 웹사이트의 보안 방화벽 정책으로 인해 웹 브라우저에서 직접 다운로드하시는 것을 권장합니다. 아래 링크를 클릭하시면 ST 공식 PDF를 바로 받으실 수 있습니다.

---

## 📚 핵심 4대 공식 문서 다운로드

| 문서 번호 | 문서명 | 파일 크기 | 설명 및 핵심 용도 | 공식 다운로드 링크 |
| :---: | :--- | :---: | :--- | :---: |
| **RM0440** | **Reference Manual** | ~18 MB | **모든 레지스터 맵 및 하드웨어 동작 원리 총망라**<br>• RCC, GPIO, EXTI, NVIC, TIM, UART, ADC, HRTIM 등 | [📥 RM0440 다운로드](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) |
| **UM2505** | **User Manual** | ~2.5 MB | **NUCLEO-G474RE 보드 하드웨어 가이드**<br>• 온보드 LED(PA5), 버튼(PC13), ST-LINK VCP 점퍼 회로도 | [📥 UM2505 다운로드](https://www.st.com/resource/en/user_manual/um2505-stm32g4-nucleo64-boards-mb1367-stmicroelectronics.pdf) |
| **DS12288** | **Datasheet** | ~5 MB | **STM32G474xB/xC/xE 칩 사양서**<br>• 핀아웃 다이어그램, 전기적 한계치, 핀별 대체 기능(AF) 표 | [📥 DS12288 다운로드](https://www.st.com/resource/en/datasheet/stm32g474re.pdf) |
| **PM0214** | **Programming Manual** | ~2 MB | **Arm Cortex-M4 코어 프로그래밍 가이드**<br>• NVIC 인터럽트 컨트롤러, SysTick 타이머, 코어 레지스터 | [📥 PM0214 다운로드](https://www.st.com/resource/en/programming_manual/pm0214-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf) |

---

## 💡 로컬 보관 안내
위 링크에서 다운로드하신 PDF 파일들을 이 폴더(`docs/manuals/`)에 넣어두시면 프로젝트를 진행하며 언제든 쉽게 참조하실 수 있습니다.
- 대용량 PDF 파일은 Git 저장소 용량 관리를 위해 `.gitignore`로 제외하도록 설정할 수 있습니다.

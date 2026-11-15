/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-6] GPIO EXTI Tutorial
 *                   Button (PC13) -> EXTI13 -> NVIC -> LED (PA5) Toggle
 ******************************************************************************
 */

#include <stdint.h>

/* 레지스터 주소 매핑 (STM32G474 레퍼런스 매뉴얼 RM0440 기준) */
// 1. Reset and Clock Control (RCC)
#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x60U))

// 2. GPIO Port A (PA5 - Green LED)
#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_ODR           (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))

// 3. GPIO Port C (PC13 - Blue Push Button)
#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))

// 4. System Configuration (SYSCFG - EXTI 라우팅 제어)
#define SYSCFG_BASE         0x40010000U
#define SYSCFG_EXTICR4      (*(volatile uint32_t *)(SYSCFG_BASE + 0x14U))

// 5. Extended Interrupt and Event Controller (EXTI)
#define EXTI_BASE           0x40010400U
#define EXTI_IMR1           (*(volatile uint32_t *)(EXTI_BASE + 0x00U))
#define EXTI_FTSR1          (*(volatile uint32_t *)(EXTI_BASE + 0x0CU))
#define EXTI_PR1            (*(volatile uint32_t *)(EXTI_BASE + 0x14U))

// 6. Cortex-M4 NVIC (Nested Vectored Interrupt Controller)
#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104U))

/* 소프트웨어 디바운싱용 간단한 딜레이 함수 */
static void delay_debounce(void)
{
    for (volatile uint32_t i = 0; i < 150000; i++) {
        __asm__("nop");
    }
}

/**
 * @brief EXTI Line 15~10 하드웨어 인터럽트 서비스 루틴 (ISR)
 *        버튼(PC13)을 누르는 순간 하드웨어가 자동으로 이 함수를 호출합니다.
 */
void EXTI15_10_IRQHandler(void)
{
    // 13번 라인에서 발생한 인터럽트인지 확인
    if (EXTI_PR1 & (1U << 13))
    {
        // [중요] 인터럽트 펜딩 플래그 클리어 (해당 비트에 1을 써야 지워짐)
        EXTI_PR1 = (1U << 13);

        // 녹색 LED (PA5) 상태 반전 (토글)
        GPIOA_ODR ^= (1U << 5);

        // 기계식 스위치 채터링 방지 딜레이
        delay_debounce();
    }
}

int main(void)
{
    // [1단계] 클럭 활성화
    // GPIOA(bit 0), GPIOC(bit 2) 클럭 인가
    RCC_AHB2ENR |= (1U << 0) | (1U << 2);
    // SYSCFG 클럭 인가 (EXTI 핀 라우팅을 위해 필수)
    RCC_APB2ENR |= (1U << 0);

    // [2단계] PA5 (LED LD2)를 범용 출력(Output, 01) 모드로 설정
    GPIOA_MODER &= ~(3U << (5 * 2));
    GPIOA_MODER |=  (1U << (5 * 2));

    // [3단계] PC13 (파란색 버튼 B1)을 입력(Input, 00) 모드로 설정
    GPIOC_MODER &= ~(3U << (13 * 2));

    // [4단계] SYSCFG에서 EXTI13 라인을 Port C(0x2)로 라우팅
    // EXTICR4의 비트 [7:4]에 0x2 기록 (0x0:PA, 0x1:PB, 0x2:PC)
    SYSCFG_EXTICR4 &= ~(0xFU << 4);
    SYSCFG_EXTICR4 |=  (0x2U << 4);

    // [5단계] EXTI 13번 라인 설정
    // 버튼을 누르는 순간(High -> Low 전이)을 포착하기 위해 Falling Trigger 활성화
    EXTI_FTSR1 |= (1U << 13);
    // 13번 라인의 인터럽트 마스크 해제 (인터럽트 허용)
    EXTI_IMR1  |= (1U << 13);

    // [6단계] Cortex-M4 NVIC 인터럽트 활성화
    // EXTI15_10의 IRQ 번호는 40번 -> ISER[1]의 8번 비트 (40 - 32 = 8)
    NVIC_ISER1 = (1U << (40 - 32));

    // [7단계] 메인 루프 - CPU는 대기(Sleep) 상태 유지
    // 버튼이 눌리면 하드웨어가 CPU를 깨워 EXTI15_10_IRQHandler를 실행합니다.
    while (1)
    {
        __asm__("wfi"); // Wait For Interrupt: 전력 절약 대기 모드
    }
}

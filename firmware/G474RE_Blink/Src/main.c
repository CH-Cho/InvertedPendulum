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

/* 레지스터 주소 매핑 (STM32G474 레퍼런스 매뉴얼 RM0440 기준) */
// 1. Reset and Clock Control (RCC)
#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB1ENR1        (*(volatile uint32_t *)(RCC_BASE + 0x58U))
#define RCC_APB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x60U))

// 2. GPIO Port A (PA2: TX, PA3: RX, PA5: LED)
#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_ODR           (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))
#define GPIOA_AFRL          (*(volatile uint32_t *)(GPIOA_BASE + 0x20U))

// 3. GPIO Port C (PC13: Button)
#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))

// 4. System Configuration (SYSCFG) & EXTI
#define SYSCFG_BASE         0x40010000U
#define SYSCFG_EXTICR4      (*(volatile uint32_t *)(SYSCFG_BASE + 0x14U))
#define EXTI_BASE           0x40010400U
#define EXTI_IMR1           (*(volatile uint32_t *)(EXTI_BASE + 0x00U))
#define EXTI_FTSR1          (*(volatile uint32_t *)(EXTI_BASE + 0x0CU))
#define EXTI_PR1            (*(volatile uint32_t *)(EXTI_BASE + 0x14U))
#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104U))

// 5. USART2 Peripheral
#define USART2_BASE         0x40004400U
#define USART2_CR1          (*(volatile uint32_t *)(USART2_BASE + 0x00U))
#define USART2_BRR          (*(volatile uint32_t *)(USART2_BASE + 0x0CU))
#define USART2_ISR          (*(volatile uint32_t *)(USART2_BASE + 0x1CU))
#define USART2_TDR          (*(volatile uint32_t *)(USART2_BASE + 0x28U))

/* 버튼 클릭 이벤트 감지 플래그 (volatile 필수) */
static volatile uint8_t g_button_pressed = 0;

/* 소프트웨어 지연 함수 */
static void delay_ms(volatile uint32_t ms)
{
    while (ms--) {
        for (volatile uint32_t i = 0; i < 2000; i++) {
            __asm__("nop");
        }
    }
}

/**
 * @brief USART2 1바이트 하드웨어 전송 함수
 */
static void uart2_write_char(uint8_t ch)
{
    while (!(USART2_ISR & (1U << 7)));
    USART2_TDR = ch;
}

/**
 * @brief 표준 C 라이브러리의 printf() 출력을 가로채는 시스템 콜
 */
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

/**
 * @brief USART2 초기화 (115200 bps, 8-N-1)
 */
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

/**
 * @brief EXTI15_10 인터럽트 핸들러 (파란색 버튼 B1)
 */
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

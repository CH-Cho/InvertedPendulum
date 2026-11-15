/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-8] TIM6 1ms (1kHz) TimeBase + EXTI Button + USART2 Printf
 *                   Dynamic Period Tuning (500ms -> 1000ms -> 1500ms -> 2000ms)
 * @repository     : https://github.com/CH-Cho/InvertedPendulum
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

/* 레지스터 주소 매핑 (STM32G474 RM0440 기준) */
// 1. RCC (Reset and Clock Control)
#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB1ENR1        (*(volatile uint32_t *)(RCC_BASE + 0x58U))
#define RCC_APB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x60U))

// 2. GPIO Port A (PA2: TX, PA3: RX, PA5: LED)
#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_ODR           (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))
#define GPIOA_AFRL          (*(volatile uint32_t *)(GPIOA_BASE + 0x20U))

// 3. USART2 Peripheral
#define USART2_BASE         0x40004400U
#define USART2_CR1          (*(volatile uint32_t *)(USART2_BASE + 0x00U))
#define USART2_BRR          (*(volatile uint32_t *)(USART2_BASE + 0x0CU))
#define USART2_ISR          (*(volatile uint32_t *)(USART2_BASE + 0x1CU))
#define USART2_TDR          (*(volatile uint32_t *)(USART2_BASE + 0x28U))

// 4. TIM6 Basic Timer
#define TIM6_BASE           0x40001000U
#define TIM6_CR1            (*(volatile uint32_t *)(TIM6_BASE + 0x00U))
#define TIM6_DIER           (*(volatile uint32_t *)(TIM6_BASE + 0x0CU))
#define TIM6_SR             (*(volatile uint32_t *)(TIM6_BASE + 0x10U))
#define TIM6_PSC            (*(volatile uint32_t *)(TIM6_BASE + 0x28U))
#define TIM6_ARR            (*(volatile uint32_t *)(TIM6_BASE + 0x2CU))

// 5. GPIO Port C (PC13: Button)
#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))

// 6. System Configuration (SYSCFG) & EXTI
#define SYSCFG_BASE         0x40010000U
#define SYSCFG_EXTICR4      (*(volatile uint32_t *)(SYSCFG_BASE + 0x14U))
#define EXTI_BASE           0x40010400U
#define EXTI_IMR1           (*(volatile uint32_t *)(EXTI_BASE + 0x00U))
#define EXTI_FTSR1          (*(volatile uint32_t *)(EXTI_BASE + 0x0CU))
#define EXTI_PR1            (*(volatile uint32_t *)(EXTI_BASE + 0x14U))
#define NVIC_ISER1          (*(volatile uint32_t *)(0xE000E104U))

/* 1ms마다 1씩 증가하는 시스템 밀리초 카운터 및 버튼 제어 변수 (volatile 필수) */
static volatile uint32_t g_system_millis = 0;
static volatile uint32_t g_button_pressed = 0;
static volatile uint32_t g_led_toggle_time = 500;

/**
 * @brief USART2 1바이트 전송
 */
static void uart2_write_char(uint8_t ch)
{
    while (!(USART2_ISR & (1U << 7)));
    USART2_TDR = ch;
}

/**
 * @brief printf 리디렉션 (_write)
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
 * @brief USART2 초기화 (115200 bps)
 */
static void uart2_init(void)
{
    // 1. GPIOA 및 USART2 주변장치 클럭 활성화
    RCC_AHB2ENR  |= (1U << 0);  // GPIOAEN
    RCC_APB1ENR1 |= (1U << 17); // USART2EN

    // 2. PA2 (TX), PA3 (RX) 핀을 Alternate Function 모드(10)로 설정
    GPIOA_MODER &= ~((3U << (2 * 2)) | (3U << (3 * 2)));
    GPIOA_MODER |=  ((2U << (2 * 2)) | (2U << (3 * 2)));

    // 3. PA2, PA3의 대체 기능을 AF7 (USART2)로 라우팅
    GPIOA_AFRL &= ~((0xFU << (2 * 4)) | (0xFU << (3 * 4)));
    GPIOA_AFRL |=  ((0x7U << (2 * 4)) | (0x7U << (3 * 4)));

    // 4. 보레이트 설정: 115200 bps @ 16MHz (16,000,000 / 115,200 = 138.88 -> 139)
    USART2_BRR = 139U;

    // 5. USART2 활성화: 모듈(UE), 송신(TE), 수신(RE) 켜기
    USART2_CR1 = (1U << 0) | (1U << 3) | (1U << 2);
}

/**
 * @brief TIM6 기본 타이머 초기화 (1ms 정주기 인터럽트 설정)
 */
static void tim6_init(void)
{
    // 1. TIM6 타이머 클럭 활성화 (APB1ENR1 비트 4)
    RCC_APB1ENR1 |= (1U << 4);

    // 2. 프리스케일러(PSC) 설정: 16MHz -> 1MHz (1us 주기 카운트)
    TIM6_PSC = 15U; // 16분주 (15 + 1)

    // 3. 자동 리로드(ARR) 설정: 1us x 1000 = 1ms (1000Hz)
    TIM6_ARR = 999U; // 1000 카운트 (999 + 1)

    // 4. 타이머 갱신 인터럽트(Update Interrupt) 허용 (UIE: 비트 0)
    TIM6_DIER |= (1U << 0);

    // 5. NVIC 인터럽트 활성화 (TIM6 IRQ 번호는 54번 -> ISER1의 22번 비트)
    NVIC_ISER1 = (1U << (54 - 32));

    // 6. 타이머 카운터 동작 시작 (CEN: 비트 0)
    TIM6_CR1 |= (1U << 0);
}

/**
 * @brief TIM6 하드웨어 인터럽트 서비스 루틴 (정확히 1ms마다 자동 호출됨)
 *        동적으로 변경되는 g_led_toggle_time 주기에 맞춰 LED를 토글합니다.
 */
void TIM6_DACUNDER_IRQHandler(void)
{
    if (TIM6_SR & (1U << 0))
    {
        TIM6_SR &= ~(1U << 0); // 펜딩 플래그 클리어 (RC_W0)
        g_system_millis++;     // 1ms 카운터 증가

        // 동적으로 변경되는 주기(g_led_toggle_time)마다 녹색 LED(PA5) 토글
        if (g_system_millis % g_led_toggle_time == 0) {
            GPIOA_ODR ^= (1U << 5);
        }
    }
}

/**
 * @brief EXTI15_10 인터럽트 핸들러 (파란색 버튼 B1)
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_PR1 & (1U << 13)) {
        EXTI_PR1 = (1U << 13); // 펜딩 플래그 클리어
        GPIOA_ODR ^= (1U << 5); // LED 토글
        g_button_pressed = 1;   // 버튼 눌림 이벤트 발생 플래그 세팅
    }
}

int main(void)
{
    // 1. 통신 및 타이머 초기화
    uart2_init();
    tim6_init();

    // 2. LED(PA5) 및 버튼(PC13) EXTI 인터럽트 설정
    RCC_AHB2ENR |= (1U << 2); // GPIOC 클럭
    RCC_APB2ENR |= (1U << 0); // SYSCFG 클럭
    GPIOA_MODER &= ~(3U << (5 * 2));
    GPIOA_MODER |=  (1U << (5 * 2));

    GPIOC_MODER &= ~(3U << (13 * 2)); // PC13 Input

    SYSCFG_EXTICR4 &= ~(0xFU << 4);
    SYSCFG_EXTICR4 |=  (0x2U << 4);  // EXTI13 -> Port C

    EXTI_FTSR1 |= (1U << 13);        // Falling Edge 트리거
    EXTI_IMR1  |= (1U << 13);        // 인터럽트 허용
    NVIC_ISER1 |= (1U << (40 - 32));  // NVIC IRQ 40 활성화

    // 3. 시작 배너 출력
    printf("\r\n");
    printf("====================================================\r\n");
    printf("  STM32G474RE Inverted Pendulum Project             \r\n");
    printf("  [CH-8] TIM6 1ms (1kHz) Hardware TimeBase Running! \r\n");
    printf("====================================================\r\n\r\n");

    uint32_t last_print_time = 0;

    // 4. 메인 루프: 1초마다 정확한 밀리초 출력 & 동적 주기 변경 처리
    while (1)
    {
        // g_led_toggle_time 주기가 지났는지 비동기 확인
        if (g_system_millis - last_print_time >= g_led_toggle_time)
        {
            last_print_time = g_system_millis;
            printf("[Real-Time Clock] Elapsed: %lu ms (Current Period: %lu ms)\r\n", g_system_millis, g_led_toggle_time);
        }

        // 버튼이 눌렸다면 주기(500ms -> 1000ms -> 1500ms -> 2000ms -> 500ms) 순환 변경
        if (g_button_pressed == 1)
        {
            g_led_toggle_time += 500;
            printf(">>> [BUTTON EVENT] Cycle Changed -> Interrupt Period: %lu ms\r\n", g_led_toggle_time);

            if (g_led_toggle_time > 2001) {
                g_led_toggle_time = 500;
            }
            g_button_pressed = 0;
        }
    }
}

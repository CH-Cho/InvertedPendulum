/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-9 / Step 05-2] TIM2 20kHz PWM + DRV8876 H-Bridge
 *                   + USART2 RX 인터럽트 기반 키보드 실시간 모터 조종
 *                   - PA0 (A0) / PA5 : TIM2_CH1 20kHz PWM (DRV8876 EN / LD2 LED)
 *                   - PB0 (A3)       : GPIO Output 모터 방향 (PH: 1=오른쪽 f, 0=왼쪽 b)
 *                   - USART2 (VCP)   : 115200 bps 키보드 원격 조종 (RXNE 인터럽트)
 *                   - TIM6           : 1ms 타임베이스 (250ms 안전 자동 정지 타이머)
 * @repository     : https://github.com/CH-Cho/InvertedPendulum
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

/* ============================================================================
 * 1. STM32G474RE 베어메탈 레지스터 정의 (RM0440 기준)
 * ========================================================================== */
// RCC
#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB1ENR1        (*(volatile uint32_t *)(RCC_BASE + 0x58U))

// GPIO A, B
#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_AFRL          (*(volatile uint32_t *)(GPIOA_BASE + 0x20U))

#define GPIOB_BASE          0x48000400U
#define GPIOB_MODER         (*(volatile uint32_t *)(GPIOB_BASE + 0x00U))
#define GPIOB_ODR           (*(volatile uint32_t *)(GPIOB_BASE + 0x14U))

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

// TIM6 (1ms System Tick)
#define TIM6_BASE           0x40001000U
#define TIM6_CR1            (*(volatile uint32_t *)(TIM6_BASE + 0x00U))
#define TIM6_DIER           (*(volatile uint32_t *)(TIM6_BASE + 0x0CU))
#define TIM6_SR             (*(volatile uint32_t *)(TIM6_BASE + 0x10U))
#define TIM6_PSC            (*(volatile uint32_t *)(TIM6_BASE + 0x28U))
#define TIM6_ARR            (*(volatile uint32_t *)(TIM6_BASE + 0x2CU))

// TIM2 (20kHz PWM: PA0 & PA5)
#define TIM2_BASE           0x40000000U
#define TIM2_CR1            (*(volatile uint32_t *)(TIM2_BASE + 0x00U))
#define TIM2_CCMR1          (*(volatile uint32_t *)(TIM2_BASE + 0x18U))
#define TIM2_CCER           (*(volatile uint32_t *)(TIM2_BASE + 0x20U))
#define TIM2_PSC            (*(volatile uint32_t *)(TIM2_BASE + 0x28U))
#define TIM2_ARR            (*(volatile uint32_t *)(TIM2_BASE + 0x2CU))
#define TIM2_CCR1           (*(volatile uint32_t *)(TIM2_BASE + 0x34U))

/* ============================================================================
 * 2. 전역 상태 변수
 * ========================================================================== */
#define PWM_ARR_MAX         799U   // 16MHz / (0+1) / (799+1) = 20,000 Hz (20kHz)

static volatile uint32_t g_system_millis = 0;
static volatile uint32_t g_motor_stop_at = 0;
static volatile uint8_t  g_motor_running = 0;
static volatile uint8_t  g_test_duty_pct = 50; // IP570 기동 마찰력 극복 테스트용 기본 50%
static volatile uint8_t  g_motor_dir     = 1;  // 1: FWD (오른쪽), 0: REV (왼쪽)
static volatile uint8_t  g_rx_cmd        = 0;

/* ============================================================================
 * 3. 유틸리티 및 드라이버 초기화 함수
 * ========================================================================== */
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
    TIM6_ARR = 999U;           // 1ms 주기 인터럽트
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
    GPIOB_ODR   |=  (1U << 0); // 초기 방향 FWD (오른쪽)

    // TIM2 20kHz PWM 설정 (16MHz / 1 / 800 = 20,000 Hz)
    TIM2_PSC   = 0U;
    TIM2_ARR   = PWM_ARR_MAX;
    TIM2_CCR1  = 0U; // 부팅 시 완전 정지!
    TIM2_CCMR1 = (0x6U << 4) | (1U << 3); // PWM Mode 1 + Preload
    TIM2_CCER  = (1U << 0);               // CH1 출력 Enable
    TIM2_CR1   = (1U << 0);               // TIM2 시작
}

static void motor_stop(void)
{
    TIM2_CCR1 = 0U;
    g_motor_running = 0U;
}

static void motor_kick(uint8_t dir, uint8_t duty_pct, uint32_t duration_ms)
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
    g_motor_stop_at = g_system_millis + duration_ms;
    g_motor_running = 1U;
}

/* ============================================================================
 * 4. 인터럽트 핸들러
 * ========================================================================== */
void TIM6_DACUNDER_IRQHandler(void)
{
    if (TIM6_SR & (1U << 0)) {
        TIM6_SR &= ~(1U << 0);
        g_system_millis++;

        // 레일 충돌 방지용 자동 정지 타이머 (250ms 펄스 킥 후 자동 정지)
        if (g_motor_running && ((int32_t)(g_system_millis - g_motor_stop_at) >= 0)) {
            motor_stop();
        }
    }
}

void USART2_IRQHandler(void)
{
    // Overrun 에러(ORE) 방어 -> 연속 키 입력 시 수신 멈춤 방지
    if (USART2_ISR & (1U << 3)) {
        USART2_ICR = (1U << 3);
    }
    if (USART2_ISR & (1U << 5)) { // RXNE
        g_rx_cmd = (uint8_t)USART2_RDR;
    }
}

/* ============================================================================
 * 5. 메인 루프 (키보드 실시간 모터 원격 조종)
 * ========================================================================== */
int main(void)
{
    tim6_init();
    uart2_init();
    motor_driver_init();

    printf("\r\n============================================================\r\n");
    printf("  STM32G474RE Inverted Pendulum Project                     \r\n");
    printf("  [Step 05-2] 20kHz PWM + DRV8876 + UART Keyboard Control   \r\n");
    printf("  Outputs : PA0 (TIM2_CH1 PWM -> EN) | PB0 (DIR -> PH)      \r\n");
    printf("  Safety  : 250ms Auto-Stop Pulse Kick (No End-Stop Switch) \r\n");
    printf("------------------------------------------------------------\r\n");
    printf("  Commands: f = Right(FWD) 250ms | b = Left(REV) 250ms      \r\n");
    printf("            s / Space = Emergency STOP                      \r\n");
    printf("            1~6 = Duty Preset (15%%,25%%,35%%,50%%,65%%,80%%) \r\n");
    printf("============================================================\r\n\r\n");

    uint32_t last_print = 0;

    while (1)
    {
        if (g_rx_cmd != 0) {
            uint8_t cmd = g_rx_cmd;
            g_rx_cmd = 0;

            if (cmd == 's' || cmd == 'S' || cmd == ' ') {
                motor_stop();
                printf("\r\n>>> [STOP] Emergency Stop! (CCR1=0)\r\n");
            } else if (cmd == 'f' || cmd == 'F') {
                printf("\r\n>>> [KICK RIGHT] Dir=1 (PH=1), Duty=%u%% (250ms)\r\n", g_test_duty_pct);
                motor_kick(1, g_test_duty_pct, 250);
            } else if (cmd == 'b' || cmd == 'B') {
                printf("\r\n>>> [KICK LEFT]  Dir=0 (PH=0), Duty=%u%% (250ms)\r\n", g_test_duty_pct);
                motor_kick(0, g_test_duty_pct, 250);
            } else if (cmd >= '1' && cmd <= '6') {
                const uint8_t duties[6] = {15, 25, 35, 50, 65, 80};
                g_test_duty_pct = duties[cmd - '1'];
                printf("\r\n>>> [DUTY SET] Test Duty = %u%% (CCR1=%u/800)\r\n",
                       g_test_duty_pct,
                       (unsigned int)(((PWM_ARR_MAX + 1U) * g_test_duty_pct) / 100U));
            }
        }

        // 1초마다 대기 상태 출력
        if ((g_system_millis - last_print) >= 1000) {
            last_print = g_system_millis;
            printf("[STATUS] Motor: %s | Last Dir: %s | Duty Preset: %2u%% (20kHz)\r\n",
                   g_motor_running ? "RUNNING" : "IDLE   ",
                   g_motor_dir ? "RIGHT(FWD)" : "LEFT (REV)",
                   g_test_duty_pct);
        }
    }
}

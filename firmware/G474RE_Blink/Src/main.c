/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-9 / Step 05-2] Pololu #4037 DRV8876 20kHz PWM + UART 조종
 *                   + [HW Bring-up] TIM3 카트 엔코더(PC6/7) & ADC1 각도센서(PA1)
 *                   - PA0 (A0) / PA5 : TIM2_CH1 20kHz PWM (EN / LD2)
 *                   - PB0 (A3)       : GPIO Output 모터 방향 (PH: 1=오른쪽 f, 0=왼쪽 b)
 *                   - PC6 / PC7 (D9) : TIM3 4체배 직교 엔코더 (FT 5V내성)
 *                   - PA1 (A1)       : ADC1_IN2 진자 각도센서 (WDD35D4-5K, 3.3V)
 *                   - USART2 (VCP)   : 115200 bps 실시간 텔레메트리 & RX 인터럽트 조종
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

// GPIO A, B, C
#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_AFRL          (*(volatile uint32_t *)(GPIOA_BASE + 0x20U))

#define GPIOB_BASE          0x48000400U
#define GPIOB_MODER         (*(volatile uint32_t *)(GPIOB_BASE + 0x00U))
#define GPIOB_ODR           (*(volatile uint32_t *)(GPIOB_BASE + 0x14U))

#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))
#define GPIOC_PUPDR         (*(volatile uint32_t *)(GPIOC_BASE + 0x0CU))
#define GPIOC_IDR           (*(volatile uint32_t *)(GPIOC_BASE + 0x10U))
#define GPIOC_AFRL          (*(volatile uint32_t *)(GPIOC_BASE + 0x20U))

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

// TIM3 (Quadrature Encoder: PC6=CH1, PC7=CH2)
#define TIM3_BASE           0x40000400U
#define TIM3_CR1            (*(volatile uint32_t *)(TIM3_BASE + 0x00U))
#define TIM3_SMCR           (*(volatile uint32_t *)(TIM3_BASE + 0x08U))
#define TIM3_CCMR1          (*(volatile uint32_t *)(TIM3_BASE + 0x18U))
#define TIM3_CCER           (*(volatile uint32_t *)(TIM3_BASE + 0x20U))
#define TIM3_CNT            (*(volatile uint32_t *)(TIM3_BASE + 0x24U))
#define TIM3_ARR            (*(volatile uint32_t *)(TIM3_BASE + 0x2CU))

// ADC1 (Pendulum Potentiometer: PA1=ADC1_IN2)
#define ADC1_BASE           0x50000000U
#define ADC1_ISR            (*(volatile uint32_t *)(ADC1_BASE + 0x00U))
#define ADC1_CR             (*(volatile uint32_t *)(ADC1_BASE + 0x08U))
#define ADC1_CFGR           (*(volatile uint32_t *)(ADC1_BASE + 0x0CU))
#define ADC1_SMPR1          (*(volatile uint32_t *)(ADC1_BASE + 0x14U))
#define ADC1_SQR1           (*(volatile uint32_t *)(ADC1_BASE + 0x30U))
#define ADC1_DR             (*(volatile uint32_t *)(ADC1_BASE + 0x40U))
#define ADC12_CCR           (*(volatile uint32_t *)(0x50000308U))

/* ============================================================================
 * 2. 전역 상태 변수
 * ========================================================================== */
#define PWM_ARR_MAX         799U   // 16MHz / 800 = 20kHz (비가청 PWM)

static volatile uint32_t g_system_millis = 0;
static volatile uint32_t g_motor_stop_at = 0;
static volatile uint8_t  g_motor_running = 0;
static volatile uint8_t  g_test_duty_pct = 25; // 기본 테스트 듀티 25%
static volatile uint8_t  g_motor_dir     = 1;  // 1: FWD (오른쪽), 0: REV (왼쪽)
static volatile uint8_t  g_rx_cmd        = 0;

/* ============================================================================
 * 3. 유틸리티 및 드라이버 초기화 함수
 * ========================================================================== */
static void delay_ms(uint32_t ms)
{
    uint32_t start = g_system_millis;
    while ((g_system_millis - start) < ms);
}

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
    TIM6_PSC = 15U;            // 16MHz / 16 = 1MHz
    TIM6_ARR = 999U;           // 1ms 주기
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

static void encoder_tim3_init(void)
{
    RCC_AHB2ENR  |= (1U << 2); // GPIOCEN
    RCC_APB1ENR1 |= (1U << 1); // TIM3EN

    // PC6 (CN10-4, A상) & PC7 (D9/CN10-19, B상) -> AF2 (TIM3_CH1/CH2)
    GPIOC_MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOC_MODER |=  ((2U << (6 * 2)) | (2U << (7 * 2)));

    // 5V 내성(FT) 보호를 위해 내부 Pull-up/down 반드시 비활성화 (00)
    GPIOC_PUPDR &= ~((3U << (6 * 2)) | (3U << (7 * 2)));

    GPIOC_AFRL  &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOC_AFRL  |=  ((0x2U << (6 * 4)) | (0x2U << (7 * 4)));

    // TIM3 Quadrature Encoder Mode 3 (TI1, TI2 양방향 4체배 카운트 + 디지털 필터)
    TIM3_ARR   = 0xFFFFU;
    TIM3_CCMR1 = (1U << 0) | (1U << 8) | (6U << 4) | (6U << 12);
    TIM3_CCER  = 0U;
    TIM3_SMCR  = 3U; // SMS = 0011 (Encoder Mode 3)
    TIM3_CNT   = 0U;
    TIM3_CR1   = 1U;
}

static void adc1_pa1_init(void)
{
    RCC_AHB2ENR |= (1U << 0) | (1U << 13); // GPIOAEN, ADC12EN

    // PA1 (A1 -> WDD35D4-5K Wiper) -> Analog Mode (11)
    GPIOA_MODER |= (3U << (1 * 2));

    // ADC 공통 클럭 설정: HCLK/1 (16MHz 동기 클럭)
    ADC12_CCR = (1U << 16);

    // 1. Deep Power-Down 해제 -> 내부 전압 레귤레이터 켜기 -> 안정화 대기
    ADC1_CR &= ~(1U << 29); // DEEPPWD = 0
    ADC1_CR |=  (1U << 28); // ADVREGEN = 1
    delay_ms(2);

    // 2. 싱글 엔드 자체 캘리브레이션 실행
    ADC1_CR |= (1U << 31); // ADCAL = 1
    while (ADC1_CR & (1U << 31));

    // 3. ADC Enable 및 Ready 대기
    ADC1_ISR = (1U << 0);  // ADRDY 플래그 클리어
    ADC1_CR |= (1U << 0);  // ADEN = 1
    while (!(ADC1_ISR & (1U << 0)));

    // 4. 채널 2 (PA1) 샘플링 타임 & 시퀀서(1채널) 설정
    ADC1_CFGR  = 0U;         // 12-bit, Right-aligned, Single conversion
    ADC1_SMPR1 = (4U << 6);  // CH2 샘플링 타임: 47.5 ADC cycles
    ADC1_SQR1  = (2U << 6);  // L = 0 (1개 변환), SQ1 = CH2
}

static uint16_t adc1_read_pa1(void)
{
    ADC1_CR |= (1U << 2); // ADSTART
    while (!(ADC1_ISR & (1U << 2))); // EOC 대기
    return (uint16_t)(ADC1_DR & 0x0FFFU);
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

        // 안전 자동 정지 타이머 (250ms 펄스 킥 후 자동 정지)
        if (g_motor_running && ((int32_t)(g_system_millis - g_motor_stop_at) >= 0)) {
            motor_stop();
        }
    }
}

void USART2_IRQHandler(void)
{
    // Overrun 에러(ORE) 방지
    if (USART2_ISR & (1U << 3)) {
        USART2_ICR = (1U << 3);
    }
    if (USART2_ISR & (1U << 5)) { // RXNE
        g_rx_cmd = (uint8_t)USART2_RDR;
    }
}

/* ============================================================================
 * 5. 메인 루프 (실시간 하드웨어 상태 모니터링 & 명령 처리)
 * ========================================================================== */
int main(void)
{
    tim6_init();
    uart2_init();
    motor_driver_init();
    encoder_tim3_init();
    adc1_pa1_init();

    printf("\r\n=================================================================\r\n");
    printf("  [HW Verification] Inverted Pendulum Hardware Diagnostic\r\n");
    printf("  - Angle Pot (PA1)    : 0~4095 (0.00V ~ 3.30V)\r\n");
    printf("  - Cart Enc  (PC6/7)  : TIM3 4x Quad Counter & Raw A/B State\r\n");
    printf("  - Motor     (PA0/PB0): 20kHz PWM | Safe 250ms Kick Mode\r\n");
    printf("-----------------------------------------------------------------\r\n");
    printf("  [Commands] f: FWD 250ms | b: REV 250ms | s/Space: STOP\r\n");
    printf("             1~6: Test Duty (15%%~80%%) | z: Zero Encoder\r\n");
    printf("=================================================================\r\n\r\n");

    uint32_t last_print = 0;

    while (1)
    {
        // 1. 키보드 명령 처리
        if (g_rx_cmd != 0) {
            uint8_t cmd = g_rx_cmd;
            g_rx_cmd = 0;

            if (cmd == 's' || cmd == 'S' || cmd == ' ') {
                motor_stop();
                printf("\r\n>>> [STOP] Emergency Stop! (CCR1=0)\r\n");
            } else if (cmd == 'f' || cmd == 'F') {
                printf("\r\n>>> [KICK FWD] Dir=1, Duty=%u%% (250ms)\r\n", g_test_duty_pct);
                motor_kick(1, g_test_duty_pct, 250);
            } else if (cmd == 'b' || cmd == 'B') {
                printf("\r\n>>> [KICK REV] Dir=0, Duty=%u%% (250ms)\r\n", g_test_duty_pct);
                motor_kick(0, g_test_duty_pct, 250);
            } else if (cmd >= '1' && cmd <= '6') {
                const uint8_t duties[6] = {15, 25, 35, 50, 65, 80};
                g_test_duty_pct = duties[cmd - '1'];
                printf("\r\n>>> [DUTY SET] Test Duty = %u%%\r\n", g_test_duty_pct);
            } else if (cmd == 'z' || cmd == 'Z') {
                TIM3_CNT = 0U;
                printf("\r\n>>> [ENC ZERO] TIM3_CNT Reset to 0\r\n");
            }
        }

        // 2. 200ms마다 실시간 센서 및 상태 텔레메트리 출력
        if ((g_system_millis - last_print) >= 200) {
            last_print = g_system_millis;

            uint16_t raw_adc = adc1_read_pa1();
            uint32_t mv = ((uint32_t)raw_adc * 3300U) / 4095U;
            int16_t  enc_cnt = (int16_t)(uint16_t)TIM3_CNT;
            uint8_t  raw_a = (uint8_t)((GPIOC_IDR >> 6) & 1U);
            uint8_t  raw_b = (uint8_t)((GPIOC_IDR >> 7) & 1U);

            printf("[LIVE] Angle(PA1): %4u (%lu.%02luV) | Enc(PC6/7): %+6d [A:%u B:%u] | Motor: %s (Duty:%2u%%)\r\n",
                   raw_adc,
                   mv / 1000U, (mv % 1000U) / 10U,
                   enc_cnt,
                   raw_a, raw_b,
                   g_motor_running ? (g_motor_dir ? "FWD " : "REV ") : "IDLE",
                   g_test_duty_pct);
        }
    }
}

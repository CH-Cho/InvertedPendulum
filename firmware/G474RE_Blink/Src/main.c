/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : [CH-10 / Step 06] TIM3 4체배 직교 엔코더 & ADC1 진자 각도계
 *                   + 가이드 위치 초기화(오른쪽/왼쪽 끝 -> 자동 중앙 복귀)
 *                   + 진자 2점 각도 캘리브레이션(6시/12시) 및 6시 간편 영점
 *                   - PA0 (A0) / PA5 : TIM2_CH1 20kHz PWM (DRV8876 EN / LD2 LED)
 *                   - PB0 (A3)       : GPIO Output 모터 방향 (PH: 1=오른쪽 f, 0=왼쪽 b)
 *                   - PC6 (CN10-4)   : TIM3_CH1 (AF2, 5V FT) 엔코더 A상
 *                   - PC7 (D9)       : TIM3_CH2 (AF2, 5V FT) 엔코더 B상
 *                   - PA1 (A1)       : ADC1_IN2 진자 포텐쇼미터(WDD35D4-5K) 각도
 *                   - USART2 (VCP)   : 115200 bps 텔레메트리 및 키보드 조종
 *                   - TIM6           : 1ms 타임베이스 (Soft Limit & 100Hz 샘플링)
 * @repository     : https://github.com/CH-Cho/InvertedPendulum
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

/* ============================================================================
 * 1. STM32G474RE 베어메탈 레지스터 정의 (RM0440 기준)
 * ========================================================================== */
// Cortex-M4 Core (FPU 접근 제어)
#define SCB_CPACR           (*(volatile uint32_t *)0xE000ED88U)

// RCC
#define RCC_BASE            0x40021000U
#define RCC_AHB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x4CU))
#define RCC_APB1ENR1        (*(volatile uint32_t *)(RCC_BASE + 0x58U))

// GPIO A, B, C
#define GPIOA_BASE          0x48000000U
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_PUPDR         (*(volatile uint32_t *)(GPIOA_BASE + 0x0CU))
#define GPIOA_AFRL          (*(volatile uint32_t *)(GPIOA_BASE + 0x20U))

#define GPIOB_BASE          0x48000400U
#define GPIOB_MODER         (*(volatile uint32_t *)(GPIOB_BASE + 0x00U))
#define GPIOB_ODR           (*(volatile uint32_t *)(GPIOB_BASE + 0x14U))

#define GPIOC_BASE          0x48000800U
#define GPIOC_MODER         (*(volatile uint32_t *)(GPIOC_BASE + 0x00U))
#define GPIOC_PUPDR         (*(volatile uint32_t *)(GPIOC_BASE + 0x0CU))
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

// TIM2 (20kHz PWM: PA0 & PA5)
#define TIM2_BASE           0x40000000U
#define TIM2_CR1            (*(volatile uint32_t *)(TIM2_BASE + 0x00U))
#define TIM2_CCMR1          (*(volatile uint32_t *)(TIM2_BASE + 0x18U))
#define TIM2_CCER           (*(volatile uint32_t *)(TIM2_BASE + 0x20U))
#define TIM2_PSC            (*(volatile uint32_t *)(TIM2_BASE + 0x28U))
#define TIM2_ARR            (*(volatile uint32_t *)(TIM2_BASE + 0x2CU))
#define TIM2_CCR1           (*(volatile uint32_t *)(TIM2_BASE + 0x34U))

// TIM3 (4체배 직교 엔코더: PC6=CH1, PC7=CH2)
#define TIM3_BASE           0x40000400U
#define TIM3_CR1            (*(volatile uint32_t *)(TIM3_BASE + 0x00U))
#define TIM3_SMCR           (*(volatile uint32_t *)(TIM3_BASE + 0x08U))
#define TIM3_CCMR1          (*(volatile uint32_t *)(TIM3_BASE + 0x18U))
#define TIM3_CCER           (*(volatile uint32_t *)(TIM3_BASE + 0x20U))
#define TIM3_CNT            (*(volatile uint32_t *)(TIM3_BASE + 0x24U))
#define TIM3_PSC            (*(volatile uint32_t *)(TIM3_BASE + 0x28U))
#define TIM3_ARR            (*(volatile uint32_t *)(TIM3_BASE + 0x2CU))

// TIM6 (1ms System Tick)
#define TIM6_BASE           0x40001000U
#define TIM6_CR1            (*(volatile uint32_t *)(TIM6_BASE + 0x00U))
#define TIM6_DIER           (*(volatile uint32_t *)(TIM6_BASE + 0x0CU))
#define TIM6_SR             (*(volatile uint32_t *)(TIM6_BASE + 0x10U))
#define TIM6_PSC            (*(volatile uint32_t *)(TIM6_BASE + 0x28U))
#define TIM6_ARR            (*(volatile uint32_t *)(TIM6_BASE + 0x2CU))

// ADC1 & ADC12 Common (PA1 = ADC1_IN2)
#define ADC1_BASE           0x50000000U
#define ADC1_ISR            (*(volatile uint32_t *)(ADC1_BASE + 0x00U))
#define ADC1_CR             (*(volatile uint32_t *)(ADC1_BASE + 0x08U))
#define ADC1_CFGR           (*(volatile uint32_t *)(ADC1_BASE + 0x0CU))
#define ADC1_SMPR1          (*(volatile uint32_t *)(ADC1_BASE + 0x14U))
#define ADC1_SQR1           (*(volatile uint32_t *)(ADC1_BASE + 0x30U))
#define ADC1_DR             (*(volatile uint32_t *)(ADC1_BASE + 0x40U))
#define ADC12_CCR           (*(volatile uint32_t *)0x50000308U)

/* ============================================================================
 * 2. 하드웨어 캘리브레이션 상수 및 전역 상태 변수
 * ========================================================================== */
#define PWM_ARR_MAX             799U    // 16MHz / (0+1) / (799+1) = 20,000 Hz (20kHz)

// IP570 카트 엔코더 기본 실측 교정치 (호밍 마법사 'h' 실행 시 자동 갱신됨)
#define RAIL_TRAVEL_MM          385
#define DEFAULT_TOTAL_COUNTS    6279
#define SOFT_LIMIT_MM           150     // 중앙 영점 기준 ±150.0 mm 소프트웨어 리미트
#define HOME_CENTER_BAND_CNT    50      // 자동 중앙 복귀 허용 오차 (±50 counts ≈ ±3.0 mm)
#define HOME_CENTER_TIMEOUT_MS  4000U   // 자동 중앙 복귀 안전 타임아웃 (4초)

// WDD35D4-5K 포텐쇼미터 기본 실측 교정치 (각도 마법사 'a' 실행 시 자동 갱신됨)
#define POT_DEFAULT_DOWN        1024
#define POT_DEFAULT_UPRIGHT     3140
#define POT_DEFAULT_180_COUNTS  2116
#define ANGLE_SIGN              1       // 오른쪽 기울임 시 부호 반전이 필요하면 -1로 변경

typedef enum {
    HOME_IDLE = 0,
    HOME_WAIT_RIGHT,
    HOME_WAIT_LEFT,
    HOME_AUTO_CENTER
} CartHomeState;

typedef enum {
    ACAL_IDLE = 0,
    ACAL_WAIT_DOWN,
    ACAL_WAIT_UP
} AngleCalState;

static volatile uint32_t g_system_millis      = 0;
static volatile uint8_t  g_tick_10ms          = 0;

// 모터 상태 및 안전 제어 변수
static volatile uint32_t g_motor_stop_at      = 0;
static volatile uint8_t  g_motor_running      = 0;
static volatile uint8_t  g_test_duty_pct      = 50; // IP570 기동 마찰력 극복 테스트용 기본 50%
static volatile uint8_t  g_motor_dir          = 1;  // 1: RIGHT(FWD), 0: LEFT(REV)
static volatile uint8_t  g_soft_limit_on      = 1;  // 1: ±150mm Soft Limit 활성화
static volatile uint8_t  g_soft_limit_trip    = 0;
static volatile uint8_t  g_rx_cmd             = 0;

// 카트 엔코더(TIM3) 확장 상태 및 동적 캘리브레이션 변수
static volatile int8_t   g_enc_sign           = 1;
static volatile int32_t  g_enc_total_counts   = DEFAULT_TOTAL_COUNTS;
static volatile int32_t  g_cart_counts        = 0;  // 중앙 0 기준 32-bit 확장 카운트
static volatile int32_t  g_cart_pos_mm_x10    = 0;  // 0.1 mm 단위 위치
static volatile int32_t  g_cart_vel_mms_x10   = 0;  // 0.1 mm/s 단위 LPF 필터링 속도
static volatile uint16_t s_prev_cnt_1ms       = 0;
static volatile int32_t  s_prev_counts_10ms   = 0;
static volatile int32_t  s_cart_vel_q4        = 0;

// 카트 호밍 마법사 상태 변수
static CartHomeState     g_home_state         = HOME_IDLE;
static int32_t           g_home_right_cnt     = 0;
static int32_t           g_home_left_cnt      = 0;
static uint32_t          g_home_center_start  = 0;

// 진자 각도계(ADC1) 상태 및 2점 캘리브레이션 변수
static AngleCalState     g_acal_state         = ACAL_IDLE;
static uint16_t          g_angle_down_adc     = POT_DEFAULT_DOWN;
static int32_t           g_angle_zero_adc     = POT_DEFAULT_UPRIGHT;
static int32_t           g_pot_180_counts     = POT_DEFAULT_180_COUNTS;
static int8_t            g_pot_up_dir_sign    = 1;  // +1: 12시가 6시보다 ADC 큼, -1: 작음
static uint16_t          g_angle_adc_raw      = POT_DEFAULT_UPRIGHT;
static int32_t           g_angle_cdeg         = 0;  // 0.01 deg (12시 = 0.00 deg)
static int32_t           g_angle_vel_x10      = 0;  // 0.1 deg/s LPF 각속도
static int32_t           g_prev_angle_cdeg    = 0;
static int32_t           g_angle_vel_q4       = 0;

/* ============================================================================
 * 3. 유틸리티 및 드라이버 초기화 함수
 * ========================================================================== */
static void fpu_enable(void)
{
    // CubeIDE 기본 fpv4-sp-d16 설정에서 부동소수점 명령 실행 시 HardFault 방지
    SCB_CPACR |= (0xFU << 20);
    __asm volatile ("dsb");
    __asm volatile ("isb");
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_system_millis;
    while ((g_system_millis - start) < ms) {
    }
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

static void print_fixed_1(int32_t val_x10)
{
    char sign = (val_x10 < 0) ? '-' : '+';
    int32_t abs_v = (val_x10 < 0) ? -val_x10 : val_x10;
    printf("%c%ld.%01ld", sign, (long)(abs_v / 10), (long)(abs_v % 10));
}

static void print_fixed_2(int32_t val_x100)
{
    char sign = (val_x100 < 0) ? '-' : '+';
    int32_t abs_v = (val_x100 < 0) ? -val_x100 : val_x100;
    printf("%c%ld.%02ld", sign, (long)(abs_v / 100), (long)(abs_v % 100));
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
    TIM6_ARR = 999U;           // 1ms 주기 인터럽트 (1kHz)
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
    GPIOB_ODR   |=  (1U << 0); // 초기 방향 RIGHT (FWD)

    // TIM2 20kHz PWM 설정 (16MHz / 1 / 800 = 20,000 Hz)
    TIM2_PSC   = 0U;
    TIM2_ARR   = PWM_ARR_MAX;
    TIM2_CCR1  = 0U; // 부팅 시 완전 정지!
    TIM2_CCMR1 = (0x6U << 4) | (1U << 3); // PWM Mode 1 + Preload
    TIM2_CCER  = (1U << 0);               // CH1 출력 Enable
    TIM2_CR1   = (1U << 0);               // TIM2 시작
}

static void motor_drive(uint8_t dir, uint8_t duty_pct)
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
}

static void motor_stop(void)
{
    TIM2_CCR1 = 0U;
    g_motor_running = 0U;
}

static uint8_t is_direction_blocked_by_limit(uint8_t dir)
{
    if (!g_soft_limit_on) return 0U;
    int32_t limit_x10 = SOFT_LIMIT_MM * 10;
    if (dir && (g_cart_pos_mm_x10 >= limit_x10))  return 1U;
    if (!dir && (g_cart_pos_mm_x10 <= -limit_x10)) return 1U;
    return 0U;
}

static void motor_kick(uint8_t dir, uint8_t duty_pct, uint32_t duration_ms)
{
    uint8_t target_dir = dir ? 1U : 0U;
    if (is_direction_blocked_by_limit(target_dir)) {
        motor_stop();
        g_soft_limit_trip = 1U;
        printf("\r\n>>> [SOFT LIMIT] Blocked %s! Pos=",
               target_dir ? "RIGHT" : "LEFT");
        print_fixed_1(g_cart_pos_mm_x10);
        printf(" mm (Limit +/-%d mm)\r\n", SOFT_LIMIT_MM);
        return;
    }

    motor_drive(target_dir, duty_pct);
    g_motor_stop_at = g_system_millis + duration_ms;
    g_motor_running = 1U;
}

static void encoder_tim3_init(void)
{
    RCC_AHB2ENR  |= (1U << 2); // GPIOCEN
    RCC_APB1ENR1 |= (1U << 1); // TIM3EN

    // PC6 (CN10-4, TIM3_CH1) & PC7 (D9, TIM3_CH2) -> AF2, Pull-Up/Down 없음 (5V FT 보호)
    GPIOC_MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOC_MODER |=  ((2U << (6 * 2)) | (2U << (7 * 2)));
    GPIOC_PUPDR &= ~((3U << (6 * 2)) | (3U << (7 * 2))); // 00: No Pull-Up/Down (5V FT 필수)
    GPIOC_AFRL  &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOC_AFRL  |=  ((0x2U << (6 * 4)) | (0x2U << (7 * 4)));

    TIM3_CR1   = 0U;
    TIM3_PSC   = 0U;
    TIM3_ARR   = 0xFFFFU; // 16-bit 전체 범위 자유 회전 카운트

    // TI1/TI2 입력 매핑(CC1S=01, CC2S=01) + 디지털 노이즈 필터(IC1F=6, IC2F=6)
    TIM3_CCMR1 = (1U << 0) | (6U << 4) | (1U << 8) | (6U << 12);

    // CC1P=1: TI1 극성 반전 -> 오른쪽(f, PH=1) 이동 시 카운트 양수(+) 증가 기준
    TIM3_CCER  = (1U << 0) | (1U << 1) | (1U << 4);

    // SMS=011 (Mode 3): 4체배 직교 인코더 모드
    TIM3_SMCR  = 3U;

    TIM3_CNT   = 0U;
    TIM3_CR1   = (1U << 0);
}

static void encoder_set_counts_atomic(int32_t new_counts, int32_t new_total_counts, int8_t new_sign)
{
    __asm volatile ("cpsid i");
    if (new_total_counts > 0) {
        g_enc_total_counts = new_total_counts;
    }
    g_enc_sign         = new_sign;
    g_cart_counts      = new_counts;
    s_prev_cnt_1ms     = (uint16_t)TIM3_CNT;
    s_prev_counts_10ms = new_counts;
    g_cart_pos_mm_x10  = (g_cart_counts * (RAIL_TRAVEL_MM * 10)) / g_enc_total_counts;
    s_cart_vel_q4      = 0;
    g_cart_vel_mms_x10 = 0;
    g_soft_limit_trip  = 0U;
    __asm volatile ("cpsie i");
}

static void encoder_zero(void)
{
    encoder_set_counts_atomic(0, g_enc_total_counts, g_enc_sign);
}

static void adc1_pa1_init(void)
{
    RCC_AHB2ENR |= (1U << 0) | (1U << 13); // GPIOAEN, ADC12EN

    GPIOA_MODER |=  (3U << (1 * 2));
    GPIOA_PUPDR &= ~(3U << (1 * 2));

    ADC12_CCR &= ~(3U << 16);
    ADC12_CCR |=  (1U << 16); // HCLK/1 동기 클록 (16MHz)

    ADC1_CR &= ~(1U << 29); // DEEPPWD = 0
    ADC1_CR |=  (1U << 28); // ADVREGEN = 1
    delay_ms(2);

    ADC1_CR &= ~(1U << 30); // ADCALDIF = 0
    ADC1_CR |=  (1U << 31); // ADCAL = 1
    while (ADC1_CR & (1U << 31));

    ADC1_ISR = (1U << 0);
    ADC1_CR |= (1U << 0); // ADEN = 1
    while (!(ADC1_ISR & (1U << 0)));

    ADC1_CFGR  = 0U;
    ADC1_SMPR1 &= ~(7U << 6);
    ADC1_SMPR1 |=  (4U << 6); // SMP2 = 100 (47.5 cycles)
    ADC1_SQR1  =  (2U << 6); // SQ1 = CH2 (PA1)
}

static uint16_t adc1_read_pa1_single(void)
{
    uint32_t timeout = 10000U;
    ADC1_CR |= (1U << 2);
    while (!(ADC1_ISR & (1U << 2)) && (--timeout > 0U));
    return (uint16_t)(ADC1_DR & 0x0FFFU);
}

static uint16_t adc1_read_pa1_avg(void)
{
    uint32_t sum = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        sum += adc1_read_pa1_single();
    }
    return (uint16_t)(sum >> 3);
}

static int32_t angle_adc_to_cdeg(uint16_t adc_raw)
{
    return ANGLE_SIGN * (((int32_t)adc_raw - g_angle_zero_adc) * 18000) / g_pot_180_counts;
}

static void angle_reseed_filter(uint16_t current_adc)
{
    g_angle_adc_raw   = current_adc;
    g_angle_cdeg      = angle_adc_to_cdeg(current_adc);
    g_prev_angle_cdeg = g_angle_cdeg;
    g_angle_vel_q4    = 0;
    g_angle_vel_x10   = 0;
}

/* ============================================================================
 * 4. 인터럽트 핸들러
 * ========================================================================== */
void TIM6_DACUNDER_IRQHandler(void)
{
    static uint8_t s_div_10ms = 0;

    if (TIM6_SR & (1U << 0)) {
        TIM6_SR &= ~(1U << 0);
        g_system_millis++;

        // (1) 1ms 단위 32-bit 확장 엔코더 위치 갱신 (부호 보정 + 오버플로우 안전)
        uint16_t curr_cnt = (uint16_t)TIM3_CNT;
        int16_t delta_1ms = (int16_t)(curr_cnt - s_prev_cnt_1ms);
        s_prev_cnt_1ms = curr_cnt;
        g_cart_counts += (int32_t)g_enc_sign * (int32_t)delta_1ms;
        g_cart_pos_mm_x10 = (g_cart_counts * (RAIL_TRAVEL_MM * 10)) / g_enc_total_counts;

        // (2) 1ms 고속 Soft Limit 레일 보호 & 250ms 펄스 킥 자동 정지
        if (g_motor_running) {
            if (is_direction_blocked_by_limit(g_motor_dir)) {
                motor_stop();
                g_soft_limit_trip = 1U;
            } else if ((int32_t)(g_system_millis - g_motor_stop_at) >= 0) {
                motor_stop();
            }
        }

        // (3) 10ms(100Hz) 속도 추정(EMA LPF alpha=0.25) 및 ADC 샘플링 트리거
        s_div_10ms++;
        if (s_div_10ms >= 10U) {
            s_div_10ms = 0U;

            int32_t delta_10ms = g_cart_counts - s_prev_counts_10ms;
            s_prev_counts_10ms = g_cart_counts;

            int32_t raw_vel_x10 = (delta_10ms * (RAIL_TRAVEL_MM * 1000)) / g_enc_total_counts;
            s_cart_vel_q4 = ((raw_vel_x10 << 4) + 3 * s_cart_vel_q4) >> 2;
            g_cart_vel_mms_x10 = (s_cart_vel_q4 + ((s_cart_vel_q4 >= 0) ? 8 : -8)) >> 4;

            g_tick_10ms = 1U;
        }
    }
}

void USART2_IRQHandler(void)
{
    if (USART2_ISR & (1U << 3)) {
        USART2_ICR = (1U << 3); // ORE 방어
    }
    if (USART2_ISR & (1U << 5)) { // RXNE
        g_rx_cmd = (uint8_t)USART2_RDR;
    }
}

/* ============================================================================
 * 5. 가이드 캘리브레이션 & 자동 중앙 복귀 핸들러
 * ========================================================================== */
static void handle_cart_homing_key(void)
{
    if (g_home_state == HOME_IDLE) {
        motor_stop();
        g_soft_limit_on = 0U; // 끝단 수동 이동 중 리미트 간섭 방지
        g_home_state = HOME_WAIT_RIGHT;
        printf("\r\n==================================================================\r\n");
        printf("  [CART HOMING 1/2] 카트를 손으로 [오른쪽 끝]까지 밀고\r\n");
        printf("                    터미널에서 'h' 를 한 번 더 누르세요. (취소: s)\r\n");
        printf("==================================================================\r\n");
    } else if (g_home_state == HOME_WAIT_RIGHT) {
        g_home_right_cnt = g_cart_counts;
        g_home_state = HOME_WAIT_LEFT;
        printf("\r\n>>> [1/2 완료] 오른쪽 끝 엔코더 기록: %ld\r\n", (long)g_home_right_cnt);
        printf("  [CART HOMING 2/2] 이제 카트를 손으로 [왼쪽 끝]까지 밀고\r\n");
        printf("                    터미널에서 'h' 를 다시 누르세요. (취소: s)\r\n");
    } else if (g_home_state == HOME_WAIT_LEFT) {
        g_home_left_cnt = g_cart_counts;
        int32_t diff = g_home_right_cnt - g_home_left_cnt;
        int8_t  new_sign = g_enc_sign;
        int32_t span = diff;

        if (diff < 0) {
            // 오른쪽이 왼쪽보다 작았다면 부호 반전으로 자동 정규화
            new_sign = (int8_t)(-g_enc_sign);
            span = -diff;
        }

        if (span < 1000) {
            g_home_state = HOME_IDLE;
            g_soft_limit_on = 1U;
            printf("\r\n>>> [HOMING 실패] 양 끝단 간격(%ld counts)이 너무 작습니다. 다시 시도하세요.\r\n",
                   (long)span);
            return;
        }

        // 현재 카트는 왼쪽 끝에 있으므로 중앙(0) 기준 위치 = -(span / 2)로 설정!
        int32_t left_origin_cnt = -(span / 2);
        encoder_set_counts_atomic(left_origin_cnt, span, new_sign);

        printf("\r\n>>> [2/2 완료] 실측 스트로크 = %ld counts (~%d mm), Sign = %+d\r\n",
               (long)span, RAIL_TRAVEL_MM, (int)new_sign);
        printf(">>> [AUTO CENTER] 중앙(Pos = 0.0 mm)으로 자동 이동합니다... (비상 정지: s)\r\n\r\n");

        g_home_center_start = g_system_millis;
        g_home_state = HOME_AUTO_CENTER;
    }
}

static void cart_auto_center_step(void)
{
    if (g_home_state != HOME_AUTO_CENTER) return;

    int32_t err = -g_cart_counts; // 목표 0 - 현재 위치
    int32_t abs_err = (err >= 0) ? err : -err;

    if (abs_err <= HOME_CENTER_BAND_CNT) {
        motor_stop();
        g_home_state = HOME_IDLE;
        g_soft_limit_on = 1U;
        printf("\r\n>>> [HOMING 완료] 중앙 위치 초기화 성공! Pos = ");
        print_fixed_1(g_cart_pos_mm_x10);
        printf(" mm (Soft Limit ON)\r\n\r\n");
        return;
    }

    if ((g_system_millis - g_home_center_start) >= HOME_CENTER_TIMEOUT_MS) {
        motor_stop();
        g_home_state = HOME_IDLE;
        g_soft_limit_on = 1U;
        printf("\r\n>>> [HOMING 타임아웃] %ums 경과로 안전 정지! Pos = ", HOME_CENTER_TIMEOUT_MS);
        print_fixed_1(g_cart_pos_mm_x10);
        printf(" mm\r\n\r\n");
        return;
    }

    // 비례 + 정지 마찰력(Stiction) 보상 듀티: 40% ~ 52%
    int32_t half_span = g_enc_total_counts / 2;
    if (half_span < 1) half_span = 1;
    uint8_t duty = (uint8_t)(40 + ((abs_err * 12) / half_span));
    if (duty > 52U) duty = 52U;

    uint8_t dir = (err > 0) ? 1U : 0U;
    motor_drive(dir, duty);
}

static void handle_angle_cal_key(void)
{
    if (g_acal_state == ACAL_IDLE) {
        g_acal_state = ACAL_WAIT_DOWN;
        printf("\r\n==================================================================\r\n");
        printf("  [ANGLE CAL 1/2] 진자가 아래(6시)에서 자연스럽게 정지하면\r\n");
        printf("                  터미널에서 'a' 를 한 번 더 누르세요. (취소: s)\r\n");
        printf("==================================================================\r\n");
    } else if (g_acal_state == ACAL_WAIT_DOWN) {
        g_angle_down_adc = adc1_read_pa1_avg();
        g_acal_state = ACAL_WAIT_UP;
        printf("\r\n>>> [1/2 완료] 6시 정지 ADC 저장: %u\r\n", (unsigned int)g_angle_down_adc);
        printf("  [ANGLE CAL 2/2] 이제 진자를 손으로 위(12시) 수직에 맞춘 뒤\r\n");
        printf("                  터미널에서 'a' 를 다시 누르세요. (취소: s)\r\n");
    } else if (g_acal_state == ACAL_WAIT_UP) {
        uint16_t up_adc = adc1_read_pa1_avg();
        int32_t  diff   = (int32_t)up_adc - (int32_t)g_angle_down_adc;
        int32_t  span   = (diff >= 0) ? diff : -diff;

        if (span < 1200 || span > 2800) {
            g_acal_state = ACAL_IDLE;
            printf("\r\n>>> [ANGLE CAL 실패] 6시(%u)와 12시(%u) 차이(%ld)가 비정상입니다.\r\n",
                   (unsigned int)g_angle_down_adc, (unsigned int)up_adc, (long)span);
            printf("                     데드존 통과 여부 확인 후 다시 시도하세요.\r\n\r\n");
            return;
        }

        g_pot_up_dir_sign = (diff >= 0) ? 1 : -1;
        g_pot_180_counts  = span;
        g_angle_zero_adc  = (int32_t)up_adc;
        g_acal_state      = ACAL_IDLE;
        angle_reseed_filter(up_adc);

        printf("\r\n>>> [ANGLE CAL 완료] 6시=%u | 12시=%ld -> 실측 180deg = %ld counts\r\n",
               (unsigned int)g_angle_down_adc, (long)g_angle_zero_adc, (long)g_pot_180_counts);
        printf("                     (이후 6시 정지 상태에서 'd'만 눌러도 자동 12시 영점 복원!)\r\n\r\n");
    }
}

/* ============================================================================
 * 6. 메인 루프 (100Hz 각도 추정 + 5Hz 텔레메트리 + 키보드 조종/초기화)
 * ========================================================================== */
int main(void)
{
    fpu_enable();
    tim6_init();
    uart2_init();
    motor_driver_init();
    encoder_tim3_init();
    adc1_pa1_init();

    angle_reseed_filter(adc1_read_pa1_avg());

    printf("\r\n==================================================================\r\n");
    printf("  STM32G474RE Inverted Pendulum Project                           \r\n");
    printf("  [Step 06 / CH-10] TIM3 4x Encoder + ADC1 Angle + Homing Wizard  \r\n");
    printf("  Sensors : PC6/PC7 (TIM3 5V FT) | PA1 (ADC1_IN2 WDD35D4-5K)      \r\n");
    printf("  Safety  : +/-%d mm Rail Soft Limit (Auto Block Outward Motion)  \r\n", SOFT_LIMIT_MM);
    printf("------------------------------------------------------------------\r\n");
    printf("  Drive   : f/b = Right/Left 250ms | s/Space = STOP | 1~6 = Duty  \r\n");
    printf("  Homing  : h = Cart Guided Homing (Right -> Left -> Auto Center) \r\n");
    printf("            a = Pendulum 2-Point Cal (6h Down -> 12h Up)          \r\n");
    printf("            d = Quick 6h Zero | u = Quick 12h Zero | z = Zero Pos \r\n");
    printf("            l = Toggle Soft Limit (Currently ON)                \r\n");
    printf("==================================================================\r\n\r\n");

    uint32_t last_print = 0U;

    while (1)
    {
        // (A) Soft Limit 긴급 차단 이벤트 즉시 알림
        if (g_soft_limit_trip) {
            g_soft_limit_trip = 0U;
            printf("\r\n>>> [SOFT LIMIT TRIGGERED] Motor cut at ");
            print_fixed_1(g_cart_pos_mm_x10);
            printf(" mm! Move opposite dir or press 'z'.\r\n");
        }

        // (B) 10ms (100Hz) 진자 각도·각속도 갱신 및 카트 자동 중앙 복귀 제어
        if (g_tick_10ms) {
            g_tick_10ms = 0U;

            g_angle_adc_raw = adc1_read_pa1_avg();
            g_angle_cdeg = angle_adc_to_cdeg(g_angle_adc_raw);

            int32_t raw_dtheta_x10 = (g_angle_cdeg - g_prev_angle_cdeg) * 10;
            g_prev_angle_cdeg = g_angle_cdeg;

            g_angle_vel_q4 = ((raw_dtheta_x10 << 4) + 3 * g_angle_vel_q4) >> 2;
            g_angle_vel_x10 = (g_angle_vel_q4 + ((g_angle_vel_q4 >= 0) ? 8 : -8)) >> 4;

            cart_auto_center_step();
        }

        // (C) USART2 키보드 명령 처리
        if (g_rx_cmd != 0U) {
            uint8_t cmd = g_rx_cmd;
            g_rx_cmd = 0U;

            if (cmd == 's' || cmd == 'S' || cmd == ' ') {
                motor_stop();
                if (g_home_state != HOME_IDLE || g_acal_state != ACAL_IDLE) {
                    g_home_state = HOME_IDLE;
                    g_acal_state = ACAL_IDLE;
                    g_soft_limit_on = 1U;
                    printf("\r\n>>> [ABORT] 비상 정지 및 캘리브레이션 마법사 취소 (Soft Limit ON)\r\n\r\n");
                } else {
                    printf("\r\n>>> [STOP] Emergency Stop! (CCR1=0)\r\n");
                }
            } else if (cmd == 'h' || cmd == 'H') {
                handle_cart_homing_key();
            } else if (cmd == 'a' || cmd == 'A') {
                handle_angle_cal_key();
            } else if (g_home_state != HOME_IDLE || g_acal_state != ACAL_IDLE) {
                // 마법사 진행 중 오조작 방지 (ENTER 키도 현재 단계 진행으로 허용)
                if (cmd == '\r' || cmd == '\n') {
                    if (g_home_state == HOME_WAIT_RIGHT || g_home_state == HOME_WAIT_LEFT) {
                        handle_cart_homing_key();
                    } else if (g_acal_state == ACAL_WAIT_DOWN || g_acal_state == ACAL_WAIT_UP) {
                        handle_angle_cal_key();
                    }
                }
            } else if (cmd == 'f' || cmd == 'F') {
                printf("\r\n>>> [KICK RIGHT] Dir=1 (PH=1), Duty=%u%% (250ms)\r\n", g_test_duty_pct);
                motor_kick(1U, g_test_duty_pct, 250U);
            } else if (cmd == 'b' || cmd == 'B') {
                printf("\r\n>>> [KICK LEFT]  Dir=0 (PH=0), Duty=%u%% (250ms)\r\n", g_test_duty_pct);
                motor_kick(0U, g_test_duty_pct, 250U);
            } else if (cmd >= '1' && cmd <= '6') {
                const uint8_t duties[6] = {15, 25, 35, 50, 65, 80};
                g_test_duty_pct = duties[cmd - '1'];
                printf("\r\n>>> [DUTY SET] Test Duty = %u%% (CCR1=%u/800)\r\n",
                       g_test_duty_pct,
                       (unsigned int)(((PWM_ARR_MAX + 1U) * g_test_duty_pct) / 100U));
            } else if (cmd == 'z' || cmd == 'Z') {
                encoder_zero();
                printf("\r\n>>> [ZERO CART] TIM3 Encoder Reset -> Pos = +0.0 mm\r\n");
            } else if (cmd == 'd' || cmd == 'D') {
                g_angle_down_adc = adc1_read_pa1_avg();
                g_angle_zero_adc = (int32_t)g_angle_down_adc + (g_pot_up_dir_sign * g_pot_180_counts);
                angle_reseed_filter(g_angle_down_adc);
                printf("\r\n>>> [6h ZERO] 6시 정지 ADC=%u 저장 -> 12시 영점(%ld) 자동 산출 (현재 Angle=",
                       (unsigned int)g_angle_down_adc, (long)g_angle_zero_adc);
                print_fixed_2(g_angle_cdeg);
                printf(" deg)\r\n");
            } else if (cmd == 'u' || cmd == 'U') {
                uint16_t up_adc = adc1_read_pa1_avg();
                g_angle_zero_adc = (int32_t)up_adc;
                angle_reseed_filter(up_adc);
                printf("\r\n>>> [12h ZERO] Upright Reference Calibrated: ADC = %ld -> +0.00 deg\r\n",
                       (long)g_angle_zero_adc);
            } else if (cmd == 'l' || cmd == 'L') {
                g_soft_limit_on = g_soft_limit_on ? 0U : 1U;
                printf("\r\n>>> [SOFT LIMIT] Protection %s (+/-%d mm)\r\n",
                       g_soft_limit_on ? "ENABLED" : "DISABLED (Caution!)",
                       SOFT_LIMIT_MM);
            }
        }

        // (D) 200ms (5Hz) 텔레메트리 출력 (안내 프롬프트 대기 중에는 화면 가독성을 위해 일시 정지)
        if ((g_home_state == HOME_IDLE || g_home_state == HOME_AUTO_CENTER) &&
            (g_acal_state == ACAL_IDLE) &&
            ((g_system_millis - last_print) >= 200U)) {
            last_print = g_system_millis;

            printf("Pos: ");
            print_fixed_1(g_cart_pos_mm_x10);
            printf(" mm (CNT:%+6ld) | Vel: ", (long)g_cart_counts);
            print_fixed_1(g_cart_vel_mms_x10);
            printf(" mm/s | Angle: ");
            print_fixed_2(g_angle_cdeg);
            printf(" deg (ADC:%4u) | dTh: ", (unsigned int)g_angle_adc_raw);
            print_fixed_1(g_angle_vel_x10);
            printf(" deg/s | Lim:%s\r\n", g_soft_limit_on ? "ON " : "OFF");
        }
    }
}

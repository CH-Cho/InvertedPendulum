/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : NUCLEO-G474RE On-board LED (PA5) Blink Tutorial
 ******************************************************************************
 */

#include <stdint.h>

/* 레지스터 주소 매핑 (STM32G474 레퍼런스 매뉴얼 RM0440 기준) */
// 1. Reset and Clock Control (RCC)
#define RCC_BASE        0x40021000U
#define RCC_AHB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x4CU))

// 2. GPIO Port A
#define GPIOA_BASE      0x48000000U
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
#define GPIOA_ODR       (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))

/* 간단한 딜레이 함수 */
static void delay(volatile uint32_t count)
{
    while (count--) {
        __asm__("nop");
    }
}

int main(void)
{
    // [1단계] GPIOA 주변장치 클럭 활성화 (RCC AHB2ENR 비트 0 = GPIOAEN)
    RCC_AHB2ENR |= (1U << 0);

    // [2단계] PA5 (녹색 LED LD2) 핀을 범용 출력(Output, 01) 모드로 설정
    // MODER 레지스터는 핀당 2비트 사용 (비트 11:10)
    GPIOA_MODER &= ~(3U << (5 * 2)); // 비트 11:10 클리어
    GPIOA_MODER |=  (1U << (5 * 2)); // 01 (General purpose output) 설정

    // [3단계] 무한 루프에서 LED 토글 및 지연
    while (1)
    {
        GPIOA_ODR ^= (1U << 5); // PA5 출력 반전 (High <-> Low)
        delay(2000000);         // 약 0.5초 대기 (기본 HSI 16MHz 기준)
    }
}

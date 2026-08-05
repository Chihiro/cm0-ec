/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : RGB LED control using STM32 LL library
 *
 *   Hardware:  STM32L051C8T6
 *   RGB LED:   3535 common anode
 *              PA4 = LED_R (Red cathode)
 *              PA5 = LED_G (Green cathode)
 *              PA6 = LED_B (Blue cathode)
 *
 *   Common anode connected to MCU_VDD:
 *     GPIO LOW  -> LED ON  (current sinks through GPIO)
 *     GPIO HIGH -> LED OFF
 *
 *   Behavior:
 *     Red (1s) -> Green (1s) -> Blue (1s) -> cycle forever
 ******************************************************************************
 */

#include "stm32l0xx_conf.h"
#include <stdbool.h>

/* Pin definitions -----------------------------------------------------------*/
#define LED_R_PIN          LL_GPIO_PIN_4   /* PA4 */
#define LED_G_PIN          LL_GPIO_PIN_5   /* PA5 */
#define LED_B_PIN          LL_GPIO_PIN_6   /* PA6 */
#define LED_ALL_PINS       (LED_R_PIN | LED_G_PIN | LED_B_PIN)
#define LED_GPIO_PORT      GPIOA

#define CYCLE_INTERVAL_MS  1000U

/* ---------------------------------------------------------------------------*/
/**
 * @brief  System Clock Configuration
 *         HSI16 (16 MHz internal RC) as system clock
 */
static void SystemClock_Config(void)
{
    /* Set flash latency: 0 wait state for 16 MHz in Range 1 */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

    /* Enable HSI16 oscillator */
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() == 0U) {
        /* Wait until HSI is ready */
    }

    /* Switch system clock to HSI16 */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
        /* Wait until HSI is used as system clock */
    }

    /* Update SystemCoreClock (used by LL_Init1msTick) */
    SystemCoreClock = 16000000U;

    /* Configure SysTick for 1 ms timebase (polling, no interrupt) */
    LL_Init1msTick(SystemCoreClock);
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  GPIO Initialization
 *         PA4, PA5, PA6 as push-pull outputs, initially HIGH (LEDs OFF)
 *
 *         For common-anode RGB LED:
 *           GPIO LOW  = LED ON
 *           GPIO HIGH = LED OFF
 */
static void LED_GPIO_Init(void)
{
    /* Enable GPIOA clock (STM32L0 uses IOP bus for GPIO) */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /* Write HIGH first, then configure as output — prevents glitches */
    LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_ALL_PINS);

    /* Configure PA4/PA5/PA6 as push-pull outputs */
    LL_GPIO_SetPinMode(LED_GPIO_PORT, LED_R_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LED_GPIO_PORT, LED_G_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LED_GPIO_PORT, LED_B_PIN, LL_GPIO_MODE_OUTPUT);

    /* No pull-up/pull-down (external current-limiting resistors on cathodes) */
    LL_GPIO_SetPinPull(LED_GPIO_PORT, LED_R_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(LED_GPIO_PORT, LED_G_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(LED_GPIO_PORT, LED_B_PIN, LL_GPIO_PULL_NO);
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  Turn all three LED channels off
 */
static void LED_AllOff(void)
{
    LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_ALL_PINS);   /* HIGH = OFF (common anode) */
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  Set red LED state
 * @param  on: true = ON (GPIO LOW), false = OFF (GPIO HIGH)
 */
static void LED_Red(bool on)
{
    if (on) {
        LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_R_PIN); /* LOW = ON */
    } else {
        LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_R_PIN);   /* HIGH = OFF */
    }
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  Set green LED state
 * @param  on: true = ON (GPIO LOW), false = OFF (GPIO HIGH)
 */
static void LED_Green(bool on)
{
    if (on) {
        LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_G_PIN); /* LOW = ON */
    } else {
        LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_G_PIN);   /* HIGH = OFF */
    }
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  Set blue LED state
 * @param  on: true = ON (GPIO LOW), false = OFF (GPIO HIGH)
 */
static void LED_Blue(bool on)
{
    if (on) {
        LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_B_PIN); /* LOW = ON */
    } else {
        LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_B_PIN);   /* HIGH = OFF */
    }
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  Main entry point
 *         Cycles RGB LED: Red -> Green -> Blue -> Red -> ...
 *         Each color stays on for 1 second.
 */
int main(void)
{
    /* Initialize system clock (HSI 16 MHz) and SysTick */
    SystemClock_Config();

    /* Initialize GPIO for RGB LED */
    LED_GPIO_Init();

    /* Main loop: cycle through colors */
    for (;;) {
        /* Red */
        LED_AllOff();
        LED_Red(true);
        LL_mDelay(CYCLE_INTERVAL_MS);

        /* Green */
        LED_AllOff();
        LED_Green(true);
        LL_mDelay(CYCLE_INTERVAL_MS);

        /* Blue */
        LED_AllOff();
        LED_Blue(true);
        LL_mDelay(CYCLE_INTERVAL_MS);
    }
}

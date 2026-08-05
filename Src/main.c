/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : RGB LED control + USART2 logging using STM32 LL library
 *
 *   Hardware:  STM32L051C8T6
 *   RGB LED:   3535 common anode
 *              PA4 = LED_R (Red cathode)
 *              PA5 = LED_G (Green cathode)
 *              PA6 = LED_B (Blue cathode)
 *   USART2:    PA2 = TX (AF4), PA3 = RX (AF4) — 115200-8-N-1
 *
 *   Common anode connected to MCU_VDD:
 *     GPIO LOW  -> LED ON  (current sinks through GPIO)
 *     GPIO HIGH -> LED OFF
 *
 *   Behavior:
 *     Red (1s) -> Green (1s) -> Blue (1s) -> cycle forever
 *     Prints color name to USART2 at each transition.
 ******************************************************************************
 */

#include "stm32l0xx_conf.h"
#include <stdbool.h>
#include <stdio.h>

/* Pin definitions -----------------------------------------------------------*/
#define LED_R_PIN          LL_GPIO_PIN_4   /* PA4 */
#define LED_G_PIN          LL_GPIO_PIN_5   /* PA5 */
#define LED_B_PIN          LL_GPIO_PIN_6   /* PA6 */
#define LED_ALL_PINS       (LED_R_PIN | LED_G_PIN | LED_B_PIN)
#define LED_GPIO_PORT      GPIOA

#define USART2_TX_PIN      LL_GPIO_PIN_2   /* PA2, AF4 */
#define USART2_TX_AF       LL_GPIO_AF_4

#define CYCLE_INTERVAL_MS  1000U

/* ---------------------------------------------------------------------------*/
/**
 * @brief  System Clock Configuration
 *         HSI16 (16 MHz) → AHB/2 → HCLK 8 MHz
 *
 *         This saves digital power (~0.8 mA typ vs 1.4 mA at 16 MHz)
 *         while keeping the HSI oscillator for good USART baud rate accuracy.
 *
 *         USART2 at PCLK 8 MHz / 115200 bps:
 *           BRR = 69, actual 115942 bps, error 0.64%
 */
static void SystemClock_Config(void)
{
    /* Set flash latency: 0 wait state for ≤ 16 MHz in Range 1 */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

    /* Enable HSI16 oscillator */
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() == 0U) {
    }

    /* Switch system clock to HSI16 (SYSCLK = 16 MHz) */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
    }

    /* AHB prescaler /2: SYSCLK 16 MHz → HCLK 8 MHz */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);   /* PCLK1 = 8 MHz (USART2, I2C1) */
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);   /* PCLK2 = 8 MHz */

    /* HCLK = 8 MHz */
    SystemCoreClock = 8000000U;

    /* Configure SysTick for 1 ms timebase (polling, no interrupt) */
    LL_Init1msTick(SystemCoreClock);
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  GPIO Initialization
 *         PA4/PA5/PA6 as push-pull outputs (RGB LED, initially HIGH = OFF)
 *
 *         For common-anode RGB LED:
 *           GPIO LOW  = LED ON
 *           GPIO HIGH = LED OFF
 */
static void LED_GPIO_Init(void)
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /* Write HIGH first, then configure as output — prevents glitch flash */
    LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_ALL_PINS);

    LL_GPIO_SetPinMode(LED_GPIO_PORT, LED_R_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LED_GPIO_PORT, LED_G_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LED_GPIO_PORT, LED_B_PIN, LL_GPIO_MODE_OUTPUT);

    LL_GPIO_SetPinPull(LED_GPIO_PORT, LED_R_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(LED_GPIO_PORT, LED_G_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(LED_GPIO_PORT, LED_B_PIN, LL_GPIO_PULL_NO);
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  USART2 Initialization (TX only, 115200-8-N-1)
 *         PA2 = TX (AF4), no flow control, no interrupt.
 *
 *         PCLK = HCLK = 8 MHz → BRR = 69 → actual 115942 bps (0.64% error)
 */
static void USART2_Init(void)
{
    /* Enable USART2 + GPIOA peripheral clocks */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /* Configure PA2 as alternate function (USART2_TX, AF4) */
    LL_GPIO_SetPinMode(GPIOA, USART2_TX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, USART2_TX_PIN, USART2_TX_AF);
    LL_GPIO_SetPinSpeed(GPIOA, USART2_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);

    /* USART2 config: 115200-8-N-1, TX only (inline LL API) */
    /* IMPORTANT: set oversampling BEFORE baud rate */
    LL_USART_Disable(USART2);
    LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_16);
    LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, 115200U);
    LL_USART_SetDataWidth(USART2, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART2, LL_USART_STOPBITS_1);
    LL_USART_SetParity(USART2, LL_USART_PARITY_NONE);
    LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX);
    LL_USART_SetHWFlowCtrl(USART2, LL_USART_HWCONTROL_NONE);
    LL_USART_Enable(USART2);

    /* Wait for USART to be ready for transmission (TXE flag) */
    while (LL_USART_IsActiveFlag_TEACK(USART2) == 0U) {
    }
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  Low-level character output for printf() / puts() / putchar()
 *
 *         Called by the Picolibc/Newlib _write() syscall stub in syscall.c.
 *         Blocks until the TX data register is empty, then sends one byte.
 */
int __io_putchar(int ch)
{
    /* Wait until TX data register empty */
    while (LL_USART_IsActiveFlag_TXE(USART2) == 0U) {
    }
    LL_USART_TransmitData8(USART2, (uint8_t)(ch & 0xFFU));
    return ch;
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  Low-level character input stub (not used — USART2 is TX-only)
 */
int __io_getchar(void)
{
    return 0;
}

/* ---------------------------------------------------------------------------*/
/* LED helpers (common anode: LOW = ON, HIGH = OFF) -------------------------*/

static void LED_AllOff(void)
{
    LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_ALL_PINS);
}

static void LED_Red(bool on)
{
    if (on) {
        LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_R_PIN);
    } else {
        LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_R_PIN);
    }
}

static void LED_Green(bool on)
{
    if (on) {
        LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_G_PIN);
    } else {
        LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_G_PIN);
    }
}

static void LED_Blue(bool on)
{
    if (on) {
        LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_B_PIN);
    } else {
        LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_B_PIN);
    }
}

/* ---------------------------------------------------------------------------*/
/**
 * @brief  Main entry point
 *         Prints boot banner to USART2, then cycles RGB LED.
 */
int main(void)
{
    SystemClock_Config();
    USART2_Init();
    LED_GPIO_Init();

    printf("\r\n");
    printf("========================================\r\n");
    printf("  EC Power Management v3.5\r\n");
    printf("  STM32L051C8T6 | HSI 8 MHz (AHB/2)\r\n");
    printf("  USART2: 115200-8-N-1\r\n");
    printf("========================================\r\n\n");

    uint32_t cycle = 0U;

    for (;;) {
        cycle++;

        /* Red */
        LED_AllOff();
        LED_Red(true);
        printf("[%4lu] LED: RED\r\n", (unsigned long)cycle);
        LL_mDelay(CYCLE_INTERVAL_MS);

        /* Green */
        LED_AllOff();
        LED_Green(true);
        printf("[%4lu] LED: GREEN\r\n", (unsigned long)cycle);
        LL_mDelay(CYCLE_INTERVAL_MS);

        /* Blue */
        LED_AllOff();
        LED_Blue(true);
        printf("[%4lu] LED: BLUE\r\n", (unsigned long)cycle);
        LL_mDelay(CYCLE_INTERVAL_MS);
    }
}

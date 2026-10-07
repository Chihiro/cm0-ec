/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : EC Power Management v8.0 — entry point.
 *
 *   MCU:  STM32L051C8T6
 *   Clock: HSI16 / 2 = 8 MHz HCLK/PCLK1
 *   USART2: PA2 TX, 115200-8-N-1 (debug output)
 ******************************************************************************
 */

#include "stm32l0xx_conf.h"
#include "ec_platform.h"
#include "ec_state_machine.h"
#include "oled_display.h"
#include <stdio.h>

int main(void)
{
    /*
     * EC_Init() internally calls EC_Platform_Init() which sets up
     * clock → GPIO → SysTick → USART2 → I2C1 → I2C2 slave, then gauge BOOT.
     * printf works after USART2 init inside EC_Platform_Init().
     */
    /* Initialize platform and USART2 before any printf. */
    EC_Init();

    printf("\r\n");
    printf("========================================\r\n");
    printf("  EC Power Management v8.0\r\n");
    printf("  STM32L051C8T6 | HSI16/2, HCLK/PCLK1 8 MHz\r\n");
    printf("  USART2: 115200-8-N-1\r\n");
    printf("========================================\r\n\n");

    printf("[BOOT] EC_Init complete. Entering main loop.\r\n\n");

    /*
     * OLED bring-up (0.91" 128x32 SSD1306 on shared I2C1, addr 0x3C).
     * Draws the dashboard immediately; short-press cycles two pages.
     * Before STOP, the panel and charge pump are powered down. SSD1306 RAM
     * retains the image, which is shown again after wake-up.
     */
#if EC_ENABLE_OLED
    {
        ec_status_t st;
        EC_GetStatus(&st);
        printf("[OLED] Initializing u8g2 2-page display...\r\n");
        if (OLED_Display_Init(&st)) {
            printf("[OLED] OK: dashboard | electrical details\r\n");
        } else {
            printf("[OLED] FAIL: no ACK at 0x3C (check wiring/pull-ups)\r\n");
        }
    }
#else
    printf("[OLED] Disabled at build time\r\n");
#endif

    /* Main loop: never blocks (except STOP which is synchronous sleep) */
    for (;;) {
        EC_Task();
    }
}

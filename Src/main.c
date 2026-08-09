/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : EC Power Management v8.0 — entry point.
 *
 *   MCU:  STM32L051C8T6
 *   Clock: MSI 2.097 MHz
 *   USART2: PA2 TX, 115200-8-N-1 (debug output)
 ******************************************************************************
 */

#include "stm32l0xx_conf.h"
#include "ec_platform.h"
#include "ec_state_machine.h"
#include <stdio.h>

int main(void)
{
    /*
     * EC_Init() internally calls EC_Platform_Init() which sets up
     * clock → GPIO → SysTick → USART2 → I2C1, then runs gauge BOOT.
     * printf works after USART2 init inside EC_Platform_Init().
     */
    printf("\r\n");
    printf("========================================\r\n");
    printf("  EC Power Management v8.0\r\n");
    printf("  STM32L051C8T6 | MSI 2.097 MHz\r\n");
    printf("  USART2: 115200-8-N-1\r\n");
    printf("========================================\r\n\n");

    /* One-time initialization: platform + gauge BOOT + enter ACTIVE_IDLE */
    printf("[BOOT] Starting EC_Init...\r\n");
    EC_Init();
    printf("[BOOT] EC_Init complete. Entering main loop.\r\n\n");

    /* Main loop: never blocks (except STOP which is synchronous sleep) */
    for (;;) {
        EC_Task();
    }
}

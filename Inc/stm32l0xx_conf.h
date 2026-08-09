/**
 ******************************************************************************
 * @file    stm32l0xx_conf.h
 * @brief   LL library configuration for STM32L051 RGB LED project.
 *
 *          Common anode: GPIO LOW = LED ON, GPIO HIGH = LED OFF
 *          PA4 = LED_R, PA5 = LED_G, PA6 = LED_B
 ******************************************************************************
 */

#ifndef __STM32L0xx_CONF_H
#define __STM32L0xx_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_i2c.h"
#include "stm32l0xx_ll_exti.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __STM32L0xx_CONF_H */

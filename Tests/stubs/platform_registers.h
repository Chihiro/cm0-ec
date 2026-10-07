/* Use the real CMSIS register types and ST LL functions with registers in RAM.
 * Replace the CPU instructions/interrupt delivery; no electrical simulation. */
#ifndef TEST_PLATFORM_REGISTERS_H
#define TEST_PLATFORM_REGISTERS_H

#include "stm32l051xx.h"
#include <stdbool.h>

extern GPIO_TypeDef test_gpioa, test_gpiob, test_gpioc;
extern I2C_TypeDef test_i2c1, test_i2c2;
extern RCC_TypeDef test_rcc;
extern PWR_TypeDef test_pwr;
extern FLASH_TypeDef test_flash;
extern USART_TypeDef test_usart2;
extern SYSCFG_TypeDef test_syscfg;
extern EXTI_TypeDef test_exti;
extern SysTick_Type test_systick;
extern SCB_Type test_scb;

/* Redefine peripheral addresses before including LL inline functions. */
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef I2C2
#undef I2C1
#undef RCC
#undef PWR
#undef FLASH
#undef USART2
#undef SYSCFG
#undef EXTI
#undef SysTick
#undef SCB
#define GPIOA (&test_gpioa)
#define GPIOB (&test_gpiob)
#define GPIOC (&test_gpioc)
#define I2C2 (&test_i2c2)
#define I2C1 (&test_i2c1)
#define RCC (&test_rcc)
#define PWR (&test_pwr)
#define FLASH (&test_flash)
#define USART2 (&test_usart2)
#define SYSCFG (&test_syscfg)
#define EXTI (&test_exti)
#define SysTick (&test_systick)
#define SCB (&test_scb)

uint32_t Test_GetPrimask(void);
void Test_DisableIRQ(void);
void Test_SetPrimask(uint32_t value);
void Test_WFI(void);
void Test_DSB(void);
void Test_ClearEXTI(uint32_t lines);
void Test_EnableIRQ(IRQn_Type irq);
void Test_DisableNVICIRQ(IRQn_Type irq);
void Test_ClearPendingIRQ(IRQn_Type irq);
void Test_SetPriority(IRQn_Type irq, uint32_t priority);
uint32_t Test_GetEnableIRQ(IRQn_Type irq);
void Test_SystemReset(void);

#undef __WFI
#undef __NOP
#define __get_PRIMASK() Test_GetPrimask()
#define __disable_irq() Test_DisableIRQ()
#define __set_PRIMASK(value) Test_SetPrimask(value)
#define __WFI() Test_WFI()
#define __DSB() Test_DSB()
#define __DMB() ((void)0)
#define __ISB() ((void)0)
#define __NOP() ((void)0)

#undef NVIC_EnableIRQ
#undef NVIC_DisableIRQ
#undef NVIC_ClearPendingIRQ
#undef NVIC_SetPriority
#undef NVIC_SystemReset
#undef NVIC_GetEnableIRQ
#undef NVIC_GetPendingIRQ
#define NVIC_EnableIRQ(irq) Test_EnableIRQ(irq)
#define NVIC_DisableIRQ(irq) Test_DisableNVICIRQ(irq)
#define NVIC_ClearPendingIRQ(irq) Test_ClearPendingIRQ(irq)
#define NVIC_SetPriority(irq, priority) Test_SetPriority(irq, priority)
#define NVIC_SystemReset() Test_SystemReset()
#define NVIC_GetEnableIRQ(irq) Test_GetEnableIRQ(irq)
#define NVIC_GetPendingIRQ(irq) ((void)(irq), 0U)

#include "../../Inc/stm32l0xx_conf.h"

/* RAM does not emulate write-one-to-clear register semantics or clock mux
 * status updates; model those effects at the hardware boundary. */
#define LL_EXTI_ClearFlag_0_31(lines) Test_ClearEXTI(lines)
#define LL_RCC_GetSysClkSource() LL_RCC_SYS_CLKSOURCE_STATUS_HSI

#endif

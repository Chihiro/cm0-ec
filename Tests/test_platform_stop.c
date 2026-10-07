#include "stubs/platform_registers.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Exercise the real platform implementation, including its file-local wake
 * flags. Only the hardware registers and interrupt delivery are mocked. */
#include "../Src/ec_platform_stm32l051.c"

GPIO_TypeDef test_gpioa, test_gpiob, test_gpioc;
I2C_TypeDef test_i2c1, test_i2c2;
RCC_TypeDef test_rcc;
PWR_TypeDef test_pwr;
FLASH_TypeDef test_flash;
USART_TypeDef test_usart2;
SYSCFG_TypeDef test_syscfg;
EXTI_TypeDef test_exti;
SysTick_Type test_systick;
SCB_Type test_scb;
uint32_t SystemCoreClock = 8000000U;

static uint32_t primask, enabled_irqs;
static unsigned wfi_calls, suspend_calls, init_calls, tick_init_calls;
static bool host_busy, host_enabled, handled_edge_during_suspend, edge_before_wfi;
static uint32_t wake_lines;

static void dispatch_irqs(void)
{
    if (primask != 0U) return;
    if ((enabled_irqs & (1U << EXTI0_1_IRQn)) && (EXTI->PR & LL_EXTI_LINE_0)) {
        EXTI0_1_IRQHandler();
    }
    if ((enabled_irqs & (1U << EXTI4_15_IRQn)) && (EXTI->PR & LL_EXTI_LINE_13)) {
        EXTI4_15_IRQHandler();
    }
}

uint32_t Test_GetPrimask(void) { return primask; }
void Test_DisableIRQ(void) { primask = 1U; }
void Test_SetPrimask(uint32_t value) { primask = value; dispatch_irqs(); }
void Test_EnableIRQ(IRQn_Type irq) { enabled_irqs |= 1U << irq; }
void Test_DisableNVICIRQ(IRQn_Type irq) { enabled_irqs &= ~(1U << irq); }
void Test_ClearPendingIRQ(IRQn_Type irq) { (void)irq; }
void Test_SetPriority(IRQn_Type irq, uint32_t priority) { (void)irq; (void)priority; }
uint32_t Test_GetEnableIRQ(IRQn_Type irq) { return (enabled_irqs & (1U << irq)) != 0U; }
void Test_SystemReset(void) { assert(!"unexpected reset"); }
void Test_ClearEXTI(uint32_t lines) { EXTI->PR &= ~lines; }

void Test_DSB(void)
{
    if (edge_before_wfi) {
        edge_before_wfi = false;
        EXTI->PR |= LL_EXTI_LINE_13;
        dispatch_irqs(); /* Must stay pending until WFI returns. */
    }
}

void Test_WFI(void)
{
    wfi_calls++;
    assert(primask == 1U); /* Prevent a wake ISR from running before sleep. */
    assert((SysTick->CTRL & SysTick_CTRL_TICKINT_Msk) == 0U);
    assert((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) != 0U);
    assert(!host_enabled);
    assert((I2C1->CR1 & I2C_CR1_PE) == 0U);
    EXTI->PR |= wake_lines;
    dispatch_irqs();
}

void LL_Init1msTick(uint32_t hclk)
{
    assert(hclk == 8000000U);
    tick_init_calls++;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->LOAD = hclk / 1000U - 1U;
}
void SystemCoreClockUpdate(void) { SystemCoreClock = 8000000U; }
bool BQ27220_Init(void)
{
    assert(host_enabled); /* Restore the slave before blocking gauge recovery. */
    I2C1->CR1 |= I2C_CR1_PE;
    return true;
}
void EC_HostI2C_Init(void)
{
    assert(SystemCoreClock == 8000000U);
    init_calls++;
    host_enabled = true;
}
bool EC_HostI2C_Suspend(void)
{
    suspend_calls++;
    if (host_busy) return false;
    host_enabled = false;
    if (handled_edge_during_suspend) {
        handled_edge_during_suspend = false;
        EXTI->PR |= LL_EXTI_LINE_0;
        dispatch_irqs();
    }
    return true;
}

static void prepare_case(void)
{
    memset(&test_gpioa, 0, sizeof(test_gpioa));
    memset(&test_gpioc, 0, sizeof(test_gpioc));
    memset(&test_exti, 0, sizeof(test_exti));
    memset(&test_scb, 0, sizeof(test_scb));
    GPIOA->IDR = LL_GPIO_PIN_0;
    GPIOC->IDR = LL_GPIO_PIN_13;
    primask = enabled_irqs = 0U;
    wfi_calls = suspend_calls = init_calls = tick_init_calls = 0U;
    host_busy = handled_edge_during_suspend = edge_before_wfi = false;
    host_enabled = true;
    wake_lines = LL_EXTI_LINE_0;
    I2C1->CR1 = I2C_CR1_PE;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
    EC_Platform_PrepareStopWake();
}

static void sample(bool expected_key, bool expected_pg)
{
    bool key_low, pg_low, key_pending, pg_pending;
    EC_Platform_SampleStopCutoff(&key_low, &pg_low, &key_pending, &pg_pending);
    assert(key_pending == expected_key && pg_pending == expected_pg);
    assert(EXTI->PR == 0U && s_stop_wake_flags == 0U);
}

int main(void)
{
    prepare_case();
    host_busy = true;
    assert(!EC_Platform_EnterStop());
    assert(wfi_calls == 0U && !EC_Platform_ServicesWereSuspended());
    assert(primask == 0U && host_enabled && (SysTick->CTRL & SysTick_CTRL_TICKINT_Msk));
    assert(I2C1->CR1 & I2C_CR1_PE); /* A cancelled STOP leaves gauge I2C enabled. */
    sample(false, false);

    /* A short key edge already handled before the last sleep check must
     * cancel sleep even if the key has returned HIGH. */
    prepare_case();
    EXTI->PR = LL_EXTI_LINE_0;
    EXTI0_1_IRQHandler();
    assert(!EC_Platform_EnterStop());
    assert(wfi_calls == 0U && host_enabled && !EC_Platform_ServicesWereSuspended());
    sample(true, false);

    prepare_case();
    GPIOA->IDR &= ~LL_GPIO_PIN_0;
    assert(!EC_Platform_EnterStop());
    assert(wfi_calls == 0U && suspend_calls == 0U && host_enabled);
    sample(false, false); /* A level alone is not edge-triggered evidence. */

    prepare_case();
    GPIOC->IDR &= ~LL_GPIO_PIN_13;
    assert(!EC_Platform_EnterStop());
    assert(wfi_calls == 0U && suspend_calls == 0U && host_enabled);
    sample(false, false);

    prepare_case();
    primask = 1U;
    EXTI->PR = LL_EXTI_LINE_13;
    assert(!EC_Platform_EnterStop() && primask == 1U);
    assert(wfi_calls == 0U && host_enabled);
    sample(false, true);

    prepare_case();
    assert(EC_Platform_EnterStop());
    assert(wfi_calls == 1U && primask == 0U && EC_Platform_ServicesWereSuspended());
    assert((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) == 0U);
    sample(true, false);
    test_rcc.CR = RCC_CR_HSIRDY;
    SystemCoreClock = 1048576U; /* Require restoring the run clock before services. */
    assert(EC_Platform_RestoreAfterStop());
    assert(host_enabled && init_calls == 1U && tick_init_calls == 1U);
    assert(SysTick->CTRL & SysTick_CTRL_TICKINT_Msk);
    assert(I2C1->CR1 & I2C_CR1_PE);
    assert(!EC_Platform_ServicesWereSuspended());

    SysTick_Handler();
    assert(EC_Platform_Millis() == 1U);

    /* Wake edge inside the I2C suspension window stays pending under PRIMASK. */
    prepare_case();
    handled_edge_during_suspend = true;
    assert(EC_Platform_EnterStop());
    assert(wfi_calls == 1U && primask == 0U);
    sample(true, false);

    /* IRQ immediately before WFI cannot run early and consume the only edge. */
    prepare_case();
    wake_lines = 0U;
    edge_before_wfi = true;
    assert(EC_Platform_EnterStop());
    assert(wfi_calls == 1U && primask == 0U);
    sample(false, true);

    /* Preserve callers' interrupt mask instead of unconditionally enabling. */
    prepare_case();
    primask = 1U;
    assert(EC_Platform_EnterStop());
    assert(primask == 1U);
    sample(true, false); /* Hardware EXTI evidence must work with IRQs masked. */

    puts("platform STOP entry, wake evidence and restore tests passed");
    return 0;
}

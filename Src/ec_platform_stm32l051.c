/**
 ******************************************************************************
 * @file           : ec_platform_stm32l051.c
 * @brief          : STM32L051 LL implementation of ec_platform.h
 *
 *   Clock:        HSI16 / 2 = 8 MHz HCLK/PCLK1/PCLK2, voltage range 1, 0 WS
 *   SysTick:      1 ms interrupt timebase, disabled before STOP
 *   I2C1:         PB6=SCL, PB7=SDA, 100 kHz, open-drain
 *   I2C2:         PB13=SCL, PB14=SDA, AF5 slave, host address 0x42
 *   USART2:       PA2=TX, 115200-8-N-1 (debug printf)
 *
 *   Pin map:
 *     PA0  — POWER_KEY (input, pull-up, active LOW)
 *     PA1  — LOAD_SW_ON (output, HIGH=ON, default LOW)
 *     PA4  — LED_R (output, common anode: LOW=ON)
 *     PA5  — LED_G (output, common anode: LOW=ON)
 *     PA6  — LED_B (output, common anode: LOW=ON)
 *     PA2  — USART2_TX (AF4)
 *     PC13 — BQ25601 PG (input, pull-up, active LOW)
 *     PB6  — I2C1_SCL (AF1, open-drain)
 *     PB7  — I2C1_SDA (AF1, open-drain)
 *     PA13 — SWDIO
 *     PA14 — SWCLK
 ******************************************************************************
 */

#include "ec_platform.h"
#include "bq27220.h"
#include "ec_host_i2c.h"
#include "stm32l0xx_conf.h"
#include <stddef.h>
#include <stdio.h>

/*============================================================================
 * Pin Definitions
 *============================================================================*/

#define KEY_GPIO_PORT       GPIOA
#define KEY_PIN             LL_GPIO_PIN_0

#define LOAD_ON_GPIO_PORT   GPIOA
#define LOAD_ON_PIN         LL_GPIO_PIN_1

#define LED_R_GPIO_PORT     GPIOA
#define LED_R_PIN           LL_GPIO_PIN_4

#define LED_G_GPIO_PORT     GPIOA
#define LED_G_PIN           LL_GPIO_PIN_5

#define LED_B_GPIO_PORT     GPIOA
#define LED_B_PIN           LL_GPIO_PIN_6

#define LED_ALL_PINS        (LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6)

#define PG_GPIO_PORT        GPIOC
#define PG_PIN              LL_GPIO_PIN_13

#define USART2_TX_PIN       LL_GPIO_PIN_2
#define USART2_TX_AF        LL_GPIO_AF_4

/*============================================================================
 * Static State
 *============================================================================*/

static volatile uint32_t s_millis;
static volatile bool     s_services_suspended;
static bool              s_usart2_ready;

#define USART2_TIMEOUT_LOOPS 50000U
#define CLOCK_TIMEOUT_LOOPS  50000U

/* STOP wake flags — set from EXTI ISRs */
static volatile uint32_t s_stop_wake_flags;

/*============================================================================
 * Clock: HSI16 SYSCLK, AHB / 2, APB1/APB2 / 1 -> 8 MHz, 0 wait states
 *============================================================================*/

static bool clock_init(void)
{
    uint32_t timeout = CLOCK_TIMEOUT_LOOPS;

    /* ES0251 2.12.3: I2C2 kernel clock must be >=4 MHz for a Standard-mode
     * transmitter with tSU;DAT=250 ns. PCLK1=8 MHz gives sampling margin.
     * Use voltage range 1 so HSI tolerance cannot exceed the 0-WS limit. */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    while (LL_PWR_IsActiveFlag_VOS()) {
        if (timeout-- == 0U) return false;
    }
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

    LL_RCC_HSI_Enable();
    timeout = CLOCK_TIMEOUT_LOOPS;
    while (!LL_RCC_HSI_IsReady()) {
        if (timeout-- == 0U) return false;
    }
    LL_RCC_HSI_DisableDivider(); /* Disable HSI's separate /4 divider. */
    timeout = CLOCK_TIMEOUT_LOOPS;
    while ((RCC->CR & RCC_CR_HSIDIVF) != 0U) {
        if (timeout-- == 0U) return false;
    }

    /* Set divisors before switching to the faster source. */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
    timeout = CLOCK_TIMEOUT_LOOPS;
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
        if (timeout-- == 0U) return false;
    }

    /* Keep MSI as the STOP wakeup source (also avoids ES0251 2.1.1 on rev A).
     * Restore HSI16/2 in RestoreAfterStop before logging or resuming I2C. */
    LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_MSI);
    LL_RCC_HSI_DisableInStopMode();
    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
    SystemCoreClockUpdate();

    return SystemCoreClock == 8000000U;
}

/*============================================================================
 * SysTick: 1 ms interrupt timebase
 *============================================================================*/

static bool systick_init(void)
{
    s_millis = 0U;

    /* Count elapsed time even while gauge, UART, or display calls block.
     * LL_mDelay() may still poll COUNTFLAG independently of this counter. */
    LL_Init1msTick(SystemCoreClock);
    NVIC_SetPriority(SysTick_IRQn, 3U); /* Below EXTI (0) and I2C2 (1). */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    LL_SYSTICK_EnableIT();

    return true;
}

void SysTick_Handler(void)
{
    s_millis++;
}

uint32_t EC_Platform_Millis(void)
{
    return s_millis;
}

/*============================================================================
 * GPIO Initialization
 *
 * SAFETY-CRITICAL ORDER:
 *   1. Write PA1 LOW first (load OFF by default)
 *   2. Write LED pins HIGH first (LED OFF, common anode)
 *   3. THEN configure pins as outputs
 *   4. Configure inputs last
 *============================================================================*/

static bool gpio_init(void)
{
    /* Enable GPIO clocks (STM32L0 uses IOP bus, not AHB) */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);

    /* ---- PA1: Write LOW first (load OFF), then configure as output ---- */
    LL_GPIO_ResetOutputPin(LOAD_ON_GPIO_PORT, LOAD_ON_PIN);
    LL_GPIO_SetPinMode(LOAD_ON_GPIO_PORT, LOAD_ON_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinPull(LOAD_ON_GPIO_PORT, LOAD_ON_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(LOAD_ON_GPIO_PORT, LOAD_ON_PIN, LL_GPIO_SPEED_FREQ_LOW);

    /* ---- LED pins: Write HIGH first (LED OFF, common anode), then output ---- */
    LL_GPIO_SetOutputPin(LED_R_GPIO_PORT, LED_ALL_PINS);

    LL_GPIO_SetPinMode(LED_R_GPIO_PORT, LED_R_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinPull(LED_R_GPIO_PORT, LED_R_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(LED_R_GPIO_PORT, LED_R_PIN, LL_GPIO_SPEED_FREQ_LOW);

    LL_GPIO_SetPinMode(LED_G_GPIO_PORT, LED_G_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinPull(LED_G_GPIO_PORT, LED_G_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(LED_G_GPIO_PORT, LED_G_PIN, LL_GPIO_SPEED_FREQ_LOW);

    LL_GPIO_SetPinMode(LED_B_GPIO_PORT, LED_B_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinPull(LED_B_GPIO_PORT, LED_B_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(LED_B_GPIO_PORT, LED_B_PIN, LL_GPIO_SPEED_FREQ_LOW);

    /* ---- PA0: KEY input with pull-up (active LOW) ---- */
    LL_GPIO_SetPinMode(KEY_GPIO_PORT, KEY_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(KEY_GPIO_PORT, KEY_PIN, LL_GPIO_PULL_UP);

    /* ---- PC13: PG input with pull-up (active LOW) ---- */
    LL_GPIO_SetPinMode(PG_GPIO_PORT, PG_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(PG_GPIO_PORT, PG_PIN, LL_GPIO_PULL_UP);

    /* PB13/PB14 stay high-Z until EC_HostI2C_Init configures AF5 open-drain. */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_14, LL_GPIO_PULL_NO);

    return true;
}

/*============================================================================
 * Input Reading
 *============================================================================*/

bool EC_Platform_IsPowerKeyPressed(void)
{
    /* PA0 LOW = key pressed */
    return (LL_GPIO_IsInputPinSet(KEY_GPIO_PORT, KEY_PIN) == 0U);
}

bool EC_Platform_IsPgActive(void)
{
    /* PC13 LOW = PG active (VBUS present) */
    return (LL_GPIO_IsInputPinSet(PG_GPIO_PORT, PG_PIN) == 0U);
}

/*============================================================================
 * Output Control (logical levels — platform handles inversion)
 *============================================================================*/

void EC_Platform_SetLoadEnabled(bool enabled)
{
    if (enabled) {
        LL_GPIO_SetOutputPin(LOAD_ON_GPIO_PORT, LOAD_ON_PIN);   /* PA1 HIGH = load ON */
    } else {
        LL_GPIO_ResetOutputPin(LOAD_ON_GPIO_PORT, LOAD_ON_PIN); /* PA1 LOW  = load OFF */
    }
}

void EC_Platform_SetLedRed(bool on)
{
    if (on) {
        LL_GPIO_ResetOutputPin(LED_R_GPIO_PORT, LED_R_PIN); /* LOW = ON (common anode) */
    } else {
        LL_GPIO_SetOutputPin(LED_R_GPIO_PORT, LED_R_PIN);   /* HIGH = OFF */
    }
}

void EC_Platform_SetLedGreen(bool on)
{
    if (on) {
        LL_GPIO_ResetOutputPin(LED_G_GPIO_PORT, LED_G_PIN);
    } else {
        LL_GPIO_SetOutputPin(LED_G_GPIO_PORT, LED_G_PIN);
    }
}

void EC_Platform_SetLedBlue(bool on)
{
    if (on) {
        LL_GPIO_ResetOutputPin(LED_B_GPIO_PORT, LED_B_PIN);
    } else {
        LL_GPIO_SetOutputPin(LED_B_GPIO_PORT, LED_B_PIN);
    }
}

/*============================================================================
 * USART2 Debug Output
 *============================================================================*/

void EC_Platform_Usart2Init(void)
{
    uint32_t timeout = USART2_TIMEOUT_LOOPS;
    s_usart2_ready = false;

    /* Enable USART2 + GPIOA clocks */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /* PA2 as alternate function (USART2_TX, AF4) */
    LL_GPIO_SetPinMode(GPIOA, USART2_TX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, USART2_TX_PIN, USART2_TX_AF);
    LL_GPIO_SetPinSpeed(GPIOA, USART2_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);

    /* USART2: 115200-8-N-1, TX only */
    LL_USART_Disable(USART2);
    LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_16);

    /*
     * PCLK1 = HCLK = 8 MHz.
     * BRR for 115200 bps @ 8 MHz, OVER8=0:
     *   BRR = 8000000 / 115200 ≈ 69.4 → BRR = 69.
     *   Nominal rate = 8000000 / 69 ≈ 115942 bps, error = +0.64%.
     *   Acceptable for debug output (tolerance typically ±3%).
     */
    LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, 115200U);

    LL_USART_SetDataWidth(USART2, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART2, LL_USART_STOPBITS_1);
    LL_USART_SetParity(USART2, LL_USART_PARITY_NONE);
    LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX);
    LL_USART_SetHWFlowCtrl(USART2, LL_USART_HWCONTROL_NONE);
    LL_USART_Enable(USART2);

    /* Wait for USART ready */
    while (LL_USART_IsActiveFlag_TEACK(USART2) == 0U) {
        if (timeout-- == 0U) {
            LL_USART_Disable(USART2);
            return;
        }
    }
    s_usart2_ready = true;
}

int EC_Platform_Putchar(int ch)
{
    uint32_t timeout = USART2_TIMEOUT_LOOPS;

    /* Early logs or a failed USART must never block power management. */
    if (!s_usart2_ready) {
        return EOF;
    }

    /* Wait for TX with a bounded timeout. */
    while (LL_USART_IsActiveFlag_TXE(USART2) == 0U) {
        if (timeout-- == 0U) {
            s_usart2_ready = false;
            return EOF;
        }
    }
    LL_USART_TransmitData8(USART2, (uint8_t)(ch & 0xFFU));
    return ch;
}

/*
 * __io_putchar / __io_getchar — called by newlib's _write() / _read()
 * syscall.c (syscall.c).  Defining them here allows printf() to work
 * over USART2 TX.
 */
int __io_putchar(int ch)
{
    return EC_Platform_Putchar(ch);
}

int __io_getchar(void)
{
    return 0;  /* USART2 is TX-only */
}

/*============================================================================
 * System Init
 *============================================================================*/

static void print_host_i2c_diagnostics(void)
{
    printf("[I2C2] HCLK/PCLK1=%lu Hz CR1=0x%08lX OAR1=0x%08lX ISR=0x%08lX TIMINGR=0x%08lX\r\n",
        (unsigned long)SystemCoreClock, (unsigned long)I2C2->CR1,
        (unsigned long)I2C2->OAR1, (unsigned long)I2C2->ISR,
        (unsigned long)I2C2->TIMINGR);
    printf("[I2C2] GPIOB MODER=0x%08lX OTYPER=0x%08lX PUPDR=0x%08lX AFRH=0x%08lX SCL=%lu SDA=%lu\r\n",
        (unsigned long)GPIOB->MODER, (unsigned long)GPIOB->OTYPER,
        (unsigned long)GPIOB->PUPDR, (unsigned long)GPIOB->AFR[1],
        (unsigned long)(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13) != 0U),
        (unsigned long)(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_14) != 0U));
    printf("[I2C2] PRIMASK=%lu IRQ_EN=%lu IRQ_PENDING=%lu APB1ENR=0x%08lX\r\n",
        (unsigned long)__get_PRIMASK(), (unsigned long)NVIC_GetEnableIRQ(I2C2_IRQn),
        (unsigned long)NVIC_GetPendingIRQ(I2C2_IRQn), (unsigned long)RCC->APB1ENR);
}

bool EC_Platform_Init(void)
{
    s_millis = 0U;
    s_services_suspended = false;
    s_stop_wake_flags = 0U;

    /* 1. Clock (HSI16 / 2 = 8 MHz) */
    if (!clock_init()) {
        return false;
    }

    /* 2. GPIO (safety order: PA1 LOW, LEDs OFF first) */
    if (!gpio_init()) {
        return false;
    }

    /* 3. SysTick (1 ms interrupt timebase) */
    if (!systick_init()) {
        return false;
    }

    /* 4. USART2 (early debug output) */
    EC_Platform_Usart2Init();

    /* 5. I2C1 (delegated to BQ27220_Init) */
    if (!BQ27220_Init()) {
        /* I2C1 init failed — gauge will be NO_GAUGE, but platform init
           is still "successful" because the system can operate without gauge */
    }

    /* 6. I2C2 slave for telemetry and delayed host load-off commands. */
    EC_HostI2C_Init();
    print_host_i2c_diagnostics();

    return true;
}

/*============================================================================
 * STOP Mode
 *============================================================================*/

/*
 * EXTI interrupt handlers for STOP wakeup.
 * These are only active during STOP — during normal run, EXTI is disabled
 * and these ISRs should never fire.
 */

void EXTI0_1_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
        s_stop_wake_flags |= 1U;  /* KEY wake */
    }
}

void EXTI4_15_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
        s_stop_wake_flags |= 2U;  /* PG wake */
    }
}

void EC_Platform_PrepareStopWake(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Enable SYSCFG clock (needed for EXTI source selection) */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    /* Map GPIO to EXTI lines */
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);   /* PA0 → EXTI0 */
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);  /* PC13 → EXTI13 */

    /* Enable falling edge trigger for KEY and PG */
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_13);

    /* Enable EXTI interrupts */
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_13);

    /* Clear any stale EXTI pending flags */
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0 | LL_EXTI_LINE_13);
    NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
    NVIC_ClearPendingIRQ(EXTI4_15_IRQn);

    /* Clear PWR wakeup flag */
    LL_PWR_ClearFlag_WU();

    /* Clear wake flags */
    s_stop_wake_flags = 0U;

    /* Enable NVIC IRQs */
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0U);
    NVIC_SetPriority(EXTI4_15_IRQn, 0U);

    s_services_suspended = false;
    __set_PRIMASK(mask);
}

bool EC_Platform_EnterStop(void)
{
    /*
     * Enter STOP mode with WFI.
     *
     * STM32L0 STOP mode:
     *   - CPU clock stops
     *   - MSI oscillator stops (we do NOT call LL_RCC_MSI_EnableInStopMode)
     *   - SRAM and registers retained
     *   - GPIO outputs maintain their state (PA1 stays LOW, LEDs stay OFF)
     *   - Wake on EXTI (PA0 or PC13 falling edge) or NVIC pending
     *
     * Mask IRQ delivery from the final activity check through WFI. Otherwise
     * an EXTI ISR could run and clear the only wake edge just before WFI.
     * WFI still wakes for an interrupt pending under PRIMASK; the handler
     * runs after we clear SLEEPDEEP and restore the caller's mask.
     */

    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Also catch short pulses already serviced since PrepareStopWake. */
    if (EC_Platform_IsPowerKeyPressed() || EC_Platform_IsPgActive() ||
        s_stop_wake_flags != 0U ||
        LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) ||
        LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13)) {
        __set_PRIMASK(mask);
        return false;
    }

    /* I2C2 has no STOP wakeup support. Cancel if a transaction began after
     * the state machine's eligibility check; otherwise release its pins. */
    if (!EC_HostI2C_Suspend()) {
        __set_PRIMASK(mask);
        return false;
    }
    s_services_suspended = true;

    /* ES0251 2.12.2: both I2C peripherals must have PE=0 in STOP.
     * Gauge polling has completed; RestoreAfterStop reinitializes I2C1. */
    LL_I2C_Disable(I2C1);

    /* Set SLEEPDEEP for STOP mode (not SLEEP) */
    LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_LOW_POWER);
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* A periodic tick must not wake STOP or leave a pending tick at WFI. */
    LL_SYSTICK_DisableIT();
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    __DSB();
    __WFI();

    /* Clear SLEEPDEEP after wake */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __set_PRIMASK(mask);
    return true;
}

void EC_Platform_SampleStopCutoff(bool *key_low, bool *pg_low,
                                   bool *key_pending, bool *pg_pending)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /*
     * 1. Read cutoff levels: sample KEY/PG BEFORE disabling EXTI.
     *    If KEY is LOW at this point, it may or may not be the wake source.
     *    Per spec §9.3: KEY LOW alone does NOT prove KEY wakeup.
     */
    if (key_low != NULL) {
        *key_low = EC_Platform_IsPowerKeyPressed();
    }
    if (pg_low != NULL) {
        *pg_low = EC_Platform_IsPgActive();
    }

    /*
     * 2. Read pending flags before clearing.
     *    KEY pending = bit0 in s_stop_wake_flags.
     *    PG pending  = bit1 in s_stop_wake_flags.
     *    Include hardware pending bits when the IRQ has not executed yet.
     *    Mask IRQ delivery so an ISR cannot clear PR after the software
     *    flags were sampled but before hardware evidence was collected.
     */
    uint32_t flags = s_stop_wake_flags;
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0)) {
        flags |= 1U;
    }
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13)) {
        flags |= 2U;
    }

    if (key_pending != NULL) {
        *key_pending = ((flags & 1U) != 0U);
    }
    if (pg_pending != NULL) {
        *pg_pending = ((flags & 2U) != 0U);
    }

    /*
     * 3. Disable EXTI triggers, NVIC, clear pending.
     */
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_0);
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_13);
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_13);

    NVIC_DisableIRQ(EXTI0_1_IRQn);
    NVIC_DisableIRQ(EXTI4_15_IRQn);

    /* Clear any remaining EXTI pending */
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0 | LL_EXTI_LINE_13);
    NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
    NVIC_ClearPendingIRQ(EXTI4_15_IRQn);

    /* Clear PWR wakeup flag */
    LL_PWR_ClearFlag_WU();

    /* Clear internal wake flags for next STOP cycle */
    s_stop_wake_flags = 0U;
    __set_PRIMASK(mask);
}

bool EC_Platform_ServicesWereSuspended(void)
{
    return s_services_suspended;
}

/*============================================================================
 * STOP Recovery
 *============================================================================*/

bool EC_Platform_RestoreAfterStop(void)
{
    /*
     * STOP wakes on MSI with the retained AHB divider. Restore HSI16/2 before
     * SysTick, I2C2, or debug output resumes; 2.097 MHz is not the run clock.
     */
    if (!clock_init()) {
        return false;
    }

    /* Re-init SysTick for 1ms timebase */
    if (!systick_init()) {
        return false;
    }

    /* Re-enable GPIO clocks (preserved in STOP but safe to re-enable) */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);

    /* Make the cached slave interface available before I2C1 bus recovery,
     * which may delay wake restoration. The ISR never accesses I2C1. */
    EC_HostI2C_Init();

    /*
     * Restore I2C1 physical configuration.
     * Per spec §4 & §9.5:
     *   - Restore I2C1 clock, registers, PB6/PB7 AF open-drain
     *   - Do NOT send BQ27220 commands
     *   - Do NOT change Gauge mode
     *   - Do NOT clear command guard or SOC state
     *   - Failure here does NOT reset MCU
     */
    if (!BQ27220_Init()) {
        /*
         * I2C1 restore failed.
         * Per spec: do NOT reset, do NOT change STOP platform result.
         * NORMAL: subsequent SOC reads will naturally fail.
         * NO_GAUGE: continues to not access Gauge.
         */
    }

    print_host_i2c_diagnostics();

    /* Clear services suspended flag */
    s_services_suspended = false;

    return true;
}

/*============================================================================
 * System Reset
 *============================================================================*/

void EC_Platform_SystemReset(void)
{
    /*
     * Try to ensure load is OFF before reset.
     * Hardware pulldown on TPS22992S ON pin will also ensure this
     * during and after reset.
     */
    EC_Platform_SetLoadEnabled(false);

    /* Small delay to let PA1 settle */
    for (volatile uint32_t d = 0U; d < 10000U; d++) {
        __NOP();
    }

    NVIC_SystemReset();
}

/**
 ******************************************************************************
 * @file           : ec_platform_stm32l051.c
 * @brief          : STM32L051 LL implementation of ec_platform.h
 *
 *   Clock:        MSI 2.097 MHz (default, Range 1, 0 WS)
 *   SysTick:      1 ms polling (TICKINT=0) — no SysTick interrupt
 *   I2C1:         PB6=SCL, PB7=SDA, 100 kHz, open-drain
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

/* I2C2 pins (PB13=SCL, PB14=SDA) — reserved for Linux host, kept analog for now */
#define I2C2_SCL_PIN        LL_GPIO_PIN_13
#define I2C2_SDA_PIN        LL_GPIO_PIN_14
#define I2C2_PINS           (LL_GPIO_PIN_13 | LL_GPIO_PIN_14)

/*============================================================================
 * Static State
 *============================================================================*/

static volatile uint32_t s_millis;
static volatile bool     s_services_suspended;

/* STOP wake flags — set from EXTI ISRs */
static volatile uint32_t s_stop_wake_flags;

/*============================================================================
 * Clock: MSI 2.097 MHz, Range 1, 0 wait states
 *============================================================================*/

static bool clock_init(void)
{
    /*
     * MSI is the default clock source after reset at 2.097 MHz (Range 1).
     * No configuration needed — just set SystemCoreClock and flash latency.
     */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

    SystemCoreClock = 2097000U;

    return true;
}

/*============================================================================
 * SysTick: 1 ms polling timebase (TICKINT=0)
 *============================================================================*/

static bool systick_init(void)
{
    s_millis = 0U;

    /*
     * LL_Init1msTick configures SysTick for 1ms period.
     * TICKINT=0 means no SysTick interrupt — we poll COUNTFLAG.
     */
    LL_Init1msTick(SystemCoreClock);

    return true;
}

uint32_t EC_Platform_Millis(void)
{
    /*
     * Check COUNTFLAG each call. Since the main loop runs faster than
     * 1 ms (no blocking delays), we catch every tick.
     */
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U) {
        s_millis++;
    }
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

    /* ---- PB13/PB14: I2C2 pins, analog high-Z (reserved for Linux host) ---- */
    LL_GPIO_SetPinMode(GPIOB, I2C2_PINS, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull(GPIOB, I2C2_PINS, LL_GPIO_PULL_NO);

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
     * PCLK = HCLK = 2.097 MHz.
     * BRR for 115200 bps @ 2.097 MHz, OVER8=0:
     *   BRR = 2097000 / 115200 ≈ 18.2 → BRR = 18.
     *   Actual rate = 2097000 / 18 = 116500 bps, error = +1.13%.
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
    }
}

int EC_Platform_Putchar(int ch)
{
    /* Block until TX data register empty */
    while (LL_USART_IsActiveFlag_TXE(USART2) == 0U) {
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

bool EC_Platform_Init(void)
{
    s_millis = 0U;
    s_services_suspended = false;
    s_stop_wake_flags = 0U;

    /* 1. Clock (MSI 2.097 MHz) */
    if (!clock_init()) {
        return false;
    }

    /* 2. GPIO (safety order: PA1 LOW, LEDs OFF first) */
    if (!gpio_init()) {
        return false;
    }

    /* 3. SysTick (1ms polling) */
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

    /* Clear PWR wakeup flag */
    LL_PWR_ClearFlag_WU();

    /* Clear wake flags */
    s_stop_wake_flags = 0U;

    /* Enable NVIC IRQs */
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0U);
    NVIC_SetPriority(EXTI4_15_IRQn, 0U);

    /* Mark services as suspended */
    s_services_suspended = true;
}

void EC_Platform_EnterStop(void)
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
     * The critical race condition on Cortex-M0+:
     *   If an edge occurs between our final check and WFI, the EXTI pending
     *   bit is latched in hardware, and WFI returns immediately.
     *   This is handled by the caller's two-stage check (PrepareStopWake →
     *   check pins → EnterStop), plus the EXTI pending bits.
     */

    /* Set SLEEPDEEP for STOP mode (not SLEEP) */
    LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_LOW_POWER);
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    __WFI();

    /* Clear SLEEPDEEP after wake */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}

void EC_Platform_SampleStopCutoff(bool *key_low, bool *pg_low,
                                   bool *key_pending, bool *pg_pending)
{
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
     *    These were set by the EXTI ISRs that fired during STOP wakeup.
     *    By reading s_stop_wake_flags BEFORE disabling EXTI, we capture
     *    the true edge-triggered evidence.
     */
    uint32_t flags = s_stop_wake_flags;

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

    /* Clear PWR wakeup flag */
    LL_PWR_ClearFlag_WU();

    /* Clear internal wake flags for next STOP cycle */
    s_stop_wake_flags = 0U;
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
     * MSI restarts automatically on STM32L0 wakeup from STOP.
     * Just update SystemCoreClock and re-init SysTick.
     */
    SystemCoreClockUpdate();
    SystemCoreClock = 2097000U;

    /* Re-init SysTick for 1ms timebase */
    if (!systick_init()) {
        return false;
    }

    /* Re-enable GPIO clocks (preserved in STOP but safe to re-enable) */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);

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

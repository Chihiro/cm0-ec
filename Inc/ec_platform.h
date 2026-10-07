/**
 ******************************************************************************
 * @file           : ec_platform.h
 * @brief          : Hardware abstraction interface for EC state machine.
 *
 *   All hardware access is routed through this interface. The state machine
 *   never touches GPIO registers, I2C registers, or SysTick directly.
 *
 *   Platform layer translates logical states to physical levels:
 *     - SetLoadEnabled(true)  → PA1 HIGH (load ON)
 *     - SetLoadEnabled(false) → PA1 LOW  (load OFF)
 *     - SetLedX(true)         → GPIO LOW (common anode: LOW = LED ON)
 *     - SetLedX(false)        → GPIO HIGH
 *     - IsPowerKeyPressed()   → true when PA0 reads LOW
 *     - IsPgActive()          → true when PC13 reads LOW
 ******************************************************************************
 */

#ifndef EC_PLATFORM_H
#define EC_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * System Initialization
 *============================================================================*/

/**
 * @brief  Full platform init: clock + GPIO + SysTick + I2C1 + I2C2 slave.
 *         Returns true if all critical subsystems initialized.
 */
bool EC_Platform_Init(void);

/**
 * @brief  Restore clock, SysTick, GPIO, I2C1 and I2C2 after STOP wakeup.
 *         Only called if running services were actually suspended.
 * @return true on success. On failure, caller should force PA1 LOW and reset.
 */
bool EC_Platform_RestoreAfterStop(void);

/*============================================================================
 * Timing
 *============================================================================*/

/**
 * @brief  Millisecond counter updated by SysTick IRQ, including time spent
 *         in blocking gauge/UART/display calls. Safe to read from I2C2 IRQ.
 *         Naturally wraps at UINT32_MAX; resets when STOP services restart.
 */
uint32_t EC_Platform_Millis(void);

/*============================================================================
 * Inputs (logical: return true when signal is active)
 *============================================================================*/

/** @return true if POWER_KEY is pressed (PA0 reads LOW) */
bool EC_Platform_IsPowerKeyPressed(void);

/** @return true if BQ25601 PG is active / VBUS present (PC13 reads LOW) */
bool EC_Platform_IsPgActive(void);

/*============================================================================
 * Outputs (logical: on = LED lit, enabled = load ON)
 *============================================================================*/

/** @param enabled  true → PA1 HIGH (TPS22992S ON), false → PA1 LOW */
void EC_Platform_SetLoadEnabled(bool enabled);

/** @param on  true → GPIO LOW (common anode LED ON), false → GPIO HIGH */
void EC_Platform_SetLedRed(bool on);
void EC_Platform_SetLedGreen(bool on);
void EC_Platform_SetLedBlue(bool on);

/*============================================================================
 * USART2 Debug Output (optional, for printf)
 *============================================================================*/

void EC_Platform_Usart2Init(void);
int  EC_Platform_Putchar(int ch);

/*============================================================================
 * STOP Mode Support
 *============================================================================*/

/**
 * @brief  Prepare wake sources before STOP:
 *         - Configure PA0 (KEY) and PC13 (PG) as falling-edge EXTI
 *         - Clear EXTI and PWR pending flags
 *         - Enable NVIC for EXTI IRQs
 *         Running services are suspended only by EC_Platform_EnterStop().
 */
void EC_Platform_PrepareStopWake(void);

/**
 * @brief  Execute STOP mode via WFI with SLEEPDEEP.
 *         Blocks until a wakeup event occurs, then returns.
 *         Caller must have called EC_Platform_PrepareStopWake() first.
 * @return false if KEY/PG activity or host I2C work prevents STOP;
 *         true after WFI. Preserves the caller's interrupt mask.
 */
bool EC_Platform_EnterStop(void);

/**
 * @brief  Sample the STOP cutoff point:
 *         1. Read KEY and PG GPIO levels (before disabling EXTI)
 *         2. Read and accumulate EXTI pending flags
 *         3. Disable EXTI triggers, NVIC IRQs, clear pending
 *         4. Clear SLEEPDEEP
 *
 * @param key_low       [out] true if KEY was LOW at cutoff
 * @param pg_low        [out] true if PG was LOW at cutoff
 * @param key_pending   [out] true if KEY EXTI was pending before clear
 * @param pg_pending    [out] true if PG EXTI was pending before clear
 */
void EC_Platform_SampleStopCutoff(bool *key_low, bool *pg_low,
                                   bool *key_pending, bool *pg_pending);

/**
 * @brief  Check if running services were marked as suspended.
 *         Used to decide whether I2C1 restore is needed.
 */
bool EC_Platform_ServicesWereSuspended(void);

/*============================================================================
 * System Reset
 *============================================================================*/

/** @brief  Trigger NVIC system reset. Called on unrecoverable errors. */
void EC_Platform_SystemReset(void);

#ifdef __cplusplus
}
#endif

#endif /* EC_PLATFORM_H */

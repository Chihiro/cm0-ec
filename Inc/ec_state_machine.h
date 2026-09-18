/**
 ******************************************************************************
 * @file           : ec_state_machine.h
 * @brief          : EC power management state machine — types and public API.
 *
 *   Implements the v8.0 logic specification:
 *     - 2 business states: ACTIVE_IDLE, LOAD_RUNNING
 *     - 2 gauge modes: NORMAL, NO_GAUGE
 *     - BOOT is a one-time sequence before the main loop
 *     - STOP is a synchronous sleep operation within the main loop
 ******************************************************************************
 */

#ifndef EC_STATE_MACHINE_H
#define EC_STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Business States
 *============================================================================*/

typedef enum {
    EC_STATE_ACTIVE_IDLE = 0,  /* Load OFF; KEY/PG/STOP logic active */
    EC_STATE_LOAD_RUNNING       /* Load ON; SOC polling active */
} ec_state_t;

/*============================================================================
 * Gauge Modes
 *============================================================================*/

typedef enum {
    EC_GAUGE_NORMAL = 0,       /* Gauge BOOT succeeded, SOC available */
    EC_GAUGE_NO_GAUGE           /* Gauge BOOT failed, no gauge access */
} ec_gauge_mode_t;

/*============================================================================
 * SOC Color (for LED indication)
 *============================================================================*/

typedef enum {
    EC_COLOR_OFF = 0,
    EC_COLOR_GREEN,
    EC_COLOR_YELLOW,
    EC_COLOR_RED
} ec_soc_color_t;

/*============================================================================
 * Status Snapshot (read-only view for display / inspection)
 *============================================================================*/

typedef struct {
    ec_state_t       state;        /* Load switch: LOAD_RUNNING = ON */
    ec_gauge_mode_t  gauge_mode;   /* NORMAL / NO_GAUGE */
    bool             soc_valid;
    uint16_t         soc_percent;  /* 0–100 % */
    bool             voltage_valid;
    uint16_t         voltage_mv;   /* mV */
    bool             current_valid;
    int16_t          current_ma;   /* + charge, - discharge */
    bool             pg_active;    /* VBUS present (BQ25601 PG) */
} ec_status_t;

/*============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief  Fill a snapshot of current business/gauge status.
 *         Safe to call from display code each refresh cycle.
 */
void EC_GetStatus(ec_status_t *out);

/**
 * @brief  One-time initialization: platform init, gauge BOOT, enter ACTIVE_IDLE.
 *         Must be called once after reset, before EC_Task().
 */
void EC_Init(void);

/**
 * @brief  Main loop iteration. Call continuously — never blocks.
 *         All timing is based on SysTick differences.
 */
void EC_Task(void);

/**
 * @brief  Get current business state (for testing/inspection).
 */
ec_state_t EC_GetState(void);

/**
 * @brief  Get current gauge mode (for testing/inspection).
 */
ec_gauge_mode_t EC_GetGaugeMode(void);

#ifdef __cplusplus
}
#endif

#endif /* EC_STATE_MACHINE_H */

/**
 ******************************************************************************
 * @file           : ec_state_machine.c
 * @brief          : EC power management state machine — core business logic.
 *
 *   Implements the v8.0 logic specification:
 *     - Key debounce (30 ms press/release, 3 s long press)
 *     - PG debounce (200 ms)
 *     - LED patterns (2 s period, SOC colors with hysteresis, blue pulse)
 *     - SOC polling (≤1 Hz, 500 ms command guard)
 *     - STOP with strict KEY/PG wakeup evidence rules
 *     - 10 s idle timer for STOP entry
 *
 *   Design rules:
 *     - No fixed delays — all timing via EC_Platform_Millis() differences
 *     - Main loop runs continuously — EC_Task() returns after each iteration
 *     - STOP is a synchronous operation that blocks within EC_Task()
 *     - BQ25601 is NOT accessed via I2C — only PG GPIO
 *     - Only RelativeStateOfCharge() is read during runtime (NORMAL mode)
 ******************************************************************************
 */

#include "ec_state_machine.h"
#include "ec_platform.h"
#include "bq27220.h"

#include <stddef.h>
#include <stdio.h>

/*============================================================================
 * Timing Constants
 *============================================================================*/

#define KEY_DEBOUNCE_MS         30U
#define KEY_LONG_PRESS_MS       3000U
#define PG_DEBOUNCE_MS          200U
#define LED_PERIOD_MS           2000U
#define LED_BLUE_PULSE_MS       150U
#define SOC_INTERVAL_MS         1000U
#define CMD_GUARD_MS            500U
#define IDLE_TIMEOUT_MS         10000U
#define DISPLAY_REQUEST_MS      2000U

/*============================================================================
 * SOC Color Hysteresis Thresholds
 *============================================================================*/

#define SOC_HYST_GREEN_ENTER    72U   /* ≥72% → GREEN */
#define SOC_HYST_GREEN_EXIT     68U   /* ≤68% → exit GREEN */
#define SOC_HYST_YELLOW_ENTER   32U   /* ≥32% → YELLOW */
#define SOC_HYST_YELLOW_EXIT    28U   /* ≤28% → exit YELLOW */
#define SOC_HYST_RED_LOW_ENTER  9U    /* ≤9%  → RED constant */
#define SOC_HYST_RED_LOW_EXIT   12U   /* ≥12% → exit RED constant */

/*============================================================================
 * Internal State Structures
 *============================================================================*/

typedef struct {
    bool     armed;              /* Must release before next long-press arm */
    bool     raw;                /* Current GPIO reading (true = pressed) */
    bool     stable;             /* Debounced state (30 ms) */
    bool     long_event;         /* Latched one-shot long-press event */
    bool     timing;             /* Currently timing a potential long press */
    uint32_t changed_at_ms;      /* When raw last changed */
    uint32_t pressed_start_ms;   /* When stable press began */
} ec_key_t;

typedef struct {
    bool     raw;                /* Current GPIO reading (true = active) */
    bool     stable;             /* Debounced state (200 ms) */
    uint32_t changed_at_ms;      /* When raw last changed */
} ec_pg_t;

typedef struct {
    bool          valid;         /* Latest SOC read was successful */
    uint16_t      value;         /* 0–100 % */
    ec_soc_color_t color;        /* Current color with hysteresis */
    bool          full;          /* Latest valid SOC was 100 */
    uint32_t      last_attempt_ms; /* When last SOC transaction began */
    bool          in_progress;   /* SOC read transaction in flight */
} ec_soc_t;

typedef struct {
    bool     display_requested;  /* Short-press display request active */
    uint32_t display_start_ms;   /* When display request started */
} ec_display_t;

typedef struct {
    ec_state_t       state;
    ec_gauge_mode_t  gauge_mode;
    bool             gauge_boot_done;

    ec_key_t         key;
    ec_pg_t          pg;
    ec_soc_t         soc;
    ec_display_t     display;

    uint32_t last_activity_ms;   /* Last event that resets idle timer */
    uint32_t last_command_ms;    /* Last BQ27220 standard command timestamp */

    uint32_t anomaly_count;      /* Unexplained STOP wakeups (diagnostic) */
} ec_ctx_t;

static ec_ctx_t g_ec;

/*============================================================================
 * Utility
 *============================================================================*/

static bool time_elapsed(uint32_t now, uint32_t start, uint32_t interval)
{
    return ((uint32_t)(now - start) >= interval);
}

static void refresh_activity(uint32_t now)
{
    g_ec.last_activity_ms = now;
}

#define LOG(fmt, ...)  printf("[%08lu] " fmt "\r\n", (unsigned long)EC_Platform_Millis(), ##__VA_ARGS__)

#ifdef EC_DEBUG_VERBOSE
#define LOGV(fmt, ...) LOG(fmt, ##__VA_ARGS__)
#else
#define LOGV(fmt, ...) ((void)0)
#endif

/*============================================================================
 * Key Debounce (30 ms press, 30 ms release, 3 s long press)
 *============================================================================*/

static void key_update(uint32_t now)
{
    bool raw = EC_Platform_IsPowerKeyPressed();

    /* Detect raw change */
    if (raw != g_ec.key.raw) {
        g_ec.key.raw = raw;
        g_ec.key.changed_at_ms = now;
    }

    /* Debounce: 30 ms stable required */
    if (time_elapsed(now, g_ec.key.changed_at_ms, KEY_DEBOUNCE_MS)) {
        bool prev_stable = g_ec.key.stable;
        g_ec.key.stable = raw;

        /* Detect edges on stable signal */
        if (!prev_stable && g_ec.key.stable) {
            /* Press detected */
            if (g_ec.key.armed) {
                g_ec.key.timing = true;
                g_ec.key.pressed_start_ms = now;
            }
        } else if (prev_stable && !g_ec.key.stable) {
            /* Release detected */
            g_ec.key.armed = true;
            g_ec.key.timing = false;
            g_ec.key.long_event = false;
        }
    }

    /* Long press detection: armed && stable pressed for ≥3s */
    if (g_ec.key.armed && g_ec.key.timing && g_ec.key.stable) {
        if (time_elapsed(now, g_ec.key.pressed_start_ms, KEY_LONG_PRESS_MS)) {
            g_ec.key.long_event = true;
            g_ec.key.armed = false;
            g_ec.key.timing = false;
        }
    }
}

static bool key_take_long_event(void)
{
    bool event = g_ec.key.long_event;
    g_ec.key.long_event = false;
    return event;
}

/*============================================================================
 * PG Debounce (200 ms)
 *============================================================================*/

static void pg_update(uint32_t now)
{
    bool raw = EC_Platform_IsPgActive();

    if (raw != g_ec.pg.raw) {
        g_ec.pg.raw = raw;
        g_ec.pg.changed_at_ms = now;
    }

    if (g_ec.pg.stable != g_ec.pg.raw) {
        if (time_elapsed(now, g_ec.pg.changed_at_ms, PG_DEBOUNCE_MS)) {
            bool prev_stable = g_ec.pg.stable;
            g_ec.pg.stable = g_ec.pg.raw;

            /* Stable transition */
            if (prev_stable && !g_ec.pg.stable) {
                /* PG became INACTIVE: restart idle timer (if load off) */
                LOG("[PG] VBUS REMOVED (PG inactive)");
                if (g_ec.state == EC_STATE_ACTIVE_IDLE) {
                    refresh_activity(now);
                }
            } else if (!prev_stable && g_ec.pg.stable) {
                /* PG became ACTIVE: refresh activity */
                LOG("[PG] VBUS PLUGGED IN (PG active)");
                refresh_activity(now);
            }
        }
    }
}

static bool pg_is_stable_active(void)
{
    return g_ec.pg.stable;
}

static void pg_force(bool active, uint32_t now)
{
    g_ec.pg.raw = active;
    g_ec.pg.stable = active;
    g_ec.pg.changed_at_ms = now;
}

/*============================================================================
 * SOC Color Determination with Hysteresis
 *============================================================================*/

static ec_soc_color_t soc_to_color(uint16_t soc, ec_soc_color_t current)
{
    if (soc >= SOC_HYST_GREEN_ENTER) {
        return EC_COLOR_GREEN;
    }
    if (soc >= SOC_HYST_GREEN_EXIT) {
        return (current == EC_COLOR_GREEN) ? EC_COLOR_GREEN : EC_COLOR_YELLOW;
    }
    if (soc >= SOC_HYST_YELLOW_ENTER) {
        return EC_COLOR_YELLOW;
    }
    if (soc >= SOC_HYST_YELLOW_EXIT) {
        return (current == EC_COLOR_YELLOW) ? EC_COLOR_YELLOW : EC_COLOR_RED;
    }
    if (soc >= SOC_HYST_RED_LOW_EXIT) {
        return EC_COLOR_RED;  /* 12-27% → RED pulse */
    }
    if (soc >= SOC_HYST_RED_LOW_ENTER) {
        return (current == EC_COLOR_RED && g_ec.soc.value <= 9U) ? EC_COLOR_RED : EC_COLOR_RED;
    }
    /* 0-9% → RED constant */
    return EC_COLOR_RED;
}

/*============================================================================
 * LED Pattern Generator
 *
 * Base period: 2000 ms.
 *
 * PG INVALID (no VBUS): SOC color single flash
 *   - Flash ON:  phase ∈ [0, 500) ms
 *   - Flash OFF: phase ∈ [500, 2000) ms
 *
 * PG VALID (VBUS present): SOC color double flash
 *   - Flash ON:  phase ∈ [0, 400) ms and [1000, 1400) ms
 *   - Flash OFF: phase ∈ [400, 1000) ms and [1400, 2000) ms
 *
 * Load ON: 150 ms blue pulse at start of period [0, 150) ms.
 *   Blue and R/G are time-multiplexed — blue takes priority.
 *
 * Constant colors (full charge, critical low):
 *   - Always ON, but yield 150 ms for blue if load is ON.
 *
 * NO_GAUGE: R/G OFF. Blue pulse only if load ON.
 * SOC invalid: R/G OFF.
 *============================================================================*/

static void led_apply(uint32_t now)
{
    uint32_t phase = now % LED_PERIOD_MS;
    bool load_on = (g_ec.state == EC_STATE_LOAD_RUNNING);
    bool gauge_ok = (g_ec.gauge_mode == EC_GAUGE_NORMAL);
    bool soc_valid = gauge_ok && g_ec.soc.valid;

    bool r_on = false, g_on = false, b_on = false;

    /* Determine R/G color */
    ec_soc_color_t color = g_ec.soc.color;
    bool is_full = soc_valid && g_ec.soc.full;               /* SOC = 100% */
    bool is_crit_low = soc_valid && (g_ec.soc.value <= 9U);   /* SOC 0-9% */

    /* Blue pulse window */
    bool blue_window = load_on && (phase < LED_BLUE_PULSE_MS);

    if (!soc_valid && !load_on) {
        /* NO_GAUGE or SOC invalid, load off: all OFF */
        r_on = false;
        g_on = false;
        b_on = false;
    } else if (!soc_valid && load_on) {
        /* NO_GAUGE or SOC invalid, load on: only blue pulse */
        b_on = blue_window;
    } else if (is_full) {
        /* Full charge: GREEN constant, yield for blue */
        if (!blue_window) {
            g_on = true;
        }
        b_on = blue_window;
    } else if (is_crit_low) {
        /* Critical low: RED constant, yield for blue */
        if (!blue_window) {
            r_on = true;
        }
        b_on = blue_window;
    } else {
        /* Flashing mode — determine flash ON/OFF */
        bool flash_on = false;

        if (pg_is_stable_active()) {
            /* PG valid: double flash */
            flash_on = (phase < 400U) || (phase >= 1000U && phase < 1400U);
        } else {
            /* PG invalid: single flash */
            flash_on = (phase < 500U);
        }

        if (flash_on && !blue_window) {
            switch (color) {
            case EC_COLOR_GREEN:
                g_on = true;
                break;
            case EC_COLOR_YELLOW:
                r_on = true;
                g_on = true;
                break;
            case EC_COLOR_RED:
                r_on = true;
                break;
            default:
                break;
            }
        }

        b_on = blue_window;
    }

    /* Apply outputs */
    EC_Platform_SetLedRed(r_on);
    EC_Platform_SetLedGreen(g_on);
    EC_Platform_SetLedBlue(b_on);
}

/*============================================================================
 * Display Request Handling
 *
 * Short press (or STOP wakeup short-click) requests a 2-second battery
 * display. If SOC is valid, display immediately. If SOC is invalid,
 * wait for next SOC result — which clears the request regardless of
 * success or failure.
 *============================================================================*/

static void display_request(uint32_t now)
{
    g_ec.display.display_requested = true;
    g_ec.display.display_start_ms = now;
}

static void display_clear(void)
{
    g_ec.display.display_requested = false;
}

/*============================================================================
 * SOC Scheduling (NORMAL mode only)
 *
 * Rules:
 *   - Maximum once per second
 *   - Minimum 500 ms between standard commands (shared BOOT + runtime)
 *   - Only RelativeStateOfCharge() is read during runtime
 *   - On success & SOC≤100: update valid, color, full flag
 *   - On failure or SOC>100: clear valid, clear full, R/G off
 *   - Display request cleared on ANY SOC result (success or failure)
 *============================================================================*/

static void soc_schedule(uint32_t now)
{
    if (g_ec.gauge_mode != EC_GAUGE_NORMAL) {
        return;
    }

    /* Guard: minimum 1s interval since last attempt */
    if (!time_elapsed(now, g_ec.soc.last_attempt_ms, SOC_INTERVAL_MS)) {
        return;
    }

    /* Guard: command guard timer (500ms between standard commands) */
    if (!time_elapsed(now, g_ec.last_command_ms, CMD_GUARD_MS)) {
        return;
    }

    /* Guard: transaction already in progress */
    if (g_ec.soc.in_progress) {
        return;
    }

    /* Mark attempt start BEFORE the transaction */
    g_ec.soc.last_attempt_ms = now;
    g_ec.soc.in_progress = true;
    g_ec.last_command_ms = now;

    /* Execute read */
    uint16_t soc_raw;
    if (BQ27220_ReadSOC(&soc_raw)) {
        if (soc_raw <= 100U) {
            /* Success: update SOC data */
            g_ec.soc.valid = true;
            g_ec.soc.value = soc_raw;
            g_ec.soc.full = (soc_raw == 100U);
            g_ec.soc.color = soc_to_color(soc_raw, g_ec.soc.color);
            LOG("[SOC] Read OK: %u%%  color=%s  full=%s",
                soc_raw,
                g_ec.soc.color == EC_COLOR_GREEN ? "GREEN" :
                g_ec.soc.color == EC_COLOR_YELLOW ? "YELLOW" :
                g_ec.soc.color == EC_COLOR_RED ? "RED" : "OFF",
                g_ec.soc.full ? "YES" : "no");
        } else {
            /* SOC > 100: invalid */
            g_ec.soc.valid = false;
            g_ec.soc.full = false;
            g_ec.soc.color = EC_COLOR_OFF;
            LOG("[SOC] Read value=%u (>100) — marked invalid", soc_raw);
        }
    } else {
        /* I2C read failed */
        g_ec.soc.valid = false;
        g_ec.soc.full = false;
        g_ec.soc.color = EC_COLOR_OFF;
        LOG("[SOC] Read FAILED — marked invalid (NORMAL kept)");
    }

    g_ec.soc.in_progress = false;

    /* Clear display request on ANY SOC result */
    display_clear();
}

/*============================================================================
 * STOP Eligibility Check
 *
 * All conditions must be met for ≥10 s:
 *   1. Business state = ACTIVE_IDLE (load OFF)
 *   2. PG raw = INACTIVE (raw, not stable — inhibit STOP immediately)
 *   3. PG NOT debouncing (edge just changed, not yet stable)
 *   4. KEY = RELEASED (stable, not in debounce)
 *   5. KEY NOT timing a long press
 *   6. No SOC transaction in progress
 *   7. 10 s elapsed since last activity
 *============================================================================*/

static bool stop_should_enter(uint32_t now)
{
    if (g_ec.state != EC_STATE_ACTIVE_IDLE) {
        return false;
    }

    /* PG raw active → no STOP (even during debounce) */
    if (g_ec.pg.raw) {
        return false;
    }

    /* PG currently debouncing → no STOP */
    if (g_ec.pg.raw != g_ec.pg.stable) {
        return false;
    }

    /* KEY pressed or debouncing → no STOP */
    if (g_ec.key.raw || g_ec.key.stable) {
        return false;
    }

    /* KEY timing a potential long press → no STOP */
    if (g_ec.key.timing) {
        return false;
    }

    /* SOC read in progress → no STOP */
    if (g_ec.soc.in_progress) {
        return false;
    }

    /* 10 s idle timeout */
    if (!time_elapsed(now, g_ec.last_activity_ms, IDLE_TIMEOUT_MS)) {
        return false;
    }

    return true;
}

/*============================================================================
 * STOP Execution
 *
 * Sequence:
 *   1. Final pre-check (KEY/PG raw) — cancel if either active
 *   2. Mark services suspended
 *   3. Prepare wake sources (EXTI falling edge)
 *   4. Second pre-check (KEY/PG raw) — cancel if activity during prep
 *   5. Execute WFI (STOP mode)
 *   6. Sample cutoff point (read levels, capture pending, disable EXTI)
 *   7. Restore services (clock, SysTick, I2C1)
 *   8. Build wake evidence from pending flags (NOT cutoff levels)
 *   9. Dispatch post-STOP actions
 *============================================================================*/

static void stop_execute(uint32_t now)
{
    /*
     * 1. Final pre-check: KEY or PG active right now → cancel STOP.
     *    Per spec §9.3: new KEY/PG activity in the preparation window
     *    establishes activity evidence and cancels WFI.
     */
    if (EC_Platform_IsPowerKeyPressed()) {
        refresh_activity(now);
        return;
    }
    if (EC_Platform_IsPgActive()) {
        pg_force(true, now);
        refresh_activity(now);
        return;
    }

    /*
     * 2. Mark services suspended.
     *    Only if we actually suspend services will we restore them.
     */
    bool actually_suspended = false;

    /*
     * 3. Prepare wake sources: enable KEY/PG falling edge EXTI.
     */
    EC_Platform_PrepareStopWake();
    actually_suspended = true;

    /*
     * 4. Second pre-check: activity during preparation?
     *    (EXTI pending would be set if an edge occurred during setup)
     */
    if (EC_Platform_IsPowerKeyPressed()) {
        /* Activity detected during prep — cancel STOP.
         * Per spec: do NOT restore not-yet-suspended services. */
        refresh_activity(now);
        return;
    }
    if (EC_Platform_IsPgActive()) {
        pg_force(true, now);
        refresh_activity(now);
        return;
    }

    /*
     * 5. Enter STOP (WFI with SLEEPDEEP).
     *    Returns after wakeup event.
     *
     *    Turn off all LEDs before entering STOP to save power.
     */
    LOG("[STOP] Entering STOP (WFI with SLEEPDEEP)...");
    EC_Platform_SetLedRed(false);
    EC_Platform_SetLedGreen(false);
    EC_Platform_SetLedBlue(false);
    EC_Platform_EnterStop();
    LOG("[STOP] Woke from STOP");

    /*
     * 6. Sample cutoff point.
     *    Read KEY/PG levels BEFORE disabling EXTI.
     *    Capture EXTI pending flags (set by ISRs during wakeup).
     *    Disable EXTI, clear pending.
     */
    bool key_cutoff_low = false, pg_cutoff_low = false;
    bool key_pending = false, pg_pending = false;

    EC_Platform_SampleStopCutoff(&key_cutoff_low, &pg_cutoff_low,
                                  &key_pending, &pg_pending);

    LOG("[STOP] Cutoff: KEY=%s PG=%s  Pending: KEY=%s PG=%s",
        key_cutoff_low ? "LOW" : "HIGH",
        pg_cutoff_low ? "LOW" : "HIGH",
        key_pending ? "YES" : "no",
        pg_pending ? "YES" : "no");

    /*
     * 7. Restore system services (only if actually suspended).
     */
    if (actually_suspended && EC_Platform_ServicesWereSuspended()) {
        if (!EC_Platform_RestoreAfterStop()) {
            /*
             * Critical recovery failure: try to ensure load OFF, then reset.
             */
            EC_Platform_SetLoadEnabled(false);
            EC_Platform_SystemReset();
            return;
        }
    }

    /* Re-read millis after STOP (SysTick was re-initialized) */
    now = EC_Platform_Millis();

    /*
     * 8. Build wake evidence.
     *
     * CRITICAL RULE (spec §9.3):
     *   - KEY pending = true  → KEY was the wake source (edge occurred)
     *   - PG pending  = true  → PG was the wake source (edge occurred)
     *   - KEY pending = false but KEy cutoff LOW → NOT KEY wakeup!
     *     The cutoff level alone does NOT prove KEY caused the wakeup.
     *     If KEY was LOW but no edge pending was latched, KEY was
     *     already LOW before STOP (or it went LOW after cutoff sample
     *     but before EXTI disable — the "tiny window" in §9.4).
     *
     *   - PG pending = false but PG cutoff LOW → handled similarly:
     *     was already LOW before STOP or fell in the tiny window.
     *     Still treated as PG activity for safety (inhibit STOP).
     */

    bool key_wake = key_pending;
    bool pg_wake = pg_pending;

    /*
     * 9. Dispatch post-STOP actions.
     */
    if (key_wake) {
        LOG("[STOP] Wake source: KEY (short-click=%s)",
            key_cutoff_low ? "no, still held" : "yes, released");
        refresh_activity(now);
        if (!key_cutoff_low) {
            /* Key already released → wakeup short-click */
            display_request(now);
        }
        return;
    }

    if (pg_wake) {
        LOG("[STOP] Wake source: PG (VBUS insert)  KEY_at_cutoff=%s",
            key_cutoff_low ? "LOW(not_armed)" : "HIGH");
        pg_force(true, now);
        refresh_activity(now);

        if (key_cutoff_low) {
            g_ec.key.armed = false;
            g_ec.key.timing = false;
            g_ec.key.long_event = false;
        }
        return;
    }

    /*
     * No evidence from either source → anomaly / spurious wakeup.
     */
    g_ec.anomaly_count++;
    LOG("[STOP] *** ANOMALY wakeup #%lu — no KEY/PG evidence ***",
        (unsigned long)g_ec.anomaly_count);
    refresh_activity(now);
}

/*============================================================================
 * State Handlers
 *============================================================================*/

static void handle_active_idle(uint32_t now)
{
    /* Long press → enable load */
    if (key_take_long_event()) {
        LOG("[STATE] Long-press → LOAD ON (PA1 HIGH)");
        EC_Platform_SetLoadEnabled(true);
        g_ec.state = EC_STATE_LOAD_RUNNING;
        refresh_activity(now);
        display_clear();
    }
}

static void handle_load_running(uint32_t now)
{
    /* Long press → disable load (direct PA1 LOW, no Linux ACK) */
    if (key_take_long_event()) {
        LOG("[STATE] Long-press → LOAD OFF (PA1 LOW)");
        EC_Platform_SetLoadEnabled(false);
        g_ec.state = EC_STATE_ACTIVE_IDLE;
        refresh_activity(now);
        display_clear();
    }
}

/*============================================================================
 * EC_Init — BOOT Sequence
 *
 * 1. Platform init (clock, GPIO, SysTick, I2C1)
 * 2. Gauge BOOT sequence (Probe → Design Capacity check → if power-loss:
 *    SEC → Unseal → FullAccess → gm.fs → Confirm → SEALED → NORMAL or NO_GAUGE)
 * 3. Initialize KEY/PG polling state
 * 4. If KEY LOW at BOOT end → NOT armed (must release first)
 * 5. Enter ACTIVE_IDLE
 *============================================================================*/

void EC_Init(void)
{
    uint32_t now;
    bool boot_key_low;

    /* Clear context */
    g_ec.state = EC_STATE_ACTIVE_IDLE;
    g_ec.gauge_mode = EC_GAUGE_NO_GAUGE;
    g_ec.gauge_boot_done = false;
    g_ec.soc.valid = false;
    g_ec.soc.value = 0U;
    g_ec.soc.color = EC_COLOR_OFF;
    g_ec.soc.full = false;
    g_ec.soc.last_attempt_ms = 0U;
    g_ec.soc.in_progress = false;
    g_ec.display.display_requested = false;
    g_ec.display.display_start_ms = 0U;
    g_ec.last_command_ms = 0U;
    g_ec.anomaly_count = 0U;

    /* 1. Platform init: clock, GPIO (PA1 LOW, LEDs OFF), SysTick, I2C1 */
    LOG("[INIT] Platform init start...");
    if (!EC_Platform_Init()) {
        /* Critical failure — cannot establish safe outputs.
         * Platform init already set PA1 LOW and LEDs OFF.
         * Reset the system as a last resort. */
        LOG("[INIT] *** FATAL: Platform init failed, resetting ***");
        EC_Platform_SystemReset();
        return;
    }
    LOG("[INIT] Platform init OK (clock=MSI 2.1MHz, GPIO, SysTick, USART2, I2C1)");

    now = EC_Platform_Millis();

    /* 2. Gauge BOOT */
    g_ec.gauge_boot_done = true;
    LOG("[INIT] Starting gauge BOOT sequence...");
    if (BQ27220_GaugeBoot(now)) {
        g_ec.gauge_mode = EC_GAUGE_NORMAL;
        LOG("[INIT] Gauge BOOT: NORMAL (full access OK)");
        BQ27220_PrintSummary();
    } else {
        g_ec.gauge_mode = EC_GAUGE_NO_GAUGE;
        LOG("[INIT] Gauge BOOT: NO_GAUGE (init failed, no further gauge access)");
    }

    /* Update command guard from BOOT activity */
    g_ec.last_command_ms = now;

    /* 3. Initialize KEY/PG polling state */
    boot_key_low = EC_Platform_IsPowerKeyPressed();

    g_ec.key.raw = boot_key_low;
    g_ec.key.stable = boot_key_low;
    g_ec.key.changed_at_ms = now;

    if (boot_key_low) {
        LOG("[INIT] KEY held at boot — NOT armed (must release first)");
        g_ec.key.armed = false;
    } else {
        LOG("[INIT] KEY released at boot — armed for long-press");
        g_ec.key.armed = true;
    }
    g_ec.key.timing = false;
    g_ec.key.long_event = false;

    pg_force(EC_Platform_IsPgActive(), now);
    LOG("[INIT] PG initial state: %s", g_ec.pg.stable ? "ACTIVE (VBUS present)" : "INACTIVE (no VBUS)");

    /* 4. Activity timer starts now */
    refresh_activity(now);

    /* 5. Enter ACTIVE_IDLE (load OFF) */
    EC_Platform_SetLoadEnabled(false);
    g_ec.state = EC_STATE_ACTIVE_IDLE;
    LOG("[INIT] Entering ACTIVE_IDLE (load OFF). Boot complete.");
}

/*============================================================================
 * EC_Task — Main Loop Iteration
 *
 * Called continuously. Returns after each iteration — never blocks
 * except during STOP (which is a synchronous sleep within the iteration).
 *
 * Order within each iteration:
 *   1. Read current time
 *   2. Update PG filter (poll raw, apply 200ms debounce)
 *   3. Update KEY debounce (poll raw, apply 30ms debounce)
 *   4. Check STOP eligibility (BEFORE SOC scheduling, per spec §9.1)
 *   5. Schedule SOC read (if NORMAL and conditions met)
 *   6. Update LED outputs
 *   7. Dispatch state handler
 *============================================================================*/

void EC_Task(void)
{
    uint32_t now = EC_Platform_Millis();

    /* 1. Update inputs */
    pg_update(now);
    key_update(now);

    /* 2. Check STOP eligibility (before SOC scheduling!) */
    if (stop_should_enter(now)) {
        stop_execute(now);
        /* After STOP, re-read time (SysTick was re-initialized) */
        now = EC_Platform_Millis();
    }

    /* 3. Schedule SOC read (NORMAL mode only) */
    soc_schedule(now);

    /* 4. Update LED outputs (every iteration — no fixed delay) */
    led_apply(now);

    /* 5. Dispatch state handler */
    switch (g_ec.state) {
    case EC_STATE_ACTIVE_IDLE:
        handle_active_idle(now);
        break;
    case EC_STATE_LOAD_RUNNING:
        handle_load_running(now);
        break;
    default:
        break;
    }
}

/*============================================================================
 * Public Accessors
 *============================================================================*/

ec_state_t EC_GetState(void)
{
    return g_ec.state;
}

ec_gauge_mode_t EC_GetGaugeMode(void)
{
    return g_ec.gauge_mode;
}

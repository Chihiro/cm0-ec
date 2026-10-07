/**
 ******************************************************************************
 * @file           : ec_state_machine.c
 * @brief          : EC power management state machine — core business logic.
 *
 *   Implements the v8.0 logic specification:
 *     - Key debounce (30 ms press/release, 3 s long press)
 *     - PG debounce (200 ms)
 *     - LED patterns (2 s period, SOC colors with hysteresis, blue pulse)
 *     - Per-cycle status print (voltage, current, SOC, PG) at ≤1 Hz
 *     - STOP with strict KEY/PG wakeup evidence rules
 *     - 10 s idle timer for STOP entry
 *     - I2C2 host command: delayed PA1 load-off, seconds supplied by host
 *
 *   Design rules:
 *     - No fixed delays — all timing via EC_Platform_Millis() differences
 *     - Main loop runs continuously — EC_Task() returns after each iteration
 *     - STOP is a synchronous operation that blocks within EC_Task()
 *     - BQ25601 is NOT accessed via I2C — only PG GPIO
 *     - Voltage, Current, and RelativeStateOfCharge() read each cycle (NORMAL mode)
 *     - Voltage and current also read each polling cycle for status print
 ******************************************************************************
 */

#include "ec_state_machine.h"
#include "ec_platform.h"
#include "bq27220.h"
#include "oled_display.h"
#include "ec_host_i2c.h"

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
#define DISPLAY_REFRESH_MS      1000U

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
    bool     short_event;        /* Latched one-shot short-press event */
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

    /* Voltage & Current (read alongside SOC, printed every polling cycle) */
    uint16_t      voltage_mv;    /* Battery voltage in mV */
    bool          voltage_valid;
    int16_t       current_ma;    /* Instantaneous current in mA (+charge, -discharge) */
    bool          current_valid;
    bool          average_current_valid;
    int16_t       average_current_ma;
    bool          remaining_capacity_valid;
    uint16_t      remaining_capacity_mah;
    bool          full_charge_capacity_valid;
    uint16_t      full_charge_capacity_mah;
    bool          battery_status_valid;
    uint16_t      battery_status;
    bool          time_to_empty_valid;
    uint16_t      time_to_empty_min;
    bool          time_to_full_valid;
    uint16_t      time_to_full_min;
    uint32_t      sample_sequence;
} ec_soc_t;

typedef struct {
    uint32_t last_refresh_ms;    /* When OLED page was last redrawn */
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
    bool power_off_pending;
    uint32_t power_off_received_ms;
    uint32_t power_off_delay_ms;

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
            /* Release detected. If we were still timing a long press,
             * it was a short press (released before 3 s). */
            bool was_short = g_ec.key.timing;
            g_ec.key.armed = true;
            g_ec.key.timing = false;
            g_ec.key.long_event = false;
            if (was_short) {
                g_ec.key.short_event = true;
            }
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

static bool key_take_short_event(void)
{
    bool event = g_ec.key.short_event;
    g_ec.key.short_event = false;
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
 * OLED Display Handling
 *
 * Short press (or STOP wakeup short-click) advances to the next page.
 * A 1 s timer redraws the current page so values track SOC/current/load.
 *============================================================================*/

static void display_next_page(uint32_t now)
{
    ec_status_t st;
    EC_GetStatus(&st);
    OLED_Display_NextPage(&st);
    g_ec.display.last_refresh_ms = now;
}

static void display_refresh(uint32_t now)
{
    ec_status_t st;
    EC_GetStatus(&st);
    OLED_Display_Refresh(&st);
    g_ec.display.last_refresh_ms = now;
}

/*============================================================================
 * SOC Scheduling (NORMAL mode only)
 *
 * Rules:
 *   - Maximum once per second
 *   - Minimum 500 ms between standard commands (shared BOOT + runtime)
 *   - Voltage, Current, and RelativeStateOfCharge() are read each cycle
 *   - On success & SOC≤100: update valid, color, full flag
 *   - On failure or SOC>100: clear valid, clear full, R/G off
 *   - Status printed every cycle: V, I, SOC%, PG state, load state
 *============================================================================*/

static void host_publish(bool force)
{
    static uint32_t last_sequence;
    static ec_state_t last_state;
    static ec_gauge_mode_t last_mode;
    static bool last_pg;
    static bool last_power_off_pending;

    if (force || last_sequence != g_ec.soc.sample_sequence ||
        last_state != g_ec.state || last_mode != g_ec.gauge_mode ||
        last_pg != g_ec.pg.stable || last_power_off_pending != g_ec.power_off_pending) {
        ec_status_t st;
        EC_GetStatus(&st);
        EC_HostI2C_Publish(&st);
        last_sequence = g_ec.soc.sample_sequence;
        last_state = g_ec.state;
        last_mode = g_ec.gauge_mode;
        last_pg = g_ec.pg.stable;
        last_power_off_pending = g_ec.power_off_pending;
    }
}

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

    /* Mark attempt start BEFORE the transactions */
    g_ec.soc.last_attempt_ms = now;
    g_ec.soc.in_progress = true;
    g_ec.last_command_ms = now;

    /* ---- Read Voltage ---- */
    uint16_t voltage_raw;
    if (BQ27220_ReadVoltage(&voltage_raw)) {
        g_ec.soc.voltage_mv = voltage_raw;
        g_ec.soc.voltage_valid = true;
    } else {
        g_ec.soc.voltage_valid = false;
    }

    /* ---- Read Current ---- */
    int16_t current_raw;
    if (BQ27220_ReadCurrent(&current_raw)) {
        g_ec.soc.current_ma = current_raw;
        g_ec.soc.current_valid = true;
    } else {
        g_ec.soc.current_valid = false;
    }

    /* ---- Read SOC ---- */
    uint16_t soc_raw;
    if (BQ27220_ReadSOC(&soc_raw)) {
        if (soc_raw <= 100U) {
            /* Success: update SOC data */
            g_ec.soc.valid = true;
            g_ec.soc.value = soc_raw;
            g_ec.soc.full = (soc_raw == 100U);
            g_ec.soc.color = soc_to_color(soc_raw, g_ec.soc.color);
        } else {
            /* SOC > 100: invalid */
            g_ec.soc.valid = false;
            g_ec.soc.full = false;
            g_ec.soc.color = EC_COLOR_OFF;
        }
    } else {
        /* I2C read failed */
        g_ec.soc.valid = false;
        g_ec.soc.full = false;
        g_ec.soc.color = EC_COLOR_OFF;
    }

    /* Host telemetry: each field has independent validity. No I2C1 access
     * happens in the I2C2 ISR; it keeps serving the previous complete sample. */
    g_ec.soc.average_current_valid = BQ27220_ReadAverageCurrent(&g_ec.soc.average_current_ma);
    g_ec.soc.remaining_capacity_valid = BQ27220_ReadRemainingCapacity(&g_ec.soc.remaining_capacity_mah);
    g_ec.soc.full_charge_capacity_valid = BQ27220_ReadFullChargeCapacity(&g_ec.soc.full_charge_capacity_mah);
    g_ec.soc.battery_status_valid = BQ27220_ReadBatteryStatus(&g_ec.soc.battery_status);
    g_ec.soc.time_to_empty_valid = BQ27220_ReadTimeToEmpty(&g_ec.soc.time_to_empty_min)
                              && g_ec.soc.time_to_empty_min != UINT16_MAX;
    g_ec.soc.time_to_full_valid = BQ27220_ReadTimeToFull(&g_ec.soc.time_to_full_min)
                             && g_ec.soc.time_to_full_min != UINT16_MAX;
    g_ec.soc.sample_sequence++;

    /* ---- Print comprehensive status every polling cycle ---- */
    {
        bool pg = pg_is_stable_active();
        const char *color_str =
            g_ec.soc.color == EC_COLOR_GREEN  ? "GREEN" :
            g_ec.soc.color == EC_COLOR_YELLOW ? "YELLOW" :
            g_ec.soc.color == EC_COLOR_RED    ? "RED" : "OFF";
        const char *state_str =
            g_ec.state == EC_STATE_LOAD_RUNNING ? "RUN" : "IDLE";

        LOG("[STATUS] V=%u mV  I=%d mA  SOC=%u%%  PG=%s  %s  %s",
            g_ec.soc.voltage_valid ? g_ec.soc.voltage_mv : 0U,
            g_ec.soc.current_valid ? (int)g_ec.soc.current_ma : 0,
            g_ec.soc.valid ? g_ec.soc.value : 0U,
            pg ? "VBUS" : "BAT",
            state_str,
            color_str);
    }

    g_ec.soc.in_progress = false;
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

    /* A live transfer prevents STOP. The main-loop health check observes
     * and recovers abandoned transfers independently of the load state. */
    if (EC_HostI2C_IsBusy()) {
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

    /*
     * 4. Second pre-check: activity during preparation?
     *    (EXTI pending would be set if an edge occurred during setup)
     */
    bool key_during_prep = EC_Platform_IsPowerKeyPressed();
    bool pg_during_prep = EC_Platform_IsPgActive();
    if (key_during_prep || pg_during_prep) {
        /* Undo wake-source preparation even when no services were suspended. */
        bool key_low, pg_low, key_pending, pg_pending;
        EC_Platform_SampleStopCutoff(&key_low, &pg_low, &key_pending, &pg_pending);
        if (pg_during_prep) {
            pg_force(true, now);
        }
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
    OLED_Display_Sleep();   /* panel + charge pump off for lowest power */
    bool entered_stop = EC_Platform_EnterStop();
    actually_suspended = EC_Platform_ServicesWereSuspended();

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

    /* USART2 uses PCLK1: print only after the run clock has been restored. */
    LOG("[STOP] %s", entered_stop ? "Woke from STOP" : "Cancelled: activity or host I2C pending");
    LOG("[STOP] Cutoff: KEY=%s PG=%s  Pending: KEY=%s PG=%s",
        key_cutoff_low ? "LOW" : "HIGH",
        pg_cutoff_low ? "LOW" : "HIGH",
        key_pending ? "YES" : "no",
        pg_pending ? "YES" : "no");

    /* Wake the OLED now that I2C1 has been re-initialized by RestoreAfterStop */
    OLED_Display_Wake();

    if (!entered_stop) {
        refresh_activity(now);
        return;
    }

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
            display_next_page(now);
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

static bool host_power_off_update(void)
{
    ec_host_power_off_request_t request;
    bool already_off_request = false;
    bool received = EC_HostI2C_TakePowerOffRequest(&request);
    if (received) {
        if (g_ec.state == EC_STATE_LOAD_RUNNING) {
            g_ec.power_off_pending = true;
            g_ec.power_off_received_ms = request.received_ms;
            g_ec.power_off_delay_ms = (uint32_t)request.delay_seconds * 1000U;
        } else {
            /* An OFF command must not turn into a timer for the next boot. */
            g_ec.power_off_pending = false;
            already_off_request = true;
        }
    }

    /* A STOP IRQ can publish a timestamp newer than EC_Task's initial now. */
    uint32_t now = EC_Platform_Millis();
    if (!already_off_request && (!g_ec.power_off_pending ||
        !time_elapsed(now, g_ec.power_off_received_ms, g_ec.power_off_delay_ms))) {
        if (received) {
            LOG("[HOST] LOAD OFF requested, delay=%u s", (unsigned)request.delay_seconds);
        }
        return false;
    }

    EC_Platform_SetLoadEnabled(false);
    g_ec.state = EC_STATE_ACTIVE_IDLE;
    g_ec.power_off_pending = false;
    /* Discard a concurrent long press; a held key must be released before
     * it can start a fresh press to turn the load back on. */
    g_ec.key.armed = !g_ec.key.raw && !g_ec.key.stable;
    g_ec.key.timing = false;
    g_ec.key.long_event = false;
    g_ec.key.short_event = false;
    refresh_activity(now);
    host_publish(true);
    if (received && !already_off_request) {
        LOG("[HOST] LOAD OFF requested, delay=%u s", (unsigned)request.delay_seconds);
    }
    LOG("[STATE] Host command -> LOAD OFF (PA1 LOW)");
    display_refresh(now);
    return true;
}

static void handle_active_idle(uint32_t now)
{
    /* Long press → enable load */
    if (key_take_long_event()) {
        LOG("[STATE] Long-press → LOAD ON (PA1 HIGH)");
        g_ec.power_off_pending = false;
        EC_Platform_SetLoadEnabled(true);
        g_ec.state = EC_STATE_LOAD_RUNNING;
        refresh_activity(now);
        display_refresh(now);
    }
}

static void handle_load_running(uint32_t now)
{
    /* Long press → disable load (direct PA1 LOW, no Linux ACK) */
    if (key_take_long_event()) {
        LOG("[STATE] Long-press → LOAD OFF (PA1 LOW)");
        g_ec.power_off_pending = false;
        EC_Platform_SetLoadEnabled(false);
        g_ec.state = EC_STATE_ACTIVE_IDLE;
        refresh_activity(now);
        display_refresh(now);
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
    g_ec.soc.voltage_mv = 0U;
    g_ec.soc.voltage_valid = false;
    g_ec.soc.current_ma = 0;
    g_ec.soc.current_valid = false;
    g_ec.soc.average_current_valid = false;
    g_ec.soc.average_current_ma = 0;
    g_ec.soc.remaining_capacity_valid = false;
    g_ec.soc.remaining_capacity_mah = 0U;
    g_ec.soc.full_charge_capacity_valid = false;
    g_ec.soc.full_charge_capacity_mah = 0U;
    g_ec.soc.battery_status_valid = false;
    g_ec.soc.battery_status = 0U;
    g_ec.soc.time_to_empty_valid = false;
    g_ec.soc.time_to_empty_min = UINT16_MAX;
    g_ec.soc.time_to_full_valid = false;
    g_ec.soc.time_to_full_min = UINT16_MAX;
    g_ec.soc.sample_sequence = 0U;
    g_ec.display.last_refresh_ms = 0U;
    g_ec.last_command_ms = 0U;
    g_ec.anomaly_count = 0U;
    g_ec.power_off_pending = false;
    g_ec.power_off_received_ms = 0U;
    g_ec.power_off_delay_ms = 0U;
    host_publish(true); /* Invalid telemetry is readable during gauge BOOT. */

    /* 1. Platform init: clock, GPIO (PA1 LOW, LEDs OFF), SysTick, I2C1 */
    if (!EC_Platform_Init()) {
        /* Critical failure — cannot establish safe outputs.
         * Platform init already set PA1 LOW and LEDs OFF.
         * Reset the system as a last resort. */
        LOG("[INIT] *** FATAL: Platform init failed, resetting ***");
        EC_Platform_SystemReset();
        return;
    }
    LOG("[INIT] Platform init OK (HSI16/2, HCLK/PCLK1=8MHz, GPIO, SysTick, USART2, I2C1, I2C2 slave 0x42)");

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

    /* Gauge BOOT and summary output take real time with the IRQ timebase. */
    now = EC_Platform_Millis();
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
    g_ec.key.short_event = false;

    pg_force(EC_Platform_IsPgActive(), now);
    LOG("[INIT] PG initial state: %s", g_ec.pg.stable ? "ACTIVE (VBUS present)" : "INACTIVE (no VBUS)");

    /* 4. Activity timer starts now */
    refresh_activity(now);

    /* 5. Enter ACTIVE_IDLE (load OFF) */
    EC_Platform_SetLoadEnabled(false);
    g_ec.state = EC_STATE_ACTIVE_IDLE;
    LOG("[INIT] Entering ACTIVE_IDLE (load OFF). Boot complete.");
    host_publish(true);
}

/*============================================================================
 * EC_Task — Main Loop Iteration
 *
 * Called continuously. Gauge, UART, and OLED calls can block; the SysTick
 * interrupt continues counting their elapsed time. STOP sleeps synchronously.
 *
 * Order within each iteration:
 *   1. Read current time
 *   2. Update PG filter (poll raw, apply 200ms debounce)
 *   3. Update KEY debounce (poll raw, apply 30ms debounce)
 *   4. Accept host writes and check the load-off deadline
 *   5. Check STOP eligibility (BEFORE SOC scheduling, per spec §9.1)
 *   6. Schedule SOC read (if NORMAL and conditions met)
 *   7. Re-check host writes/deadline after blocking I/O
 *   8. Update LED outputs and dispatch state handler
 *============================================================================*/

void EC_Task(void)
{
    uint32_t now = EC_Platform_Millis();

    /* 1. Update inputs */
    pg_update(now);
    key_update(now);
    bool host_power_off_applied = host_power_off_update();
    /* A stalled bus after wake also needs recovery while the load is ON;
     * waiting for STOP eligibility leaves the host unable to read forever. */
    (void)EC_HostI2C_RecoverStalled();
    /* The host handler can refresh activity after an intervening tick. */
    now = EC_Platform_Millis();

    /* 2. Check STOP eligibility (before SOC scheduling!) */
    if (stop_should_enter(now)) {
        stop_execute(now);
        /* After STOP, re-read time (SysTick was re-initialized) */
        now = EC_Platform_Millis();
    }

    /* 3. Schedule SOC read (NORMAL mode only) */
    soc_schedule(now);

    /* A host write or the deadline may arrive during blocking gauge reads. */
    now = EC_Platform_Millis();
    host_power_off_applied |= host_power_off_update();
    now = EC_Platform_Millis();

    /* 4. Update LED outputs (every iteration — no fixed delay) */
    led_apply(now);

    /* 5. Dispatch state handler */
    if (!host_power_off_applied) {
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

    /* 6. OLED: short press advances page; 1 s timer redraws current page */
    if (key_take_short_event()) {
        display_next_page(now);
    }
    if (time_elapsed(now, g_ec.display.last_refresh_ms, DISPLAY_REFRESH_MS)) {
        display_refresh(now);
    }

    host_publish(false);
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

void EC_GetStatus(ec_status_t *out)
{
    if (out == NULL) {
        return;
    }

    out->state         = g_ec.state;
    out->gauge_mode    = g_ec.gauge_mode;
    out->soc_valid     = (g_ec.gauge_mode == EC_GAUGE_NORMAL) && g_ec.soc.valid;
    out->soc_percent   = g_ec.soc.value;
    out->voltage_valid = g_ec.soc.voltage_valid;
    out->voltage_mv    = g_ec.soc.voltage_mv;
    out->current_valid = g_ec.soc.current_valid;
    out->current_ma    = g_ec.soc.current_ma;
    out->average_current_valid = g_ec.soc.average_current_valid;
    out->average_current_ma = g_ec.soc.average_current_ma;
    out->remaining_capacity_valid = g_ec.soc.remaining_capacity_valid;
    out->remaining_capacity_mah = g_ec.soc.remaining_capacity_mah;
    out->full_charge_capacity_valid = g_ec.soc.full_charge_capacity_valid;
    out->full_charge_capacity_mah = g_ec.soc.full_charge_capacity_mah;
    out->battery_status_valid = g_ec.soc.battery_status_valid;
    out->battery_status = g_ec.soc.battery_status;
    out->time_to_empty_valid = g_ec.soc.time_to_empty_valid;
    out->time_to_empty_min = g_ec.soc.time_to_empty_min;
    out->time_to_full_valid = g_ec.soc.time_to_full_valid;
    out->time_to_full_min = g_ec.soc.time_to_full_min;
    out->sample_sequence = g_ec.soc.sample_sequence;
    out->pg_active     = pg_is_stable_active();
    out->power_off_pending = g_ec.power_off_pending;
}

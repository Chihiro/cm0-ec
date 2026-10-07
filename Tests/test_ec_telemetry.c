/* Integration of actual EC scheduling/status publication with mocked platform
 * and gauge. Compile with the same forced LL stub header as test_host_i2c.c. */
#include "ec_state_machine.h"
#include "ec_platform.h"
#include "ec_host_i2c.h"
#include "bq27220.h"
#include <assert.h>
#include <stdio.h>

I2C_TypeDef mock_i2c2;
GPIO_TypeDef mock_gpiob;
uint32_t mock_primask;
bool mock_irq_enabled, mock_start_on_address_disable;
uint32_t mock_events_on_address_disable;
static uint32_t now;
static bool gauge_ok, pg, key, suspended, cancel_at_prepare, cancel_at_enter;
static bool fail_voltage, fail_time_to_full;
static bool wake_clock_needs_restore;
static bool reset_time_after_stop;
static unsigned gauge_reads, restores, cutoffs, stop_entries, resets;
static bool load_enabled, inject_on_key_read, inject_on_gauge_read;
static uint32_t gauge_read_delay_ms;
static uint32_t millis_read_step, boot_delay_ms;
void I2C2_IRQHandler(void);
static void power_off_write(uint16_t delay_seconds);

bool EC_Platform_Init(void) { EC_HostI2C_Init(); return true; }
uint32_t EC_Platform_Millis(void) {
    /* LOG() reads millis: any output before restoring the PCLK1-based USART
     * after WFI is a regression even when the clock/sensors are mocked. */
    assert(!wake_clock_needs_restore);
    now += millis_read_step;
    return now;
}
bool EC_Platform_IsPowerKeyPressed(void) {
    if (inject_on_key_read) {
        inject_on_key_read = false;
        now++; /* An IRQ arrives after EC_Task sampled its initial now. */
        power_off_write(5);
    }
    return key;
}
bool EC_Platform_IsPgActive(void) { return pg; }
void EC_Platform_SetLoadEnabled(bool value) { load_enabled = value; }
void EC_Platform_SetLedRed(bool value) { (void)value; }
void EC_Platform_SetLedGreen(bool value) { (void)value; }
void EC_Platform_SetLedBlue(bool value) { (void)value; }
void EC_Platform_SystemReset(void) { resets++; }
void EC_Platform_PrepareStopWake(void) {
    suspended = false;
    if (cancel_at_prepare) key = true;
}
bool EC_Platform_EnterStop(void) {
    stop_entries++;
    if (cancel_at_enter) return false;
    suspended = true;
    wake_clock_needs_restore = true;
    return true;
}
void EC_Platform_SampleStopCutoff(bool *kl, bool *pl, bool *kp, bool *pp) {
    cutoffs++;
    *kl = key; *pl = pg; *kp = false; *pp = false;
}
bool EC_Platform_ServicesWereSuspended(void) { return suspended; }
bool EC_Platform_RestoreAfterStop(void) {
    restores++;
    suspended = false;
    wake_clock_needs_restore = false;
    if (reset_time_after_stop) now = 0;
    EC_HostI2C_Init();
    return true;
}
bool BQ27220_GaugeBoot(uint32_t tick) { (void)tick; now += boot_delay_ms; return gauge_ok; }
void BQ27220_PrintSummary(void) {}
bool BQ27220_ReadVoltage(uint16_t *v) {
    gauge_reads++;
    now += gauge_read_delay_ms;
    if (inject_on_gauge_read) {
        inject_on_gauge_read = false;
        power_off_write(0);
        assert(load_enabled); /* The I2C ISR must not cut power. */
    }
    if (fail_voltage) return false;
    *v = 3900;
    return true;
}
bool BQ27220_ReadCurrent(int16_t *v) { gauge_reads++; *v = 500; return true; }
bool BQ27220_ReadSOC(uint16_t *v) { gauge_reads++; *v = 50; return true; }
bool BQ27220_ReadAverageCurrent(int16_t *v) { gauge_reads++; *v = 480; return true; }
bool BQ27220_ReadRemainingCapacity(uint16_t *v) { gauge_reads++; *v = 1650; return true; }
bool BQ27220_ReadFullChargeCapacity(uint16_t *v) { gauge_reads++; *v = 3300; return true; }
bool BQ27220_ReadBatteryStatus(uint16_t *v) { gauge_reads++; *v = 1U << 3; return true; }
bool BQ27220_ReadTimeToEmpty(uint16_t *v) { gauge_reads++; *v = UINT16_MAX; return true; }
bool BQ27220_ReadTimeToFull(uint16_t *v) { gauge_reads++; if (fail_time_to_full) return false; *v = 210; return true; }

static void rx(uint8_t value)
{
    I2C2->RXDR = value;
    I2C2->ISR = I2C_ISR_RXNE | I2C_ISR_BUSY;
    I2C2_IRQHandler();
}

static void power_off_write(uint16_t delay_seconds)
{
    I2C2->ISR = I2C_ISR_ADDR | I2C_ISR_BUSY;
    I2C2_IRQHandler();
    rx(EC_HOST_REG_CONTROL);
    rx(EC_HOST_CMD_POWER_OFF);
    rx((uint8_t)delay_seconds);
    rx((uint8_t)(delay_seconds >> 8));
    I2C2->ISR = I2C_ISR_STOPF;
    I2C2_IRQHandler();
}

static void long_press(void)
{
    key = true;
    now++;
    EC_Task();
    now += 31;
    EC_Task();
    now += 3000;
    EC_Task();
    key = false;
    EC_Task();
    now += 31;
    EC_Task();
}

static void host_read_map(uint8_t *map, bool finish)
{
    I2C2->ISR = I2C_ISR_ADDR | I2C_ISR_BUSY;
    I2C2_IRQHandler();
    rx(0);
    I2C2->ISR = I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY;
    I2C2_IRQHandler();
    for (unsigned i = 0; i < EC_HOST_REGISTER_COUNT; i++) {
        I2C2->ISR = I2C_ISR_TXIS | I2C_ISR_BUSY;
        I2C2_IRQHandler();
        map[i] = I2C2->TXDR;
    }
    if (finish) {
        I2C2->ISR = I2C_ISR_NACKF | I2C_ISR_STOPF;
        I2C2_IRQHandler();
    }
}

static void start_case(bool with_gauge)
{
    GPIOB->IDR = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    now = 0;
    gauge_ok = with_gauge;
    pg = true;
    key = false;
    cancel_at_prepare = cancel_at_enter = false;
    fail_voltage = fail_time_to_full = false;
    inject_on_key_read = inject_on_gauge_read = false;
    gauge_read_delay_ms = 0;
    millis_read_step = boot_delay_ms = 0;
    reset_time_after_stop = false;
    EC_Init();
}

static void start_running(bool with_gauge)
{
    start_case(with_gauge);
    long_press();
    assert(load_enabled && EC_GetState() == EC_STATE_LOAD_RUNNING);
}

static void test_delayed_power_off(void)
{
    ec_status_t st;
    start_running(false);
    uint32_t receipt = now;
    power_off_write(5);
    assert(load_enabled);
    EC_Task();
    EC_GetStatus(&st);
    assert(load_enabled && st.power_off_pending);
    now = receipt + 4999;
    EC_Task();
    assert(load_enabled);
    now++;
    EC_Task();
    EC_GetStatus(&st);
    assert(!load_enabled && !st.power_off_pending && st.state == EC_STATE_ACTIVE_IDLE);

    /* Receipt time, not the main-loop consumption time, starts the countdown. */
    start_running(false);
    power_off_write(1);
    now += 1000;
    EC_Task();
    assert(!load_enabled);

    /* A later valid command replaces the deadline; malformed writes do not. */
    start_running(false);
    power_off_write(5);
    EC_Task();
    now += 4000;
    receipt = now;
    power_off_write(2);
    EC_Task();
    now += 1000;
    EC_Task();
    assert(load_enabled);
    I2C2->ISR = I2C_ISR_ADDR | I2C_ISR_BUSY;
    I2C2_IRQHandler();
    rx(EC_HOST_REG_CONTROL); rx(EC_HOST_CMD_POWER_OFF); rx(30); /* Missing high byte. */
    I2C2->ISR = I2C_ISR_STOPF;
    I2C2_IRQHandler();
    now = receipt + 2000;
    EC_Task();
    assert(!load_enabled);

    /* A held concurrent long press cannot re-enable power after a zero delay. */
    start_running(false);
    key = true;
    now++; EC_Task();
    now += 31; EC_Task();
    now += 3000;
    power_off_write(0);
    EC_Task();
    assert(!load_enabled);
    now += 10000; EC_Task();
    assert(!load_enabled);
    key = false; EC_Task(); now += 31; EC_Task();
    long_press();
    assert(load_enabled); /* A fresh press still works. */

    start_running(false);
    power_off_write(30); EC_Task();
    long_press();
    assert(!load_enabled); /* Manual OFF cancels the host timer. */
    long_press();
    now += 30000; EC_Task();
    assert(load_enabled);

    start_case(false);
    power_off_write(1); EC_Task();
    long_press();
    now += 1000; EC_Task();
    assert(load_enabled); /* An already-OFF command cannot affect a future boot. */

    start_running(false);
    now = UINT32_MAX - 1000U;
    power_off_write(2); EC_Task();
    now = 998; EC_Task();
    assert(load_enabled);
    now = 999; EC_Task();
    assert(!load_enabled); /* 32-bit millisecond wrap. */

    start_running(false);
    receipt = now;
    power_off_write(UINT16_MAX); EC_Task();
    now = receipt + 65535000U - 1U; EC_Task();
    assert(load_enabled);
    now++; EC_Task();
    assert(!load_enabled);

    start_running(false);
    inject_on_key_read = true;
    EC_Task();
    EC_GetStatus(&st);
    assert(load_enabled && st.power_off_pending); /* IRQ timestamp newer than task now. */
    now += 5000; EC_Task();
    assert(!load_enabled);

    start_running(true);
    power_off_write(2); EC_Task();
    now += 1999;
    gauge_read_delay_ms = 50;
    EC_Task();
    assert(!load_enabled); /* Deadline crossed during a blocking gauge read. */

    start_running(true);
    inject_on_gauge_read = true;
    now += 1000; EC_Task();
    assert(!load_enabled); /* A new zero-delay command during gauge I/O is handled this task. */

    start_case(false);
    pg = false; EC_Task(); now += 200; EC_Task();
    long_press();
    receipt = now;
    power_off_write(1); EC_Task();
    now = receipt + 1000; EC_Task();
    assert(!load_enabled);
    unsigned previous_stops = stop_entries;
    now += 9999; EC_Task();
    assert(stop_entries == previous_stops);
    now++; EC_Task();
    assert(stop_entries == previous_stops + 1); /* Idle timeout starts at actual OFF. */

    /* A tick during host processing cannot make a stale task timestamp appear
     * to be 2^32 ms after freshly updated activity, causing immediate STOP. */
    start_running(false);
    pg = false; EC_Task(); now += 200; EC_Task();
    previous_stops = stop_entries;
    millis_read_step = 1;
    power_off_write(0); EC_Task();
    assert(!load_enabled && stop_entries == previous_stops);
    millis_read_step = 0;

    /* The IRQ clock now includes gauge BOOT time. Idle timeout starts after
     * BOOT rather than expiring immediately after a slow initialization. */
    start_case(false);
    pg = false;
    boot_delay_ms = 15000;
    EC_Init();
    previous_stops = stop_entries;
    EC_Task();
    assert(stop_entries == previous_stops);
    now += 9999; EC_Task();
    assert(stop_entries == previous_stops);
    now++; EC_Task();
    assert(stop_entries == previous_stops + 1);
    boot_delay_ms = 0;
}

static void test_read_shutdown_stop_cycles(void)
{
    uint8_t map[EC_HOST_REGISTER_COUNT];
    for (unsigned reset_tick = 0; reset_tick < 2; reset_tick++) {
        start_running(false);
        reset_time_after_stop = reset_tick != 0;
        pg = false; EC_Task(); now += 200; EC_Task();
        for (unsigned cycle = 0; cycle < 3; cycle++) {
            host_read_map(map, true);
            assert(map[0] == 1 && (map[4] & EC_HOST_FLAG_LOAD_ON));
            uint32_t receipt = now;
            power_off_write(1); EC_Task();
            host_read_map(map, true);
            assert(map[4] & EC_HOST_FLAG_POWER_OFF_PENDING);
            now = receipt + 999; EC_Task();
            assert(load_enabled);
            /* A telemetry read in progress at the cutoff may never get STOP
             * after PA1 cuts power to the master. */
            host_read_map(map, false);
            now++; EC_Task();
            assert(!load_enabled && EC_HostI2C_IsBusy());
            unsigned previous_stops = stop_entries;
            now += 9999; EC_Task();
            assert(stop_entries == previous_stops && !EC_HostI2C_IsBusy());
            now++; EC_Task();
            assert(stop_entries == previous_stops + 1 && !EC_HostI2C_IsBusy());
            host_read_map(map, true);
            assert(map[0] == 1 && !(map[4] & (EC_HOST_FLAG_LOAD_ON | EC_HOST_FLAG_POWER_OFF_PENDING)));
            long_press();
            assert(load_enabled);
        }
    }

    /* Recover while running and with PG present; keep load power intact. */
    start_running(false);
    host_read_map(map, false);
    EC_Task(); /* First observation must not reset a new transfer. */
    assert(load_enabled && EC_HostI2C_IsBusy());
    unsigned previous_stops = stop_entries;
    now += EC_HOST_STALL_TIMEOUT_MS - 1U; EC_Task();
    assert(load_enabled && EC_HostI2C_IsBusy());
    now++; EC_Task();
    assert(load_enabled && !EC_HostI2C_IsBusy() && stop_entries == previous_stops);
    host_read_map(map, true);
    assert(map[4] & EC_HOST_FLAG_LOAD_ON);

    I2C2->stuck_scl = true; /* BUSY clear, SCL low after wake. */
    EC_Task();
    now += EC_HOST_STALL_TIMEOUT_MS; EC_Task();
    assert(load_enabled && !I2C2->stuck_scl && stop_entries == previous_stops);
    power_off_write(1); EC_Task(); /* Commands remain usable after recovery. */
    now += 1000; EC_Task();
    assert(!load_enabled);
}

int main(void)
{
    GPIOB->IDR = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    ec_status_t st;
    gauge_ok = true;
    pg = true;
    EC_Init();
    EC_GetStatus(&st);
    assert(st.sample_sequence == 0 && !st.time_to_full_valid);
    now = 1000;
    EC_Task();
    EC_GetStatus(&st);
    assert(st.gauge_mode == EC_GAUGE_NORMAL && st.sample_sequence == 1 && gauge_reads == 9);
    assert(st.soc_valid && st.soc_percent == 50 && st.voltage_valid && st.voltage_mv == 3900);
    assert(st.current_valid && st.current_ma == 500 && st.average_current_ma == 480);
    assert(st.time_to_full_valid && st.time_to_full_min == 210);
    assert(!st.time_to_empty_valid && st.time_to_empty_min == UINT16_MAX);
    assert(st.remaining_capacity_valid && st.remaining_capacity_mah == 1650);
    assert(st.full_charge_capacity_valid && st.full_charge_capacity_mah == 3300);
    now = 1500;
    EC_Task();
    assert(gauge_reads == 9); /* Host publication cannot add gauge accesses. */
    fail_voltage = true;
    fail_time_to_full = true;
    now = 2000;
    EC_Task();
    EC_GetStatus(&st);
    assert(st.sample_sequence == 2 && gauge_reads == 18);
    assert(!st.voltage_valid && !st.time_to_full_valid && st.current_valid && st.soc_valid);

    /* NO_GAUGE stays readable and never starts any runtime gauge reads. */
    now = 0;
    gauge_ok = false;
    pg = false;
    EC_Init();
    now = 1000;
    EC_Task();
    EC_GetStatus(&st);
    assert(st.gauge_mode == EC_GAUGE_NO_GAUGE && st.sample_sequence == 0 && gauge_reads == 18);
    assert(!st.current_valid && !st.battery_status_valid && !st.remaining_capacity_valid);

    /* In-flight host access prevents STOP before any preparation. */
    now = 10999;
    I2C2->ISR = I2C_ISR_ADDR | I2C_ISR_BUSY;
    I2C2_IRQHandler(); /* A live transfer, not an abandoned BUSY flag. */
    now = 11000;
    EC_Task();
    assert(stop_entries == 0 && cutoffs == 0);
    I2C2->ISR = I2C_ISR_STOPF;
    I2C2_IRQHandler();
    cancel_at_prepare = true;
    EC_Task();
    assert(stop_entries == 0 && cutoffs == 1 && restores == 0);

    /* A transaction racing with the last STOP check must cancel cleanly. */
    cancel_at_prepare = false;
    key = false;
    now = 0;
    EC_Init();
    cancel_at_enter = true;
    now = 11000;
    EC_Task();
    assert(stop_entries == 1 && cutoffs == 2 && restores == 0);
    cancel_at_enter = false;
    now = 22000;
    EC_Task();
    assert(stop_entries == 2 && cutoffs == 3 && restores == 1 && resets == 0);
    assert(gauge_reads == 18);
    test_delayed_power_off();
    test_read_shutdown_stop_cycles();
    puts("EC telemetry scheduling and STOP tests passed");
    return 0;
}

/* Build from ec/: cc -std=c11 -Wall -Wextra -Werror -IInc
 *   -include Tests/stubs/stm32l0xx_conf.h
 *   Tests/test_host_i2c.c Src/ec_host_i2c.c -o /tmp/ec-test-host-i2c
 */
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
uint32_t EC_Platform_Millis(void) { return now; }
void I2C2_IRQHandler(void);

static void irq(uint32_t flags)
{
    I2C2->ISR = flags;
    I2C2_IRQHandler();
}

static void pointer(uint8_t offset, bool stop)
{
    irq(I2C_ISR_ADDR | I2C_ISR_BUSY);
    I2C2->RXDR = offset;
    irq(I2C_ISR_RXNE | (stop ? I2C_ISR_STOPF : I2C_ISR_BUSY));
}

static uint8_t read_byte(void)
{
    assert((I2C2->CR1 & I2C_CR1_TXIE) != 0U);
    irq(I2C_ISR_TXIS | I2C_ISR_BUSY);
    return I2C2->TXDR;
}

static void read_map(uint8_t *map)
{
    pointer(0U, false);
    irq(I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY);
    for (unsigned i = 0; i < EC_HOST_REGISTER_COUNT; i++) map[i] = read_byte();
    irq(I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_TXIS);
    assert((I2C2->CR1 & I2C_CR1_TXIE) == 0U);
    assert(!EC_HostI2C_IsBusy());
}

static uint16_t word(const uint8_t *map, unsigned offset)
{
    return (uint16_t)(map[offset] | ((uint16_t)map[offset + 1U] << 8U));
}

static void write_bytes(const uint8_t *data, unsigned length, uint32_t ending)
{
    irq(I2C_ISR_ADDR | I2C_ISR_BUSY);
    for (unsigned i = 0; i < length; i++) {
        I2C2->RXDR = data[i];
        irq(I2C_ISR_RXNE | I2C_ISR_BUSY);
    }
    if (ending) irq(ending);
}

static void test_power_off_writes(void)
{
    ec_host_power_off_request_t request;
    uint8_t command[] = { EC_HOST_REG_CONTROL, EC_HOST_CMD_POWER_OFF, 0x34, 0x12 };
    now = 1000;
    write_bytes(command, sizeof(command), 0);
    assert(!EC_HostI2C_TakePowerOffRequest(&request)); /* No STOP yet. */
    now = 1250;
    irq(I2C_ISR_STOPF);
    assert(!EC_HostI2C_Suspend() && I2C2->own_enabled); /* Process before STOP sleep. */
    assert(!EC_HostI2C_TakePowerOffRequest(NULL));
    mock_primask = 1;
    assert(EC_HostI2C_TakePowerOffRequest(&request) && mock_primask == 1);
    mock_primask = 0;
    assert(request.delay_seconds == 0x1234 && request.received_ms == 1250);
    assert(!EC_HostI2C_TakePowerOffRequest(&request));

    /* Final delay byte arrives together with STOP; latest valid command wins. */
    command[2] = command[3] = 0xFF;
    write_bytes(command, 3, 0);
    I2C2->RXDR = command[3];
    irq(I2C_ISR_RXNE | I2C_ISR_STOPF);
    assert(EC_HostI2C_TakePowerOffRequest(&request) && request.delay_seconds == 65535);
    write_bytes(command, 4, I2C_ISR_STOPF);
    command[2] = command[3] = 0;
    now = 1300;
    write_bytes(command, 4, I2C_ISR_STOPF);
    assert(EC_HostI2C_TakePowerOffRequest(&request));
    assert(request.delay_seconds == 0 && request.received_ms == 1300);

    for (unsigned length = 0; length < 4; length++) {
        write_bytes(command, length, I2C_ISR_STOPF);
        assert(!EC_HostI2C_TakePowerOffRequest(&request));
    }
    uint8_t oversized[260] = { EC_HOST_REG_CONTROL, EC_HOST_CMD_POWER_OFF, 5, 0 };
    write_bytes(oversized, 5, I2C_ISR_STOPF);
    assert(!EC_HostI2C_TakePowerOffRequest(&request));
    write_bytes(oversized, sizeof(oversized), I2C_ISR_STOPF);
    assert(!EC_HostI2C_TakePowerOffRequest(&request)); /* Count must not wrap. */
    command[1] = 0;
    write_bytes(command, 4, I2C_ISR_STOPF);
    assert(!EC_HostI2C_TakePowerOffRequest(&request));
    command[1] = EC_HOST_CMD_POWER_OFF;
    command[0] = EC_HOST_REG_VOLTAGE_MV;
    write_bytes(command, 4, I2C_ISR_STOPF);
    assert(!EC_HostI2C_TakePowerOffRequest(&request));
    command[0] = 0x21;
    write_bytes(command, 4, I2C_ISR_STOPF);
    assert(!EC_HostI2C_TakePowerOffRequest(&request));
    command[0] = EC_HOST_REG_CONTROL;

    uint32_t errors[] = { I2C_ISR_BERR, I2C_ISR_ARLO, I2C_ISR_OVR, I2C_ISR_NACKF };
    for (unsigned i = 0; i < sizeof(errors) / sizeof(errors[0]); i++) {
        write_bytes(command, 3, 0);
        I2C2->RXDR = command[3];
        irq(I2C_ISR_RXNE | I2C_ISR_STOPF | errors[i]);
        assert(!EC_HostI2C_TakePowerOffRequest(&request));
    }
    write_bytes(command, 4, 0);
    irq(I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY); /* Repeated START discards command. */
    assert(read_byte() == 0xFF);
    irq(I2C_ISR_NACKF | I2C_ISR_STOPF);
    assert(!EC_HostI2C_TakePowerOffRequest(&request));

    write_bytes(command, 4, I2C_ISR_STOPF | I2C_ISR_ADDR | I2C_ISR_BUSY);
    assert(EC_HostI2C_TakePowerOffRequest(&request)); /* STOP before next ADDR still commits. */
    irq(I2C_ISR_STOPF);
    write_bytes(command, 4, 0);
    EC_HostI2C_Init();
    irq(I2C_ISR_STOPF);
    assert(!EC_HostI2C_TakePowerOffRequest(&request)); /* Restore discards incomplete frames. */

    /* Hardware STOP clears BUSY before the IRQ commits the control frame.
     * Suspending in that window must preserve the received command. */
    command[2] = 5;
    write_bytes(command, 4, 0);
    I2C2->ISR = I2C_ISR_STOPF; /* The ISR has not run yet. */
    assert(!EC_HostI2C_Suspend());
    assert(I2C2->enabled && I2C2->own_enabled && mock_irq_enabled);
    I2C2_IRQHandler();
    assert(EC_HostI2C_TakePowerOffRequest(&request) && request.delay_seconds == 5);

    /* The same STOP/RXNE can arrive inside the address-disable window. */
    write_bytes(command, 3, 0);
    I2C2->RXDR = command[3];
    I2C2->ISR = 0;
    mock_events_on_address_disable = I2C_ISR_RXNE | I2C_ISR_STOPF;
    assert(!EC_HostI2C_Suspend() && I2C2->own_enabled);
    mock_events_on_address_disable = 0;
    I2C2_IRQHandler();
    assert(EC_HostI2C_TakePowerOffRequest(&request) && request.delay_seconds == 5);

    /* A read's NACK/STOP coinciding with the next write address must not
     * poison that write or leave its RX bytes interpreted as read data. */
    pointer(0, false);
    irq(I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY);
    assert(read_byte() == EC_HOST_PROTOCOL_VERSION);
    irq(I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_ADDR | I2C_ISR_BUSY);
    for (unsigned i = 0; i < sizeof(command); i++) {
        I2C2->RXDR = command[i];
        irq(I2C_ISR_RXNE | (i + 1U == sizeof(command) ? I2C_ISR_STOPF : I2C_ISR_BUSY));
    }
    assert(EC_HostI2C_TakePowerOffRequest(&request) && request.delay_seconds == 5);

    write_bytes(command, 4, I2C_ISR_STOPF);
    uint8_t map[EC_HOST_REGISTER_COUNT];
    read_map(map);
    assert(EC_HostI2C_TakePowerOffRequest(&request) && request.delay_seconds == 5);
}

static void test_stalled_recovery(void)
{
    ec_host_power_off_request_t request;
    uint8_t command[] = { EC_HOST_REG_CONTROL, EC_HOST_CMD_POWER_OFF, 5, 0 };
    now = 100;
    EC_HostI2C_Init();
    now = 200;
    write_bytes(command, 3, 0); /* Partial write, then master loses power. */
    assert(!EC_HostI2C_RecoverStalled()); /* Start observing the stalled bus. */
    now = 1199;
    assert(!EC_HostI2C_RecoverStalled() && I2C2->enabled);
    now++;
    assert(EC_HostI2C_RecoverStalled() && !EC_HostI2C_IsBusy());
    assert(mock_irq_enabled && I2C2->own_enabled && I2C2->CR1 == 0xBDU);
    irq(I2C_ISR_STOPF);
    assert(!EC_HostI2C_TakePowerOffRequest(&request));
    uint8_t map[EC_HOST_REGISTER_COUNT];
    read_map(map);
    assert(map[0] == 1 && map[1] == 28 && word(map, 24) == 0x5678);

    /* All unserviced events must get an ISR, even if BUSY is stale. */
    const uint32_t events[] = { I2C_ISR_ADDR, I2C_ISR_RXNE, I2C_ISR_TXIS,
        I2C_ISR_STOPF, I2C_ISR_NACKF, I2C_ISR_BERR, I2C_ISR_ARLO, I2C_ISR_OVR };
    now += 10000;
    for (unsigned i = 0; i < sizeof(events) / sizeof(events[0]); i++) {
        I2C2->ISR = I2C_ISR_BUSY | events[i];
        assert(!EC_HostI2C_RecoverStalled());
        assert(!EC_HostI2C_Suspend());
    }
    I2C2->ISR = 0;
    EC_HostI2C_Init();
    write_bytes(command, 4, I2C_ISR_STOPF);
    I2C2->ISR = I2C_ISR_BUSY;
    now += 10000;
    assert(!EC_HostI2C_RecoverStalled()); /* Do not lose completed command. */
    assert(EC_HostI2C_TakePowerOffRequest(&request) && request.delay_seconds == 5);
    mock_primask = 1;
    assert(!EC_HostI2C_RecoverStalled());
    now += EC_HOST_STALL_TIMEOUT_MS;
    assert(EC_HostI2C_RecoverStalled() && mock_primask == 1);
    mock_primask = 0;

    now = UINT32_MAX - 500U;
    EC_HostI2C_Init();
    I2C2->ISR = I2C_ISR_BUSY;
    assert(!EC_HostI2C_RecoverStalled());
    now = 498;
    assert(!EC_HostI2C_RecoverStalled());
    now = 499;
    assert(EC_HostI2C_RecoverStalled()); /* Millisecond wrap. */

    /* The previous event may be old, but a START observed just now must
     * still get a full timeout. Do not reset before the address arrives. */
    now = 0;
    EC_HostI2C_Init();
    now = 30000;
    I2C2->ISR = I2C_ISR_BUSY;
    assert(!EC_HostI2C_RecoverStalled());
    now += EC_HOST_STALL_TIMEOUT_MS - 1U;
    assert(!EC_HostI2C_RecoverStalled());
    irq(I2C_ISR_ADDR | I2C_ISR_BUSY); /* Actual progress restarts observation. */
    assert(!EC_HostI2C_RecoverStalled());
    now++;
    assert(!EC_HostI2C_RecoverStalled());
    now += EC_HOST_STALL_TIMEOUT_MS - 1U;
    assert(EC_HostI2C_RecoverStalled());

    /* Reported hardware symptom: SCL low with BUSY/ADDR/TXIS all clear.
     * Resetting must not depend exclusively on the BUSY flag. */
    EC_HostI2C_Init();
    I2C2->stuck_scl = true;
    assert(!EC_HostI2C_RecoverStalled());
    now += EC_HOST_STALL_TIMEOUT_MS;
    assert(EC_HostI2C_RecoverStalled());
    assert(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13));
    read_map(map); /* Telemetry is usable again. */
    I2C2->ISR = 0U; /* Completed read; no synthetic simultaneous TXIS remains. */

    /* If the host holds SCL low, one reset cannot release it. Do not reset
     * the slave every second throughout that same external fault. */
    GPIOB->IDR &= ~LL_GPIO_PIN_13;
    assert(!EC_HostI2C_RecoverStalled());
    now += EC_HOST_STALL_TIMEOUT_MS;
    assert(EC_HostI2C_RecoverStalled());
    now += 10000;
    assert(!EC_HostI2C_RecoverStalled());
    GPIOB->IDR |= LL_GPIO_PIN_13;
    assert(!EC_HostI2C_RecoverStalled());
    GPIOB->IDR &= ~LL_GPIO_PIN_13;
    assert(!EC_HostI2C_RecoverStalled());
    now += EC_HOST_STALL_TIMEOUT_MS;
    assert(EC_HostI2C_RecoverStalled()); /* A later fault gets a new attempt. */
    GPIOB->IDR |= LL_GPIO_PIN_13;
    EC_HostI2C_Init();
}

int main(void)
{
    GPIOB->IDR = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    ec_status_t st = {
        .state = EC_STATE_LOAD_RUNNING, .gauge_mode = EC_GAUGE_NORMAL,
        .soc_valid = true, .soc_percent = 73,
        .voltage_valid = true, .voltage_mv = 3812,
        .current_valid = true, .current_ma = -1234,
        .average_current_valid = true, .average_current_ma = -1000,
        .remaining_capacity_valid = true, .remaining_capacity_mah = 2409,
        .full_charge_capacity_valid = true, .full_charge_capacity_mah = 3300,
        .battery_status_valid = true, .battery_status = BQ27220_BATTSTAT_DSG,
        .time_to_empty_valid = true, .time_to_empty_min = 145,
        .time_to_full_valid = false, .time_to_full_min = UINT16_MAX,
        .sample_sequence = 0x12345678U, .pg_active = true
    };
    uint8_t map[EC_HOST_REGISTER_COUNT];
    EC_HostI2C_Publish(&st);
    mock_primask = 1U;
    EC_HostI2C_Init();
    assert(mock_primask == 1U); /* Init preserves an already masked caller. */
    mock_primask = 0U;
    EC_HostI2C_Init();
    assert(mock_primask == 0U);
    assert(mock_irq_enabled && I2C2->enabled && I2C2->own_enabled && I2C2->stretching);
    assert(I2C2->OAR1 == 0x8084U && I2C2->CR1 == 0xBDU);
    assert(I2C2->TIMINGR == 0x00303F3FU);
    assert(GPIOB->af[13] == 5 && GPIOB->af[14] == 5);
    assert(GPIOB->mode[13] == LL_GPIO_MODE_ALTERNATE && GPIOB->mode[14] == LL_GPIO_MODE_ALTERNATE);
    assert(GPIOB->pull[13] == 0 && GPIOB->pull[14] == 0 && GPIOB->output_type == LL_GPIO_OUTPUT_OPENDRAIN);
    assert((I2C2->CR1 & I2C_CR1_TXIE) == 0U);

    read_map(map);
    assert(map[0] == 1 && map[1] == 28 && map[4] == (7 | EC_HOST_FLAG_POWER_OFF_SUPPORTED));
    assert(word(map, 2) == (0x1FFU & ~EC_HOST_VALID_TIME_TO_FULL));
    assert(map[5] == EC_HOST_BATTERY_DISCHARGING);
    assert(word(map, 6) == 3812 && (int16_t)word(map, 8) == -1234);
    assert(word(map, 10) == 73 && word(map, 12) == UINT16_MAX && word(map, 14) == 145);
    assert(word(map, 16) == 2409 && word(map, 18) == 3300 && (int16_t)word(map, 20) == -1000);
    assert(word(map, 22) == 1 && word(map, 24) == 0x5678 && word(map, 26) == 0x1234);

    /* Publishing twice during a read must not tear the old sample, even when
     * main reuses the bank from which this transaction initially copied. */
    pointer(6, true);
    irq(I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY);
    assert(read_byte() == (3812 & 0xFF));
    st.voltage_mv = 4200;
    st.current_ma = 123;
    st.time_to_full_valid = true;
    st.time_to_full_min = 0;
    EC_HostI2C_Publish(&st);
    st.voltage_mv = 4100;
    EC_HostI2C_Publish(&st);
    assert(read_byte() == (3812 >> 8));
    assert(read_byte() == ((uint16_t)-1234 & 0xFF));
    irq(I2C_ISR_NACKF); /* A NACK without STOP must stop TX interrupts. */
    assert((I2C2->CR1 & I2C_CR1_TXIE) == 0U);
    irq(I2C_ISR_STOPF);
    read_map(map);
    assert(word(map, 6) == 4100 && map[5] == EC_HOST_BATTERY_CHARGING);
    assert((word(map, 2) & EC_HOST_VALID_TIME_TO_FULL) && word(map, 12) == 0);

    /* Last pointer byte and repeated read address in the same interrupt. */
    irq(I2C_ISR_ADDR | I2C_ISR_BUSY);
    I2C2->RXDR = 10;
    irq(I2C_ISR_RXNE | I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY);
    assert(read_byte() == 73 && read_byte() == 0);
    irq(I2C_ISR_NACKF | I2C_ISR_STOPF);

    /* Additional writes cannot modify telemetry or move the register pointer. */
    pointer(6, false);
    I2C2->RXDR = 0;
    irq(I2C_ISR_RXNE | I2C_ISR_STOPF);
    irq(I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY);
    assert(read_byte() == (4100 & 0xFF));
    irq(I2C_ISR_NACKF | I2C_ISR_STOPF);

    pointer(255, true);
    irq(I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY);
    for (unsigned i = 0; i < 300; i++) assert(read_byte() == 0xFF);
    irq(I2C_ISR_NACKF | I2C_ISR_STOPF);

    /* Recover cleanly after errors and after consecutive one-byte reads. */
    const uint32_t errors[] = { I2C_ISR_BERR, I2C_ISR_ARLO, I2C_ISR_OVR };
    for (unsigned i = 0; i < 3; i++) {
        pointer(0, false);
        irq(I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY);
        irq(errors[i] | I2C_ISR_TXIS | I2C_ISR_BUSY);
        assert((I2C2->ISR & errors[i]) == 0 && (I2C2->CR1 & I2C_CR1_TXIE) == 0);
        assert(I2C2->enabled && !EC_HostI2C_IsBusy());
        pointer(0, true);
        irq(I2C_ISR_ADDR | I2C_ISR_DIR | I2C_ISR_BUSY);
        assert(read_byte() == 1);
        irq(I2C_ISR_NACKF | I2C_ISR_STOPF);
    }

    st.time_to_full_min = UINT16_MAX; /* Sentinel clears validity despite true flag. */
    st.voltage_valid = false;
    st.battery_status = 1U << 9; /* Correct BQ27220 FC bit, not bit 3. */
    EC_HostI2C_Publish(&st);
    read_map(map);
    assert(word(map, 6) == UINT16_MAX && word(map, 12) == UINT16_MAX);
    assert((word(map, 2) & (EC_HOST_VALID_VOLTAGE | EC_HOST_VALID_TIME_TO_FULL)) == 0);
    assert(map[5] == EC_HOST_BATTERY_FULL);
    st.current_ma = -1; /* FC can remain set just after discharge starts. */
    EC_HostI2C_Publish(&st);
    read_map(map);
    assert(map[5] == EC_HOST_BATTERY_DISCHARGING);
    st.current_ma = 0;
    st.battery_status = 1U << 3; /* BATTPRES must not be treated as FC. */
    EC_HostI2C_Publish(&st);
    read_map(map);
    assert(map[5] == EC_HOST_BATTERY_IDLE);
    st.current_valid = false;
    EC_HostI2C_Publish(&st);
    read_map(map);
    assert(map[5] == EC_HOST_BATTERY_UNKNOWN && (word(map, 2) & EC_HOST_VALID_CURRENT) == 0);
    st.gauge_mode = EC_GAUGE_NO_GAUGE;
    EC_HostI2C_Publish(&st);
    read_map(map);
    assert(word(map, 2) == 0 && map[4] == (3 | EC_HOST_FLAG_POWER_OFF_SUPPORTED) && map[5] == EC_HOST_BATTERY_UNKNOWN);
    EC_HostI2C_Publish(NULL);
    st.power_off_pending = true;
    EC_HostI2C_Publish(&st);
    read_map(map);
    assert((map[4] & EC_HOST_FLAG_POWER_OFF_PENDING) != 0);
    st.power_off_pending = false;
    EC_HostI2C_Publish(&st);
    test_power_off_writes();
    test_stalled_recovery();

    I2C2->ISR = I2C_ISR_BUSY;
    mock_primask = 1;
    assert(!EC_HostI2C_Suspend() && mock_primask == 1 && mock_irq_enabled && I2C2->enabled);
    mock_primask = 0;
    I2C2->ISR = 0;
    mock_start_on_address_disable = true;
    assert(!EC_HostI2C_Suspend() && I2C2->own_enabled && mock_primask == 0);
    mock_start_on_address_disable = false;
    I2C2->ISR = 0;
    assert(EC_HostI2C_Suspend() && !mock_irq_enabled && !I2C2->enabled && mock_primask == 0);
    assert(GPIOB->mode[13] == LL_GPIO_MODE_ANALOG && GPIOB->mode[14] == LL_GPIO_MODE_ANALOG);
    EC_HostI2C_Init();
    read_map(map);
    assert(word(map, 2) == 0 && word(map, 24) == 0x5678); /* Restore keeps publication. */
    puts("host I2C protocol tests passed");
    return 0;
}

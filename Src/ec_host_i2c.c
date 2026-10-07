#include "ec_host_i2c.h"
#include "bq27220.h"
#include "ec_platform.h"
#include "stm32l0xx_conf.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* Double buffering lets main publish without masking interrupts. At each read
 * address match, the ISR freezes one bank for the entire read transaction. */
static uint8_t s_published[2][EC_HOST_REGISTER_COUNT] = {
    { EC_HOST_PROTOCOL_VERSION, EC_HOST_REGISTER_COUNT },
    { EC_HOST_PROTOCOL_VERSION, EC_HOST_REGISTER_COUNT }
};
static volatile uint8_t s_active_bank;
static uint8_t s_read_snapshot[EC_HOST_REGISTER_COUNT];
static uint16_t s_offset; /* Saturates at 256: out-of-range reads never wrap. */
static bool s_expect_pointer;
static bool s_transmitting;
static bool s_write_is_control;
static uint8_t s_write_count; /* Payload bytes, saturating at 4 (invalid). */
static uint8_t s_power_off_payload[3];
static ec_host_power_off_request_t s_power_off_request;
static volatile bool s_power_off_requested;
static volatile bool s_stall_observed;
static uint32_t s_stall_since_ms;
static volatile bool s_recovery_latched;

#define HOST_EVENT_FLAGS (I2C_ISR_ADDR | I2C_ISR_RXNE | I2C_ISR_TXIS | \
    I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)

static void put_word(uint8_t *map, uint8_t offset, uint16_t value)
{
    map[offset] = (uint8_t)value;
    map[offset + 1U] = (uint8_t)(value >> 8U);
}

static uint8_t battery_state(const ec_status_t *st)
{
    if (st->gauge_mode != EC_GAUGE_NORMAL || !st->current_valid) {
        return EC_HOST_BATTERY_UNKNOWN;
    }
    /* Current sign takes priority: a full battery can already be discharging. */
    if (st->current_ma < 0) {
        return EC_HOST_BATTERY_DISCHARGING;
    }
    if (st->battery_status_valid && (st->battery_status & BQ27220_BATTSTAT_FC)) {
        return EC_HOST_BATTERY_FULL;
    }
    if (st->current_ma > 0) {
        return EC_HOST_BATTERY_CHARGING;
    }
    return EC_HOST_BATTERY_IDLE;
}

void EC_HostI2C_Publish(const ec_status_t *st)
{
    if (st == NULL) {
        return;
    }

    uint8_t bank = s_active_bank ^ 1U;
    uint8_t *map = s_published[bank];
    uint16_t valid = 0U;
    bool normal = st->gauge_mode == EC_GAUGE_NORMAL;
    bool ttf_valid = normal && st->time_to_full_valid && st->time_to_full_min != UINT16_MAX;
    bool tte_valid = normal && st->time_to_empty_valid && st->time_to_empty_min != UINT16_MAX;

    if (normal && st->voltage_valid)            valid |= EC_HOST_VALID_VOLTAGE;
    if (normal && st->current_valid)            valid |= EC_HOST_VALID_CURRENT;
    if (normal && st->soc_valid)                valid |= EC_HOST_VALID_SOC;
    if (ttf_valid)                             valid |= EC_HOST_VALID_TIME_TO_FULL;
    if (tte_valid)                             valid |= EC_HOST_VALID_TIME_TO_EMPTY;
    if (normal && st->remaining_capacity_valid) valid |= EC_HOST_VALID_REMAINING;
    if (normal && st->full_charge_capacity_valid) valid |= EC_HOST_VALID_FULL_CHARGE;
    if (normal && st->average_current_valid)    valid |= EC_HOST_VALID_AVERAGE_CURRENT;
    if (normal && st->battery_status_valid)     valid |= EC_HOST_VALID_BATTERY_STATUS;

    map[EC_HOST_REG_VERSION] = EC_HOST_PROTOCOL_VERSION;
    map[EC_HOST_REG_LENGTH] = EC_HOST_REGISTER_COUNT;
    put_word(map, EC_HOST_REG_VALID, valid);
    map[EC_HOST_REG_FLAGS] = (st->pg_active ? EC_HOST_FLAG_VBUS : 0U)
                          | (st->state == EC_STATE_LOAD_RUNNING ? EC_HOST_FLAG_LOAD_ON : 0U)
                          | (normal ? EC_HOST_FLAG_GAUGE_NORMAL : 0U)
                          | EC_HOST_FLAG_POWER_OFF_SUPPORTED
                          | (st->power_off_pending ? EC_HOST_FLAG_POWER_OFF_PENDING : 0U);
    map[EC_HOST_REG_BATTERY_STATE] = battery_state(st);
    put_word(map, EC_HOST_REG_VOLTAGE_MV, (valid & EC_HOST_VALID_VOLTAGE) ? st->voltage_mv : UINT16_MAX);
    put_word(map, EC_HOST_REG_CURRENT_MA, (valid & EC_HOST_VALID_CURRENT) ? (uint16_t)st->current_ma : 0U);
    put_word(map, EC_HOST_REG_SOC_PERCENT, (valid & EC_HOST_VALID_SOC) ? st->soc_percent : UINT16_MAX);
    put_word(map, EC_HOST_REG_TIME_TO_FULL_MIN, ttf_valid ? st->time_to_full_min : UINT16_MAX);
    put_word(map, EC_HOST_REG_TIME_TO_EMPTY_MIN, tte_valid ? st->time_to_empty_min : UINT16_MAX);
    put_word(map, EC_HOST_REG_REMAINING_MAH, (valid & EC_HOST_VALID_REMAINING) ? st->remaining_capacity_mah : UINT16_MAX);
    put_word(map, EC_HOST_REG_FULL_CHARGE_MAH, (valid & EC_HOST_VALID_FULL_CHARGE) ? st->full_charge_capacity_mah : UINT16_MAX);
    put_word(map, EC_HOST_REG_AVERAGE_CURRENT_MA, (valid & EC_HOST_VALID_AVERAGE_CURRENT) ? (uint16_t)st->average_current_ma : 0U);
    put_word(map, EC_HOST_REG_BATTERY_STATUS, (valid & EC_HOST_VALID_BATTERY_STATUS) ? st->battery_status : UINT16_MAX);
    put_word(map, EC_HOST_REG_SAMPLE_SEQUENCE, (uint16_t)st->sample_sequence);
    put_word(map, EC_HOST_REG_SAMPLE_SEQUENCE + 2U, (uint16_t)(st->sample_sequence >> 16U));

    __DMB();
    s_active_bank = bank;
}

void EC_HostI2C_Init(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    NVIC_DisableIRQ(I2C2_IRQn);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
    /* Disconnect the old peripheral output before restoring its registers. */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ANALOG);
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C2);
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C2);

    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_13, LL_GPIO_AF_5);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_14, LL_GPIO_AF_5);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_13 | LL_GPIO_PIN_14, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_14, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);

    /* I2C2 is clocked by PCLK1=8 MHz (HSI16, AHB /2, APB1 /1).
     * ES0251 2.12.3 requires >=4 MHz for tSU;DAT=250 ns. Standard mode only.
     * SCLDEL = 4 * 125 ns; SDADEL = 0, analog filter enabled.
     * SCL high/low fields are unused as a slave; master supplies SCL. */
    LL_I2C_SetTiming(I2C2, __LL_I2C_CONVERT_TIMINGS(0U, 3U, 0U, 0x3FU, 0x3FU));
    LL_I2C_SetMode(I2C2, LL_I2C_MODE_I2C);
    LL_I2C_EnableAnalogFilter(I2C2);
    LL_I2C_SetDigitalFilter(I2C2, 0U);
    LL_I2C_EnableClockStretching(I2C2);
    LL_I2C_SetOwnAddress1(I2C2, EC_HOST_I2C_ADDRESS << 1U, LL_I2C_OWNADDRESS1_7BIT);
    LL_I2C_EnableOwnAddress1(I2C2);

    s_offset = 0U;
    s_expect_pointer = false;
    s_transmitting = false;
    s_write_is_control = false;
    s_write_count = 0U;
    s_power_off_requested = false;
    s_stall_observed = false;
    s_recovery_latched = false;
    LL_I2C_EnableIT_ADDR(I2C2);
    LL_I2C_EnableIT_RX(I2C2);
    LL_I2C_EnableIT_NACK(I2C2);
    LL_I2C_EnableIT_STOP(I2C2);
    LL_I2C_EnableIT_ERR(I2C2);
    NVIC_ClearPendingIRQ(I2C2_IRQn);
    NVIC_SetPriority(I2C2_IRQn, 1U);
    NVIC_EnableIRQ(I2C2_IRQn);
    /* Accept addresses only after software state and IRQ delivery are ready.
     * Never clear a live address-match interrupt after setting PE. */
    LL_I2C_Enable(I2C2);
    __DSB();
    __set_PRIMASK(mask);
}

bool EC_HostI2C_IsBusy(void)
{
    return LL_I2C_IsActiveFlag_BUSY(I2C2) != 0U;
}

bool EC_HostI2C_RecoverStalled(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t flags = I2C2->ISR;
    bool scl_high = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13) != 0U;
    bool idle = (flags & I2C_ISR_BUSY) == 0U && scl_high;
    if (idle) {
        s_recovery_latched = false;
    }
    if ((I2C2->CR1 & I2C_CR1_PE) == 0U || idle ||
        (flags & HOST_EVENT_FLAGS) != 0U || s_power_off_requested) {
        /* Preserve unserviced events and completed commands. In particular,
         * BUSY can already be clear while the last RXNE/STOP awaits its ISR. */
        s_stall_observed = false;
        __set_PRIMASK(mask);
        return false;
    }
    uint32_t now = EC_Platform_Millis();
    if (!s_stall_observed) {
        /* Time from first observation, not an old IRQ timestamp: a START
         * that has just arrived must have time to reach address matching. */
        s_stall_since_ms = now;
        s_stall_observed = true;
    }
    if (s_recovery_latched ||
        (uint32_t)(now - s_stall_since_ms) < EC_HOST_STALL_TIMEOUT_MS) {
        __set_PRIMASK(mask);
        return false;
    }

    uint32_t cr1 = I2C2->CR1;
    uint32_t irq_enabled = NVIC_GetEnableIRQ(I2C2_IRQn);
    /* RM0377: PE=0 releases both lines. Keep it low for >=3 APB cycles.
     * GPIO input mode also disconnects the AF driver for a physical check;
     * do not generate clocks on a bus that belongs to the host. */
    LL_I2C_Disable(I2C2);
    (void)I2C2->CR1;
    (void)I2C2->CR1;
    (void)I2C2->CR1;
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_INPUT);
    __DSB();
    uint32_t released_scl = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13) != 0U;
    uint32_t released_sda = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_14) != 0U;
    EC_HostI2C_Init(); /* Preserve telemetry; discard only the partial frame. */
    s_recovery_latched = true; /* One attempt until the bus progresses/releases. */
    __set_PRIMASK(mask);
    printf("[I2C2] RECOVERY CR1=0x%08lX ISR=0x%08lX IRQ_EN=%lu pins-released SCL=%lu SDA=%lu\r\n",
        (unsigned long)cr1, (unsigned long)flags, (unsigned long)irq_enabled,
        (unsigned long)released_scl, (unsigned long)released_sda);
    return true;
}

bool EC_HostI2C_TakePowerOffRequest(ec_host_power_off_request_t *request)
{
    if (request == NULL) {
        return false;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool pending = s_power_off_requested;
    if (pending) {
        __DMB();
        *request = s_power_off_request;
        s_power_off_requested = false;
    }
    __set_PRIMASK(mask);
    return pending;
}

static bool stop_is_blocked(void)
{
    /* BUSY drops in hardware at STOP, before the ISR necessarily consumes
     * RXNE/STOPF or commits a command. Do not discard any unserviced event.
     * TXE alone is normal while idle and must not prevent sleep. */
    const uint32_t pending = I2C_ISR_BUSY | HOST_EVENT_FLAGS;
    return (I2C2->ISR & pending) != 0U || s_power_off_requested;
}

bool EC_HostI2C_Suspend(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (stop_is_blocked()) {
        __set_PRIMASK(mask);
        return false;
    }
    /* Removing own-address acknowledgement first closes the START race. */
    LL_I2C_DisableOwnAddress1(I2C2);
    if (stop_is_blocked()) {
        LL_I2C_EnableOwnAddress1(I2C2);
        __set_PRIMASK(mask);
        return false;
    }
    NVIC_DisableIRQ(I2C2_IRQn);
    LL_I2C_Disable(I2C2);
    NVIC_ClearPendingIRQ(I2C2_IRQn);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ANALOG);
    __set_PRIMASK(mask);
    return true;
}

void I2C2_IRQHandler(void)
{
    uint32_t flags = I2C2->ISR;
    if ((flags & HOST_EVENT_FLAGS) != 0U) {
        s_stall_observed = false;
        s_recovery_latched = false;
    }

    /* RXNE can coincide with STOP or a repeated START. Consume the byte
     * before processing those boundaries so the final received byte is kept. */
    if ((flags & I2C_ISR_RXNE) != 0U) {
        uint8_t value = LL_I2C_ReceiveData8(I2C2);
        if (s_expect_pointer) {
            s_offset = value;
            s_expect_pointer = false;
            s_write_is_control = value == EC_HOST_REG_CONTROL;
            s_write_count = 0U;
        } else if (s_write_is_control) {
            if (s_write_count < sizeof(s_power_off_payload)) {
                s_power_off_payload[s_write_count] = value;
            }
            if (s_write_count < 4U) {
                s_write_count++;
            }
        }
        /* Writes to telemetry and unknown offsets are still ignored. */
    }

    if ((flags & (I2C_ISR_NACKF | I2C_ISR_STOPF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)) != 0U) {
        /* Commit only an exact, error-free standalone write after STOP.
         * The ISR never switches off the host's power. */
        if ((flags & I2C_ISR_STOPF) != 0U &&
            (flags & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)) == 0U &&
            s_write_is_control && s_write_count == sizeof(s_power_off_payload) &&
            s_power_off_payload[0] == EC_HOST_CMD_POWER_OFF) {
            s_power_off_request.delay_seconds = (uint16_t)(s_power_off_payload[1] |
                ((uint16_t)s_power_off_payload[2] << 8U));
            s_power_off_request.received_ms = EC_Platform_Millis();
            __DMB();
            s_power_off_requested = true;
        }
        s_write_is_control = false;
        s_write_count = 0U;
        LL_I2C_DisableIT_TX(I2C2);
        s_transmitting = false;
        LL_I2C_ClearFlag_TXE(I2C2); /* Flush a possibly unconsumed TXDR byte. */
        if (flags & I2C_ISR_NACKF) LL_I2C_ClearFlag_NACK(I2C2);
        if (flags & I2C_ISR_STOPF) LL_I2C_ClearFlag_STOP(I2C2);
        if (flags & I2C_ISR_BERR)  LL_I2C_ClearFlag_BERR(I2C2);
        if (flags & I2C_ISR_ARLO)  LL_I2C_ClearFlag_ARLO(I2C2);
        if (flags & I2C_ISR_OVR)   LL_I2C_ClearFlag_OVR(I2C2);
        s_expect_pointer = false;
        if (flags & (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)) {
            /* Abort the damaged transfer and release SCL. Merely masking TXIE
             * with a pending TXIS could otherwise stretch the bus forever. */
            LL_I2C_Disable(I2C2);
            /* A PE toggle is a software reset only if PE remains clear for
             * at least three APB clocks, including on the error path. */
            (void)I2C2->CR1;
            (void)I2C2->CR1;
            (void)I2C2->CR1;
            if (flags & I2C_ISR_ADDR) LL_I2C_ClearFlag_ADDR(I2C2);
            LL_I2C_Enable(I2C2);
            return;
        }
    }

    if ((flags & I2C_ISR_ADDR) != 0U) {
        /* A repeated START cannot complete a control write. */
        s_write_is_control = false;
        s_write_count = 0U;
        LL_I2C_DisableIT_TX(I2C2);
        LL_I2C_ClearFlag_TXE(I2C2);
        s_transmitting = (flags & I2C_ISR_DIR) != 0U;
        s_expect_pointer = !s_transmitting;
        if (s_transmitting) {
            memcpy(s_read_snapshot, s_published[s_active_bank], sizeof(s_read_snapshot));
            LL_I2C_EnableIT_TX(I2C2);
        }
        LL_I2C_ClearFlag_ADDR(I2C2); /* Releases SCL after the snapshot is ready. */
    }

    if (s_transmitting && LL_I2C_IsActiveFlag_TXIS(I2C2)) {
        uint8_t value = s_offset < EC_HOST_REGISTER_COUNT ? s_read_snapshot[s_offset] : 0xFFU;
        LL_I2C_TransmitData8(I2C2, value);
        if (s_offset < 256U) {
            s_offset++;
        }
    }
}

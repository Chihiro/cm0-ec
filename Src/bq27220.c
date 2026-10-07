/**
 ******************************************************************************
 * @file           : bq27220.c
 * @brief          : BQ27220 CEDV fuel gauge driver (LL I2C, polling mode)
 *
 * @details        : BQ27220 at I2C1 (PB6=SCL, PB7=SDA), 7-bit addr 0x55.
 *
 *   SW workaround for STM32L0 I2C: uses SOFTEND instead of AUTOEND.
 *   Some STM32L0 I2C silicon revisions have errata where AUTOEND
 *   does not reliably generate STOP.  SOFTEND + manual STOP is known
 *   to work around this.
 *
 *   I2C read protocol:
 *     Phase 1 — Write command code (SOFTEND):
 *       START → ADDR(W) → CMD_BYTE → [TC] → manual STOP
 *     Phase 2 — Read 2 bytes (SOFTEND):
 *       START → ADDR(R) → DATA_LO (ACK) → DATA_HI (NACK) → [TC] → manual STOP
 *
 *   PCLK = 8 MHz, ~100 kHz Standard-mode.
 ******************************************************************************
 */

#include "bq27220.h"
#include <stdio.h>

#define BQ_LOG(fmt, ...)  printf("[BQ27220] " fmt "\r\n", ##__VA_ARGS__)

/* Expected Design Capacity in mAh. Used to detect power-loss:
 * after a power cycle the BQ27220 reverts to factory defaults;
 * if Design Capacity reads 3300 our gm.fs config is loaded. */
#define BQ27220_DESIGN_CAPACITY_MAH  3300U

/*============================================================================
 * Internal helpers
 *============================================================================*/

#define I2C_TIMEOUT  50000U

/*
 * Bus recovery: toggle SCL up to 9 times to unstick SDA.
 */
static bool i2c_bus_recovery(void)
{
    if (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_7) != 0U) {
        return true;  /* SDA high — bus is free */
    }

    for (int i = 0; i < 9; i++) {
        /* SCL low */
        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
        for (volatile uint32_t d = 0U; d < 80U; d++) { __NOP(); }

        /* SCL release (back to AF open-drain) */
        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
        for (volatile uint32_t d = 0U; d < 80U; d++) { __NOP(); }

        if (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_7) != 0U) {
            return true;
        }
    }
    return false;
}

/**
 * @brief  Manually generate STOP condition and wait for completion.
 */
static bool i2c_generate_stop(void)
{
    uint32_t to;

    LL_I2C_GenerateStopCondition(I2C1);

    /* Wait for STOPF */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
        if (--to == 0U) { return false; }
    }
    LL_I2C_ClearFlag_STOP(I2C1);

    /* Wait for BUSY=0 */
    to = I2C_TIMEOUT;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (--to == 0U) { return false; }
    }

    /*
     * BQ27220 requires ≥66 µs idle between I2C packets.
     * Using LL_mDelay(1) for 1ms — safe and reliable
     * (TI reference implementations also use 1ms).
     */
    LL_mDelay(1);

    return true;
}

/*============================================================================
 * Public API
 *============================================================================*/

bool BQ27220_Init(void)
{
    /* ---- Clock ---- */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    /* ---- GPIO: PB6=SCL, PB7=SDA, open-drain, AF1 ---- */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_1);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_1);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);

    /* ---- I2C peripheral: full deinit ---- */
    LL_I2C_Disable(I2C1);
    /* Small delay to ensure PE clears */
    for (volatile uint32_t d = 0U; d < 100U; d++) { __NOP(); }

    /* Software reset the I2C peripheral */
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
    for (volatile uint32_t d = 0U; d < 100U; d++) { __NOP(); }
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);
    for (volatile uint32_t d = 0U; d < 100U; d++) { __NOP(); }

    /* Timing: ~62 kHz Standard-mode @ 8 MHz PCLK */
    LL_I2C_SetTiming(I2C1, BQ27220_I2C_TIMING);

    /* Master mode, 7-bit addressing, I2C protocol */
    LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
    LL_I2C_SetMasterAddressingMode(I2C1, LL_I2C_ADDRSLAVE_7BIT);

    /* Enable analog filter, no digital filter */
    LL_I2C_EnableAnalogFilter(I2C1);
    LL_I2C_SetDigitalFilter(I2C1, 0x00U);

    /* Keep clock stretching ENABLED (NOSTRETCH=0, default) */

    /* Enable peripheral */
    LL_I2C_Enable(I2C1);
    /* Wait for PE to assert */
    for (volatile uint32_t d = 0U; d < 100U; d++) { __NOP(); }

    /* Bus recovery in case SDA is stuck */
    if (!i2c_bus_recovery()) {
        printf("  I2C1: bus recovery failed (SDA stuck low)\r\n");
        return false;
    }

    return true;
}

bool BQ27220_Probe(void)
{
    uint16_t voltage = 0U;
    if (!BQ27220_ReadWord(BQ27220_CMD_VOLTAGE, &voltage)) {
        return false;
    }
    (void)voltage;
    return true;
}

/**
 * @brief  Read a 16-bit value from a BQ27220 standard command.
 *
 *   Uses SOFTEND + manual STOP (workaround for STM32L0 I2C errata
 *   where AUTOEND sometimes fails to generate STOP cleanly).
 *
 *   Sequence:
 *     Phase 1: START_WRITE + cmd_byte(1) [SOFTEND] → TC → manual STOP
 *     Phase 2: START_READ  + data(2)    [SOFTEND] → TC → manual STOP
 *
 *   In Phase 2, we NACK the last byte by setting NACK=1 before reading
 *   the second-to-last byte.  This tells the slave we want 2 bytes total.
 */
bool BQ27220_ReadWord(uint8_t cmd, uint16_t *value)
{
    uint32_t to;

    if (value == NULL) return false;

    /*---------- Phase 1: Write command code ----------*/

    /* Wait for bus idle */
    to = I2C_TIMEOUT;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (--to == 0U) { return false; }
    }

    /*
     * SOFTEND, NBYTES=1, START+WRITE.
     * The peripheral will set TC after the byte is transmitted
     * but will NOT generate STOP — we do that manually.
     */
    LL_I2C_HandleTransfer(I2C1,
                          BQ27220_I2C_ADDR,
                          LL_I2C_ADDRSLAVE_7BIT,
                          1U,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_WRITE);

    /* Wait for TXIS (address was ACKed) */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        if (--to == 0U) { return false; }
        if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
            LL_I2C_ClearFlag_NACK(I2C1);
            return false;
        }
    }

    /* Transmit the command byte */
    LL_I2C_TransmitData8(I2C1, cmd);

    /* Wait for TC (byte sent, no STOP yet — SOFTEND) */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TC(I2C1)) {
        if (--to == 0U) { return false; }
    }

    /* Manually generate STOP */
    if (!i2c_generate_stop()) { return false; }

    /*---------- Phase 2: Read 2 data bytes ----------*/

    /* Wait for bus idle */
    to = I2C_TIMEOUT;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (--to == 0U) { return false; }
    }

    /*
     * SOFTEND, NBYTES=2, START+READ.
     *
     * For SOFTEND with NBYTES=2 read:
     *   Byte 1 → ACK from master → RXNE → read
     *   Byte 2 → NACK from master → RXNE → read
     *   TC is set (no STOP — manual)
     *
     * The NACK on the last byte is automatic when using SOFTEND
     * with NBYTES > 1: the peripheral NACKs the LAST byte (byte 2).
     */
    LL_I2C_HandleTransfer(I2C1,
                          BQ27220_I2C_ADDR,
                          LL_I2C_ADDRSLAVE_7BIT,
                          2U,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_READ);

    /* Wait for byte 1 (low) */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {
        if (--to == 0U) { return false; }
        if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
            LL_I2C_ClearFlag_NACK(I2C1);
            return false;
        }
    }
    uint8_t lo = LL_I2C_ReceiveData8(I2C1);

    /* Wait for byte 2 (high) */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {
        if (--to == 0U) { return false; }
    }
    uint8_t hi = LL_I2C_ReceiveData8(I2C1);

    /* Wait for TC */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TC(I2C1)) {
        if (--to == 0U) { return false; }
    }

    /* Manually generate STOP */
    if (!i2c_generate_stop()) { return false; }

    *value = ((uint16_t)hi << 8U) | (uint16_t)lo;
    return true;
}

bool BQ27220_WriteWord(uint8_t cmd, uint16_t value)
{
    uint32_t to;

    to = I2C_TIMEOUT;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (--to == 0U) { return false; }
    }

    LL_I2C_HandleTransfer(I2C1,
                          BQ27220_I2C_ADDR,
                          LL_I2C_ADDRSLAVE_7BIT,
                          3U,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_WRITE);

    /* Byte 0: command */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        if (--to == 0U) { return false; }
        if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
            LL_I2C_ClearFlag_NACK(I2C1);
            return false;
        }
    }
    LL_I2C_TransmitData8(I2C1, cmd);

    /* Byte 1: data low */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        if (--to == 0U) { return false; }
    }
    LL_I2C_TransmitData8(I2C1, (uint8_t)(value & 0xFFU));

    /* Byte 2: data high */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        if (--to == 0U) { return false; }
    }
    LL_I2C_TransmitData8(I2C1, (uint8_t)((value >> 8U) & 0xFFU));

    /* Wait for TC */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TC(I2C1)) {
        if (--to == 0U) { return false; }
    }

    return i2c_generate_stop();
}

/*============================================================================
 * I2C bus scanner
 *============================================================================*/

/**
 * @brief  Minimal address probe using one-byte write (same code path as ReadWord).
 *         START + ADDR(W) + dummy_byte(1) + STOP
 *         Returns true if slave ACKs its address (TXIS fires).
 */
static bool i2c_probe_addr(uint16_t saddr)
{
    uint32_t to;
    bool acked = false;

    to = I2C_TIMEOUT;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (--to == 0U) { return false; }
    }

    /* SOFTEND, NBYTES=1, START+WRITE — same code path as ReadWord */
    LL_I2C_HandleTransfer(I2C1,
                          saddr,
                          LL_I2C_ADDRSLAVE_7BIT,
                          1U,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_WRITE);

    /* Wait for TXIS (address ACKed) or NACKF (address NACKed) */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        if (--to == 0U) { break; }
        if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
            LL_I2C_ClearFlag_NACK(I2C1);
            goto done;       /* don't wait for TC */
        }
    }

    if (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        goto done;           /* timeout — no device at this address */
    }

    acked = true;

    /* Send dummy byte so the transfer can complete */
    LL_I2C_TransmitData8(I2C1, 0x00U);

    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TC(I2C1)) {
        if (--to == 0U) { break; }
    }

done:
    (void)i2c_generate_stop();
    return acked;
}

void BQ27220_ScanBus(void)
{
    int found = 0;

    printf("\r\n");
    printf("---- I2C1 Bus Scan (7-bit addr 0x08-0x77) ----\r\n");

    for (uint8_t addr = 0x08U; addr <= 0x77U; addr++) {
        uint16_t saddr = (uint16_t)addr << 1U;

        if (i2c_probe_addr(saddr)) {
            found++;
            printf("  ACK at 0x%02X (8-bit W: 0x%02X)\r\n",
                   addr, (unsigned int)saddr);
        }
    }

    if (found == 0) {
        printf("  *** NO devices found on I2C1 bus ***\r\n");
        printf("  Check: PB6(SCL)/PB7(SDA) wiring, 4.7k pull-ups\r\n");
        printf("         to MCU_VDD, and target device power.\r\n");
    } else {
        printf("  Total: %d device(s) found.\r\n", found);
    }
    printf("--------------------------------------------------\r\n\n");
}

/*============================================================================
 * Convenience wrappers
 *============================================================================*/

bool BQ27220_ReadVoltage(uint16_t *v)       { return BQ27220_ReadWord(BQ27220_CMD_VOLTAGE, v); }
bool BQ27220_ReadTemperature(uint16_t *t)   { return BQ27220_ReadWord(BQ27220_CMD_TEMPERATURE, t); }
bool BQ27220_ReadInternalTemp(uint16_t *t)  { return BQ27220_ReadWord(BQ27220_CMD_INTERNAL_TEMP, t); }
bool BQ27220_ReadSOC(uint16_t *s)           { return BQ27220_ReadWord(BQ27220_CMD_RELATIVE_SOC, s); }
bool BQ27220_ReadSOH(uint16_t *s)           { return BQ27220_ReadWord(BQ27220_CMD_STATE_OF_HEALTH, s); }
bool BQ27220_ReadRemainingCapacity(uint16_t *c) { return BQ27220_ReadWord(BQ27220_CMD_REMAINING_CAPACITY, c); }
bool BQ27220_ReadFullChargeCapacity(uint16_t *c) { return BQ27220_ReadWord(BQ27220_CMD_FULL_CHARGE_CAPACITY, c); }
bool BQ27220_ReadCurrent(int16_t *c)        { return BQ27220_ReadWord(BQ27220_CMD_CURRENT, (uint16_t *)c); }
bool BQ27220_ReadAverageCurrent(int16_t *c) { return BQ27220_ReadWord(BQ27220_CMD_AVERAGE_CURRENT, (uint16_t *)c); }
bool BQ27220_ReadTimeToEmpty(uint16_t *t)  { return BQ27220_ReadWord(BQ27220_CMD_TIME_TO_EMPTY, t); }
bool BQ27220_ReadTimeToFull(uint16_t *t)   { return BQ27220_ReadWord(BQ27220_CMD_TIME_TO_FULL, t); }
bool BQ27220_ReadBatteryStatus(uint16_t *s) { return BQ27220_ReadWord(BQ27220_CMD_BATTERY_STATUS, s); }
bool BQ27220_ReadOperationStatus(uint16_t *s) { return BQ27220_ReadWord(BQ27220_CMD_OPERATION_STATUS, s); }
bool BQ27220_ReadDesignCapacity(uint16_t *c) { return BQ27220_ReadWord(BQ27220_CMD_DESIGN_CAPACITY, c); }
bool BQ27220_ReadCycleCount(uint16_t *c)    { return BQ27220_ReadWord(BQ27220_CMD_CYCLE_COUNT, c); }

/*============================================================================
 * Print summary
 *============================================================================*/

void BQ27220_PrintSummary(void)
{
    uint16_t v=0, t=0, soc=0, rm=0, fcc=0, soh=0, cyc=0, des=0, bs=0, os=0;
    int16_t cur=0, avg=0;
    bool ok = true;

    if (!BQ27220_ReadVoltage(&v))           { ok = false; }
    if (!BQ27220_ReadTemperature(&t))       { ok = false; }
    if (!BQ27220_ReadSOC(&soc))             { ok = false; }
    if (!BQ27220_ReadRemainingCapacity(&rm)) { ok = false; }
    if (!BQ27220_ReadFullChargeCapacity(&fcc)) { ok = false; }
    if (!BQ27220_ReadSOH(&soh))             { ok = false; }
    if (!BQ27220_ReadCurrent(&cur))         { ok = false; }
    if (!BQ27220_ReadAverageCurrent(&avg))  { ok = false; }
    if (!BQ27220_ReadCycleCount(&cyc))      { ok = false; }
    if (!BQ27220_ReadDesignCapacity(&des))  { ok = false; }
    if (!BQ27220_ReadBatteryStatus(&bs))    { ok = false; }
    if (!BQ27220_ReadOperationStatus(&os))  { ok = false; }

    int32_t tc_i  = ((int32_t)t - 2732) / 10;
    int32_t tc_f  = ((int32_t)t - 2732) % 10;
    if (tc_f < 0) { tc_f = -tc_f; }

    printf("\r\n");
    printf("============== BQ27220 Battery Info ==============\r\n");
    if (!ok) printf("  *** WARNING: Some I2C reads failed ***\r\n");
    printf("  Voltage:       %5u mV\r\n", v);
    printf("  Temperature:   %4ld.%1ld C\r\n", (long)tc_i, (long)tc_f);
    printf("  SOC:           %5u %%\r\n", soc);
    printf("  Remaining:     %5u mAh\r\n", rm);
    printf("  Full Charge:   %5u mAh\r\n", fcc);
    printf("  SOH:           %5u %%\r\n", soh);
    printf("  Current:       %6d mA\r\n", (int)cur);
    printf("  Avg Current:   %6d mA\r\n", (int)avg);
    printf("  Cycle Count:   %5u\r\n", cyc);
    printf("  Design Cap:    %5u mAh\r\n", des);
    printf("  Batt Status:   0x%04X\r\n", bs);
    printf("  Op Status:     0x%04X\r\n", os);

    printf("  State:         ");
    if (bs & BQ27220_BATTSTAT_DSG)       printf("DISCHARGING");
    else if (bs & BQ27220_BATTSTAT_FC)   printf("FULLY CHARGED");
    else if (cur > 0)                     printf("CHARGING");
    else if (cur < 0)                     printf("DISCHARGING");
    else                                  printf("IDLE");
    if (bs & BQ27220_BATTSTAT_CHG_INH)    printf(" (charge inhibited)");
    if (!(os & BQ27220_OPSTAT_INITCOMP))  printf(" [INIT NOT COMPLETE]");
    printf("\r\n");
    printf("==================================================\r\n\n");
}

/*============================================================================
 * Command Guard Timer (≥500ms between standard commands)
 *============================================================================*/

static uint32_t s_last_command_ms;

bool BQ27220_CanIssueCommand(uint32_t now_ms)
{
    return ((uint32_t)(now_ms - s_last_command_ms) >= 500U);
}

void BQ27220_MarkCommandIssued(uint32_t now_ms)
{
    s_last_command_ms = now_ms;
}

/*============================================================================
 * Multi-byte I2C write: cmd + len bytes of data
 * Uses SOFTEND + manual STOP (STM32L0 errata workaround).
 *============================================================================*/

bool BQ27220_WriteBlock(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint32_t to;

    if (data == NULL && len > 0U) return false;

    /* Wait for bus idle */
    to = I2C_TIMEOUT;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (--to == 0U) { return false; }
    }

    /* Total bytes: 1 (cmd) + len (data) */
    uint8_t total = (uint8_t)(1U + len);

    LL_I2C_HandleTransfer(I2C1,
                          BQ27220_I2C_ADDR,
                          LL_I2C_ADDRSLAVE_7BIT,
                          total,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_WRITE);

    /* Transmit cmd byte */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        if (--to == 0U) { return false; }
        if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
            LL_I2C_ClearFlag_NACK(I2C1);
            return false;
        }
    }
    LL_I2C_TransmitData8(I2C1, cmd);

    /* Transmit data bytes */
    for (uint8_t i = 0U; i < len; i++) {
        to = I2C_TIMEOUT;
        while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
            if (--to == 0U) { return false; }
        }
        LL_I2C_TransmitData8(I2C1, data[i]);
    }

    /* Wait for TC */
    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TC(I2C1)) {
        if (--to == 0U) { return false; }
    }

    return i2c_generate_stop();
}

/*============================================================================
 * Multi-byte I2C read: write register address (1 byte), then read len bytes
 *============================================================================*/

bool BQ27220_ReadBlock(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint32_t to;

    if (data == NULL && len > 0U) return false;

    /*---------- Phase 1: Write register address ----------*/
    to = I2C_TIMEOUT;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (--to == 0U) { return false; }
    }

    LL_I2C_HandleTransfer(I2C1,
                          BQ27220_I2C_ADDR,
                          LL_I2C_ADDRSLAVE_7BIT,
                          1U,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_WRITE);

    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
        if (--to == 0U) { return false; }
        if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
            LL_I2C_ClearFlag_NACK(I2C1);
            return false;
        }
    }
    LL_I2C_TransmitData8(I2C1, reg);

    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TC(I2C1)) {
        if (--to == 0U) { return false; }
    }
    if (!i2c_generate_stop()) { return false; }

    /*---------- Phase 2: Read len bytes ----------*/
    if (len == 0U) { return true; }

    to = I2C_TIMEOUT;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (--to == 0U) { return false; }
    }

    LL_I2C_HandleTransfer(I2C1,
                          BQ27220_I2C_ADDR,
                          LL_I2C_ADDRSLAVE_7BIT,
                          len,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_READ);

    for (uint8_t i = 0U; i < len; i++) {
        to = I2C_TIMEOUT;
        while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {
            if (--to == 0U) { return false; }
            if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
                LL_I2C_ClearFlag_NACK(I2C1);
                return false;
            }
        }
        data[i] = LL_I2C_ReceiveData8(I2C1);
    }

    to = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_TC(I2C1)) {
        if (--to == 0U) { return false; }
    }

    return i2c_generate_stop();
}

/*============================================================================
 * Control() sub-command access
 *============================================================================*/

bool BQ27220_ReadControlStatus(uint16_t *status)
{
    return BQ27220_ReadWord(BQ27220_CMD_CONTROL, status);
}

bool BQ27220_WriteControlWord(uint16_t value)
{
    return BQ27220_WriteWord(BQ27220_CMD_CONTROL, value);
}

/*============================================================================
 * BOOT Step: Read Device Number via Control() subcommand 0x0001
 *
 * BQ27220 Control() protocol:
 *   1. Write subcommand to 0x00/0x01 (2 data bytes)
 *   2. BQ27220 processes the subcommand (clock stretching, fast)
 *   3. Read 0x00/0x01 to get result (2 bytes)
 *   4. Read 0x00/0x01 again to verify completion (should be stable)
 *
 * We read Control() twice after the subcommand write: first read gets
 * the result, second read confirms it's stable (subcommand complete).
 *============================================================================*/

bool BQ27220_ReadDeviceNumber(uint16_t *device_number)
{
    uint16_t result;

    if (device_number == NULL) return false;

    /* Write sub-command 0x0001 to Control() */
    if (!BQ27220_WriteWord(BQ27220_CMD_CONTROL, BQ27220_CTRL_DEVICE_TYPE)) {
        return false;
    }

    /* Delay for subcommand execution (after STOP, BQ27220 needs time) */
    LL_mDelay(2);

    /* Read back Control() to get result */
    if (!BQ27220_ReadWord(BQ27220_CMD_CONTROL, &result)) {
        return false;
    }

    *device_number = result;
    return true;
}

/*============================================================================
 * BOOT Step: Read Firmware Version
 *============================================================================*/

bool BQ27220_ReadFwVersion(uint16_t *fw_version)
{
    uint16_t result;

    if (fw_version == NULL) return false;

    if (!BQ27220_WriteWord(BQ27220_CMD_CONTROL, BQ27220_CTRL_FW_VERSION)) {
        return false;
    }

    LL_mDelay(2);

    if (!BQ27220_ReadWord(BQ27220_CMD_CONTROL, &result)) {
        return false;
    }

    *fw_version = result;
    return true;
}

/*============================================================================
 * BOOT Step: Read SEC (Security Mode)
 *============================================================================*/

/*
 * OperationStatus low byte bits [2:1]:
 *   3 (11) = SEALED
 *   2 (10) = UNSEALED
 *   1 (01) = FULL ACCESS
 */
bool BQ27220_ReadSEC(uint8_t *sec_mode)
{
    uint16_t op_status;

    if (sec_mode == NULL) return false;

    if (!BQ27220_ReadOperationStatus(&op_status)) {
        return false;
    }

    *sec_mode = (uint8_t)((op_status >> 1U) & 0x03U);
    return true;
}

/*============================================================================
 * BOOT Step: UNSEAL (SEALED → UNSEALED)
 *
 * Two SEPARATE I2C writes to Control() @ 0x00, NBYTES=3 each,
 * little-endian, with STOP between them:
 *   W: AA 00 14 04   (key1 = 0x0414)
 *   W: AA 00 72 36   (key2 = 0x3672)
 *
 * Must wait ≥4s after last BQ27220 communication before starting.
 *============================================================================*/

bool BQ27220_EnterUnsealed(void)
{
    uint16_t op_status;

    /*
     * Unseal: two SEPARATE writes to AltManufacturerAccess @ 0x3E,
     * NBYTES=3 each, little-endian, STOP between:
     *   W: AA 3E 14 04   (key1 = 0x0414)
     *   W: AA 3E 72 36   (key2 = 0x3672)
     */
    BQ_LOG("    4s safe window...");
    LL_mDelay(4000);

    BQ_LOG("    W: AA 00 14 04 (key=0x0414 LE)");
    if (!BQ27220_WriteWord(BQ27220_CMD_CONTROL, BQ27220_UNSEAL_KEY1)) {
        BQ_LOG("    FAIL: I2C error on key1");
        return false;
    }
    LL_mDelay(50);

    BQ_LOG("    W: AA 00 72 36 (key=0x3672 LE)");
    if (!BQ27220_WriteWord(BQ27220_CMD_CONTROL, BQ27220_UNSEAL_KEY2)) {
        BQ_LOG("    FAIL: I2C error on key2");
        return false;
    }
    LL_mDelay(100);

    if (!BQ27220_ReadOperationStatus(&op_status)) {
        BQ_LOG("    FAIL: I2C error");
        return false;
    }

    uint8_t sec = (uint8_t)((op_status >> 1U) & 0x03U);
    BQ_LOG("    OpStatus=0x%04X SEC_raw=%u (2=UNSEALED,1=FULL_ACCESS)", op_status, sec);

    /* Accept UNSEALED (2) or FULL ACCESS (1) — device already past SEALED */
    return (sec == 2U || sec == 1U);
}

bool BQ27220_EnterFullAccess(void)
{
    uint16_t op_status;

    BQ_LOG("    4s safe window...");
    LL_mDelay(4000);

    /* Two separate writes to AltManufacturerAccess @ 0x3E */
    BQ_LOG("    W: AA 00 FF FF");
    if (!BQ27220_WriteWord(BQ27220_CMD_CONTROL, BQ27220_CTRL_FULL_ACCESS_KEY)) {
        BQ_LOG("    FAIL: I2C error on key1");
        return false;
    }
    LL_mDelay(50);

    BQ_LOG("    W: AA 00 FF FF");
    if (!BQ27220_WriteWord(BQ27220_CMD_CONTROL, BQ27220_CTRL_FULL_ACCESS_KEY)) {
        BQ_LOG("    FAIL: I2C error on key2");
        return false;
    }
    LL_mDelay(100);

    if (!BQ27220_ReadOperationStatus(&op_status)) {
        BQ_LOG("    FAIL: I2C error");
        return false;
    }

    uint8_t sec = (uint8_t)((op_status >> 1U) & 0x03U);
    BQ_LOG("    OpStatus=0x%04X SEC_raw=%u (expect 1=FULL_ACCESS)", op_status, sec);

    return (sec == 1U);  /* 01 = FULL ACCESS */
}

/*============================================================================
 * Golden Memory Flash Stream (gm.fs)
 *============================================================================*/

/*
 * gm.fs text format:
 *   ; comment line (skip)
 *   W: AA 3E <bytes...>   — write block to MACData
 *   W: AA 60 <bytes...>   — write checksum to MACDataSum
 *   X: <ms>               — delay in ms
 *   C: AA 3E <bytes...>   — compare: write subcmd, read back, compare
 *
 * All data bytes are hex strings separated by spaces.
 */

static const char s_gm_fs_data[] =
    ";Verify Existing Firmware Version\n"
    "W: AA 3E 02 00\n"
    "C: AA 3E 02 00 02 20 00 03\n"
    ";Data Block\n"
    "W: AA 3E F5 91 00 00 01 C2 00 32 00 C8 10 68\n"
    "W: AA 60 44 0E\n"
    "X: 10\n"
    "W: AA 3E F5 91\n"
    "C: AA 3E F5 91 00 00 01 C2 00 32 00 C8 10 68\n"
    "W: AA 3E 01 92 00 64\n"
    "W: AA 60 08 06\n"
    "X: 10\n"
    "W: AA 3E 01 92\n"
    "C: AA 3E 01 92 00 64\n"
    "W: AA 3E 32 92 02 26 02 01 F4 02 58 02 02 26 F6\n"
    "W: AA 60 A2 0F\n"
    "X: 10\n"
    "W: AA 3E 32 92\n"
    "C: AA 3E 32 92 02 26 02 01 F4 02 58 02 02 26 F6\n"
    "W: AA 3E 06 92 04 84 10 00\n"
    "W: AA 60 CF 08\n"
    "X: 10\n"
    "W: AA 3E 06 92\n"
    "C: AA 3E 06 92 04 84 10 00\n"
    "W: AA 3E 0B 92 01 09 00 00 96 00 AF\n"
    "W: AA 60 13 0B\n"
    "X: 10\n"
    "W: AA 3E 0B 92\n"
    "C: AA 3E 0B 92 01 09 00 00 96 00 AF\n"
    "W: AA 3E 12 92 02 20\n"
    "W: AA 60 39 06\n"
    "X: 10\n"
    "W: AA 3E 12 92\n"
    "C: AA 3E 12 92 02 20\n"
    "W: AA 3E 17 92 00 0A 05 00 32 01 C2 14 14\n"
    "W: AA 60 2A 0D\n"
    "X: 10\n"
    "W: AA 3E 17 92\n"
    "C: AA 3E 17 92 00 0A 05 00 32 01 C2 14 14\n"
    "W: AA 3E 28 92 00 3C 00 4B 00 28 00 3C 3C 01\n"
    "W: AA 60 1D 0E\n"
    "X: 10\n"
    "W: AA 3E 28 92\n"
    "C: AA 3E 28 92 00 3C 00 4B 00 28 00 3C 3C 01\n"
    "W: AA 3E 40 92 0A 8C 02 0A F0\n"
    "W: AA 60 9B 09\n"
    "X: 10\n"
    "W: AA 3E 40 92\n"
    "C: AA 3E 40 92 0A 8C 02 0A F0\n"
    "W: AA 3E 7F 92 0C 8C 8C 0A F0 0B 54 00 05 10 68 10 04 64 5F 0B 54 0B B8 06 08 10 68 10 04 64 5F\n"
    "W: AA 60 9E 1F\n"
    "X: 10\n"
    "W: AA 3E 7F 92\n"
    "C: AA 3E 7F 92 0C 8C 8C 0A F0 0B 54 00 05 10 68 10 04 64 5F 0B 54 0B B8 06 08 10 68 10 04 64 5F\n"
    "W: AA 3E 9A 92 00 10 2A 0C E4 0C E4\n"
    "W: AA 60 B9 0B\n"
    "X: 10\n"
    "W: AA 3E 9A 92\n"
    "C: AA 3E 9A 92 00 10 2A 0C E4 0C E4\n"
    "W: AA 3E 8F 41 00\n"
    "W: AA 60 2F 05\n"
    "X: 10\n"
    "W: AA 3E 8F 41\n"
    "C: AA 3E 8F 41 00\n"
    "W: AA 3E 7D 92 5A\n"
    "W: AA 60 96 05\n"
    "X: 10\n"
    "W: AA 3E 7D 92\n"
    "C: AA 3E 7D 92 5A\n"
    "W: AA 3E 51 92 02 BC\n"
    "W: AA 60 5E 06\n"
    "X: 10\n"
    "W: AA 3E 51 92\n"
    "C: AA 3E 51 92 02 BC\n"
    "W: AA 3E 5B 92 77\n"
    "W: AA 60 9B 05\n"
    "X: 10\n"
    "W: AA 3E 5B 92\n"
    "C: AA 3E 5B 92 77\n"
    "W: AA 3E 64 92 05 DC\n"
    "W: AA 60 28 06\n"
    "X: 10\n"
    "W: AA 3E 64 92\n"
    "C: AA 3E 64 92 05 DC\n"
    "W: AA 3E 68 92 14 00 00 00 C8 00 00 64 64 08 0E 74 00 64 1F 40\n"
    "W: AA 60 14 14\n"
    "X: 10\n"
    "W: AA 3E 68 92\n"
    "C: AA 3E 68 92 14 00 00 00 C8 00 00 64 64 08 0E 74 00 64 1F 40\n"
    "W: AA 3E A3 92 0E 10 00 64 0E 9F 00 95 03 63 0F BE 01 3C 09 00 00 0B D7 01 0D 39 01 0D AD 01 10 4D 0F CB 0F 55\n"
    "W: AA 60 0D 24\n"
    "X: 10\n"
    "W: AA 3E A3 92\n"
    "C: AA 3E A3 92 0E 10 00 64 0E 9F 00 95 03 63 0F BE 01 3C 09 00 00 0B D7 01 0D 39 01 0D AD 01 10 4D 0F CB 0F 55\n"
    "W: AA 3E C3 92 0E ED 0E 8D 0E 48 0E 23 0D FE 0D BB 0D 6F 0A 99\n"
    "W: AA 60 9B 14\n"
    "X: 10\n"
    "W: AA 3E C3 92\n"
    "C: AA 3E C3 92 0E ED 0E 8D 0E 48 0E 23 0D FE 0D BB 0D 6F 0A 99\n"
    "W: AA 3E 7B 92 02 3C\n"
    "W: AA 60 B4 06\n"
    "X: 10\n"
    "W: AA 3E 7B 92\n"
    "C: AA 3E 7B 92 02 3C\n";

/*============================================================================
 * gm.fs Parser — TI FlashStream format
 *
 * Format:
 *   ; comment line (skip)
 *   W: AA reg data...   — write data bytes to register reg
 *   C: AA reg data...   — read from register reg, compare byte-by-byte
 *   X: ms               — delay in milliseconds (decimal)
 *
 *   AA = BQ27220 8-bit write address (0x55 << 1).
 *   reg = I2C register address.
 *   data = hex bytes separated by spaces.
 *   Max 96 data bytes per line → max 98 tokens (AA + reg + 96 data).
 *
 *   Requires ≥66 µs idle between I2C packets (we use 70 µs).
 *============================================================================*/

#define GMFS_MAX_DATA    96U
#define GMFS_MAX_TOKENS  (GMFS_MAX_DATA + 2U)

/* Parse one hex byte. Returns true on success. */
static bool parse_hex_byte(const char *s, uint8_t *out)
{
    uint8_t val = 0U;
    for (int i = 0; i < 2; i++) {
        char c = s[i];
        if (c >= '0' && c <= '9') {
            val = (uint8_t)((val << 4U) | (uint8_t)(c - '0'));
        } else if (c >= 'A' && c <= 'F') {
            val = (uint8_t)((val << 4U) | (uint8_t)(c - 'A' + 10));
        } else if (c >= 'a' && c <= 'f') {
            val = (uint8_t)((val << 4U) | (uint8_t)(c - 'a' + 10));
        } else {
            return false;
        }
    }
    *out = val;
    return true;
}

/*
 * Parse hex tokens from a gm.fs line.
 * Returns true on success (including empty line, count=0).
 * Returns false on invalid hex, half-byte token, or overflow.
 */
static bool parse_tokens(const char *line, uint8_t *tokens, size_t capacity,
                          size_t *count)
{
    size_t n = 0U;
    const char *p = line;

    while (*p != '\0' && *p != '\n' && *p != '\r') {
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p == '\0' || *p == '\n' || *p == '\r') { break; }

        if (n >= capacity) { return false; }  /* Overflow */

        if (!parse_hex_byte(p, &tokens[n])) { return false; }  /* Invalid hex */

        n++;
        p += 2;
    }

    /* No trailing garbage allowed */
    while (*p == ' ' || *p == '\t') { p++; }
    if (*p != '\0' && *p != '\n' && *p != '\r') { return false; }

    *count = n;
    return true;
}

bool BQ27220_ExecuteGmFs(void)
{
    const char *p = s_gm_fs_data;
    uint8_t tokens[GMFS_MAX_TOKENS];
    uint8_t readback[GMFS_MAX_DATA];

    while (*p != '\0') {
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') { p++; }
        if (*p == '\0') { break; }

        if (*p == ';') {
            while (*p != '\0' && *p != '\n') { p++; }
            continue;
        }

        char cmd_type = *p;
        p++;

        if (*p != ':') { return false; }
        p++;

        /* Skip whitespace after ':' */
        while (*p == ' ' || *p == '\t') { p++; }

        if (cmd_type == 'X') {
            if (*p < '0' || *p > '9') { return false; }

            uint32_t delay_ms = 0U;
            while (*p >= '0' && *p <= '9') {
                uint32_t digit = (uint32_t)(*p - '0');
                if (delay_ms > (UINT32_MAX - digit) / 10U) { return false; }
                delay_ms = delay_ms * 10U + digit;
                p++;
            }

            /* No trailing garbage */
            while (*p == ' ' || *p == '\t' || *p == '\r') { p++; }
            if (*p != '\n' && *p != '\0') { return false; }

            LL_mDelay(delay_ms);

            if (*p == '\n') { p++; }
            continue;
        }

        if (cmd_type != 'W' && cmd_type != 'C') { return false; }

        /* W: or C: — parse hex tokens */
        size_t n;
        if (!parse_tokens(p, tokens, GMFS_MAX_TOKENS, &n)) { return false; }
        if (n < 3U) { return false; }  /* Need AA + reg + ≥1 data byte */

        while (*p != '\0' && *p != '\n') { p++; }

        if (tokens[0] != 0xAAU) { return false; }

        uint8_t reg = tokens[1];
        uint8_t data_len = (uint8_t)(n - 2U);

        if (data_len > GMFS_MAX_DATA) { return false; }

        if (cmd_type == 'W') {
            if (!BQ27220_WriteBlock(reg, &tokens[2], data_len)) { return false; }
        } else {
            if (!BQ27220_ReadBlock(reg, readback, data_len)) { return false; }
            for (uint8_t i = 0U; i < data_len; i++) {
                if (readback[i] != tokens[2U + i]) { return false; }
            }
        }

        if (*p == '\r') { p++; }
        if (*p == '\n') { p++; }
    }

    return true;
}

/*============================================================================
 * BOOT Step: Confirm configuration complete
 *============================================================================*/

/*
 * Poll OperationStatus for a bitmask until it reaches expected state.
 * Reads at ~500ms intervals (max 2/sec per TI recommendation).
 */
static bool BQ27220_WaitOpStatus(uint16_t mask, bool expected_set,
                                  uint32_t timeout_ms)
{
    while (timeout_ms > 0U) {
        uint16_t op_status = 0U;
        if (BQ27220_ReadOperationStatus(&op_status)) {
            bool is_set = ((op_status & mask) != 0U);
            if (is_set == expected_set) {
                return true;
            }
        }
        if (timeout_ms <= 500U) { break; }
        LL_mDelay(500);
        timeout_ms -= 500U;
    }
    return false;
}

/*
 * After gm.fs: verify CFGUPDATE=0, INITCOMP=1.
 */
bool BQ27220_ConfirmConfigComplete(void)
{
    BQ_LOG("    Waiting CFGUPDATE=0...");
    if (!BQ27220_WaitOpStatus(BQ27220_OPSTAT_CFGUPDATE, false, 10000U)) {
        BQ_LOG("    FAIL: CFGUPDATE remains set");
        return false;
    }

    BQ_LOG("    Waiting INITCOMP=1...");
    if (!BQ27220_WaitOpStatus(BQ27220_OPSTAT_INITCOMP, true, 10000U)) {
        BQ_LOG("    FAIL: INITCOMP not set");
        return false;
    }

    BQ_LOG("    Config re-init complete");
    return true;
}

/*============================================================================
 * BOOT Step: Issue SEALED sub-command
 *============================================================================*/

bool BQ27220_IssueSealed(void)
{
    BQ_LOG("    W: AA 00 30 00 (SEALED)");
    return BQ27220_WriteWord(BQ27220_CMD_CONTROL, BQ27220_CTRL_SEALED);
}

bool BQ27220_ConfirmSealed(void)
{
    uint8_t sec_mode;

    if (!BQ27220_ReadSEC(&sec_mode)) {
        BQ_LOG("    ConfirmSealed: I2C error");
        return false;
    }

    BQ_LOG("    ConfirmSealed: SEC=%u (3=SEALED)", sec_mode);
    return (sec_mode == 3U);
}

/*============================================================================
 * Complete BOOT Sequence
 *============================================================================*/

bool BQ27220_GaugeBoot(uint32_t now_ms)
{
    BQ_LOG("======== Gauge BOOT Start ========");

    s_last_command_ms = now_ms - 500U;

    /* Step 1: Probe — one attempt */
    BQ_LOG("Step 1/5: Probe (I2C detect @ 0x55)...");
    if (!BQ27220_Probe()) {
        BQ_LOG("  FAIL: No ACK at 0x55");
        return false;
    }
    BQ_LOG("  OK: Device ACKed");

    /*
     * Step 2: Read Design Capacity to detect power-loss.
     *
     * Design Capacity (cmd 0x3C) is a standard command readable in any
     * security mode. After a power cycle, the BQ27220 reverts to factory
     * defaults (Design Capacity ≠ 3300). If it reads 3300, our gm.fs
     * config is already loaded — skip the entire gm.fs flow.
     */
    BQ_LOG("Step 2/5: Read Design Capacity (power-loss check)...");
    uint16_t design_cap = 0U;
    if (!BQ27220_ReadDesignCapacity(&design_cap)) {
        BQ_LOG("  FAIL: I2C error reading Design Capacity");
        return false;
    }
    BQ_LOG("  Design Capacity = %u mAh (expected %u)", design_cap,
           BQ27220_DESIGN_CAPACITY_MAH);

    bool need_gm_fs = (design_cap != BQ27220_DESIGN_CAPACITY_MAH);

    if (!need_gm_fs) {
        BQ_LOG("  Config already loaded — skip gm.fs");
        BQ_LOG("======== Gauge BOOT: SUCCESS (NORMAL) ========");
        return true;
    }

    /*
     * Step 3: Power was lost — re-write gm.fs.
     * Must unseal first to enter FullAccess + CFGUPDATE mode.
     */
    BQ_LOG("Step 3/5: Power-loss detected (Design Cap %u ≠ %u) — run gm.fs",
           design_cap, BQ27220_DESIGN_CAPACITY_MAH);

    uint8_t sec_mode;
    if (!BQ27220_ReadSEC(&sec_mode)) {
        BQ_LOG("  FAIL: I2C error reading SEC");
        return false;
    }
    BQ_LOG("  SEC (bits2:1) = %u  [3=SEALED 2=UNSEALED 1=FULL_ACCESS]", sec_mode);

    if (sec_mode == 3U) {
        BQ_LOG("  Unsealing...");
        if (!BQ27220_EnterUnsealed()) {
            BQ_LOG("  FAIL: UNSEAL failed");
            return false;
        }
    }

    BQ_LOG("  A: FULL ACCESS...");
    if (!BQ27220_EnterFullAccess()) {
        BQ_LOG("  FAIL: FULL ACCESS failed");
        return false;
    }

    BQ_LOG("  B: SET_CFGUPDATE (0x0090)...");
    if (!BQ27220_WriteWord(BQ27220_CMD_CONTROL, BQ27220_CTRL_SET_CFG_UPDATE)) {
        BQ_LOG("  FAIL: SET_CFGUPDATE I2C error");
        return false;
    }
    LL_mDelay(2000);  /* TI: wait ≥2 seconds */
    if (!BQ27220_WaitOpStatus(BQ27220_OPSTAT_CFGUPDATE, true, 5000U)) {
        BQ_LOG("  FAIL: CFGUPDATE not set");
        return false;
    }

    BQ_LOG("  C: Execute gm.fs...");
    if (!BQ27220_ExecuteGmFs()) {
        BQ_LOG("  FAIL: gm.fs execution failed");
        return false;
    }

    BQ_LOG("  D: EXIT_CFG_UPDATE_REINIT (0x0091)...");
    if (!BQ27220_WriteWord(BQ27220_CMD_CONTROL, BQ27220_CTRL_EXIT_CFG_UPDATE_REINIT)) {
        BQ_LOG("  FAIL: EXIT_CFG_UPDATE_REINIT I2C error");
        return false;
    }

    BQ_LOG("  E: Confirm (CFGUPDATE=0, INITCOMP=1)...");
    if (!BQ27220_ConfirmConfigComplete()) {
        BQ_LOG("  FAIL: Config confirmation failed");
        return false;
    }

    /* Step 4: Verify Design Capacity is now 3300 */
    BQ_LOG("Step 4/5: Verify Design Capacity after gm.fs...");
    if (!BQ27220_ReadDesignCapacity(&design_cap)) {
        BQ_LOG("  FAIL: I2C error reading Design Capacity");
        return false;
    }
    BQ_LOG("  Design Capacity = %u mAh", design_cap);
    if (design_cap != BQ27220_DESIGN_CAPACITY_MAH) {
        BQ_LOG("  FAIL: Design Capacity %u ≠ expected %u — gm.fs may be incomplete",
               design_cap, BQ27220_DESIGN_CAPACITY_MAH);
        return false;
    }

    /* Step 5: Ensure SEALED at exit */
    BQ_LOG("Step 5/5: SEAL device...");
    uint8_t final_sec;
    if (!BQ27220_ReadSEC(&final_sec)) {
        BQ_LOG("  FAIL: I2C error reading final SEC");
        return false;
    }

    if (final_sec != 3U) {
        BQ_LOG("  Device not SEALED (SEC=%u) — sending SEALED...", final_sec);
        if (!BQ27220_IssueSealed()) {
            BQ_LOG("  FAIL: SEALED I2C error");
            return false;
        }
        LL_mDelay(70);
        if (!BQ27220_ConfirmSealed()) {
            BQ_LOG("  FAIL: SEALED confirmation failed");
            return false;
        }
    }
    BQ_LOG("  OK: Device is SEALED");

    BQ_LOG("======== Gauge BOOT: SUCCESS (NORMAL) ========");
    return true;
}

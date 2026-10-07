/**
 ******************************************************************************
 * @file           : bq27220.h
 * @brief          : BQ27220 CEDV fuel gauge driver (LL I2C, polling mode)
 *
 * @details        : Reads battery SOC, voltage, temperature, current,
 *                   remaining/full capacity, and SOH via I2C1.
 *                   I2C1: PB6=SCL, PB7=SDA, 7-bit addr 0x55.
 *
 *   References:
 *     - BQ27220 Technical Reference Manual SLUUBD4A
 *     - STM32L051C8T6 datasheet
 ******************************************************************************
 */

#ifndef BQ27220_H
#define BQ27220_H

#include "stm32l0xx_conf.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * I2C address
 *============================================================================*/
/*
 * LL_I2C_HandleTransfer() writes directly to CR2.SADD[7:1].
 * SADD[7:1] must contain the 7-bit slave address, so we pass
 * the address LEFT-SHIFTED by 1.
 *
 *   Raw 7-bit addr: 0x55 = 0b1010101
 *   SADD[7:1]     : 0b1010101 → SADD = 0xAA
 */
#define BQ27220_I2C_ADDR          (0x55U << 1U)  /* SADD[7:1] = 0x55 */

/*
 * Conservative Standard-mode timing @ 8 MHz PCLK (~62 kHz):
 *   PRESC=0  → t_presc = 125 ns
 *   SCLDEL=3 → t_SU;DAT = 500 ns
 *   SDADEL=0 → t_HD;DAT = 0 ns (analog filter provides hold)
 *   SCLH=0x3F=63 → t_SCL_H = 8000 ns
 *   SCLL=0x3F=63 → t_SCL_L = 8000 ns
 *
 * Result: f_SCL ≈ 62 kHz — safe margin for any slave.
 * BQ27220 supports 100 kHz Standard and 400 kHz Fast mode.
 */
#define BQ27220_I2C_TIMING \
    __LL_I2C_CONVERT_TIMINGS(0U, 3U, 0U, 0x3FU, 0x3FU)

/*============================================================================
 * Standard command codes (Table 2-1 of TRM)
 *============================================================================*/
#define BQ27220_CMD_CONTROL             0x00U
#define BQ27220_CMD_TEMPERATURE         0x06U
#define BQ27220_CMD_VOLTAGE             0x08U
#define BQ27220_CMD_BATTERY_STATUS      0x0AU
#define BQ27220_CMD_CURRENT             0x0CU
#define BQ27220_CMD_REMAINING_CAPACITY  0x10U
#define BQ27220_CMD_FULL_CHARGE_CAPACITY 0x12U
#define BQ27220_CMD_AVERAGE_CURRENT     0x14U
#define BQ27220_CMD_TIME_TO_EMPTY       0x16U
#define BQ27220_CMD_TIME_TO_FULL        0x18U
#define BQ27220_CMD_STANDBY_CURRENT     0x1AU
#define BQ27220_CMD_INTERNAL_TEMP       0x28U
#define BQ27220_CMD_CYCLE_COUNT         0x2AU
#define BQ27220_CMD_RELATIVE_SOC        0x2CU
#define BQ27220_CMD_STATE_OF_HEALTH     0x2EU
#define BQ27220_CMD_OPERATION_STATUS    0x3AU
#define BQ27220_CMD_DESIGN_CAPACITY     0x3CU

/*============================================================================
 * Control() sub-commands (Table 2-2 of TRM)
 *============================================================================*/
#define BQ27220_CTRL_DEVICE_TYPE        0x0001U
#define BQ27220_CTRL_FW_VERSION         0x0002U
#define BQ27220_CTRL_SEC                0x0030U
#define BQ27220_CTRL_SEALED             0x0030U
#define BQ27220_CTRL_FULL_ACCESS_KEY    0xFFFFU
#define BQ27220_CTRL_STATUS             0x0000U

/* Default UNSEAL keys for BQ27220: 0x0414, 0x3672 */
#define BQ27220_UNSEAL_KEY1             0x0414U
#define BQ27220_UNSEAL_KEY2             0x3672U

/* MACData command for Data Flash access */
#define BQ27220_CMD_MAC_DATA            0x3EU
#define BQ27220_CMD_MAC_DATA_SUM        0x60U

/*============================================================================
 * BatteryStatus bit definitions (Table 2-6 of TRM SLUUBD4A)
 *============================================================================*/
#define BQ27220_BATTSTAT_DSG            (1U << 0)   /* Discharging detected */
#define BQ27220_BATTSTAT_FC             (1U << 9)   /* Fully charged */
#define BQ27220_BATTSTAT_CHG_INH        (1U << 8)   /* Charge inhibited */
#define BQ27220_BATTSTAT_TDA            (1U << 2)   /* Terminate discharge alarm */

/*============================================================================
 * OperationStatus bit definitions (Table 2-5 of TRM)
 *============================================================================*/
#define BQ27220_OPSTAT_INITCOMP         (1U << 5)   /* bit 5: Initialization complete */
#define BQ27220_OPSTAT_CFGUPDATE        0x0400U     /* bit 10: CONFIG UPDATE active */

/* Control() subcommands for CONFIG UPDATE flow */
#define BQ27220_CTRL_SET_CFG_UPDATE          0x0090U
#define BQ27220_CTRL_EXIT_CFG_UPDATE_REINIT  0x0091U
#define BQ27220_OPSTAT_SS               (1U << 13)  /* Sealed state */
#define BQ27220_OPSTAT_FULL_ACCESS      (1U << 14)  /* Full access */
#define BQ27220_OPSTAT_SEC_MODE(n)      (((n) >> 13U) & 0x03U)
#define BQ27220_OPSTAT_SEC_SEALED       0x0000U     /* 00 = SEALED */
#define BQ27220_OPSTAT_SEC_UNSEALED     0x2000U     /* 10 = UNSEALED */
#define BQ27220_OPSTAT_SEC_FULL_ACCESS  0x6000U     /* 11 = FULL ACCESS */

/*============================================================================
 * Power-Loss Detection
 *
 * BQ27220 ITPOR bit position is NOT publicly documented in the TRM.
 * Instead, we detect power-loss by reading Design Capacity (cmd 0x3C):
 *   - Design Capacity == 3300 mAh → gm.fs config is loaded
 *   - Design Capacity ≠ 3300 mAh → power was lost, re-write gm.fs
 *
 * Design Capacity is a standard command readable in any security mode,
 * so this check can be done before unsealing.
 *============================================================================*/

/*============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief  Initialize I2C1 peripheral for BQ27220 communication.
 *         PB6=SCL, PB7=SDA, open-drain, 100 kHz Standard mode.
 * @return true on success.
 */
bool BQ27220_Init(void);

/**
 * @brief  Probe the BQ27220 device by reading Control() status.
 * @return true if device ACKs at address 0x55.
 */
bool BQ27220_Probe(void);

/**
 * @brief  Read a 16-bit value from a standard command register.
 * @param  cmd     Command code (0x00–0xFF).
 * @param  value   Output pointer for the 16-bit result.
 * @return true on success.
 * @note   Blocks until I2C transaction completes or timeout (~5ms).
 */
bool BQ27220_ReadWord(uint8_t cmd, uint16_t *value);

/**
 * @brief  Write a 16-bit value to a standard command register.
 * @param  cmd     Command code.
 * @param  value   16-bit value to write.
 * @return true on success.
 * @note   Only for writable commands (AtRate, BTP sets, etc.).
 */
bool BQ27220_WriteWord(uint8_t cmd, uint16_t value);

/* ---- Convenience wrappers ---- */

/**
 * @brief  Read battery voltage in mV.
 */
bool BQ27220_ReadVoltage(uint16_t *voltage_mv);

/**
 * @brief  Read pack temperature in 0.1 K units.
 *         Convert to °C: t_celsius = (temp_dk / 10.0f) - 273.15f.
 */
bool BQ27220_ReadTemperature(uint16_t *temp_deci_kelvin);

/**
 * @brief  Read internal (die) temperature in 0.1 K units.
 */
bool BQ27220_ReadInternalTemp(uint16_t *temp_deci_kelvin);

/**
 * @brief  Read Relative State of Charge in percent.
 */
bool BQ27220_ReadSOC(uint16_t *soc_percent);

/**
 * @brief  Read State of Health in percent.
 *         Output = SOH(%) = SOH% /  CCC%.
 */
bool BQ27220_ReadSOH(uint16_t *soh_percent);

/**
 * @brief  Read remaining capacity in mAh.
 */
bool BQ27220_ReadRemainingCapacity(uint16_t *capacity_mah);

/**
 * @brief  Read full charge capacity in mAh.
 */
bool BQ27220_ReadFullChargeCapacity(uint16_t *capacity_mah);

/**
 * @brief  Read instantaneous current in mA.
 *         Positive = charging, negative = discharging.
 */
bool BQ27220_ReadCurrent(int16_t *current_ma);

/**
 * @brief  Read average current in mA.
 */
bool BQ27220_ReadAverageCurrent(int16_t *current_ma);

/** @brief Predicted minutes to empty; 0xFFFF = not discharging. */
bool BQ27220_ReadTimeToEmpty(uint16_t *minutes);

/** @brief Predicted minutes to full; 0xFFFF = not charging. */
bool BQ27220_ReadTimeToFull(uint16_t *minutes);

/**
 * @brief  Read BatteryStatus register.
 */
bool BQ27220_ReadBatteryStatus(uint16_t *status);

/**
 * @brief  Read OperationStatus register.
 */
bool BQ27220_ReadOperationStatus(uint16_t *status);

/**
 * @brief  Read Design Capacity in mAh.
 */
bool BQ27220_ReadDesignCapacity(uint16_t *capacity_mah);

/**
 * @brief  Read Cycle Count.
 */
bool BQ27220_ReadCycleCount(uint16_t *cycles);

/**
 * @brief  Scan I2C1 bus (addresses 0x08–0x77) and print which devices ACK.
 *
 *         Uses a minimal START+ADDR+STOP probe — no data transfer.
 *         Useful for diagnosing wiring / address issues.
 */
void BQ27220_ScanBus(void);

/**
 * @brief  Print a battery info summary to USART2 via printf.
 *         Reads all key parameters and formats them in a compact table.
 */
void BQ27220_PrintSummary(void);

/*============================================================================
 * BOOT Sequence API
 *============================================================================*/

/**
 * @brief  Set command guard timestamp. Called before any standard command.
 *         If guard not yet expired, returns false — caller must wait.
 * @param  now_ms  Current millisecond timestamp.
 * @return true if ≥500ms since last standard command, false otherwise.
 */
bool BQ27220_CanIssueCommand(uint32_t now_ms);

/**
 * @brief  Mark that a standard command was issued at this time.
 */
void BQ27220_MarkCommandIssued(uint32_t now_ms);

/**
 * @brief  Read Control() status word (sub-command 0x0000).
 */
bool BQ27220_ReadControlStatus(uint16_t *status);

/**
 * @brief  Write a value to Control() command (cmd 0x00).
 */
bool BQ27220_WriteControlWord(uint16_t value);

/**
 * @brief  Read Device Number via Control(0x0001). Expects 0x0220 for BQ27220.
 */
bool BQ27220_ReadDeviceNumber(uint16_t *device_number);

/**
 * @brief  Read Firmware Version via Control(0x0002).
 */
bool BQ27220_ReadFwVersion(uint16_t *fw_version);

/**
 * @brief  Read SEC (security mode) bits from OperationStatus.
 *         Returns SEALED(0), UNSEALED(2), or FULL_ACCESS(3).
 */
bool BQ27220_ReadSEC(uint8_t *sec_mode);

/**
 * @brief  Send UNSEAL keys (0x0414, 0x3672) to transition SEALED → UNSEALED.
 */
bool BQ27220_EnterUnsealed(void);

/**
 * @brief  Enter FULL ACCESS mode via freeze key sequence.
 *         Writes 0xFFFF to Control() twice, then confirms FULL ACCESS.
 */
bool BQ27220_EnterFullAccess(void);

/**
 * @brief  Read len bytes from a register via I2C.
 *         Writes 1-byte register address, then reads len bytes.
 */
bool BQ27220_ReadBlock(uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief  Execute the golden memory flash stream (gm.fs).
 *         Parses and applies the embedded gm.fs byte stream.
 *         W:/C: commands are executed; X: commands are delays.
 * @return true if all W: writes and C: compares succeeded.
 */
bool BQ27220_ExecuteGmFs(void);

/**
 * @brief  Confirm configuration complete after gm.fs:
 *         CFGUPDATE=0, INITCOMP=1, config signature valid.
 */
bool BQ27220_ConfirmConfigComplete(void);

/**
 * @brief  Send SEALED sub-command (0x0020) to Control().
 */
bool BQ27220_IssueSealed(void);

/**
 * @brief  Confirm device is SEALED by reading OperationStatus SEC bits.
 */
bool BQ27220_ConfirmSealed(void);

/**
 * @brief  Complete BOOT sequence for BQ27220.
 *
 *         Flow:
 *         1. Probe → Design Capacity (power-loss check)
 *         2. If Design Capacity == 3300: config already loaded → SUCCESS
 *         3. If Design Capacity ≠ 3300: power was lost →
 *            SEC → Unseal → FullAccess → CFGUPDATE → gm.fs →
 *            EXIT_REINIT → Confirm → Verify Design Capacity → SEALED
 *
 *         Any failure → return false, NO_GAUGE forever.
 *
 * @param  now_ms  Current timestamp for command guard initialization.
 * @return true if NORMAL mode achieved.
 */
bool BQ27220_GaugeBoot(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* BQ27220_H */

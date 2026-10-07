/* Host telemetry and delayed load-off control: PB13 SCL / PB14 SDA, AF5. */
#ifndef EC_HOST_I2C_H
#define EC_HOST_I2C_H

#include "ec_state_machine.h"

/* Unshifted 7-bit address; wire address bytes are 0x84 / 0x85. */
#define EC_HOST_I2C_ADDRESS              0x42U
#define EC_HOST_PROTOCOL_VERSION         1U
#define EC_HOST_REGISTER_COUNT           0x1CU
#define EC_HOST_STALL_TIMEOUT_MS          1000U

/* Byte offsets. Multi-byte values are little-endian, without struct padding. */
#define EC_HOST_REG_VERSION              0x00U
#define EC_HOST_REG_LENGTH               0x01U
#define EC_HOST_REG_VALID                0x02U  /* uint16_t */
#define EC_HOST_REG_FLAGS                0x04U
#define EC_HOST_REG_BATTERY_STATE        0x05U
#define EC_HOST_REG_VOLTAGE_MV           0x06U  /* uint16_t */
#define EC_HOST_REG_CURRENT_MA           0x08U  /* int16_t */
#define EC_HOST_REG_SOC_PERCENT          0x0AU  /* uint16_t */
#define EC_HOST_REG_TIME_TO_FULL_MIN     0x0CU  /* uint16_t */
#define EC_HOST_REG_TIME_TO_EMPTY_MIN    0x0EU  /* uint16_t */
#define EC_HOST_REG_REMAINING_MAH        0x10U  /* uint16_t */
#define EC_HOST_REG_FULL_CHARGE_MAH      0x12U  /* uint16_t */
#define EC_HOST_REG_AVERAGE_CURRENT_MA   0x14U  /* int16_t */
#define EC_HOST_REG_BATTERY_STATUS       0x16U  /* uint16_t */
#define EC_HOST_REG_SAMPLE_SEQUENCE      0x18U  /* uint32_t */

/* Separate write-only command: [0x20, 0xA5, delay_s_lo, delay_s_hi] + STOP.
 * Reading 0x20 returns 0xFF; the 28-byte telemetry layout stays unchanged. */
#define EC_HOST_REG_CONTROL              0x20U
#define EC_HOST_CMD_POWER_OFF            0xA5U

#define EC_HOST_VALID_VOLTAGE            (1U << 0)
#define EC_HOST_VALID_CURRENT            (1U << 1)
#define EC_HOST_VALID_SOC                (1U << 2)
#define EC_HOST_VALID_TIME_TO_FULL       (1U << 3)
#define EC_HOST_VALID_TIME_TO_EMPTY      (1U << 4)
#define EC_HOST_VALID_REMAINING          (1U << 5)
#define EC_HOST_VALID_FULL_CHARGE        (1U << 6)
#define EC_HOST_VALID_AVERAGE_CURRENT    (1U << 7)
#define EC_HOST_VALID_BATTERY_STATUS     (1U << 8)

#define EC_HOST_FLAG_VBUS                (1U << 0)
#define EC_HOST_FLAG_LOAD_ON             (1U << 1)
#define EC_HOST_FLAG_GAUGE_NORMAL        (1U << 2)
#define EC_HOST_FLAG_POWER_OFF_SUPPORTED (1U << 3)
#define EC_HOST_FLAG_POWER_OFF_PENDING   (1U << 4)

typedef struct {
    uint16_t delay_seconds;
    uint32_t received_ms; /* SysTick timestamp of the completed write's STOP. */
} ec_host_power_off_request_t;

typedef enum {
    EC_HOST_BATTERY_UNKNOWN = 0,
    EC_HOST_BATTERY_IDLE = 1,
    EC_HOST_BATTERY_CHARGING = 2,
    EC_HOST_BATTERY_DISCHARGING = 3,
    EC_HOST_BATTERY_FULL = 4
} ec_host_battery_state_t;

/* Init/restore the peripheral, preserving the published telemetry. */
void EC_HostI2C_Init(void);
/* Main-loop only. The ISR serves a frozen snapshot, never accesses I2C1. */
void EC_HostI2C_Publish(const ec_status_t *status);
bool EC_HostI2C_IsBusy(void);
/* Call each main-loop iteration, including while running/PG active. Reset
 * BUSY or SCL-low after observing >=1 s without progress. Preserve telemetry,
 * unserviced events and completed commands; no change to load power.
 * One recovery attempt per stall; UART reports levels with our pins released. */
bool EC_HostI2C_RecoverStalled(void);
/* Main-loop only. Latest completed valid write wins; incomplete writes are
 * discarded. Atomically consumes the mailbox without changing load GPIOs. */
bool EC_HostI2C_TakePowerOffRequest(ec_host_power_off_request_t *request);
/* Release pins for STOP. Returns false while a transaction, unserviced I2C
 * event, or unconsumed power-off request is pending. */
bool EC_HostI2C_Suspend(void);

#endif

/**
 ******************************************************************************
 * @file           : oled_ssd1306.c
 * @brief          : u8g2 hardware-I2C port for SSD1306 128x32.
 *
 *   u8g2 owns rendering and the SSD1306 command protocol. This file adapts
 *   U8X8 byte messages to the existing STM32L051 I2C1 LL peripheral shared
 *   with the BQ27220. Transfers use SOFTEND followed by a manual STOP, matching
 *   the workaround already used by the fuel-gauge driver.
 ******************************************************************************
 */

#include "oled_ssd1306.h"
#include "stm32l0xx_conf.h"

#include <stddef.h>

#define OLED_I2C_ADDR          (0x3CU << 1U)
#define OLED_I2C_TIMEOUT_LOOPS 50000U
#define OLED_TX_BUFFER_SIZE       32U
#define OLED_FRAME_BUFFER_SIZE  ((OLED_WIDTH * OLED_HEIGHT) / 8U)

static u8g2_t s_u8g2;
static uint8_t s_frame_buffer[OLED_FRAME_BUFFER_SIZE];
static uint8_t s_tx_buffer[OLED_TX_BUFFER_SIZE];
static uint8_t s_tx_length;
static bool s_transport_ok;
static bool s_initialized;

static void oled_i2c_clear_errors(void)
{
    if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
        LL_I2C_ClearFlag_NACK(I2C1);
    }
    if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
        LL_I2C_ClearFlag_STOP(I2C1);
    }
    if (LL_I2C_IsActiveFlag_BERR(I2C1)) {
        LL_I2C_ClearFlag_BERR(I2C1);
    }
    if (LL_I2C_IsActiveFlag_ARLO(I2C1)) {
        LL_I2C_ClearFlag_ARLO(I2C1);
    }
    if (LL_I2C_IsActiveFlag_OVR(I2C1)) {
        LL_I2C_ClearFlag_OVR(I2C1);
    }
}

static void oled_i2c_abort(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT_LOOPS;

    if (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        LL_I2C_GenerateStopCondition(I2C1);
        while (!LL_I2C_IsActiveFlag_STOP(I2C1) && (timeout > 0U)) {
            timeout--;
        }
    }
    oled_i2c_clear_errors();
}

static bool oled_i2c_has_error(void)
{
    return LL_I2C_IsActiveFlag_NACK(I2C1) ||
           LL_I2C_IsActiveFlag_BERR(I2C1) ||
           LL_I2C_IsActiveFlag_ARLO(I2C1) ||
           LL_I2C_IsActiveFlag_OVR(I2C1);
}

static bool oled_i2c_write(uint16_t address, const uint8_t *data, uint8_t length)
{
    uint32_t timeout;

    if ((data == NULL) || (length == 0U)) {
        return false;
    }

    oled_i2c_clear_errors();

    timeout = OLED_I2C_TIMEOUT_LOOPS;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (timeout-- == 0U) {
            oled_i2c_abort();
            return false;
        }
    }

    LL_I2C_HandleTransfer(I2C1,
                          address,
                          LL_I2C_ADDRSLAVE_7BIT,
                          length,
                          LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_WRITE);

    for (uint8_t i = 0U; i < length; i++) {
        timeout = OLED_I2C_TIMEOUT_LOOPS;
        while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {
            if (oled_i2c_has_error() || (timeout-- == 0U)) {
                oled_i2c_abort();
                return false;
            }
        }
        LL_I2C_TransmitData8(I2C1, data[i]);
    }

    timeout = OLED_I2C_TIMEOUT_LOOPS;
    while (!LL_I2C_IsActiveFlag_TC(I2C1)) {
        if (oled_i2c_has_error() || (timeout-- == 0U)) {
            oled_i2c_abort();
            return false;
        }
    }

    LL_I2C_GenerateStopCondition(I2C1);
    timeout = OLED_I2C_TIMEOUT_LOOPS;
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
        if (timeout-- == 0U) {
            oled_i2c_abort();
            return false;
        }
    }
    LL_I2C_ClearFlag_STOP(I2C1);

    timeout = OLED_I2C_TIMEOUT_LOOPS;
    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        if (timeout-- == 0U) {
            oled_i2c_abort();
            return false;
        }
    }

    return true;
}

static void oled_set_charge_pump(bool enabled)
{
    u8x8_t *u8x8 = u8g2_GetU8x8(&s_u8g2);

    (void)u8x8_cad_StartTransfer(u8x8);
    (void)u8x8_cad_SendCmd(u8x8, 0x8DU);
    (void)u8x8_cad_SendArg(u8x8, enabled ? 0x14U : 0x10U);
    (void)u8x8_cad_EndTransfer(u8x8);
}

static uint8_t oled_u8x8_byte_cb(u8x8_t *u8x8, uint8_t msg,
                                  uint8_t arg_int, void *arg_ptr)
{
    switch (msg) {
    case U8X8_MSG_BYTE_INIT:
        return 1U;

    case U8X8_MSG_BYTE_START_TRANSFER:
        s_tx_length = 0U;
        return 1U;

    case U8X8_MSG_BYTE_SEND: {
        const uint8_t *src = (const uint8_t *)arg_ptr;

        if ((src == NULL) ||
            ((uint16_t)s_tx_length + arg_int > OLED_TX_BUFFER_SIZE)) {
            s_transport_ok = false;
            return 0U;
        }
        for (uint8_t i = 0U; i < arg_int; i++) {
            s_tx_buffer[s_tx_length++] = src[i];
        }
        return 1U;
    }

    case U8X8_MSG_BYTE_END_TRANSFER:
        if ((s_tx_length == 0U) ||
            !oled_i2c_write(u8x8_GetI2CAddress(u8x8),
                            s_tx_buffer, s_tx_length)) {
            s_transport_ok = false;
            return 0U;
        }
        return 1U;

    default:
        return 0U;
    }
}

static uint8_t oled_u8x8_gpio_delay_cb(u8x8_t *u8x8, uint8_t msg,
                                        uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8;
    (void)arg_ptr;

    switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        return 1U;

    case U8X8_MSG_DELAY_MILLI:
        LL_mDelay(arg_int);
        return 1U;

    case U8X8_MSG_DELAY_10MICRO: {
        uint32_t loops = (uint32_t)arg_int *
                         ((SystemCoreClock / 100000U) + 1U);
        while (loops-- > 0U) {
            __NOP();
        }
        return 1U;
    }

    case U8X8_MSG_DELAY_100NANO:
    case U8X8_MSG_DELAY_NANO:
    case U8X8_MSG_GPIO_RESET:
        return 1U;

    default:
        /* No reset GPIO is connected on the four-pin I2C module. */
        return 1U;
    }
}

bool OLED_Init(void)
{
    s_initialized = false;
    s_transport_ok = true;
    s_tx_length = 0U;

    u8g2_SetupDisplay(&s_u8g2,
                      u8x8_d_ssd1306_128x32_univision,
                      u8x8_cad_ssd13xx_fast_i2c,
                      oled_u8x8_byte_cb,
                      oled_u8x8_gpio_delay_cb);
    u8g2_SetupBuffer(&s_u8g2,
                     s_frame_buffer,
                     OLED_HEIGHT / 8U,
                     u8g2_ll_hvline_vertical_top_lsb,
                     &u8g2_cb_r0);
    u8x8_SetI2CAddress(u8g2_GetU8x8(&s_u8g2), OLED_I2C_ADDR);

    u8g2_InitDisplay(&s_u8g2);
    if (!s_transport_ok) {
        return false;
    }

    u8g2_SetPowerSave(&s_u8g2, 0U);
    if (!s_transport_ok) {
        return false;
    }

    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SendBuffer(&s_u8g2);
    if (!s_transport_ok) {
        return false;
    }

    s_initialized = true;
    return true;
}

u8g2_t *OLED_GetContext(void)
{
    return &s_u8g2;
}

bool OLED_Present(void)
{
    if (!s_initialized) {
        return false;
    }

    s_transport_ok = true;
    u8g2_SendBuffer(&s_u8g2);
    return s_transport_ok;
}

bool OLED_PresentArea(uint8_t tile_x, uint8_t tile_y,
                      uint8_t tile_width, uint8_t tile_height)
{
    if (!s_initialized || (tile_width == 0U) || (tile_height == 0U) ||
        (tile_x >= OLED_TILE_WIDTH) || (tile_y >= OLED_TILE_HEIGHT) ||
        ((uint16_t)tile_x + tile_width > OLED_TILE_WIDTH) ||
        ((uint16_t)tile_y + tile_height > OLED_TILE_HEIGHT)) {
        return false;
    }

    s_transport_ok = true;
    u8g2_UpdateDisplayArea(&s_u8g2, tile_x, tile_y,
                           tile_width, tile_height);
    return s_transport_ok;
}

void OLED_Sleep(void)
{
    if (s_initialized) {
        s_transport_ok = true;
        u8g2_SetPowerSave(&s_u8g2, 1U);
        oled_set_charge_pump(false);
    }
}

void OLED_Wake(void)
{
    if (s_initialized) {
        s_transport_ok = true;
        oled_set_charge_pump(true);
        u8g2_SetPowerSave(&s_u8g2, 0U);
    }
}

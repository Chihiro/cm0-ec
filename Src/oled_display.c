/**
 ******************************************************************************
 * @file           : oled_display.c
 * @brief          : u8g2 UI for the 128x32 SSD1306 status display.
 *
 *   Page 0 is a two-column charge/load dashboard. Page 1 keeps the electrical
 *   detail values available without crowding the dashboard. All formatting
 *   is integer-only.
 ******************************************************************************
 */

#include "oled_display.h"
#include "oled_ssd1306.h"

#include <stddef.h>
#include <stdint.h>

#define PAGE_COUNT        2U
#define OLED_TILE_BYTES   8U
#define OLED_BUFFER_SIZE  (OLED_WIDTH * OLED_TILE_HEIGHT)
#define CHARGING_CURRENT_MIN_MA 20

static uint8_t s_page;
static bool s_ready;
static bool s_shadow_valid;
static uint8_t s_display_shadow[OLED_BUFFER_SIZE];

extern const uint8_t u8g2_font_ec_ui_12[];
extern const uint8_t u8g2_font_ec_values_9x15[];

/* UTF-8 text encoded explicitly so source-file encoding cannot alter it. */
#define UI_TEXT_CHARGE   "\xE7\x94\xB5\xE9\x87\x8F" /* 电量 */
#define UI_TEXT_LOAD     "\xE8\xB4\x9F\xE8\xBD\xBD" /* 负载 */

/* 7x14 charging bolt, placed between the charge text and the divider. */
static const uint8_t CHARGE_BOLT_XBM[14] = {
    0x30U, 0x38U, 0x18U, 0x1CU, 0x0CU, 0x3EU, 0x38U,
    0x18U, 0x1CU, 0x0CU, 0x06U, 0x07U, 0x03U, 0x01U
};

static void draw_cjk_text(u8g2_t *u8g2, uint8_t x, uint8_t y,
                          const char *text)
{
    u8g2_SetFont(u8g2, u8g2_font_ec_ui_12);
    u8g2_SetFontPosTop(u8g2);
    u8g2_DrawUTF8(u8g2, x, y, text);
}

static void append_str(char **dst, const char *src)
{
    while (*src != '\0') {
        *(*dst)++ = *src++;
    }
}

static void set_str(char *dst, const char *src)
{
    append_str(&dst, src);
    *dst = '\0';
}

static void append_u32(char **dst, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (value == 0U) {
        *(*dst)++ = '0';
        return;
    }

    while (value > 0U) {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count > 0U) {
        *(*dst)++ = digits[--count];
    }
}

static void format_soc(char *dst, uint16_t soc)
{
    char *p = dst;

    append_u32(&p, soc);
    append_str(&p, "%");
    *p = '\0';
}

static void format_voltage(char *dst, uint16_t millivolts)
{
    char *p = dst;
    uint16_t fraction = (uint16_t)((millivolts % 1000U) / 10U);

    append_u32(&p, millivolts / 1000U);
    append_str(&p, ".");
    if (fraction < 10U) {
        append_str(&p, "0");
    }
    append_u32(&p, fraction);
    append_str(&p, "V");
    *p = '\0';
}

static void format_current_amps(char *dst, int16_t milliamps)
{
    int32_t signed_ma = milliamps;
    uint32_t magnitude = (signed_ma < 0) ? (uint32_t)(-signed_ma)
                                         : (uint32_t)signed_ma;
    uint32_t hundredths = (magnitude + 5U) / 10U;
    uint32_t fraction = hundredths % 100U;
    char *p = dst;

    append_u32(&p, hundredths / 100U);
    append_str(&p, ".");
    if (fraction < 10U) {
        append_str(&p, "0");
    }
    append_u32(&p, fraction);
    append_str(&p, "A");
    *p = '\0';
}

static void format_power(char *dst, int16_t milliamps, uint16_t millivolts)
{
    int64_t microwatts = (int64_t)millivolts * (int64_t)milliamps;
    uint64_t magnitude = (microwatts < 0) ? (uint64_t)(-microwatts)
                                          : (uint64_t)microwatts;
    uint32_t hundredths = (uint32_t)((magnitude + 5000ULL) / 10000ULL);
    uint32_t fraction = hundredths % 100U;
    char *p = dst;

    append_u32(&p, hundredths / 100U);
    append_str(&p, ".");
    if (fraction < 10U) {
        append_str(&p, "0");
    }
    append_u32(&p, fraction);
    append_str(&p, "W");
    *p = '\0';
}

static void draw_battery(u8g2_t *u8g2, uint16_t soc, bool valid)
{
    uint8_t segments = 0U;

    u8g2_DrawFrame(u8g2, 3U, 6U, 43U, 20U);
    u8g2_DrawFrame(u8g2, 46U, 12U, 3U, 8U);

    if (valid) {
        segments = (uint8_t)((soc + 24U) / 25U);
        if (segments > 4U) {
            segments = 4U;
        }
    }

    for (uint8_t i = 0U; i < segments; i++) {
        u8g2_DrawBox(u8g2, (u8g2_uint_t)(5U + (10U * i)), 8U, 8U, 16U);
    }
}

static void draw_load_state(u8g2_t *u8g2, bool on)
{
    u8g2_SetFont(u8g2, u8g2_font_5x7_tr);
    u8g2_SetFontPosTop(u8g2);

    if (on) {
        u8g2_DrawBox(u8g2, 95U, 17U, 26U, 13U);
        u8g2_SetDrawColor(u8g2, 0U);
        u8g2_DrawStr(u8g2, 102U, 20U, "ON");
        u8g2_SetDrawColor(u8g2, 1U);
    } else {
        u8g2_DrawFrame(u8g2, 95U, 17U, 26U, 13U);
        u8g2_DrawStr(u8g2, 99U, 20U, "OFF");
    }
}

static void draw_dashboard(u8g2_t *u8g2, const ec_status_t *status)
{
    char value[8];
    bool load_on = (status->state == EC_STATE_LOAD_RUNNING);
    bool charging = status->pg_active && status->current_valid &&
                    (status->current_ma > CHARGING_CURRENT_MIN_MA);

    u8g2_DrawVLine(u8g2, 87U, 2U, 28U);

    draw_battery(u8g2, status->soc_percent, status->soc_valid);
    if (status->soc_valid) {
        format_soc(value, status->soc_percent);
    } else {
        set_str(value, "--");
    }
    u8g2_SetFont(u8g2, u8g2_font_6x12_tr);
    u8g2_SetFontPosTop(u8g2);
    u8g2_DrawStr(u8g2, 54U, 3U, value);
    draw_cjk_text(u8g2, 54U, 18U, UI_TEXT_CHARGE);
    if (charging) {
        u8g2_DrawXBMP(u8g2, 79U, 9U, 7U, 14U, CHARGE_BOLT_XBM);
    }

    draw_cjk_text(u8g2, 96U, 1U, UI_TEXT_LOAD);
    draw_load_state(u8g2, load_on);
}

static void join_values(char *dst, const char *left, const char *right)
{
    char *p = dst;

    append_str(&p, left);
    append_str(&p, " ");
    append_str(&p, right);
    *p = '\0';
}

static void draw_details(u8g2_t *u8g2, const ec_status_t *status)
{
    char left[16];
    char right[16];
    char line[32];

    u8g2_SetFont(u8g2, u8g2_font_ec_values_9x15);
    u8g2_SetFontPosTop(u8g2);

    if (status->voltage_valid) {
        format_voltage(left, status->voltage_mv);
    } else {
        set_str(left, "--");
    }
    if (status->current_valid && status->voltage_valid) {
        format_power(right, status->current_ma, status->voltage_mv);
    } else {
        set_str(right, "--");
    }
    join_values(line, left, right);
    u8g2_DrawStr(u8g2, 8U, 1U, line);

    if (status->current_valid) {
        format_current_amps(left, status->current_ma);
    } else {
        set_str(left, "--");
    }
    u8g2_DrawStr(u8g2, 8U, 17U, left);
}

static void copy_bytes(uint8_t *dst, const uint8_t *src, uint16_t count)
{
    while (count > 0U) {
        *dst++ = *src++;
        count--;
    }
}

static bool tile_changed(const uint8_t *frame, uint16_t offset)
{
    for (uint8_t i = 0U; i < OLED_TILE_BYTES; i++) {
        if (frame[offset + i] != s_display_shadow[offset + i]) {
            return true;
        }
    }
    return false;
}

static bool present_full(u8g2_t *u8g2)
{
    if (!OLED_Present()) {
        s_shadow_valid = false;
        return false;
    }

    copy_bytes(s_display_shadow, u8g2_GetBufferPtr(u8g2), OLED_BUFFER_SIZE);
    s_shadow_valid = true;
    return true;
}

static bool present_changes(u8g2_t *u8g2)
{
    const uint8_t *frame = u8g2_GetBufferPtr(u8g2);

    if (!s_shadow_valid) {
        return present_full(u8g2);
    }

    for (uint8_t tile_y = 0U; tile_y < OLED_TILE_HEIGHT; tile_y++) {
        uint8_t tile_x = 0U;

        while (tile_x < OLED_TILE_WIDTH) {
            uint16_t offset = ((uint16_t)tile_y * OLED_WIDTH) +
                              ((uint16_t)tile_x * OLED_TILE_BYTES);

            if (!tile_changed(frame, offset)) {
                tile_x++;
                continue;
            }

            uint8_t run_start = tile_x;
            do {
                tile_x++;
                if (tile_x >= OLED_TILE_WIDTH) {
                    break;
                }
                offset = ((uint16_t)tile_y * OLED_WIDTH) +
                         ((uint16_t)tile_x * OLED_TILE_BYTES);
            } while (tile_changed(frame, offset));

            uint8_t run_width = (uint8_t)(tile_x - run_start);
            if (!OLED_PresentArea(run_start, tile_y, run_width, 1U)) {
                return false;
            }

            offset = ((uint16_t)tile_y * OLED_WIDTH) +
                     ((uint16_t)run_start * OLED_TILE_BYTES);
            copy_bytes(&s_display_shadow[offset], &frame[offset],
                       (uint16_t)run_width * OLED_TILE_BYTES);
        }
    }

    return true;
}

static bool render_page(const ec_status_t *status, bool force_full)
{
    u8g2_t *u8g2;

    if (status == NULL) {
        return false;
    }

    u8g2 = OLED_GetContext();
    u8g2_ClearBuffer(u8g2);
    if (s_page == 0U) {
        draw_dashboard(u8g2, status);
    } else {
        draw_details(u8g2, status);
    }
    return force_full ? present_full(u8g2) : present_changes(u8g2);
}

bool OLED_Display_Init(const ec_status_t *status)
{
    s_page = 0U;
    s_ready = false;
    s_shadow_valid = false;

    if ((status == NULL) || !OLED_Init()) {
        return false;
    }

    s_ready = true;
    if (!render_page(status, true)) {
        s_ready = false;
        return false;
    }
    return true;
}

void OLED_Display_NextPage(const ec_status_t *status)
{
    if (!s_ready || (status == NULL)) {
        return;
    }

    s_page = (uint8_t)((s_page + 1U) % PAGE_COUNT);
    (void)render_page(status, true);
}

void OLED_Display_Refresh(const ec_status_t *status)
{
    if (s_ready && (status != NULL)) {
        (void)render_page(status, false);
    }
}

void OLED_Display_Sleep(void)
{
    if (s_ready) {
        OLED_Sleep();
    }
}

void OLED_Display_Wake(void)
{
    if (s_ready) {
        OLED_Wake();
    }
}

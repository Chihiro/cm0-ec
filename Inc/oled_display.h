/**
 ******************************************************************************
 * @file           : oled_display.h
 * @brief          : u8g2 page controller for the SSD1306 128x32 OLED.
 *
 *   Page 0 is a two-column dashboard with a dominant charge area:
 *     x=0..86   large battery icon, percentage and charge label
 *     x=88..127 compact load title and ON/OFF state
 *   The divider is one pixel wide at x=87, y=2..29.
 *   Page 1 follows the reference layout: voltage/power on row 1 and current
 *   on row 2, both left-aligned with a large font. No time value is shown.
 *
 *   u8g2 renders into a 512-byte full buffer. Periodic refreshes compare it
 *   with a 512-byte display shadow and send only changed 8x8 tile runs.
 *   Initialization and page changes intentionally send a complete frame.
 ******************************************************************************
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdbool.h>
#include "ec_state_machine.h"

/* CMake supplies 0/1; other build systems default to OLED disabled. */
#ifndef EC_ENABLE_OLED
#define EC_ENABLE_OLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if EC_ENABLE_OLED

/** Initialize the OLED and draw page 0. */
bool OLED_Display_Init(const ec_status_t *status);

/** Advance to the next page and redraw; no-op if initialization failed. */
void OLED_Display_NextPage(const ec_status_t *status);

/** Redraw the current page with the latest status snapshot. */
void OLED_Display_Refresh(const ec_status_t *status);

/** Enter the SSD1306 power-save state before MCU STOP mode. */
void OLED_Display_Sleep(void);

/** Leave the SSD1306 power-save state after MCU wake-up. */
void OLED_Display_Wake(void);

#else

/* Keep state-machine calls harmless when the display is excluded. */
static inline bool OLED_Display_Init(const ec_status_t *status)
{
    (void)status;
    return false;
}
static inline void OLED_Display_NextPage(const ec_status_t *status)
{
    (void)status;
}
static inline void OLED_Display_Refresh(const ec_status_t *status)
{
    (void)status;
}
static inline void OLED_Display_Sleep(void) {}
static inline void OLED_Display_Wake(void) {}

#endif /* EC_ENABLE_OLED */

#ifdef __cplusplus
}
#endif

#endif /* OLED_DISPLAY_H */

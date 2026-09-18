/**
 ******************************************************************************
 * @file           : oled_ssd1306.h
 * @brief          : u8g2 port for the 0.91" SSD1306 128x32 OLED.
 *
 *   Bus: I2C1, PB6/PB7, 7-bit address 0x3C. The peripheral is initialized
 *   by BQ27220_Init(); this module only supplies the u8g2 byte callback and
 *   owns the 512-byte full-frame buffer.
 ******************************************************************************
 */

#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include <stdbool.h>
#include "u8g2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_WIDTH   128U
#define OLED_HEIGHT   32U
#define OLED_TILE_WIDTH  (OLED_WIDTH / 8U)
#define OLED_TILE_HEIGHT (OLED_HEIGHT / 8U)

/** Initialize u8g2, SSD1306 and the full-frame buffer. */
bool OLED_Init(void);

/** Return the application-owned u8g2 drawing context. */
u8g2_t *OLED_GetContext(void);

/** Transfer the current u8g2 full buffer to the panel. */
bool OLED_Present(void);

/** Transfer one tile-aligned rectangle from the current u8g2 buffer. */
bool OLED_PresentArea(uint8_t tile_x, uint8_t tile_y,
                      uint8_t tile_width, uint8_t tile_height);

/** Enter/leave the SSD1306 power-save state. */
void OLED_Sleep(void);
void OLED_Wake(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_SSD1306_H */

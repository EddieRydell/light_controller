/* Modified from: https://github.com/nopnop2002/esp-idf-st7789 */

#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

#define rgb565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

#define RED    rgb565(255,   0,   0) // 0xf800
#define GREEN  rgb565(  0, 255,   0) // 0x07e0
#define BLUE   rgb565(  0,   0, 255) // 0x001f
#define BLACK  rgb565(  0,   0,   0) // 0x0000
#define WHITE  rgb565(255, 255, 255) // 0xffff
#define GRAY   rgb565(128, 128, 128) // 0x8410
#define YELLOW rgb565(255, 255,   0) // 0xFFE0
#define CYAN   rgb565(  0, 156, 209) // 0x04FA
#define PURPLE rgb565(128,   0, 128) // 0x8010

#define LCD_CHAR_W 6
#define LCD_CHAR_H 8

#ifdef CONFIG_WIDTH
#define LCD_W CONFIG_WIDTH
#else
#define LCD_W 320
#endif

#ifdef CONFIG_HEIGHT
#define LCD_H CONFIG_HEIGHT
#else
#define LCD_H 240
#endif

typedef enum {DIRECTION0, DIRECTION90, DIRECTION180, DIRECTION270} direction_t;

typedef enum {
	SCROLL_RIGHT = 1,
	SCROLL_LEFT = 2,
	SCROLL_DOWN = 3,
	SCROLL_UP = 4,
} scroll_t;

typedef struct {
	int32_t     _width;
	int32_t     _height;
	int32_t     _offsetx;
	int32_t     _offsety;
	direction_t _font_direction;
	uint8_t     _font_size;
	bool        _font_back_en;
	uint16_t    _font_back_color;
	int8_t      _dc;
	int8_t      _bl;
	spi_device_handle_t _SPIHandle;
	bool        _use_frame_buffer;
	uint16_t   *_frame_buffer;
} TFT_t;

void lcdInit(TFT_t *dev);

// Draw (outline) and fill primitives
void lcdFillScreen(TFT_t *dev, uint16_t color);
void lcdDrawPixel(TFT_t *dev, int32_t x, int32_t y, uint16_t color);
void lcdDrawMultiPixels(TFT_t *dev, int32_t x, int32_t y, int32_t size, uint16_t *colors);
void lcdDrawHLine(TFT_t *dev, int32_t x, int32_t y, int32_t w, uint16_t color);
void lcdDrawVLine(TFT_t *dev, int32_t x, int32_t y, int32_t h, uint16_t color);
void lcdDrawLine(TFT_t *dev, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color);
void lcdDrawRect(TFT_t *dev, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color);
void lcdFillRect(TFT_t *dev, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color);
void lcdDrawTri(TFT_t *dev, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color);
void lcdFillTri(TFT_t *dev, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color);
void lcdDrawCircle(TFT_t *dev, int32_t x0, int32_t y0, int32_t r, uint16_t color);
void lcdFillCircle(TFT_t *dev, int32_t x0, int32_t y0, int32_t r, uint16_t color);
void lcdDrawRoundRect(TFT_t *dev, int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t r, uint16_t color);
void lcdDrawArrow(TFT_t *dev, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t w, uint16_t color);
void lcdFillArrow(TFT_t *dev, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t w, uint16_t color);

// Specify center and size of shape
void lcdDrawRectangle(TFT_t *dev, int32_t xc, int32_t yc, int32_t w, int32_t h, int32_t angle, uint16_t color);
void lcdDrawTriangle(TFT_t *dev, int32_t xc, int32_t yc, int32_t w, int32_t h, int32_t angle, uint16_t color);
void lcdDrawRegularPolygon(TFT_t *dev, int32_t xc, int32_t yc, int32_t n, int32_t r, int32_t angle, uint16_t color);

// Characters and strings
int32_t lcdDrawChar(TFT_t *dev, int32_t x, int32_t y, char ascii, uint16_t color);
int32_t lcdDrawString(TFT_t *dev, int32_t x, int32_t y, char *ascii, uint16_t color);

// Font parameters
void lcdSetFontDirection(TFT_t *dev, direction_t dir); // not implemented, always 0
void lcdSetFontSize(TFT_t *dev, uint8_t size);
void lcdSetFontBackground(TFT_t *dev, uint16_t color);
void lcdNoFontBackground(TFT_t *dev);

// Display configuration
void lcdSPIClockSpeed(int32_t speed);
void lcdDisplayOff(TFT_t *dev);
void lcdDisplayOn(TFT_t *dev);
void lcdBacklightOff(TFT_t *dev);
void lcdBacklightOn(TFT_t *dev);
void lcdInversionOff(TFT_t *dev);
void lcdInversionOn(TFT_t *dev);
void lcdFrameEnable(TFT_t *dev);
void lcdFrameDisable(TFT_t *dev);
void lcdWrapArround(TFT_t *dev, scroll_t scroll, int32_t start, int32_t end);
void lcdWriteFrame(TFT_t *dev);

#endif // LCD_H_

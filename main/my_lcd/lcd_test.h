#ifndef LCD_TEST_H_
#define LCD_TEST_H_

#include <stdint.h>
#include "freertos/FreeRTOS.h" // TickType_t
#include "my_lcd.h" // TFT_t

TickType_t LineTestHV(TFT_t *dev, int32_t width, int32_t height);

TickType_t LineTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t FillTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t ColorBarTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t ColorBandTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t FillRectTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t FillTriTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t FillCircleTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t CircleTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t RoundRectTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t ArrowTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t RectangleTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t TriangleTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t TextDirTest(TFT_t *dev, int32_t width, int32_t height);

TickType_t TextParamTest(TFT_t *dev, int32_t width, int32_t height);

// Calls all the tests in a forever loop
void LCD(void *pvParameters);

#endif // LCD_TEST_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h> // strcpy, strlen
#include <time.h> // time

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "my_lcd.h"

#define INTERVAL 200
#define WAIT vTaskDelay(INTERVAL)


TickType_t LineTestHV(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(dev, BLACK);
	color=RED;
	for(int32_t ypos=0;ypos<height;ypos=ypos+10) {
		lcdDrawHLine(dev, 0, ypos, width, color);
	}

	for(int32_t xpos=0;xpos<width;xpos=xpos+10) {
		lcdDrawVLine(dev, xpos, 0, height, color);
	}
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t LineTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(dev, BLACK);

	uint16_t red;
	uint16_t green;
	uint16_t blue;
	srand( (unsigned int)time( NULL ) );
	for(int32_t i=1;i<100;i++) {
		red=rand()&0xFFU;
		green=rand()&0xFFU;
		blue=rand()&0xFFU;
		color=rgb565(red, green, blue);
		int32_t x0=rand()%width;
		int32_t y0=rand()%height;
		int32_t x1=rand()%width;
		int32_t y1=rand()%height;
		lcdDrawLine(dev, x0, y0, x1, y1, color);
	}
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t FillTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	lcdFillScreen(dev, RED);
	lcdWriteFrame(dev);
	vTaskDelay(50);
	lcdFillScreen(dev, GREEN);
	lcdWriteFrame(dev);
	vTaskDelay(50);
	lcdFillScreen(dev, BLUE);
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t ColorBarTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	if (width < height) {
		int32_t y1,y2;
		y1 = height/3;
		y2 = (height/3)*2;
		lcdFillRect(dev, 0, 0, width-1, y1-1, RED);
		lcdFillRect(dev, 0, y1, width-1, y2-1, GREEN);
		lcdFillRect(dev, 0, y2, width-1, height-1, BLUE);
		lcdWriteFrame(dev);
	} else {
		int32_t x1,x2;
		x1 = width/3;
		x2 = (width/3)*2;
		lcdFillRect(dev, 0, 0, x1-1, height-1, RED);
		lcdFillRect(dev, x1, 0, x2-1, height-1, GREEN);
		lcdFillRect(dev, x2, 0, width-1, height-1, BLUE);
		lcdWriteFrame(dev);
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t ColorBandTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(dev, BLACK);
	color = RED;
	int32_t delta = height/16;
	int32_t ypos = 0;
	for(int32_t i=0;i<16;i++) {
		//ESP_LOGI(__FUNCTION__, "color=0x%x",color);
		lcdFillRect(dev, 0, ypos, width-1, ypos+delta, color);
		color = color >> 1;
		ypos = ypos + delta;
	}
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t FillRectTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(dev, CYAN);

	uint16_t red;
	uint16_t green;
	uint16_t blue;
	srand( (unsigned int)time( NULL ) );
	for(int32_t i=1;i<100;i++) {
		red=rand()&0xFFU;
		green=rand()&0xFFU;
		blue=rand()&0xFFU;
		color=rgb565(red, green, blue);
		int32_t xpos=rand()%width;
		int32_t ypos=rand()%height;
		int32_t size=rand()%(width/5);
		lcdFillRect(dev, xpos, ypos, xpos+size, ypos+size, color);
	}
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t FillTriTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(dev, CYAN);

	uint16_t red;
	uint16_t green;
	uint16_t blue;
	srand( (unsigned int)time( NULL ) );
	for(int32_t i=1;i<100;i++) {
		red=rand()&0xFFU;
		green=rand()&0xFFU;
		blue=rand()&0xFFU;
		color=rgb565(red, green, blue);
		int32_t x0=rand()%width;
		int32_t y0=rand()%height;
		int32_t x1=rand()%width;
		int32_t y1=rand()%height;
		int32_t x2=rand()%width;
		int32_t y2=rand()%height;
		lcdFillTri(dev, x0, y0, x1, y1, x2, y2, color);
	}
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t FillCircleTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(dev, CYAN);

	uint16_t red;
	uint16_t green;
	uint16_t blue;
	srand( (unsigned int)time( NULL ) );
	for(int32_t i=1;i<100;i++) {
		red=rand()&0xFFU;
		green=rand()&0xFFU;
		blue=rand()&0xFFU;
		color=rgb565(red, green, blue);
		int32_t radius=rand()%(width/5);
		int32_t xpos=rand()%width;
		if (xpos < radius) xpos = radius; // clip
		else if (xpos > width-1-radius) xpos = width-1-radius;
		int32_t ypos=rand()%height;
		if (ypos < radius) ypos = radius; // clip
		else if (ypos > height-1-radius) ypos = height-1-radius;
		lcdFillCircle(dev, xpos, ypos, radius, color);
	}
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t CircleTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	int32_t limit = width;
	if (height < width) limit = height;
	limit /= 2;
	lcdFillScreen(dev, BLACK);
	color = CYAN;
	int32_t xpos = width/2;
	int32_t ypos = height/2;
	for(int32_t i=5;i<limit;i=i+5) {
		lcdDrawCircle(dev, xpos, ypos, i, color);
	}
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t RoundRectTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();


	uint16_t color;
	int32_t limit = width;
	if (width > height) limit = height;
	lcdFillScreen(dev, BLACK);
	color = BLUE;
	for(int32_t i=5;i<limit;i=i+5) {
		if (i > (limit-i-1) ) break;
		//ESP_LOGI(__FUNCTION__, "i=%d, width-i-1=%d",(int)i, (int)width-i-1);
		lcdDrawRoundRect(dev, i, i, (width-i-1), (height-i-1), 10, color);
	}
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t ArrowTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t fontSize = 2;
	uint8_t fontWidth = LCD_CHAR_W * fontSize;
	uint8_t fontHeight = LCD_CHAR_H * fontSize;

	int32_t xpos;
	int32_t ypos;
	int32_t	stlen;
	char ascii[24];
	uint16_t color;

	lcdFillScreen(dev, BLACK);
	lcdSetFontSize(dev, fontSize);

	strcpy(ascii, "LCD");
	ypos = ((height - fontHeight) / 2) - 1;
	xpos = (width - (strlen(ascii) * fontWidth)) / 2;
	lcdSetFontDirection(dev, DIRECTION0);
	color = WHITE;
	lcdDrawString(dev, xpos, ypos, ascii, color);

	lcdSetFontDirection(dev, 0);
	color = RED;
	lcdFillArrow(dev, 10, 10, 0, 0, 5, color);
	strcpy(ascii, "0,0");
	lcdDrawString(dev, 0, 10+8, ascii, color);

	color = GREEN;
	lcdFillArrow(dev, width-11, 10, width-1, 0, 5, color);
	sprintf(ascii, "%d,0",(int)width-1);
	stlen = strlen(ascii);
	xpos = (width-1) - (fontWidth*stlen);
	lcdDrawString(dev, xpos, 10+8, ascii, color);

	color = GRAY;
	lcdFillArrow(dev, 10, height-11, 0, height-1, 5, color);
	sprintf(ascii, "0,%d",(int)height-1);
	ypos = (height-11) - 8 - fontHeight;
	lcdDrawString(dev, 0, ypos, ascii, color);

	color = CYAN;
	lcdFillArrow(dev, width-11, height-11, width-1, height-1, 5, color);
	sprintf(ascii, "%d,%d",(int)width-1, (int)height-1);
	stlen = strlen(ascii);
	xpos = (width-1) - (fontWidth*stlen);
	lcdDrawString(dev, xpos, ypos, ascii, color);

	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t RectangleTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(dev, BLACK);
	color = CYAN;
	int32_t xpos = width/2;
	int32_t ypos = height/2;

	int32_t h = ((height < width) ? height : width) * 0.7;
	int32_t w = h * 0.5;
	int32_t angle;

	for(angle = 0; angle < (360*3); angle += 30) {
		lcdDrawRectangle(dev, xpos, ypos, w, h, angle, color);
		lcdDrawRectangle(dev, xpos, ypos, w, h, angle, BLACK);
	}

	for(angle = 0; angle < 180; angle += 30) {
		lcdDrawRectangle(dev, xpos, ypos, w, h, angle, color);
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t TriangleTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(dev, BLACK);
	color = CYAN;
	int32_t xpos = width/2;
	int32_t ypos = height/2;

	int32_t h = ((height < width) ? height : width) * 0.7;
	int32_t w = h * 0.7;
	int32_t angle;

	for(angle = 0; angle < (360*3); angle += 30) {
		lcdDrawTriangle(dev, xpos, ypos, w, h, angle, color);
		lcdDrawTriangle(dev, xpos, ypos, w, h, angle, BLACK);
	}

	for(angle = 0; angle < 360; angle += 30) {
		lcdDrawTriangle(dev, xpos, ypos, w, h, angle, color);
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t TextDirTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t fontSize = (LCD_W > 240) ? 3 : 2;

	uint16_t color;
	char ascii[20];

	lcdFillScreen(dev, BLACK);
	lcdSetFontSize(dev, fontSize);

	color = RED;
	strcpy(ascii, "Direction=0");
	lcdSetFontDirection(dev, 0);
	lcdDrawString(dev, 0, 0, ascii, color);

#if 0 // other directions unimplemented
	color = BLUE;
	strcpy(ascii, "Direction=2");
	lcdSetFontDirection(dev, 2);
	lcdDrawString(dev, width-1, height-1, ascii, color);

	color = CYAN;
	strcpy(ascii, "Direction=1");
	lcdSetFontDirection(dev, 1);
	lcdDrawString(dev, width-1, 0, ascii, color);

	color = GREEN;
	strcpy(ascii, "Direction=3");
	lcdSetFontDirection(dev, 3);
	lcdDrawString(dev, 0, height-1, ascii, color);
#endif
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t TextParamTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint8_t i;
	int32_t xpos, ypos;
	char ascii[40];
	uint16_t ctab[] = {RED,WHITE,BLACK,GREEN,BLUE,GRAY,YELLOW,CYAN,PURPLE};

	lcdFillScreen(dev, BLACK);
	lcdSetFontDirection(dev, 0);

	for (xpos = 0, ypos = 0, i = 1; ; xpos += 5, ypos += LCD_CHAR_H*i, i++) {
		lcdSetFontSize(dev, i);
		lcdSetFontBackground(dev, ctab[(i+1)%9]);
		sprintf(ascii, "size:%hhu", i);
		if (strlen(ascii)*LCD_CHAR_W*i+xpos > LCD_W) break;
		lcdDrawString(dev, xpos, ypos, ascii, ctab[i%9]);
	}

	lcdNoFontBackground(dev);
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t TextTest(TFT_t *dev, int32_t width, int32_t height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(dev, BLACK);
	lcdSetFontDirection(dev, 0);

	uint16_t red;
	uint16_t green;
	uint16_t blue;
	uint8_t size;
	char text[] = "Carpe Diem!";
	uint32_t tlen = strlen(text);
	uint16_t bgtab[] = {RED,GREEN,BLUE,BLACK,GRAY,YELLOW,CYAN,PURPLE};
	srand( (unsigned int)time( NULL ) );
	for(int32_t i=1;i<100;i++) {
		red=rand()&0xFFU;
		green=rand()&0xFFU;
		blue=rand()&0xFFU;
		color=rgb565(red, green, blue);
		size = (i&0x3)+1;
		int32_t xpos=rand()%(width-LCD_CHAR_W*size*tlen+1);
		int32_t ypos=rand()%(height-LCD_CHAR_H*size+1);
		lcdSetFontSize(dev, size);
		lcdSetFontBackground(dev, bgtab[i%8]);
		lcdDrawString(dev, xpos, ypos, text, color);
	}
	lcdWriteFrame(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

void LCD(void *pvParameters)
{
	TFT_t dev;

	lcdInit(&dev);

	while(1) {

		FillTest(&dev, LCD_W, LCD_H);
		WAIT;

		ColorBarTest(&dev, LCD_W, LCD_H);
		WAIT;

		ColorBandTest(&dev, LCD_W, LCD_H);
		WAIT;

		ArrowTest(&dev, LCD_W, LCD_H);
		WAIT;

		LineTestHV(&dev, LCD_W, LCD_H);
		WAIT;

		LineTest(&dev, LCD_W, LCD_H);
		WAIT;

		CircleTest(&dev, LCD_W, LCD_H);
		WAIT;

		RoundRectTest(&dev, LCD_W, LCD_H);
		WAIT;

		FillRectTest(&dev, LCD_W, LCD_H);
		WAIT;

		FillTriTest(&dev, LCD_W, LCD_H);
		WAIT;

		FillCircleTest(&dev, LCD_W, LCD_H);
		WAIT;

		if (dev._use_frame_buffer == false) {
			RectangleTest(&dev, LCD_W, LCD_H);
			WAIT;

			TriangleTest(&dev, LCD_W, LCD_H);
			WAIT;
		}

		TextDirTest(&dev, LCD_W, LCD_H);
		WAIT;

		TextParamTest(&dev, LCD_W, LCD_H);
		WAIT;

		TextTest(&dev, LCD_W, LCD_H);
		WAIT;

	} // end while
}

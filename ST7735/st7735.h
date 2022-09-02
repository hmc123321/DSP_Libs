/*
 * st7735.h
 *
 *  Created on: 2022年6月10日
 *      Author: HMC
 */

#ifndef LIB_ST7735_H_
#define LIB_ST7735_H_

// #define ADAFRUIT_ST7735

#include "driverlib.h"
#include "device.h"
#include "board.h"

#ifdef ADAFRUIT_ST7735
#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

#define ST7735_IS_160X80 		1
#define ST7735_XSTART 			26
#define ST7735_YSTART 			1
#define ST7735_WIDTH  			80
#define ST7735_HEIGHT 			160
//#define ST7735_VALUE_ROTATION	0
#define ST7735_DATA_ROTATION 	(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR)
#endif

// Color definitions
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF



void  SPI_WriteData(uint16_t data);
void  Lcd_WriteCmd(uint16_t Data);
void  Lcd_WriteDat(uint16_t Data);
void  LCD_WriteDat_16Bit(unsigned int Data);
void  Reset();
void  lcd_initial();
#ifdef ADAFRUIT_ST7735
void ST7735_Init();
static void ST7735_ExecuteCommandList(const uint8_t *addr);
static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color) ;
void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_FillScreen(uint16_t color);
#endif
void Lcd_SetRegion(unsigned int x_start,unsigned int y_start,unsigned int x_end,unsigned int y_end);
void dsp_single_colour(int color);
void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data);
void Lcd_SetXY(uint16_t x,uint16_t y);

#endif /* LIB_ST7735_H_ */

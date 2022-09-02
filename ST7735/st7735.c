/*
 * st7735.c
 *
 *  Created on: 2022年6月10日
 *      Author: HMC
 */

#include "st7735.h"

#ifdef ADAFRUIT_ST7735
#define DELAY 0x80

// based on Adafruit ST7735 library for Arduino
static const uint8_t
init_cmds1[] = {            		// Init for 7735R, part 1 (red or green tab)
		  15,                       // 15 commands in list:
		  ST7735_SWRESET, DELAY,  	//  1: Software reset, 0 args, w/delay
		  150,                    	//     150 ms delay
		  ST7735_SLPOUT, DELAY,  	//  2: Out of sleep mode, 0 args, w/delay
		  255,                    	//     500 ms delay
		  ST7735_FRMCTR1, 3,		//  3: Frame rate ctrl - normal mode, 3 args:
		  0x01, 0x2C, 0x2D,       	//     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		  ST7735_FRMCTR2, 3,  		//  4: Frame rate control - idle mode, 3 args:
		  0x01, 0x2C, 0x2D,       	//     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		  ST7735_FRMCTR3, 6,  		//  5: Frame rate ctrl - partial mode, 6 args:
		  0x01, 0x2C, 0x2D,       	//     Dot inversion mode
		  0x01, 0x2C, 0x2D,       	//     Line inversion mode
		  ST7735_INVCTR, 1,  		//  6: Display inversion ctrl, 1 arg, no delay:
		  0x07,                   	//     No inversion
		  ST7735_PWCTR1, 3,  		//  7: Power control, 3 args, no delay:
		  0xA2,
		  0x02,                   	//     -4.6V
		  0x84,                   	//     AUTO mode
		  ST7735_PWCTR2, 1,  		//  8: Power control, 1 arg, no delay:
		  0xC5,                   	//     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
		  ST7735_PWCTR3, 2,  		//  9: Power control, 2 args, no delay:
		  0x0A,                   	//     Opamp current small
		  0x00,                   	//     Boost frequency
		  ST7735_PWCTR4, 2,  		// 10: Power control, 2 args, no delay:
		  0x8A,                   	//     BCLK/2, Opamp current small & Medium low
		  0x2A,
		  ST7735_PWCTR5, 2,  		// 11: Power control, 2 args, no delay:
		  0x8A, 0xEE,
		  ST7735_VMCTR1, 1,  		// 12: Power control, 1 arg, no delay:
		  0x0E,
		  ST7735_INVOFF, 0,  		// 13: Don't invert display, no args, no delay
		  ST7735_MADCTL, 1,  		// 14: Memory access control (directions), 1 arg:
		  ST7735_DATA_ROTATION,     //     row addr/col addr, bottom to top refresh
		  ST7735_COLMOD, 1,  		// 15: set color mode, 1 arg, no delay:
		  0x05},                 	//     16-bit color

init_cmds2[] = {					// Init for 7735S, part 2 (160x80 display)
		3,                        	//  3 commands in list:
		ST7735_CASET, 4,  			//  1: Column addr set, 4 args, no delay:
		0x00, 0x00,             	//     XSTART = 0
		0x00, 0x4F,             	//     XEND = 79
		ST7735_RASET, 4,  			//  2: Row addr set, 4 args, no delay:
		0x00, 0x00,             	//     XSTART = 0
		0x00, 0x9F ,            	//     XEND = 159
		ST7735_INVON, 0 },        //  3: Invert colors

init_cmds3[] = {            		// Init for 7735R, part 3 (red or green tab)
		4,                        	//  4 commands in list:
		ST7735_GMCTRP1, 16, 		//  1: Magical unicorn dust, 16 args, no delay:
		0x02, 0x1c, 0x07, 0x12,
		0x37, 0x32, 0x29, 0x2d,
		0x29, 0x25, 0x2B, 0x39,
		0x00, 0x01, 0x03, 0x10,
		ST7735_GMCTRN1, 16, 		//  2: Sparkles and rainbows, 16 args, no delay:
		0x03, 0x1d, 0x07, 0x06,
		0x2E, 0x2C, 0x29, 0x2D,
		0x2E, 0x2E, 0x37, 0x3F,
		0x00, 0x00, 0x02, 0x10,
		ST7735_NORON, DELAY, 		//  3: Normal display on, no args, w/delay
		10,                     	//     10 ms delay
		ST7735_DISPON, DELAY, 		//  4: Main screen turn on, no args w/delay
		100 };                  	//     100 ms delay


static void ST7735_ExecuteCommandList(const uint8_t *addr)
{
    uint8_t i,numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--)
    {
    	uint8_t cmd = *addr++;
        Lcd_WriteCmd(cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if(numArgs)
        {
            for (i=0;i<numArgs;i++)
                Lcd_WriteDat(*((uint16_t*)addr+i));
            addr += numArgs;
        }

        if(ms)
        {
            ms = *addr++;
            if(ms == 255) ms = 500;
            DEVICE_DELAY_US(ms*1000);
        }
    }
}


void ST7735_Init()
{
	GPIO_writePin(LCD_CS,0);
    Reset();
    ST7735_ExecuteCommandList(init_cmds1);
    ST7735_ExecuteCommandList(init_cmds2);
    ST7735_ExecuteCommandList(init_cmds3);
    GPIO_writePin(LCD_CS,1);
}

static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    // column address set
    Lcd_WriteCmd(ST7735_CASET);
    uint8_t data[] = { 0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART };
    Lcd_WriteDat(data[0]);
    Lcd_WriteDat(data[1]);
    Lcd_WriteDat(data[2]);
    Lcd_WriteDat(data[3]);
    // row address set
    Lcd_WriteCmd(ST7735_RASET);
    data[1] = y0 + ST7735_YSTART;
    data[3] = y1 + ST7735_YSTART;

    Lcd_WriteDat(data[0]);
    Lcd_WriteDat(data[1]);
    Lcd_WriteDat(data[2]);
    Lcd_WriteDat(data[3]);

    // write to RAM
    Lcd_WriteCmd(ST7735_RAMWR);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;

    GPIO_writePin(LCD_CS,0);

    ST7735_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    Lcd_WriteDat(data[0]);
    Lcd_WriteDat(data[1]);
    GPIO_writePin(LCD_CS,1);
}

void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    GPIO_writePin(LCD_CS,0);
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);

    uint8_t data[] = { color >> 8, color & 0xFF };
    GPIO_writePin(LCD_DC,1);
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            Lcd_WriteDat(data[0]);
            Lcd_WriteDat(data[1]);
        }
    }

    GPIO_writePin(LCD_CS,1);
}

void ST7735_FillScreen(uint16_t color) {
    ST7735_FillRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}
#endif

void  SPI_WriteData(uint16_t Data)
{
    SPI_writeDataBlockingNonFIFO(mySPI0_BASE, Data<<8);//写入完成前会SPI_STS_INT_FLAG为0，完成后会置一，由读取清零
    SPI_readDataBlockingNonFIFO(mySPI0_BASE);
    DEVICE_DELAY_US(5);//200KHz,一个时钟是5us

//    sendFlag=1;
//    SPI_writeDataNonBlocking(mySPI0_BASE, Data<<8);
//    Interrupt_enable(INT_SPIA_TX);
//    while (sendFlag==1);
}
//向液晶屏写一个8位指令
void  Lcd_WriteCmd(uint16_t Data)
{
		
//		GPIO_writePin(LCD_CS,0);
		GPIO_writePin(LCD_DC,0);
		SPI_WriteData(Data);
		// GPIO_writePin(LCD_CS,1);
}
//向液晶屏写一个8位数据
void  Lcd_WriteDat(uint16_t Data)
{	
//		GPIO_writePin(LCD_CS,0);
		GPIO_writePin(LCD_DC,1);
        SPI_WriteData(Data);
		// GPIO_writePin(LCD_CS,1);
}
//向液晶屏写一个16位数据
void  Lcd_WriteDat_16Bit(unsigned int Data)
{
//	GPIO_writePin(LCD_CS,0);
	GPIO_writePin(LCD_DC,1);
	Lcd_WriteDat(Data>>8);
	Lcd_WriteDat(Data);
	// GPIO_writePin(LCD_CS,1);
}

void Reset()
{
    GPIO_writePin(LCD_RES,0);
    DEVICE_DELAY_US(100000);
    GPIO_writePin(LCD_RES,1);
    DEVICE_DELAY_US(50000);
}

void lcd_initial()
{	
	Reset(); //Reset before LCD Init.
	Lcd_WriteCmd(0x11);//Sleep exit 
	DEVICE_DELAY_US (120000);
	
	
	Lcd_WriteCmd(0xfe);     //Inter register enable 1
    Lcd_WriteCmd(0xfe);     
    Lcd_WriteCmd(0xfe);
    Lcd_WriteCmd(0xef);     //Inter register enable 2

    Lcd_WriteCmd(0xb3);     //设置伽马调整允许  Frame Rate Control (In Partial mode/ full colors)
    Lcd_WriteDat(0x03);
        
    Lcd_WriteCmd(0x36);     //Memory Access Ctrl
    Lcd_WriteDat(0x68);        //MY MX MV ML BGR MH 0 0  竖屏D8 
        
    Lcd_WriteCmd(0x3a);             //Pixel Format Set
    Lcd_WriteDat(0x05);            //16BIT  565格式

    Lcd_WriteCmd(0xb6);           //命令设置  Display Function set 5
    Lcd_WriteDat(0x11);
    Lcd_WriteCmd(0xac);  
    Lcd_WriteDat(0x0b);

    Lcd_WriteCmd(0xb4);             //反相显示控制
    Lcd_WriteDat(0x21);

    Lcd_WriteCmd(0xb1);             //命令设置 Frame Rate Control (In normal mode/ Full colors)
    Lcd_WriteDat(0xc0);

    Lcd_WriteCmd(0xe6);              //VREG1_CTL   
    Lcd_WriteDat(0x50);
    Lcd_WriteDat(0x43);          //默认值
    Lcd_WriteCmd(0xe7);             //VREG2_CTL
    Lcd_WriteDat(0x56);            //-3V
    Lcd_WriteDat(0x43);          //

    Lcd_WriteCmd(0xE0);             //SET_GAMMA1 
    Lcd_WriteDat(0x1f);
    Lcd_WriteDat(0x41);
    Lcd_WriteDat(0x1B);
    Lcd_WriteDat(0x55);
    Lcd_WriteDat(0x36);
    Lcd_WriteDat(0x3d);
    Lcd_WriteDat(0x3e);
    Lcd_WriteDat(0x0); 
    Lcd_WriteDat(0x16);
    Lcd_WriteDat(0x08);
    Lcd_WriteDat(0x09);
    Lcd_WriteDat(0x15);
    Lcd_WriteDat(0x14);
    Lcd_WriteDat(0xf); 

    Lcd_WriteCmd(0xE1);             //SET_GAMMA2
    Lcd_WriteDat(0x1f);
    Lcd_WriteDat(0x41);
    Lcd_WriteDat(0x1B);
    Lcd_WriteDat(0x55);
    Lcd_WriteDat(0x36);
    Lcd_WriteDat(0x3d);
    Lcd_WriteDat(0x3e);
    Lcd_WriteDat(0x0); 
    Lcd_WriteDat(0x16);
    Lcd_WriteDat(0x08);
    Lcd_WriteDat(0x09);
    Lcd_WriteDat(0x15);
    Lcd_WriteDat(0x14);
    Lcd_WriteDat(0xf); 

    Lcd_WriteCmd(0xfe);         //关闭Inter register enable
    Lcd_WriteCmd(0xff);

    Lcd_WriteCmd(0x35);         //传动效应线 ON
    Lcd_WriteDat(0x00);
    Lcd_WriteCmd(0x44);         //Scan line set
    Lcd_WriteDat(0x00);
    Lcd_WriteCmd(0x11);         //Sleep Out
    DEVICE_DELAY_US (120000);
    Lcd_WriteCmd(0x29);         //显示开
	//设置初始行列起始地址
    Lcd_WriteCmd(0x2A); //Set Column Address 
    Lcd_WriteDat(0x00);
    Lcd_WriteDat(0x00); 
    Lcd_WriteDat(0x00); 
    Lcd_WriteDat(0x9f); 
    
    Lcd_WriteCmd(0x2B); //Set Page Address 
    Lcd_WriteDat(0x00); 
    Lcd_WriteDat(0x18); 
    Lcd_WriteDat(0x00); 
    Lcd_WriteDat(0x67); 
    Lcd_WriteCmd(0x2c);
	

/*
	Lcd_WriteCmd(0x21);//反相显示开【20H：反相显示关】

	Lcd_WriteCmd(0xB1); // Frame Rate Control (In normal mode/ Full colors)
	Lcd_WriteDat(0x05);
	Lcd_WriteDat(0x3A);
	Lcd_WriteDat(0x3A);

	Lcd_WriteCmd(0xB2);// Frame Rate Control (In Idle mode/ 8-colors)
	Lcd_WriteDat(0x05);
	Lcd_WriteDat(0x3A);
	Lcd_WriteDat(0x3A);

	Lcd_WriteCmd(0xB3); // Frame Rate Control (In Partial mode/ full colors)
	Lcd_WriteDat(0x05);  
	Lcd_WriteDat(0x3A);
	Lcd_WriteDat(0x3A);
	Lcd_WriteDat(0x05);
	Lcd_WriteDat(0x3A);
	Lcd_WriteDat(0x3A);

	Lcd_WriteCmd(0xB4);//反相显示控制
	Lcd_WriteDat(0x03);

	Lcd_WriteCmd(0xC0);//Power Control 1
	Lcd_WriteDat(0x62);
	Lcd_WriteDat(0x02);
	Lcd_WriteDat(0x04);

	Lcd_WriteCmd(0xC1);//Power Control 2
	Lcd_WriteDat(0xC0);

	Lcd_WriteCmd(0xC2);//Power Control 3 [in Normal mode]
	Lcd_WriteDat(0x0D);
	Lcd_WriteDat(0x00);

	Lcd_WriteCmd(0xC3);//Power Control 4 [in Idle mode]
	Lcd_WriteDat(0x8D);
	Lcd_WriteDat(0x6A);   

	Lcd_WriteCmd(0xC4);//Power Control 5 [in Partial mode]
	Lcd_WriteDat(0x8D); 
	Lcd_WriteDat(0xEE); 

	Lcd_WriteCmd(0xC5);// VCOM Control 1
	Lcd_WriteDat(0x0E);    

//设置Gamma START
	Lcd_WriteCmd(0xE0);//Gamma (‘+’polarity) Correction Characteristics Setting
	Lcd_WriteDat(0x10);
	Lcd_WriteDat(0x0E);
	Lcd_WriteDat(0x02);
	Lcd_WriteDat(0x03);
	Lcd_WriteDat(0x0E);
	Lcd_WriteDat(0x07);
	Lcd_WriteDat(0x02);
	Lcd_WriteDat(0x07);
	Lcd_WriteDat(0x0A);
	Lcd_WriteDat(0x12);
	Lcd_WriteDat(0x27);
	Lcd_WriteDat(0x37);
	Lcd_WriteDat(0x00);
	Lcd_WriteDat(0x0D);
	Lcd_WriteDat(0x0E);
	Lcd_WriteDat(0x10);

	Lcd_WriteCmd(0xE1);//Gamma ‘-’polarity Correction Characteristics Setting
	Lcd_WriteDat(0x10);
	Lcd_WriteDat(0x0E);
	Lcd_WriteDat(0x03);
	Lcd_WriteDat(0x03);
	Lcd_WriteDat(0x0F);
	Lcd_WriteDat(0x06);
	Lcd_WriteDat(0x02);
	Lcd_WriteDat(0x08);
	Lcd_WriteDat(0x0A);
	Lcd_WriteDat(0x13);
	Lcd_WriteDat(0x26);
	Lcd_WriteDat(0x36);
	Lcd_WriteDat(0x00);
	Lcd_WriteDat(0x0D);
	Lcd_WriteDat(0x0E);
	Lcd_WriteDat(0x10);
	//设置GAMMA END

	Lcd_WriteCmd(0x29);//显示开启 

*/
	}

/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点
返回值：无
*************************************************/
void Lcd_SetRegion(unsigned int x_start,unsigned int y_start,unsigned int x_end,unsigned int y_end)
{	
	//硬件选择GM = 011，分辨率为128*160，屏幕是80*160，因此需要在起始地址上加24个像素。
	Lcd_WriteCmd(0x2a);			//列地址设置
	Lcd_WriteDat(0x00);			//起始高8位
	Lcd_WriteDat(x_start);	//起始低8位
	Lcd_WriteDat(0x00);			//终止高8位
	Lcd_WriteDat(x_end);		//终止低8位

	Lcd_WriteCmd(0x2b);			//行地址设置
	Lcd_WriteDat(0x00);			//起始高8位
	Lcd_WriteDat(y_start+24);		//起始低8位
	Lcd_WriteDat(0x00);			//终止高8位
	Lcd_WriteDat(y_end+24);		//终止低8位

	Lcd_WriteCmd(0x2c);			//存储器写入数据
}

void Lcd_SetXY(uint16_t x,uint16_t y)
{
    Lcd_SetRegion(x,y,x,y);
}

void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data)
{
	Lcd_SetRegion(x,y,x+1,y+1);
	LCD_WriteDat_16Bit(Data);
}    

void dsp_single_colour(int color)
{
    uint16_t i,j;
    Lcd_SetRegion(0,0,160-1,80-1);
    // Lcd_WriteCmd(0x2C);
    for (i=0;i<160;i++)
        for (j=0;j<80;j++)
            Lcd_WriteDat_16Bit(color);
}


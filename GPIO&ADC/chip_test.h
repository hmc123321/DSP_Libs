#ifndef __CHIP_TEST_H__
#define __CHIP_TEST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "stdlib.h"

//MAX1: 46   - 0    = 46
//MAX2: 538  - 512  = 26
//MAX3: 1068 - 1024 = 44
//MAX4: 1604 - 1536 = 68
//MAX5: 2154 - 2048 = 106
//MAX6: # 2731 - 2560 = 171

#define EN_GPIO_MUX_TABLE    // 取消注释此行以启用对IO复用器查表

#define RED_LED DACA_BASE
#define GREEN_LED DACB_BASE
#define TEST_PASSED 1
#define TEST_FAILED 0
#define ADC_MAX  2500
//*****************************************************************************
//
//! 传入pinMode函数的参数，设置IO的输入输出-上拉下拉-推挽开漏等组合状态
//
//*****************************************************************************
typedef enum {
    INPUT,    
    OUTPUT,
    INPUT_PULLUP,
    OUTPUT_OD,
    OUTPUT_OD_PULLUP,
} Tester_GPIOPinState;
//*****************************************************************************
//
//! ADC通道的索引，其中ADCA：0-15，ADCB：16-31，ADCC：32-47
//! 传入初始化函数进行计算后获取基地址与偏移通道，将地址写入Tester_GpioGroup->adcPinBaseAddr
//
//*****************************************************************************
typedef enum 
{
    A0  ,    A1  ,    A2  ,    A3  ,
    A4  ,    A5  ,    A6  ,    A7  ,
    A8  ,    A9  ,    A10 ,    A11 ,
    A12 ,    A13 ,    A14 ,    A15 ,

    B0  ,    B1  ,    B2  ,    B3  ,
    B4  ,    B5  ,    B6  ,    B7  ,
    B8  ,    B9  ,    B10 ,    B11 ,
    B12 ,    B13 ,    B14 ,    B15 ,

    C0  ,    C1  ,    C2  ,    C3  ,
    C4  ,    C5  ,    C6  ,    C7  ,
    C8  ,    C9  ,    C10 ,    C11 ,
    C12 ,    C13 ,    C14 ,    C15  
}Tester_ADCChannelIndex;

/*********************************extern variables*************************/
extern uint32_t GPIO_Pin_Mux[60];

/*********************************Function declarations*************************/

void pinMode(uint32_t pin,Tester_GPIOPinState mode);
void ADC_AllModulesInit();//1.初始化全部ADC模块
void ADC_SOCChannelLink(uint32_t base,ADC_Channel socNumber);//2.绑定ADC通道到对应的SOC
void ADC_SOCInterruptLink(uint32_t base,ADC_Channel socNumber);//3.链接对应的SOC到中断
uint16_t analogRead(uint32_t base,ADC_Channel socNumber);//4.获取ADC采样值

#ifdef __cplusplus
}
#endif

#endif 



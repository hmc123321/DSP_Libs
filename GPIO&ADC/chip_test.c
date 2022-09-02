/**
 * @file chip_test.c
 * @author HMC
 * @brief 
 * @version 0.1
 * @date 2022-04-25
 * @copyright Copyright (c) 2022
 * 
 */

#include "chip_test.h"

#ifdef EN_GPIO_MUX_TABLE

/*******************************************************************************
 *  定义常量，开辟空间
 ******************************************************************************/

/**
 * @brief 将从0-59的GPIO序号对应到传入GPIO_setPinConfig函数
 * 
 */
uint32_t GPIO_Pin_Mux[60]=
{
    //0x00060000-0x00061E00
    GPIO_0_GPIO0    ,   GPIO_1_GPIO1    ,   GPIO_2_GPIO2    ,   GPIO_3_GPIO3    ,
    GPIO_4_GPIO4    ,   GPIO_5_GPIO5    ,   GPIO_6_GPIO6    ,   GPIO_7_GPIO7    ,
    GPIO_8_GPIO8    ,   GPIO_9_GPIO9    ,   GPIO_10_GPIO10  ,   GPIO_11_GPIO11  ,
    GPIO_12_GPIO12  ,   GPIO_13_GPIO13  ,   GPIO_14_GPIO14  ,   GPIO_15_GPIO15  ,
    //0x00080000-0x00081E00
    GPIO_16_GPIO16  ,   GPIO_17_GPIO17  ,   GPIO_18_GPIO18  ,   0            ,
    GPIO_20_GPIO20  ,   GPIO_21_GPIO21  ,   GPIO_22_GPIO22  ,   GPIO_23_GPIO23  ,
    GPIO_24_GPIO24  ,   GPIO_25_GPIO25  ,   GPIO_26_GPIO26  ,   GPIO_27_GPIO27  ,
    GPIO_28_GPIO28  ,   GPIO_29_GPIO29  ,   GPIO_30_GPIO30  ,   GPIO_31_GPIO31  ,
    //0x00460000-0x00461E00
    GPIO_32_GPIO32  ,   GPIO_33_GPIO33  ,   GPIO_34_GPIO34  ,   GPIO_35_GPIO35  ,
    0               ,   GPIO_37_GPIO37  ,   0               ,   GPIO_39_GPIO39  ,
    GPIO_40_GPIO40  ,   GPIO_41_GPIO41  ,   GPIO_42_GPIO42  ,   GPIO_43_GPIO43  ,
    GPIO_44_GPIO44  ,   GPIO_45_GPIO45  ,   GPIO_46_GPIO46  ,   GPIO_47_GPIO47  ,
     //0x00480000-0x00481600
    GPIO_48_GPIO48  ,   GPIO_49_GPIO49  ,   GPIO_50_GPIO50  ,   GPIO_51_GPIO51  ,
    GPIO_52_GPIO52  ,   GPIO_53_GPIO53  ,   GPIO_54_GPIO54  ,   GPIO_55_GPIO55  ,
    GPIO_56_GPIO56  ,   GPIO_57_GPIO57  ,   GPIO_58_GPIO58  ,   GPIO_59_GPIO59  
};
#endif

/*******************************************************************************
 *  基本操作部分
 ******************************************************************************/
/**
 * @brief 配置GPOIO的输入输出-上拉下拉-推挽开漏等组合状态
 * 
 * @param pin   输入的IO序号，从0-59
 * @param mode  IO配置模式，参数如下：
 * \b INPUT, \b OUTPUT, \b INPUT_PULLUP, \b OUTPUT_OD, \b OUTPUT_OD_PULLUP
 */
void pinMode(uint32_t pin,Tester_GPIOPinState mode)
{
    //计算该引脚的MUX地址
    uint32_t gpioPinMuxBase;
    #ifdef EN_GPIO_MUX_TABLE
    gpioPinMuxBase=GPIO_Pin_Mux[pin];

    #else
	if (pin<16)
	{
		gpioPinMuxBase=GPIO_0_GPIO0;
	}
	else if (pin<32)
	{
		gpioPinMuxBase=GPIO_16_GPIO16;
	}
	else if (pin<48)
	{
		gpioPinMuxBase=GPIO_32_GPIO32;
	}
	else 
	{
		gpioPinMuxBase=GPIO_48_GPIO48;
	}
    gpioPinMuxBase=gpioPinMuxBase+((pin%16)*0x200);
    if (pin==23)
	{
		gpioPinMuxBase+=0x04;
	}
    #endif

    EALLOW;
    GPIO_setPinConfig(gpioPinMuxBase);
    switch (mode)
    {
        case INPUT:
            GPIO_setDirectionMode(pin, GPIO_DIR_MODE_IN);
            GPIO_setPadConfig(pin, GPIO_PIN_TYPE_STD);
            break;
        case OUTPUT:
            GPIO_setDirectionMode(pin, GPIO_DIR_MODE_OUT);
            GPIO_setPadConfig(pin, GPIO_PIN_TYPE_STD);
            break;
        case INPUT_PULLUP:
            GPIO_setDirectionMode(pin, GPIO_DIR_MODE_IN);
            GPIO_setPadConfig(pin, GPIO_PIN_TYPE_PULLUP);
            break;
        case OUTPUT_OD:
            GPIO_setDirectionMode(pin, GPIO_DIR_MODE_OUT);
            GPIO_setPadConfig(pin, GPIO_PIN_TYPE_OD);
            break;
        case OUTPUT_OD_PULLUP:
            GPIO_setDirectionMode(pin, GPIO_DIR_MODE_OUT);
            GPIO_setPadConfig(pin, GPIO_PIN_TYPE_OD|GPIO_PIN_TYPE_PULLUP);
            break;
    }
    GPIO_setMasterCore(pin, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(pin, GPIO_QUAL_SYNC);
    EDIS;
}
/*******************************************************************************
 *  其他外设剥离独立初始化
 ******************************************************************************/


/*******************************************************************************
 * ADC部分
 ******************************************************************************/

/**
 * @brief 使能全部三组ADC，均配置为：
 *          1.外置电压基准，1.65v;
 *          2.100M二分频成50M ADCCLK;
 *          3.优先级采用轮转模式;
 *          4.ADC中断发生在电压转换完成时;
 *          5.提前中断产生的时间为0.
 */
void ADC_AllModulesInit()
{
    EALLOW;
    //myADC0 initialization

	// ADC Initialization: Write ADC configurations and power up the ADC
	// Configures the ADC module's offset trim
	ADC_setOffsetTrimAll(ADC_REFERENCE_INTERNAL,ADC_REFERENCE_3_3V);

	// Configures the analog-to-digital converter module prescaler.
	ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);
	// Sets the timing of the end-of-conversion pulse
	ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
	// Sets the timing of early interrupt generation.
	ADC_setInterruptCycleOffset(ADCA_BASE, 0U);
	// Powers up the analog-to-digital converter core.
	ADC_enableConverter(ADCA_BASE);
	// Delay for 1ms to allow ADC time to power up
	DEVICE_DELAY_US(5000);

    // Configures the analog-to-digital converter module prescaler.
	ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_2_0);
	// Sets the timing of the end-of-conversion pulse
	ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
	// Sets the timing of early interrupt generation.
	ADC_setInterruptCycleOffset(ADCB_BASE, 0U);
	// Powers up the analog-to-digital converter core.
	ADC_enableConverter(ADCB_BASE);
	// Delay for 1ms to allow ADC time to power up
	DEVICE_DELAY_US(5000);

    // Configures the analog-to-digital converter module prescaler.
	ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_2_0);
	// Sets the timing of the end-of-conversion pulse
	ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);
	// Sets the timing of early interrupt generation.
	ADC_setInterruptCycleOffset(ADCC_BASE, 0U);
	// Powers up the analog-to-digital converter core.
	ADC_enableConverter(ADCC_BASE);
	// Delay for 1ms to allow ADC time to power up
	DEVICE_DELAY_US(5000);

    // SOC Configuration: Setup ADC EPWM channel and trigger settings
	// Disables SOC burst mode.
	ADC_disableBurstMode(ADCA_BASE);
    ADC_disableBurstMode(ADCB_BASE);
    ADC_disableBurstMode(ADCC_BASE);
	// Sets the priority mode of the SOCs.
	ADC_setSOCPriority(ADCA_BASE, ADC_PRI_ALL_ROUND_ROBIN);
    ADC_setSOCPriority(ADCB_BASE, ADC_PRI_ALL_ROUND_ROBIN);
    ADC_setSOCPriority(ADCC_BASE, ADC_PRI_ALL_ROUND_ROBIN);

    EDIS;
}

/**
 * @brief 配置一个ADC通道到其对应的SOC，例如A7配置到ADCA的SOC7。
 *          软件触发采样，80ns采样窗口
 *          
 * @param base ADC模块的基地址
 * @param socNumber 该模块的第几个通道。
 */
void ADC_SOCChannelLink(uint32_t base,ADC_Channel socNumber)
{
    EALLOW;
    //链接ADC通道与SOC
    ADC_setupSOC(base, (ADC_SOCNumber)socNumber, ADC_TRIGGER_SW_ONLY, socNumber, 10U);
	ADC_setInterruptSOCTrigger(base, (ADC_SOCNumber)socNumber, ADC_INT_SOC_TRIGGER_NONE);
    EDIS;  
}

/**
 * @brief 将某一组的第N路ADC通道配置为中断源，不启用PIE中断，仅启用ADC_INT中断
 * @param base ADC组的基地址
 * @param socNumber ADC通道。
 */
void ADC_SOCInterruptLink(uint32_t base,ADC_Channel socNumber)
{
    EALLOW;
    //配置ADC的中断源，默认启用该组ADC的中断1,通道序号就是SOC的序号
    ADC_setInterruptSource(base, ADC_INT_NUMBER1, (ADC_SOCNumber)socNumber);
    ADC_enableInterrupt(base, ADC_INT_NUMBER1);
	ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);
    ADC_disableContinuousMode(base, ADC_INT_NUMBER1);//禁用该组的连续中断
    EDIS;
} 
/**
 * @brief 使用阻塞的方式获取某通道的ADC采样值
 * 
 * @param base ADC组的基地址
 * @param socNumber ADC通道
 * @return uint16_t 
 */
uint16_t analogRead(uint32_t base,ADC_Channel socNumber)
{
    // EALLOW;
    // //配置ADC的中断源，默认启用该组ADC的中断1,通道序号就是SOC的序号
    // ADC_setInterruptSource(base, ADC_INT_NUMBER1, (ADC_SOCNumber)socNumber);
	// ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);
    // ADC_enableInterrupt(base, ADC_INT_NUMBER1);
    // EDIS;
    ADC_SOCInterruptLink(base,socNumber);

    ADC_forceSOC(base, (ADC_SOCNumber)socNumber); //启用软件强制转换
    while(ADC_getInterruptStatus(base, ADC_INT_NUMBER1) == false)
    {
    }//等待触发INT1的EOC中断
    ADC_clearInterruptStatus(base, ADC_INT_NUMBER1);//清除中断标志
    return ADC_readResult(((base-ADCA_BASE)/0x80)*0x20+0xB00, (ADC_SOCNumber)socNumber);
}

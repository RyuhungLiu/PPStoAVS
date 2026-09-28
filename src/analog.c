#include "analog.h"
#include "board.h"

/* 参考手册 §17.3.2 ISP_CTLR */
#define ISP2_EN             (1u << 16)
#define ISP2_GAIN_55        (3u << 17)
#define ISP2_VFBIAS_1V6     (1u << 19)
#define ISP2_SEL_IO         (1u << 21)

static uint16_t isp_zero_code = 0;

static uint16_t adc_read(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_59Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
        ;
    return ADC_GetConversionValue(ADC1);
}

static uint16_t adc_read_avg(uint8_t channel, uint8_t n)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        sum += adc_read(channel);
    }
    return sum / n;
}

void analog_init(void)
{
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_ADC1, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_OPCM, ENABLE);
    RCC_ADCCLKConfig(RCC_HB_Div8);   /* 48MHz / 8 = 6MHz */

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin = VBUS_ADC_PIN | ISP_GPIO_PINS;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    /* OPA4/ISP2：差分（ISN2=PA11），偏置 1.6V，增益 55，输出接 ADC_IN10 */
    OPA->ISP_CTLR = (OPA->ISP_CTLR & 0x0000FFFF) | ISP2_EN | ISP2_GAIN_55 | ISP2_VFBIAS_1V6 | ISP2_SEL_IO;

    ADC_InitTypeDef adc = {0};
    ADC_StructInit(&adc);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
        ;
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
        ;
}

void analog_calibrate_current_zero(void)
{
    isp_zero_code = adc_read_avg(ISP_ADC_CHANNEL, 16);
}

uint16_t analog_vbus_mv(void)
{
    uint32_t code = adc_read_avg(VBUS_ADC_CHANNEL, 4);
    return code * ADC_VREF_MV * VBUS_DIVIDER_NUM / 4096;
}

/* 单次采样：正好采在 BMC 报文上时读数低于 Rp 电平 */
uint16_t analog_fe_cc_mv(void)
{
    return (uint32_t)adc_read(FE_CC_ADC_CHANNEL) * ADC_VREF_MV / 4096;
}

/*
 * OPA4 输出 = 1.6V −（ISP − ISN）× 55。原假设电流越大输出越低，实测读数恒为 0（R 补偿无效），
 * 说明 R4 上的压降方向相反。电流只会从充电器流向设备，取与零点的绝对差，与极性无关
 */
uint16_t analog_current_ma(void)
{
    uint16_t code = adc_read_avg(ISP_ADC_CHANNEL, 4);
    uint16_t diff = code >= isp_zero_code ? code - isp_zero_code : isp_zero_code - code;
    uint32_t diff_uv = (uint32_t)diff * ADC_VREF_MV * 1000 / 4096;
    return diff_uv / (ISP_RSENSE_MOHM * ISP_GAIN);
}

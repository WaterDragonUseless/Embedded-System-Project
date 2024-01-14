#include "stm32f10x.h" // Device header
#include "AD.h"

// Initialize the Analog to Digital Converter
void AD_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // Enable ADC clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // Enable GPIOB clock

    RCC_ADCCLKConfig(RCC_PCLK2_Div6); // Set ADC clock division factor to 6

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Set GPIO mode to analog input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // Select pins 0 and 1
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // Set GPIO speed
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // Set ADC to independent mode
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // Right alignment of data
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // Software triggering
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Disable continuous conversion
    ADC_InitStructure.ADC_NbrOfChannel = 1; // Number of channels to use
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // Disable scan mode
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE); // Enable ADC

    // Calibration process for ADC
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // Start ADC conversion
}

// Get ADC value in default mode
uint16_t AD_GetValue_D(void) {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

// Get ADC value in continuous mode
uint16_t AD_GetValue_E(void) {
    return ADC_GetConversionValue(ADC1);
}

// Get ADC value from a specific channel
uint16_t AD_GetValue(uint8_t ADC_Channel) {
    ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}
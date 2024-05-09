#include "main.h"
#include "adc.h"
#include "tim.h"
#include "fsm.h"
#include "ntc.h"
#include "stdbool.h"


extern volatile uint16_t ntc_value;
extern volatile uint8_t ntc_temp;
volatile bool ADC_conv_flag = 0;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    
  ntc_temp = get_ntc_temperature(ntc_value);
  HAL_GPIO_TogglePin(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin);
  ADC_conv_flag = 1;
  
}
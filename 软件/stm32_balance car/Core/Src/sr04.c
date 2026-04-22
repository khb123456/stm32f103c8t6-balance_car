#include "sr04.h"
uint16_t count;
float distance;

void HAL_Delay_us(uint32_t us)
{
	uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000) * us;
    uint32_t start = SysTick->VAL;
    while((start - SysTick->VAL) < ticks);
}

 void SR04_Trigger(void)
 {
	 TRIG_H;
	 HAL_Delay_us(15);
	 TRIG_L;
 }
 
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	 if(HAL_GPIO_ReadPin(ECHO_GPIO_Port,ECHO_Pin)==GPIO_PIN_SET)
	 {
		 __HAL_TIM_SetCounter(&htim3,0);
		 HAL_TIM_Base_Start(&htim3);
	 }
	 else
	 {
		 HAL_TIM_Base_Stop(&htim3);
		 count=__HAL_TIM_GetCounter(&htim3);
		 distance=count*0.017/100.0;
	 }
 }
 
 
 
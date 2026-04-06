#ifndef __SR04_H
#define __SR04_H

#include "main.h"
#include "tim.h"

#define TRIG_H  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET)
#define TRIG_L  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET)
void HAL_Delay_us(uint32_t us);
void SR04_Trigger(void);

#endif



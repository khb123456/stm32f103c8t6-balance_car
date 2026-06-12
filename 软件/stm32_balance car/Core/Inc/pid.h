#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float target;
	float actual;
	float error;
	float error_last;
	float integral;
	float integral_max;
	float dif;
	float out;
	float out_max;
}PID_TypeDef;
void PID_Calculate(PID_TypeDef *pid);
void PID_Init(PID_TypeDef *pid);
#endif

#include "pid.h"
#include "math.h"
#include "encoder.h"
#include "tim.h"
#include "task.h"
#include "mpu6050.h"

/* PID 采样周期（秒），与定时器触发控制周期保持一致 */
#ifndef PID_DT
#define PID_DT 0.005f
#endif

void PID_Init(PID_TypeDef *pid)
{
	pid->target=0;
	pid->actual=0;
	pid->error_last=0;
	pid->error=0;
	pid->integral=0;
}
void PID_Calculate(PID_TypeDef *pid)
{
	pid->error = pid->target - pid->actual;
	float Pout = pid->Kp * pid->error;
	if(pid->Ki!=0)
	{
	pid->integral += pid->error ;
	}
	else
	{
		pid->integral=0;
	}
//	if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
//	if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
	float Iout = pid->Ki * pid->integral;

	float Dout = pid->Kd * (pid->error - pid->error_last);
	pid->error_last = pid->error;

	pid->out = Pout + Iout + Dout;
	if(pid->out>pid->out_max) pid->out=pid->out_max;
	if(pid->out<-pid->out_max) pid->out=-pid->out_max;
}

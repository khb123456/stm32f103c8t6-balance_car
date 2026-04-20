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
	/* 给积分上限一个合理默认，避免默认 0 导致积分始终不被限制 */
	if (pid->integral_max == 0) pid->integral_max = pid->out_max;
}
void PID_Calculate(PID_TypeDef *pid)
{
	pid->error = pid->target - pid->actual;
	float Pout = pid->Kp * pid->error;

	/* 积分先累加并限幅（按时间尺度积分），再计算 I 输出 */
	pid->integral += pid->error * PID_DT;
	if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
	if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
	float Iout = pid->Ki * pid->integral;

	/* 计算微分：误差变化率（按照时间尺度），再更新上次误差 */
	float derivative = (pid->error - pid->error_last) / PID_DT;
	float Dout = pid->Kd * derivative;
	pid->error_last = pid->error;

	pid->out = Pout + Iout + Dout;
	if(pid->out>pid->out_max) pid->out=pid->out_max;
	if(pid->out<-pid->out_max) pid->out=-pid->out_max;
}

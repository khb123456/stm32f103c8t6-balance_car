#include "pid.h"
#include "math.h"
#include "encoder.h"
#include "tim.h"
#include "task.h"
#include "mpu6050.h"

//float angle_kp=120.0f,angle_kd=-0.75;
//float speed_kp=0.0,speed_ki=0.0;
//float turn_kp=0.0,turn_kd=0.0;
int speed_left,speed_right;
int pwm_left,pwm_right;
int ave_speed,dif_speed,ave_pwm,dif_pwm;
int target_speed;

int GetCurrentSpeed()
{
	 speed_left=Read_Speed(&htim2);
	 speed_right=-Read_Speed(&htim4);
	ave_speed=(speed_left+speed_right)/2.0f;
	return ave_speed;
}

int GetDifSpeed()
{
	 speed_left=Read_Speed(&htim2);
	 speed_right=-Read_Speed(&htim4);
	 dif_speed=speed_left-speed_right;
	 return dif_speed;
}

void PID_Calculate(PID_TypeDef *pid)
{
	pid->error=pid->target-pid->actual;
	float Pout=pid->Kp*pid->error;
	
	pid->integral+=pid->error;
	float Iout=pid->Ki*pid->integral;
	if(pid->integral>pid->integral_max) pid->integral=pid->integral_max;
	if(pid->integral<-pid->integral_max) pid->integral=-pid->integral_max;
	
	float Dout=pid->Kd*pid->dif;
	pid->out=Pout+Iout+Dout;
	if(pid->out>pid->out_max) pid->out=pid->out_max;
	if(pid->out<-pid->out_max) pid->out=-pid->out_max;
}

////角度环PD控制器
//int Vertical(float target,float actual,float Groy)
//{
//	int temp;
//	temp=angle_kp*(target-actual)+angle_kd*Groy;
//	return temp;
//}

////速度环PI控制器
//int Speed_Loop(int target,int actual)
//{
//	static int error,integral;
//	error=target-actual;
//	integral+=error;
//	if(integral>20000) integral=20000;
//	if(integral<-20000) integral=-20000;
//	
//	return speed_kp*error+speed_ki*integral;
//	
//}

////转向环PD控制器
//int Turn_Loop(int target,int dif_speed)
//{
//	return turn_kp*target+turn_kd*dif_speed;
//}

//void Balance(void)
//{
//	PERIODIC(10);
//	int actual_speed=GetCurrentSpeed();
//	int dif_speed=GetDifSpeed();
//	Speed_Loop(target_speed,ave_speed);
//	
//}
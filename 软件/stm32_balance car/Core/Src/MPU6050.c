#include "mpu6050.h"
#include "task.h"
#include "math.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

KalmanFilter kf_pitch,kf_roll;
float a=0.95238; //互补滤波系数
static AttitudeMode currentMode=MODE_KALMAN;
void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	IIC_Start();
	IIC_SendByte(MPU6050_ADDRESS);
	IIC_ReceiveAck();
	IIC_SendByte(RegAddress);
	IIC_ReceiveAck();
	IIC_SendByte(Data);
	IIC_ReceiveAck();
	IIC_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	IIC_Start();
	IIC_SendByte(MPU6050_ADDRESS);
	IIC_ReceiveAck();
	IIC_SendByte(RegAddress);
	IIC_ReceiveAck();
	
	IIC_Start();
	IIC_SendByte(MPU6050_ADDRESS|0x01);
	IIC_ReceiveAck();
	Data=IIC_ReceiveByte();
	IIC_SendAck(1);
	IIC_Stop();
	
	return Data;
	
}
uint8_t MPU6050_ReadRegs(uint8_t RegAddress,uint8_t *DataArray,uint8_t Count)
{
	uint8_t i;
	IIC_Start();
	IIC_SendByte(MPU6050_ADDRESS);
	IIC_ReceiveAck();
	IIC_SendByte(RegAddress);
	IIC_ReceiveAck();
	
	IIC_Start();
	IIC_SendByte(MPU6050_ADDRESS|0x01);
	IIC_ReceiveAck();
	
	for(i=0;i<Count;i++)
	{
		DataArray[i]=IIC_ReceiveByte();
		if(i<Count-1)
		{
			IIC_SendAck(0);
		}
		else
		{
			IIC_SendAck(1);
		}
	}
	IIC_Stop();
}
void MPU6050_Init(void)
{
	IIC_Init();
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);		
	HAL_Delay(100);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);		
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);		
	MPU6050_WriteReg(MPU6050_CONFIG, 0x00);			
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//将陀螺仪的量程设置为+-2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);	//将加速度计的量程设置为+-2g
}

//MPU6050互补滤波的进程函数
void MPU6050_Proc_Complementary(void)
{
//	static uint32_t  nxt=0;
//	if(HAL_GetTick()<nxt)
//		return;
	PERIODIC(5);
	//互补滤波
	MPU6050_UpDate();
	//通过陀螺仪的测量结果计算欧拉角
	float yaw_g=yaw+gz*0.005;
	float pitch_g=pitch+gx*0.005;
	float roll_g=roll-gy*0.005;
	
	//通过加速度计计算欧拉角
	float pitch_a=atan2(ay,az)/3.1415727f*180.0f;
	float roll_a=atan2(ax,az)/3.1415727f*180.0f;
	
	//使用互补滤波器对陀螺仪和加速度计的计算结果进行融合
	
	yaw=yaw_g;
	pitch=a*pitch_g+(1-a)*pitch_a;
	roll=a*roll_g+(1-a)*roll_a;
	
//	nxt+=5;
	
}

//初始化卡尔曼滤波器
void Kalman_Init(KalmanFilter *kf)
{
	kf->Q_angle=0.001f;  //过程噪声（角度）
	kf->Q_bias=0.003f;   //过程噪声（漂移）
	kf->R_measure=0.03f;  //测量噪声（加速度计）
	kf->angle=0.0f;		//初始角度
	kf->bias=0.0f;		//初始漂移
	kf->P[0][0]=0.0f;	//初始化误差协方差矩阵
	kf->P[0][1]=0.0f;
	kf->P[1][0]=0.0f;
	kf->P[1][1]=0.0f;
	
}

//卡尔曼滤波器预测步骤
void Kalman_Predict(KalmanFilter *kf,float gyro_rate,float dt)
{
	//预测角度和漂移
	kf->angle+=(gyro_rate-kf->bias)*dt;
	
	//更新协方差误差矩阵
	kf->P[0][0]+=dt*(dt*kf->P[1][1]-kf->P[0][1]-kf->P[1][0]+kf->Q_angle);
	kf->P[0][1]-=dt*kf->P[1][1];
	kf->P[1][0]-=dt*kf->P[1][1];
	kf->P[1][1]+=kf->Q_bias*dt;
}

//卡尔曼滤波器更新步骤
void Kalman_Update(KalmanFilter *kf,float measured_angle)
{
	//计算卡尔曼增益
	float S=kf->P[0][0]+kf->R_measure;
	float K[2];
	K[0]=kf->P[0][0]/S;
	K[1]=kf->P[1][0]/S;
	
	//更新角度估计
	float y=measured_angle-kf->angle;
	kf->angle+=K[0]*y;
	kf->bias+=K[1]*y;
	
	//更新误差协方差矩阵
	float P00_temp=kf->P[0][0];
	float P01_temp=kf->P[0][1];
	float P10_temp=kf->P[1][0];
	float P11_temp=kf->P[1][1];
	
	kf->P[0][0]=(1-K[0])*P00_temp;
	kf->P[0][1]=(1-K[0])*P01_temp;
	kf->P[1][0]=P10_temp-K[1]*P00_temp;
	kf->P[1][1]=P11_temp-K[1]*P01_temp;
}

//卡尔曼滤波进程函数
void MPU6050_Proc_Kalman(void)
{
	PERIODIC(5);
	MPU6050_UpDate();  //更新传感器数据
	//计算加速度度计角度（用于卡尔曼滤波更新）
	float pitch_a=atan2(ay,az)/3.1415727f*180.0f;
	float roll_a=atan2(ax,az)/3.1415727f*180.0f;
	
	//陀螺仪积分角度（用于卡尔曼滤波更新）
	static float yaw_g=0,pitch_g=0,roll_g=0;
	yaw_g=yaw+gz*0.005;
	pitch_g=pitch+gx*0.005;
	roll_g=roll-gy*0.005;
	
	//初始化卡尔曼滤波器（首次运行时）
	static uint8_t kalman_init=0;
	if(!kalman_init)
	{
		Kalman_Init(&kf_pitch);
		Kalman_Init(&kf_roll);
		kalman_init=1;
	}
	
	//执行卡尔曼滤波（Pitch和Roll轴）
	Kalman_Predict(&kf_pitch,gx,0.005f);  //预测Pitch
	Kalman_Update(&kf_pitch,pitch_a);     //用加速度计更新Pitch
	
	Kalman_Predict(&kf_roll,-gy,0.005f);  //预测Roll
	Kalman_Update(&kf_roll,roll_a);     //用加速度计更新Roll
	
	//更新全局姿态角
	yaw=yaw_g;   //Yaw轴无加速度计观测，保持陀螺仪积分
	pitch=kf_pitch.angle;  //Pitch轴使用卡尔曼滤波结果
	roll=kf_roll.angle;		//Roll轴使用卡尔曼滤波结果
	
}

int MPU6050_DMP_Init(void)
{
	struct int_param_s int_param;
	int_param.cb=NULL;   //如果不使用中断，可以设为NULL
	if(mpu_init(&int_param)) return -1;    //初始化MPU（复位、配置时钟等）
	if(dmp_load_motion_driver_firmware()) return -1; //加载DMP固件
	if(dmp_set_fifo_rate(5)) return -1;   //设置DMP输出速率（例如50Hz）
	//设置DMP功能：六轴低功耗四元数、发送原始加速度、发送校准陀螺仪
	unsigned short dmp_features=DMP_FEATURE_6X_LP_QUAT|		
					DMP_FEATURE_SEND_RAW_ACCEL|
					DMP_FEATURE_SEND_CAL_GYRO;
	if(dmp_enable_feature(dmp_features)) return -1;
	//设置传感器方向
	if(dmp_set_orientation(0)) return -1;
	//使能DMP
	if(mpu_set_dmp_state(1)) return -1;
	return 0;
}

static void MPU6050_Update_DMP(void){
	short gyro[3],accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensor;
	unsigned char more;
	//从FIFO读取DMP数据
	if(dmp_read_fifo(gyro,accel,quat,&timestamp,&sensor,&more)==0)
	{
		if(sensor&INV_WXYZ_QUAT){
			//四元数转换到欧拉角（单位：度）
			float q0=quat[0]/1073741824.0f;
			float q1=quat[1]/1073741824.0f;
			float q2=quat[2]/1073741824.0f;
			float q3=quat[3]/1073741824.0f;
			
			//计算欧拉角
			roll=atan2f(2.0f*(q0*q1+q2*q3),1.0f-2.0f*(q1*q1+q2*q2))*57.29578f;
			pitch=asinf(2.0f*(q0*q2-q3*q1)*57.29578);
			yaw=atan2f(2.0f*(q0*q3+q1*q2),1.0f-2.0f*(q2*q2+q3*q3))*57.29578f;
		}
	}
}

void MPU6050_Mode_Update(void)
{
	switch(currentMode)
	{
		case MODE_COMPLEMENTARY:
			MPU6050_Proc_Complementary();
		    break;
		case MODE_KALMAN:
			MPU6050_Proc_Kalman();
			break;
		case MODE_DMP:
			MPU6050_Update_DMP();
			break;
	}
}

void MPU6050_SetMode(AttitudeMode mode)
{
	if(mode==currentMode) return;
	
	//根据模式重新初始化传感器或者DMP
	if(mode==MODE_DMP){
		if(MPU6050_DMP_Init()!=0){
			return;
		}
		currentMode=MODE_DMP;
	}
	else
	{
		if(currentMode==MODE_DMP)
		{
			mpu_set_dmp_state(0);
		}
		MPU6050_Init();
		currentMode=mode;
	}
}
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		
}


void MPU6050_UpDate(void)
{
	uint8_t Data[14];
	MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H,Data,14);
	int16_t ax_raw=(Data[0]<<8)|Data[1];
	int16_t ay_raw=(Data[2]<<8)|Data[3];
	int16_t az_raw=(Data[4]<<8)|Data[5];
	
	int16_t gx_raw=(Data[8]<<8)|Data[9];
	int16_t gy_raw=(Data[10]<<8)|Data[11];
	int16_t gz_raw=(Data[12]<<8)|Data[13];
//	//单次读取寄存器，然后进行数据拼接
//	int16_t ax_raw=(int16_t)((MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H)<<8)|MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L));
//	int16_t ay_raw=(int16_t)((MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H)<<8)|MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L));
//	int16_t az_raw=(int16_t)((MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H)<<8)|MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L));
	
	ax=ax_raw*6.1035e-5f;
	ay=ay_raw*6.1035e-5f;
	az=az_raw*6.1035e-5f;
	

	
//	int16_t temperatue_raw=(int16_t)((MPU6050_ReadReg(MPU6050_TEMP_OUT_H)<<8)|MPU6050_ReadReg(MPU6050_TEMP_OUT_L));
////	temperature=temperatue_raw/340+36.53;		//MPU6050
//	temperature=temperatue_raw/333.87f+21.0f;  //MPU6500
//	
//	int16_t gx_raw=(int16_t)((MPU6050_ReadReg(MPU6050_GYRO_XOUT_H)<<8)|MPU6050_ReadReg(MPU6050_GYRO_XOUT_L));
//	int16_t gy_raw=(int16_t)((MPU6050_ReadReg(MPU6050_GYRO_YOUT_H)<<8)|MPU6050_ReadReg(MPU6050_GYRO_YOUT_L));
//	int16_t gz_raw=(int16_t)((MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H)<<8)|MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L));
	
	gx=gx_raw*6.1035e-2f;
	gy=gy_raw*6.1035e-2f;
	gz=gz_raw*6.1035e-2f;


}

void MPU6050_GetData(float *AccX, float *AccY, float *AccZ,
					float *GyroX, float *GyroY, float *GyroZ,float *Tem)
{
	*AccX=ax;	*AccY=ay;	*AccZ=az;
	*GyroX=gx;	*GyroY=gy;	*GyroZ=gz;
	*Tem=temperature;
	
}

float MPU6050_GetYaw(void)
{
	return yaw;
}
float MPU6050_GetPitch(void)
{
	return pitch;
}

float MPU6050_GetRoll(void)
{
	return roll;
}

float MPU6050_GetAccX(void)
{
	return ax;
}

float MPU6050_GetAccY(void)
{
	return ay;
}
float MPU6050_GetAccZ(void)
{
	return az;
}
float MPU6050_GetGyroX(void)
{
	return gx;
}
float MPU6050_GetGyroY(void)
{
	return gy;
}
float MPU6050_GetGyroZ(void)
{
	return gz;
}
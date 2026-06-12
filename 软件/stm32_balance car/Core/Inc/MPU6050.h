#ifndef __MPU6050_H
#define __MPU6050_H

#include "MyI2C.h"
#include "task.h"
#include "math.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
/*
 * MPU6050寄存器地址定义
 * 适用场景：STM32/51/Arduino等单片机与MPU6050通信时使用
 */

// MPU6050的I2C设备地址（AD0引脚接GND时为0x68，左移一位后为0xD0；AD0接VCC时为0x69，左移一位后为0xD2）
#define MPU6050_ADDRESS         0xD0  

// 采样率分频寄存器地址：设置采样率 = 陀螺仪输出率 / (1 + SMPLRT_DIV)
// 陀螺仪输出率默认是1kHz（当DLPF使能时）或8kHz（当DLPF禁用时）
#define	MPU6050_SMPLRT_DIV		0x19  

// 配置寄存器地址：设置数字低通滤波器(DLPF)的参数，影响加速度计和陀螺仪的带宽
#define	MPU6050_CONFIG			0x1A  

// 陀螺仪配置寄存器地址：设置陀螺仪的满量程范围（±250/±500/±1000/±2000 °/s）
#define	MPU6050_GYRO_CONFIG		0x1B  

// 加速度计配置寄存器地址：设置加速度计的满量程范围（±2/±4/±8/±16 g）
#define	MPU6050_ACCEL_CONFIG	0x1C  

// 加速度计X轴输出数据寄存器（高8位）：存储X轴加速度数据的高字节
#define	MPU6050_ACCEL_XOUT_H	0x3B  
// 加速度计X轴输出数据寄存器（低8位）：存储X轴加速度数据的低字节
#define	MPU6050_ACCEL_XOUT_L	0x3C  
// 加速度计Y轴输出数据寄存器（高8位）：存储Y轴加速度数据的高字节
#define	MPU6050_ACCEL_YOUT_H	0x3D  
// 加速度计Y轴输出数据寄存器（低8位）：存储Y轴加速度数据的低字节
#define	MPU6050_ACCEL_YOUT_L	0x3E  
// 加速度计Z轴输出数据寄存器（高8位）：存储Z轴加速度数据的高字节
#define	MPU6050_ACCEL_ZOUT_H	0x3F  
// 加速度计Z轴输出数据寄存器（低8位）：存储Z轴加速度数据的低字节
#define	MPU6050_ACCEL_ZOUT_L	0x40  

// 温度传感器输出数据寄存器（高8位）：存储温度数据的高字节
#define	MPU6050_TEMP_OUT_H		0x41  
// 温度传感器输出数据寄存器（低8位）：存储温度数据的低字节
#define	MPU6050_TEMP_OUT_L		0x42  

// 陀螺仪X轴输出数据寄存器（高8位）：存储X轴角速度数据的高字节
#define	MPU6050_GYRO_XOUT_H		0x43  
// 陀螺仪X轴输出数据寄存器（低8位）：存储X轴角速度数据的低字节
#define	MPU6050_GYRO_XOUT_L		0x44  
// 陀螺仪Y轴输出数据寄存器（高8位）：存储Y轴角速度数据的高字节
#define	MPU6050_GYRO_YOUT_H		0x45  
// 陀螺仪Y轴输出数据寄存器（低8位）：存储Y轴角速度数据的低字节
#define	MPU6050_GYRO_YOUT_L		0x46  
// 陀螺仪Z轴输出数据寄存器（高8位）：存储Z轴角速度数据的高字节
#define	MPU6050_GYRO_ZOUT_H		0x47  
// 陀螺仪Z轴输出数据寄存器（低8位）：存储Z轴角速度数据的低字节
#define	MPU6050_GYRO_ZOUT_L		0x48  

// 电源管理寄存器1地址：设置设备的睡眠模式、时钟源选择、复位等功能
// 关键：上电后需要将该寄存器的BIT6（睡眠位）置0，才能唤醒MPU6050
#define	MPU6050_PWR_MGMT_1		0x6B  

// 电源管理寄存器2地址：设置加速度计和陀螺仪各轴的待机模式
#define	MPU6050_PWR_MGMT_2		0x6C  

// WHO_AM_I寄存器地址：读取该寄存器可获取设备ID（默认值0x68），用于验证通信是否正常
#define	MPU6050_WHO_AM_I		0x75  

static float ax,ay,az;  //加速度计的结果，单位g
static float temperature;  //温度计的结果，单位摄氏度
static float gx,gy,gz;   //单位°/s
static float yaw,pitch,roll;  //欧拉角，单位： °


//卡尔曼滤波器参数(针对Pitch轴和Roll轴)
typedef struct{
	float Q_angle;  //过程噪声协方差（陀螺仪噪声）
	float Q_bias;   //过程噪声协方差（陀螺仪漂移）
	float R_measure;   //测量噪声协方差（加速度计噪声）
	float angle;		//当前估计角度
	float bias;			//陀螺仪漂移估计
	float P[2][2];      //协方差矩阵
}KalmanFilter;

typedef enum{
	MODE_COMPLEMENTARY,
	MODE_KALMAN,
	MODE_DMP
}AttitudeMode;

void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
uint8_t MPU6050_ReadRegs(uint8_t RegAddress,uint8_t *DataArray,uint8_t Count);
void MPU6050_Init(void);

void MPU6050_Proc(void);
uint8_t MPU6050_GetID(void);

void MPU6050_UpDate(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
					int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_Proc_Complementary(void);
void Kalman_Init(KalmanFilter *kf);
void Kalman_Predict(KalmanFilter *kf,float gyro_rate,float dt);
void Kalman_Update(KalmanFilter *kf,float measured_angle);
void MPU6050_Proc_Kalman(void);

void MPU6050_SetMode(AttitudeMode mode);
void MPU6050_Mode_Update(void);

float MPU6050_GetYaw(void);
float MPU6050_GetPitch(void);
float MPU6050_GetRoll(void);
float MPU6050_GetAccX(void);
float MPU6050_GetAccY(void);
float MPU6050_GetAccZ(void);
float MPU6050_GetGyroX(void);
float MPU6050_GetGyroY(void);
float MPU6050_GetGyroZ(void);

#endif

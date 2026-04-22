/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "MPU6050.h"
#include "stdio.h"
#include "sr04.h"
#include "motor.h"
#include "encoder.h"
#include "string.h"
#include "task.h"
#include "pid.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t AX, AY, AZ, GX, GY, GZ,Tem;



int Encoder_Left,Encoder_Right;
extern float distance;
uint8_t rx_buf[2];
char uart_buf[128];
uint8_t Rx_Flag;
char RxPacket[100];
int16_t LeftPWM, RightPWM;			//左PWM，右PWM
int16_t Ave_PWM, Dif_PWM;				//平均PWM，差分PWM

float LeftSpeed, RightSpeed;		//左速度，右速度
float AveSpeed, DifSpeed;			//平均速度，差分速度
uint8_t Run_Flag=1;
volatile uint8_t control_flag = 0;    // 新增：由定时器置位，主循环执行 Control
PID_TypeDef angle_pid={
	.Kp=5.0f,
	.Ki=0.0f,
	.Kd=0.1f,
	.out_max=900.0f
};
PID_TypeDef speed_pid={
	.Kp=0.0f,
	.Ki=0.0f,
	.Kd=0.0f,
	.out_max=10.0f
};
PID_TypeDef turn_pid=
{
	.Kp=0.0f,
	.Ki=0.0f,
	.Kd=0.0f,
	.out_max=200.0f
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read(void);
int fputc(int c,FILE *stream);
static void OLED_Proc(void);
void Set_params(void);
void Control(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  IIC_Init();
  OLED_Init();  
  MPU6050_Init();
  OLED_Clear();
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart3,rx_buf,1);
  Load(0,0);
  MPU6050_SetMode(MODE_COMPLEMENTARY);
  HAL_TIM_Base_Start_IT(&htim3);
  PID_Init(&angle_pid);
  PID_Init(&speed_pid);
  PID_Init(&turn_pid);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (control_flag)
    {
      control_flag = 0;
      Control();
    }
	  Set_params();          // 非阻塞地更新 PID/目标参数

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Read(void)
{
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
	
	LeftSpeed=Encoder_Left/13.0f/20.0f/0.05f;
	RightSpeed=Encoder_Right/13.0f/20.0f/0.05f;
	
	AveSpeed=(LeftSpeed+RightSpeed)/2.0f;
	DifSpeed=LeftSpeed-RightSpeed;
	
}

int fputc(int c,FILE *stream)
{
	uint8_t ch[1]={c};
	HAL_UART_Transmit(&huart3,ch,1,0xFFFF);
	return c;
}

static void OLED_Proc(void)
{
	OLED_Clear();
	OLED_Printf(0,0,0,12,"Angle");
	OLED_Printf(0,1,0,12,"P:%05.2f",angle_pid.Kp);
	OLED_Printf(0,2,0,12,"I:%05.2f",angle_pid.Ki);
	OLED_Printf(0,3,0,12,"D:%05.2f",angle_pid.Kd);
	OLED_Printf(43,0,0,12,"Speed");
	OLED_Printf(43,1,0,12,"P:%05.2f",speed_pid.Kp);
	OLED_Printf(43,2,0,12,"I:%05.2f",speed_pid.Ki);
	OLED_Printf(43,3,0,12,"D:%05.2f",speed_pid.Kd);
	OLED_Printf(85,0,0,12,"Turn");
	OLED_Printf(85,1,0,12,"P:%05.2f",turn_pid.Kp);
	OLED_Printf(85,2,0,12,"I:%05.2f",turn_pid.Ki);
	OLED_Printf(85,3,0,12,"D:%05.2f",turn_pid.Kd);
	OLED_Printf(0,4,0,12,"Pitch:%05.2f",angle_pid.actual);
	OLED_Printf(0,5,0,12,"Speed:%05.2f",AveSpeed);
	SR04_Trigger();
	OLED_Printf(0,6,0,12,"Distance:%05.2f",distance);
}
void Set_params(void)
{
	if(Rx_Flag==1)
	{
		char *Tag=strtok(RxPacket,",");
		if(strcmp(Tag,"slider")==0)
		{
			char *Name=strtok(NULL,",");
			char *Value=strtok(NULL,",");
			
			if(strcmp(Name,"angle_kp")==0)
			{
				angle_pid.Kp=atof(Value);
			}
			else if(strcmp(Name,"angle_ki")==0)
			{
				angle_pid.Ki=atof(Value);
			}
			else if(strcmp(Name,"angle_kd")==0)
			{
				angle_pid.Kd=atof(Value);
			}
			else if(strcmp(Name,"speed_kp")==0)
			{
				speed_pid.Kp=atof(Value);
			}
			else if(strcmp(Name,"speed_ki")==0)
			{
				speed_pid.Ki=atof(Value);
			}
			else if(strcmp(Name,"speed_kd")==0)
			{
				speed_pid.Kd=atof(Value);
			}
			else if(strcmp(Name,"turn_kp")==0)
			{
				turn_pid.Kp=atof(Value);
			}
			else if(strcmp(Name,"turn_ki")==0)
			{
				turn_pid.Ki=atof(Value); 
			}
			else if(strcmp(Name,"turn_kd")==0)
			{
				turn_pid.Kd=atof(Value);
			}

		}
		else if (strcmp(Tag, "joystick") == 0)			//Tag为joystick，收到摇杆数据包
			{
				int8_t LH = atoi(strtok(NULL, ","));		//提取数据2，定义为摇杆值LH
				int8_t LV = atoi(strtok(NULL, ","));		//提取数据3，定义为摇杆值LV
				int8_t RH = atoi(strtok(NULL, ","));		//提取数据4，定义为摇杆值RH
				int8_t RV = atoi(strtok(NULL, ","));		//提取数据5，定义为摇杆值RV
				
				/*执行摇杆操作*/
				speed_pid.target = LV / 25.0;	//摇杆值LV缩放后，控制速度环目标值，前后行进控制
				turn_pid.target = RH / 2;				//摇杆值RH缩放后，控制差分PWM，左右转弯控制
			}
		
			Rx_Flag=0;

	}
//	printf("[plot,%f,%f]",angle_pid.target,angle_pid.actual);
}

void Control(void)
{
		MPU6050_Mode_Update();
		angle_pid.actual = MPU6050_GetPitch();
		 /* 超限保护 */
		if (angle_pid.actual > 50.0f || angle_pid.actual < -50.0f) 
		{
			Load(0,0);
			Run_Flag=0;
		}
		if(Run_Flag)
		{
		speed_pid.actual=AveSpeed;
		PID_Calculate(&speed_pid);
		angle_pid.target=speed_pid.out-0.2f;
			
		PID_Calculate(&angle_pid);
		Ave_PWM = -((int)angle_pid.out);
					
		turn_pid.actual=DifSpeed;
		PID_Calculate(&turn_pid);
		Dif_PWM=turn_pid.out;
			
		LeftPWM = Ave_PWM + Dif_PWM/2;
		RightPWM = Ave_PWM - Dif_PWM/2;

		/* 输出限幅 */
		if (LeftPWM > 100) LeftPWM = 100;
		if (LeftPWM < -100) LeftPWM = -100;
		if (RightPWM > 100) RightPWM = 100;
		if (RightPWM < -100) RightPWM = -100;

		Load(LeftPWM, RightPWM);
	    }
		else
		{
		Load(0,0);
		}
	
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t RxState=0;
	static uint8_t pRxPacket=0;

	if(huart->Instance==USART3)
	{
		if(RxState==0)
		{
			if(rx_buf[0]=='['&&Rx_Flag==0)
			{
				RxState=1;
				pRxPacket=0;
			}
		}
		
		else if(RxState==1)
		{
			if(rx_buf[0]==']')
			{
				RxState=0;
				RxPacket[pRxPacket]='\0';
				Rx_Flag=1;
			}
			else
		{
			RxPacket[pRxPacket]=rx_buf[0];
			pRxPacket++;
		}
		}

		HAL_UART_Receive_IT(&huart3,rx_buf,1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t Count0,Count1;
	if (htim->Instance == TIM3) 
	 {
		Count0++;
		 if(Count0>=10)
		 {
			Count0=0;
			control_flag=1;
		 }
		 Count1++;
		 if(Count1>=50)
		 {
			 Count1=0;
			 Read();
		 }
	 
	 }

	
  
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

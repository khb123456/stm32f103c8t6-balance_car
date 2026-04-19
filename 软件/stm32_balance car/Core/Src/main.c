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
uint8_t ID;								
float AX, AY, AZ, GX, GY, GZ,Tem;

uint32_t sys_ticks=0;
int Encoder_Left,Encoder_Right;
uint8_t display_buf[20];
extern float distance;
extern uint8_t rx_buf[2];
char uart_buf[128];
uint8_t Rx_Flag;
char RxPacket[100];
int Ave_PWM;
uint8_t Run_Flag=1;
PID_TypeDef angle_pid={
	.Kp=0.0f,
	.Ki=0.0f,
	.Kd=0.0f,
	.out_max=800.0f
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read(void);
int fputc(int c,FILE *stream);
static void USART3_Proc(void);
void Control();
void Set_params();
void DWT_Init(void)
{
    DEM_CR |= DEM_CR_TRCENA;   
    DWT_CYCCNT = 0;           
    DWT_CR |= DWT_CR_CYCCNTENA;
}

// 获取当前CPU周期
uint32_t DWT_GetCycle(void)
{
    return DWT_CYCCNT;
}

// 转微秒
uint32_t DWT_Cycle2Us(uint32_t cycle)
{
    return cycle / (SystemCoreClock / 1000000);
}
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
  DWT_Init();
  MPU6050_SetMode(MODE_KALMAN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	SR04_Trigger();
//	sprintf((char *)display_buf,"distance:%.1f cm ",distance);
//	OLED_ShowString(0,0,(char *)display_buf,16,1);
//	  Read();
//	  printf("Encoder_L: %d \n  Encoder_R:%d \r\n",Encoder_Left,Encoder_Right);
//	  sprintf((char *)display_buf,"Encoder_L:%d\r\n",Encoder_Left);
//	  OLED_ShowString(0,0,(char*)display_buf,16,0);
//	  sprintf((char *)display_buf,"Encoder_R:%d\r\n",Encoder_Right);
//	  OLED_ShowString(0,2,(char*)display_buf,16,0);
//	 HAL_UART_Transmit(&huart3,display_buf,sizeof(display_buf),1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	Set_params();
	Control();



  }
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
	if(uwTick-sys_ticks<10)
		return;
	sys_ticks=uwTick;
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
}

int fputc(int c,FILE *stream)
{
	uint8_t ch[1]={c};
	HAL_UART_Transmit(&huart3,ch,1,0xFFFF);
	return c;
}

static void USART3_Proc(void)
{
	PERIODIC(1);
	uint32_t star_time,end_time,time;
	star_time=HAL_GetTick();
	MPU6050_Mode_Update();
	yaw=MPU6050_GetYaw();
	pitch=MPU6050_GetPitch();
	roll=MPU6050_GetRoll();
	end_time=HAL_GetTick();
	time=end_time-star_time;
	printf("%f,%f,%f\r\ntime:%d\n",yaw,pitch,roll,time);
	
}
void Set_params()
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

		}
		
			Rx_Flag=0;

	}
//	printf("[plot,%f,%f]",angle_pid.target,angle_pid.actual);
	
}
void Control()
{
	PERIODIC(10);
	MPU6050_Mode_Update();
	angle_pid.target=-3.0f;
	angle_pid.actual=MPU6050_GetPitch();
	angle_pid.dif=MPU6050_GetGyroX();
//	sprintf((char *)display_buf,"Pitch:%f\r\n",angle_pid.actual);
//	OLED_ShowString(0,2,(char*)display_buf,16,0);
//	printf("angle:%f\n time:%d\n",angle_pid.actual,HAL_GetTick());
	if(angle_pid.actual>30.0f||angle_pid.actual<-30.0f)
	{
		Run_Flag=0;
	}
	printf("Kp=%f,Kd=%f\nactual=%f,dif=%f\r\n",angle_pid.Kp,angle_pid.Kd,
			angle_pid.actual,angle_pid.dif);
	if(Run_Flag)
	{
	PID_Calculate(&angle_pid);
	Ave_PWM=-((int)angle_pid.out);
	Load(Ave_PWM,Ave_PWM);
	}
	else
	{
		Load(0,0);
	}
	printf("AvePWM:%d\n",Ave_PWM);
	 
	
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

#ifdef  USE_FULL_ASSERT
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

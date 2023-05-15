/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	MPU6050_Accelerometer_2G = 0x00, /* Range is +- 2G */
	MPU6050_Accelerometer_4G = 0x08, /* Range is +- 4G */
	MPU6050_Accelerometer_8G = 0x10, /* Range is +- 8G */
	MPU6050_Accelerometer_16G = 0x18 /* Range is +- 16G */
}MPU6050_Accelerometer;


typedef enum {
	MPU6050_Gyroscope_250s = 0x00,  /* Range is +- 250 degrees/s */
	MPU6050_Gyroscope_500s = 0x08,  /* Range is +- 500 degrees/s */
	MPU6050_Gyroscope_1000s = 0x10, /* Range is +- 1000 degrees/s */
	MPU6050_Gyroscope_2000s = 0x18  /* Range is +- 2000 degrees/s */
}MPU6050_Gyroscope;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68<<1
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define WHO_AM_I_REG 0x75
#define ACCEL_CONFIG_REG 0x1C
#define GYRO_CONFIG_REG 0x1B
#define LPF_REG 0x1A
#define RAD_TO_DEG 57.295779513082320876798154814105
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t Data, Check = 104;
uint8_t Acc_Set[6], Gyro_Set[6];
int16_t Gyro_Raw[3], Acc_Raw[3];
float Gyro_Cal[3];
int16_t Acc_Total_Vector;
float Gyro_Pitch_Angle, Gyro_Roll_Angle;
float Acc_Pitch_Angle, Acc_Roll_Angle;
float Pitch_Angle, Roll_Angle;
int i;
float PrevTime, PrevTime_1, Time_1, ElapsedTime_1, PrevTime_Cal, Time_Cal, ElapsedTime_Cal;
uint8_t Address;
float Gyro_Mult;
float Acce_Mult;
HAL_StatusTypeDef Set_Gyro;

double Proportional_Factor=30,Integral_Factor=1,Derivative_Factor=5;
double Target_Angle = 0;
float Proportional=0, Integral=0, Derivative=0;
float PID;
float Angle_Error;
float Previous_Error = 0;
unsigned long Time_Instant, Time_Last, ElapsedTime_PID;

uint16_t ADC_Value[3];
uint16_t Pulse_Value[3];

uint16_t PWM_M1, PWM_M2, PWM_M3, PWM_M4;

int Exti = 0;
char Mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU6050_Initialize(I2C_HandleTypeDef *hi2c, MPU6050_Accelerometer AccSens, MPU6050_Gyroscope GyroSens)
{
	/* Is device ready*/
	HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR, 3, HAL_MAX_DELAY);

	/* Check who I am */
	if(HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &Check, 1, HAL_MAX_DELAY))
	{
		/* Wake up */
		Data = 0x00;
	    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, HAL_MAX_DELAY);

	    //GYRO CNFG --> +-500 degree/second --> 0x08
	    Data = (uint8_t)AccSens;
	    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, HAL_MAX_DELAY);

	    // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
	    Data = (uint8_t)GyroSens;
	    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, HAL_MAX_DELAY);
	/*
	    Data = 0x03;
	    HAL_I2C_Mem_Write (hi2c, MPU6050_ADDR, LPF_REG, 1, &Data, 1, HAL_MAX_DELAY);
	    */
	}

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccSens) {
		case MPU6050_Accelerometer_2G:
			Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2;
			break;
		case MPU6050_Accelerometer_4G:
			Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4;
			break;
		case MPU6050_Accelerometer_8G:
			Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
			break;
		case MPU6050_Accelerometer_16G:
			Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16;
		default:
			break;
	}

	switch (GyroSens) {
		case MPU6050_Gyroscope_250s:
			Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
			break;
		case MPU6050_Gyroscope_500s:
			Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
			break;
		case MPU6050_Gyroscope_1000s:
			Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
			break;
		case MPU6050_Gyroscope_2000s:
			Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
		default:
			break;
	}
}
void MPU6050_Calibration(I2C_HandleTypeDef *hi2c)
{
  for(i=0; i<2000; i++)
  {
    PrevTime_Cal = Time_Cal;
    Time_Cal = HAL_GetTick();
    ElapsedTime_Cal = (Time_Cal-PrevTime_Cal)*1000;

    Gyro_Set[0]=0x43;
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, Gyro_Set, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, Gyro_Set, 6, HAL_MAX_DELAY);

    Gyro_Raw[0] = (Gyro_Set[0] << 8 | Gyro_Set[1]);
    Gyro_Raw[1] = (Gyro_Set[2] << 8 | Gyro_Set[3]);
    Gyro_Raw[2] = (Gyro_Set[4] << 8 | Gyro_Set[5]);

    Gyro_Cal[0] += Gyro_Raw[0];
    Gyro_Cal[1] += Gyro_Raw[1];
    Gyro_Cal[2] += Gyro_Raw[2];

    HAL_Delay(3);
  }

  Gyro_Cal[0] /= 2000;
  Gyro_Cal[1] /= 2000;
  Gyro_Cal[2] /= 2000;
}
void MPU6050_ComputeAngle(I2C_HandleTypeDef *hi2c)
{
  PrevTime_1 = Time_1;
  Time_1 = HAL_GetTick();
  ElapsedTime_1 = (Time_1 -PrevTime_1) * 1000;

  Acc_Set[0] = 0x3B;
  HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, Acc_Set, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, Acc_Set, 6, HAL_MAX_DELAY);

  Acc_Raw[0] = (Acc_Set[0] << 8 | Acc_Set[1]);
  Acc_Raw[1] = (Acc_Set[2] << 8 | Acc_Set[3]);
  Acc_Raw[2] = (Acc_Set[4] << 8 | Acc_Set[5]);

  Gyro_Set[0] = 0x43;
  HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, Gyro_Set, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, Gyro_Set, 6, HAL_MAX_DELAY);

  Gyro_Raw[0] = (Gyro_Set[0] << 8 | Gyro_Set[1]);
  Gyro_Raw[1] = (Gyro_Set[2] << 8 | Gyro_Set[3]);
  Gyro_Raw[2] = (Gyro_Set[4] << 8 | Gyro_Set[5]);

  Gyro_Raw[0] -= Gyro_Cal[0];
  Gyro_Raw[1] -= Gyro_Cal[1];
  Gyro_Raw[2] -= Gyro_Cal[2];

  Gyro_Pitch_Angle += Gyro_Raw[0] * Gyro_Mult * 0.004;
  Gyro_Roll_Angle += Gyro_Raw[1] * Gyro_Mult * 0.004;

  Gyro_Pitch_Angle += Gyro_Roll_Angle * sin(Gyro_Raw[2] * Gyro_Mult * 0.004 / RAD_TO_DEG);
  Gyro_Roll_Angle -= Gyro_Pitch_Angle * sin(Gyro_Raw[2] * Gyro_Mult * 0.004 / RAD_TO_DEG);

  Acc_Total_Vector = sqrt((Acc_Raw[0] * Acc_Raw[0]) + (Acc_Raw[1] * Acc_Raw[1]) + (Acc_Raw[2] * Acc_Raw[2]));

  Acc_Pitch_Angle = asin((float)Acc_Raw[1]/Acc_Total_Vector)* RAD_TO_DEG;
  Acc_Roll_Angle = asin((float)Acc_Raw[0]/Acc_Total_Vector)* - RAD_TO_DEG;

  Acc_Pitch_Angle -= 0.00;
  Acc_Roll_Angle -= 0.00;

  if(Set_Gyro)
  {
  	Pitch_Angle = Gyro_Pitch_Angle * 0.9996 + Acc_Pitch_Angle * 0.0004;
  	Roll_Angle = Gyro_Roll_Angle * 0.9996 + Acc_Pitch_Angle * 0.0004;
  }
  else
  {
  	Pitch_Angle = Acc_Pitch_Angle;
  	Set_Gyro = true;
  }

  while((HAL_GetTick() - PrevTime)*1000 < 4000);
  PrevTime = HAL_GetTick();
}
void ComputePID()
{
    Time_Last = Time_Instant;
    Time_Instant = HAL_GetTick();
    ElapsedTime_PID = (Time_Instant - Time_Last) / 1000;
    Angle_Error = Roll_Angle - Target_Angle;
    Proportional = Proportional_Factor * Angle_Error;
    if( -10 < Angle_Error && Angle_Error < 10)
    {
    	Integral = Integral + (Integral_Factor * Angle_Error);
    }
    Derivative = Derivative_Factor * ((Angle_Error - Previous_Error) / ElapsedTime_PID);
    PID = Proportional + Integral; //+ Derivative;
    if(PID < -2500)
    {
    	PID = -2500;
    }
    if(PID > 2500)
    {
    	PID = 2500;
    }
    Previous_Error = Angle_Error;
}
void Read_ADC(ADC_HandleTypeDef *hadc)
{
   	HAL_ADC_Start(&hadc1);
   	if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) != RESET)
    {
    	ADC_Value[0] = HAL_ADC_GetValue(&hadc1);
    }
   	HAL_ADC_Stop(&hadc1);
}
/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) != RESET)
	{
		ADC_Value[0] = HAL_ADC_GetValue(&hadc1);
	}
	if(__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_EOC) != RESET)
	{
		ADC_Value[1] = HAL_ADC_GetValue(&hadc2);
	}
	if(__HAL_ADC_GET_FLAG(&hadc3, ADC_FLAG_EOC) != RESET)
	{
		ADC_Value[2] = HAL_ADC_GetValue(&hadc3);
	}
}*/
uint16_t map(int In, int Inmin, int Inmax, int Outmin, int Outmax)
{
   	return (In - Inmin) * (Outmax - Outmin) / (Inmax - Inmin) + Outmin;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Exti++;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		if(Exti == 1)
		{
			Mode = 'Drone';
		}
		else if(i == 2)
		{
			Mode = 'Yatay';
		}
		else if(i == 3)
		{
			Mode = 'Sabit Kanat';
			Exti = 0;
		}
	}
}
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc2);
  HAL_ADC_Start_IT(&hadc3);

  MPU6050_Initialize(&hi2c1, MPU6050_Accelerometer_8G, MPU6050_Gyroscope_500s);
  MPU6050_Calibration(&hi2c1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MPU6050_ComputeAngle(&hi2c1);
	  ComputePID();
	  //Read_ADC();
	  Pulse_Value[0] = map(ADC_Value[0], 0, 4095, 0, 5000);
	  Pulse_Value[1] = map(ADC_Value[1], 0, 4095, 1000, 2000);
	  Pulse_Value[2] = map(ADC_Value[2], 0, 4095, 1000, 2000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  PWM_M1 = Pulse_Value[0] + PID;
	  PWM_M3 = Pulse_Value[0] - PID;
	  */
		switch (Mode) {
			case 'Drone':
				Target_Angle = 0;
				PWM_M1 = Pulse_Value[0] + Pulse_Value[1] + Pulse_Value[2] + PID;
				//PWM_M2 = Pulse_Value[0] - Pulse_Value[1] + Pulse_Value[1] + PID;
				PWM_M3 = Pulse_Value[0] + Pulse_Value[1] - Pulse_Value[1] - PID;
				//PWM_M4 = Pulse_Value[0] - Pulse_Value[1] - Pulse_Value[1] - PID;
				break;
			case 'Yatay':
				Target_Angle = 90;
				if( -10 < Angle_Error && Angle_Error < 10){
					Exti++;
				}
				break;
			case 'Sabit Kanat':
			  	PWM_M1 = Pulse_Value[0] + Pulse_Value[1] + Pulse_Value[2];
			  	//PWM_M2 = Pulse_Value[0] - Pulse_Value[1] + Pulse_Value[1];
			  	PWM_M3 = Pulse_Value[0] + Pulse_Value[1] - Pulse_Value[1];
			  	//PWM_M4 = Pulse_Value[0] - Pulse_Value[1] - Pulse_Value[1];
				break;
			default:
				break;
		}
	  /*
	  if(Exti = 1)
	  {
		  PWM_M1 = Pulse_Value[0] + Pulse_Value[1] + Pulse_Value[2] + PID;
		  //PWM_M2 = Pulse_Value[0] - Pulse_Value[1] + Pulse_Value[1] + PID;
		  PWM_M3 = Pulse_Value[0] + Pulse_Value[1] - Pulse_Value[1] - PID;
		  //PWM_M4 = Pulse_Value[0] - Pulse_Value[1] - Pulse_Value[1] - PID;
	  }
	  else if(Exti = 2)
	  {
	  	  Target_Angle = 90;
	  	  if( -10 < Angle_Error && Angle_Error < 10)
	  		  Exti++;
	  }
	  else if(Exti = 3)
	  {
	  	  PWM_M1 = Pulse_Value[0] + Pulse_Value[1] + Pulse_Value[2];
	  	  //PWM_M2 = Pulse_Value[0] - Pulse_Value[1] + Pulse_Value[1];
	  	  PWM_M3 = Pulse_Value[0] + Pulse_Value[1] - Pulse_Value[1];
	  	  //PWM_M4 = Pulse_Value[0] - Pulse_Value[1] - Pulse_Value[1];
	  }
	  else
	  {
		  Exti = 1;
	  }
	  */

	  if(PWM_M1 > 5000)
	  {
		  PWM_M1 = 5000;
	  }
	  else if(PWM_M1 < 5)
	  {
		  PWM_M1 = 5;
	  }
	  if(PWM_M3 > 5000)
	  {
		  PWM_M3 = 5000;
	  }
	  else if(PWM_M3 < 5)
	  {
		  PWM_M3 = 5;
	  }

	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_M1);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_M3);
	  //__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_M2);
	  //__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_M4);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

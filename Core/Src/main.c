/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "ble_remote.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define T1 &htim1
#define T2 &htim2
#define T3 &htim3
#define T4 &htim4
#define T5 &htim5
#define T6 &htim6
#define T7 &htim7
#define T8 &htim8
#define T9 &htim9
#define T10 &htim10
#define T11 &htim11
#define T12 &htim12
#define T13 &htim13
#define T14 &htim14
#define T15 &htim15
#define T16 &htim16
#define T17 &htim17

#define C1 TIM_CHANNEL_1
#define C2 TIM_CHANNEL_2
#define C3 TIM_CHANNEL_3
#define C4 TIM_CHANNEL_4
#define C5 TIM_CHANNEL_5
#define C6 TIM_CHANNEL_6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t data[11];
uint8_t swi[4];
int16_t xyz[3];
int16_t chassis[4];

/*set steering angle*/
int PWMS[4],angle[4]={25,25,25,25},angle1[4]={125,125,125,125};

int PWMLF1,PWMLF2,PWMRF1,PWMRF2,PWMLB1,PWMLB2,PWMRB1,PWMRB2;
int x,y,z;
int16_t fdb[4];
int number[3],receive[4];
int16_t message[8];

	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int16_t Max(int16_t in, int16_t m) 
    {               
        if (in > m)      
        {                    
           return m;      
        }                     
        else if (in < -m)
        {                     
           return -m;   
        } 
				else
				{
					return in;
				}
    }

void Start_Pwm()
{
	HAL_TIM_Base_Start(T3);
	HAL_TIM_PWM_Start(T3,C4);		//m1+
	HAL_TIM_PWM_Start(T3,C2);		//m1-
	
	HAL_TIM_Base_Start(T15);
	HAL_TIM_Base_Start(T8);
	HAL_TIM_PWM_Start(T15,C2);	//m2+
	HAL_TIM_PWM_Start(T8,C4);		//m2-
	
	HAL_TIM_Base_Start(T12);
	HAL_TIM_PWM_Start(T12,C1);	//m3+
	HAL_TIM_PWM_Start(T12,C2);	//m3-
	
	HAL_TIM_Base_Start(T15);
	HAL_TIM_Base_Start(T17);
	HAL_TIM_PWM_Start(T15,C1);	//m4+
	HAL_TIM_PWM_Start(T17,C1);	//m4-
	
	HAL_TIM_Base_Start(T16);
	
	HAL_TIM_PWM_Start(T3,C3);		//S1
	HAL_TIM_PWM_Start(T8,C3);		//S2
	HAL_TIM_PWM_Start(T3,C1);		//S3
	HAL_TIM_PWM_Start(T16,C1);		//S4
}

void Set_Pwm(int m1,int m2,int m3,int m4,int s0,int s1,int s2,int s3)
{
	  	if(m1>0)			    PWMLF1=m1,PWMLF2=0;
			else 	          PWMLF2=m1,PWMLF1=0;
		
		  if(m2>0)		    	PWMLB1=m2,PWMLB2=0;
			else 	          PWMLB2=m2,PWMLB1=0;
	
	    if(m3>0)			    PWMRB1=m3,PWMRB2=0;
			else 	          PWMRB2=m3,PWMRB1=0;
		
	    if(m4>0)			    PWMRF1=m4,PWMRF2=0;
			else 	          PWMRF2=m4,PWMRF1=0;
		  
			if(s0!=0)			    PWMS[0]=angle1[0];
			else 	          PWMS[0]=angle[0];
	
			if(s1!=0)			    PWMS[1]=angle1[1];
			else 	          PWMS[1]=angle[1];
	
			if(s2!=0)			    PWMS[2]=angle1[2];
			else 	          PWMS[2]=angle[2];
	
			if(s3!=0)			    PWMS[3]=angle1[3];
			else 	          PWMS[3]=angle[3];
	
}


void run_Pwm()
{
    __HAL_TIM_SetCompare(T3, C4, PWMLF1);	//M1+
		__HAL_TIM_SetCompare(T3, C2, PWMLF2);	//M1-
	
		__HAL_TIM_SetCompare(T15, C2, PWMLB1);	//M2+
		__HAL_TIM_SetCompare(T8, C4, PWMLB2);		//M2-
	
	/*reverse motor*/
		__HAL_TIM_SetCompare(T12,C2, PWMRB1);	//M3-
		__HAL_TIM_SetCompare(T12,C1, PWMRB2);	//M3+
	
		__HAL_TIM_SetCompare(T17,C1, PWMRF1);	//M4-
		__HAL_TIM_SetCompare(T15,C1, PWMRF2);	//M4+
	
	/*steering motor*/
		__HAL_TIM_SetCompare(T3, C3, PWMS[0]);	//S1
	__HAL_TIM_SetCompare(T8, C3, PWMS[1]);	//S2
	__HAL_TIM_SetCompare(T3, C1, PWMS[2]);	//S3
	__HAL_TIM_SetCompare(T16, C1, PWMS[3]);	//S4
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

/**************************************************************/
/*Init Begin*/

	Start_Pwm();
	HAL_UART_Receive_DMA(&huart4,data,11);
	
	/*************************
	PID SETTING
	*************************/
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
	PID_Regulator_t pid0;
	PID_Regulator_t pid1;
	PID_Regulator_t pid2;
	PID_Regulator_t pid3;
	PID_Init( &pid0,0, 500,100,4.5,0.4,0.4);
	PID_Init( &pid1,0, 500,100,4.5,0.4,0.4);
		/*reverse motor*/
	PID_Init( &pid2,0, 500,100,4.5,0.4,0.4);
	PID_Init( &pid3,0, 500,100,4.5,0.4,0.4);
	
	
	
/*Init End*/
/**************************************************************/

	const remote_t *r=get_remote_control_point();
	for(int i=0;i<4;i++)
{
  chassis[i]=0;
	receive[i]=0;
}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		/*chassis control*/
		xyz[0]=Max(r->rocker[0].y_position,150);
		xyz[1]=Max(r->rocker[0].x_position,150);
		xyz[2]=Max(r->rocker[1].x_position,150);
		chassis[0]=xyz[0]+xyz[1]+xyz[2];			//MT1
    chassis[1]=xyz[0]-xyz[1]+xyz[2];			//MT2
    chassis[2]=xyz[0]+xyz[1]-xyz[2];		//MT3
    chassis[3]=xyz[0]-xyz[1]-xyz[2];		//MT4
		/*********************
		********PID***********
		*********************/
		  fdb[0]=__HAL_TIM_GET_COUNTER(&htim1);		//MT1
		  fdb[1]=__HAL_TIM_GET_COUNTER(&htim2);		//MT2
		/*reverse PID*/
		  fdb[2]=-__HAL_TIM_GET_COUNTER(&htim4);		//MT3
		  fdb[3]=-__HAL_TIM_GET_COUNTER(&htim5);		//MT4
		
		__HAL_TIM_SetCounter(&htim1,0);
		__HAL_TIM_SetCounter(&htim2,0);
		__HAL_TIM_SetCounter(&htim4,0);
		__HAL_TIM_SetCounter(&htim5,0);
		

		  receive[0]=PID_Calculate(&pid0, fdb[0], chassis[0]);
		  receive[1]=PID_Calculate(&pid1, fdb[1], chassis[1]);
		  receive[2]=PID_Calculate(&pid2, fdb[2], chassis[2]);
			receive[3]=PID_Calculate(&pid3, fdb[3], chassis[3]);
			
		message[0]=chassis[0];
		message[1]=fdb[0];
		sendware(message,sizeof(message));
		
		/*steering control*/
		swi[0]=r->Switch[0];
		swi[1]=r->Switch[1];
		swi[2]=r->Switch[2];
		swi[3]=r->Switch[3];
		
		/*move*/
    Set_Pwm(receive[0],receive[1],receive[2],receive[3],swi[0],swi[1],swi[2],swi[3]);		//bi  huan
//		Set_Pwm(0,0,0,0,swi[0],swi[1],swi[2],swi[3]);		//kai huan
	  run_Pwm(); 
		
		
		HAL_Delay(30);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART4)
	{
		uart_to_remote(data);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

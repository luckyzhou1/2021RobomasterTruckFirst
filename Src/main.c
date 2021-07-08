/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "RC_Task.h"
#include "CanBus_Task.h"
#include "APPInteraction.h"
#include "ChassisControl.h"
#include "Ano_Dt.h"
#include "stdio.h"
#include "ControlTask.h"
#include "Paraminit.h"
#include "ramp.h"
#include "CatchingTask.h"
#include "DataScope_DP.h"
#include "Rescue_Task.h"
#include "Vision_interact.h"
#include "GimbalControl.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CAN2_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT_IDLE(&huart2,UART_Buffer,100);   /*�������ڿ����жϣ����ڽ���ң��������������*/
  HAL_TIM_Base_Start_IT(&htim3);  //ʹ��TIM3�жϣ����ʱ��Ϊ1ms
  HAL_TIM_Base_Start_IT(&htim5); //ʹ��TIM5�жϣ����ʱ��Ϊ500ms
  CANFilterInit();     /*CAN�˲��������úͿ���CANͨ��*/
  APPInteractionInit(); /*��ʼ���������*/
  ParamInit();
  HAL_UART3_Receive_IT_IDLE();//�Ӿ�����DMA���п����ж�
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      AllTask();
      
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t ms1 = 0,ms2 = 0,ms3 = 0,ms4 = 0,ms50 = 0;
    if(htim->Instance == TIM3)
    {
        ms1++;
        ms2++;
		ms3++;
        ms50++;
        gimbal_delay++;//��̨����������ʱ
        rescue_delay++;//��Ԯ������ʱ
        rescue_in_delay++;//ˢ��������ʱ(����)
        rescue_out_delay++;//ˢ��������ʱ(���)
        catching_reset_delay++;//��ȡ��λ������ʱ
        conversion_delay++;//�һ���ʯ������λ��ͨ������ʱ
        z_group_set_delay++;//��������
        x_group_set_delay++;//��������
        c_group_set_delay++;//��������
        if(ms1 >= 1)
        {
            ms1 = 0;
            Control_Ctrl = 1;
        }
        if(ms2 >= 2)
        {
            ms2 = 0;
            vision_send_flag = 1;
            Data_Send_ANO_DT = 1;
        }
	   if(ms3 >= 100)
        {
            ms3 = 0;
            delay_100ms  = 1;
        }
        if(ms50 >= 50)
        {
            ms50 = 0;
            Data_Scope_DP_Send = 1;
        }
        
    }
		if(htim->Instance == TIM5)
		{
			
            ms4++;
            flip_box_delay++;
            rotational_delay++;
            
            swip_card_ctrl_delay++;//ˢ����ϰ�������
            rescue_ctrl_delay++;//��Ԯ��ϰ�������
            KEY_CTRL_A_count++;//����
            KEY_A_count++;      //����
            catching_island_delay++;//��ȡ��Դ���Ͽ�ʯ������ʱ
            chassis_pos_delay++;
          if(ms4 >= 6)
          {
             ms4 = 0;
             delay_3000ms = 1;//����̧���������ϼ�����
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

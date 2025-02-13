/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "REG.h"
#include "JY_Serial.h"
#include "OLED.h"
#include <stdio.h> 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t rx_data;		
typedef struct
{
		float Angle[3];		/* 三轴角度 */
		float Accel[3];		/* 三轴加速度 */
		float Gyro[3];		/* 三轴角速度 */
		
}JY901_GetData;

JY901_GetData IMU;

uint8_t z_AngleUnlock[5]={0xFF,0xAA,0x69,0x88,0xB5};
uint8_t z_AngleCalibration[5]={0xFF,0xAA,0x01,0x04,0x00};
uint8_t z_AngleSave[5]={0xFF,0xAA,0x00,0x00,0x00};


/* 串口重定向 */

#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
 
}; 
 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
    //二选一,功能一样
    HAL_UART_Transmit (&huart2 ,(uint8_t *)&ch,1,HAL_MAX_DELAY );
	return ch;
    
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//    USART1->DR = (uint8_t) ch;      
//	return ch;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		static uint8_t rx_buffer[11],rx_cnt;		/* 串口数据接收缓冲区 */
		
		HAL_UART_Receive_DMA(&huart1,&rx_data,sizeof(rx_data));
		
		rx_buffer[rx_cnt++]=rx_data;
		if(rx_buffer[0]!=0x55) rx_cnt=0;
		
		if(rx_cnt>=11)		/* 一组数据接收完毕 */
		{
				switch(rx_buffer[1])
				{
						case 0x51:			/* 三轴加速度获取 */
								IMU.Accel[0]=(short)(((short)rx_buffer[3]<<8)|rx_buffer[2])/32768.0*16.0;	/* x轴加速度 */
						    IMU.Accel[1]=(short)(((short)rx_buffer[5]<<8)|rx_buffer[4])/32768.0*16.0;	/* y轴加速度 */
						    IMU.Accel[2]=(short)(((short)rx_buffer[7]<<8)|rx_buffer[6])/32768.0*16.0;	/* z轴加速度 */
								break;
						
						case 0x52:			/* 三轴角速度获取 */
								IMU.Gyro[0]=(short)(((short)rx_buffer[3]<<8)|rx_buffer[2])/32768.0*2000.0;	/* x轴角速度 */
						    IMU.Gyro[1]=(short)(((short)rx_buffer[5]<<8)|rx_buffer[4])/32768.0*2000.0;	/* y轴角速度 */
						    IMU.Gyro[2]=(short)(((short)rx_buffer[7]<<8)|rx_buffer[6])/32768.0*2000.0;	/* z轴角速度 */
								break;
						
						case 0x53:			/* 三轴角度获取 */
								IMU.Angle[0]=(short)(((short)rx_buffer[3]<<8)|rx_buffer[2])/32768.0*180.0;	/* 滚转角读取 */
						    IMU.Angle[1]=(short)(((short)rx_buffer[5]<<8)|rx_buffer[4])/32768.0*180.0;	/* 俯仰角读取 */
						    IMU.Angle[2]=(short)(((short)rx_buffer[7]<<8)|rx_buffer[6])/32768.0*180.0;	/* 偏航角读取 */
								break;
				}
				rx_cnt=0;			/* 清零 */
		}		
		
}

/* z轴角度校准(6轴) */
void z_Angle_Calibration(void)
{
		HAL_UART_Transmit (&huart1 ,z_AngleUnlock,sizeof(z_AngleUnlock),0xff);
		HAL_Delay(200);
		HAL_UART_Transmit (&huart1 ,z_AngleCalibration,sizeof(z_AngleCalibration),0xff);
		HAL_Delay(4000);
		HAL_UART_Transmit (&huart1 ,z_AngleSave,sizeof(z_AngleSave),0xff);
}

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	z_Angle_Calibration();		/* z轴角度校准 */
	
//	HAL_UART_Receive_IT(&huart1,&rx_data,1);
	HAL_UART_Receive_DMA(&huart1,&rx_data,sizeof(rx_data));
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		HAL_Delay(400);
		
//		printf("Ag_x:%f  ",IMU.Angle[0]);
//		printf("Ag_y:%f  ",IMU.Angle[1]);
//		printf("Ag_z:%f  \n\n",IMU.Angle[2]);
//		
//		printf("Ac_x:%f  ",IMU.Accel[0]);
//		printf("Ac_y:%f  ",IMU.Accel[1]);
//		printf("Ac_z:%f  \n\n",IMU.Accel[2]);

//		printf("Gy_x:%f  ",IMU.Gyro[0]);
//		printf("Gy_y:%f  ",IMU.Gyro[1]);
//		printf("Gy_z:%f  \n\n\n\n",IMU.Gyro[2]);
				
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

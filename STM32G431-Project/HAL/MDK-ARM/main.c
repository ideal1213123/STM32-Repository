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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "BaseDriver.h"

void SystemClock_Config(void);

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();

	////////////////////
	TIM1_PWM_Start();
	TIM2_IC_Start();
	TIM3_PWM_Start();
	unsigned short pwmCmp = 1000;
	unsigned short tmVal[2] = {0};
	
	ADC1_Calibration();
	ADC2_Calibration();
	unsigned short ad1v0 = 0,ad1v1 = 0,ad2v0 = 0;
	
	char showMode = 1;
	LCD_Init(showMode);
	LCD_Clear( MAKE_RGB565(255,255,0) );
	
	unsigned char restart_cnt = 0;
	MemRead(1,&restart_cnt,1);
	++restart_cnt;
	MemWrite(1,&restart_cnt,1);
	
	char buff[64] = "";
	sprintf(buff,"[%d]",restart_cnt);
	LCD_DrawString(showMode,160,20,buff,0,0xFFFF);

	while (1)
	{
		//Test Key and LED
		unsigned char key,keyEdge;
		GetKeyState(key,keyEdge);
		SetLeds(key);
		
		static unsigned char mcpVal = 0;
		if( keyEdge&0x01 ){//ADC TEST
			mcpVal++;
			if(mcpVal>0x8F){ mcpVal = 0; }
			MCP_Write(mcpVal);
			
			ADC1_Read(ad1v0,ad1v1);
			ADC2_Read(ad2v0);	
		}
		if( keyEdge&0x02 ){//TIM TEST
			pwmCmp += 1000;
			if( 10000 == pwmCmp ){ pwmCmp = 1000; }
			TIM1_SetCompare1(pwmCmp>>1);
			TIM3_SetCompare1(pwmCmp);
		}		
		TIM2_GetCapture(tmVal);
		//Uart Test
		EchoUart(&huart1);
		
		static char showFrame = 0;
		if( ++showFrame && showFrame == 4 ){
			showFrame = 0;

			LCDPrintf(showMode,20,20,0,0xFFFF,"[%d,%d,%d]",ad1v0,ad1v1,ad2v0);		
			LCDPrintf(showMode,20,50,0,0xFFFF,"[%d,%d]",100000/tmVal[0],pwmCmp);
			LCDPrintf(showMode,20,80,0,0xFFFF,"[%d,%d]",tmVal[0],tmVal[1]);
		}
		
	}
}







/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

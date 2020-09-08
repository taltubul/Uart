
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "Defines.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t phy_alive = 0;
uint8_t interface_clock = 0;
uint8_t phy_tx_busy = 0;
uint8_t Phy_Tx_clock_status = 0;
uint8_t Phy_Rx_clock_status = 0;
uint8_t Phy_Rx_clock_pin =0;
uint8_t interface_clock_falling_edge;
uint8_t interface_clock_rising_edge;
uint8_t Phy_Tx_falling_edge = 0;
uint8_t Phy_Tx_rising_edge = 0;
uint8_t Rx_value = 0;
uint8_t Tx_value = 0;
uint8_t phy_to_dll_rx_bus = 0;
uint8_t phy_to_dll_rx_bus_valid = 0;
uint8_t dll_to_phy_tx_bus = 0;	
uint8_t dll_to_phy_tx_bus_valid = 0;	
GPIO_A* gpioa = ((GPIO_A *)GPIOA_Base);
GPIO_B* gpiob = ((GPIO_B *)GPIOB_Base);
TIM_4* tim4 = ((TIM_4 *)TIM_4_Base);
TIM_3* tim3 = ((TIM_3 *)TIM_3_Base);
uint8_t phy_to_dll_rx_bus_valid_signal = 0;
uint8_t dll_to_phy_tx_bus_valid_signal = 0;
uint32_t counterperiod4;
uint8_t sample_counter = 0;
uint8_t one_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void phy_TX(){	
	static uint8_t parity = 0;
	static uint8_t counter = 0;
	static uint8_t temp_data = 0;
	uint8_t mask = 0x01;
	if(dll_to_phy_tx_bus_valid&&!phy_tx_busy){
		tim3->CNT = Counter_period;
		HAL_TIM_Base_Start_IT(&htim3);
		phy_tx_busy = 1;
		temp_data = dll_to_phy_tx_bus;
		dll_to_phy_tx_bus_valid = 0;
		HAL_GPIO_WritePin(phy_tx_data_GPIO_Port,phy_tx_data_Pin,GPIO_PIN_RESET);
		}
	if(phy_tx_busy&&Phy_Tx_clock_status){
		Phy_Tx_clock_status = 0;
		counter++;
		switch(counter){
			case 1:
				HAL_GPIO_WritePin(phy_tx_data_GPIO_Port,phy_tx_data_Pin,GPIO_PIN_RESET);
				break;
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				if (((temp_data>>(counter-2)) & mask)){
					HAL_GPIO_WritePin(phy_tx_data_GPIO_Port,phy_tx_data_Pin,GPIO_PIN_SET);
					parity++;
				}
				else{
					HAL_GPIO_WritePin(phy_tx_data_GPIO_Port,phy_tx_data_Pin,GPIO_PIN_RESET);
				}		
				break;
			
			case 10:
				if (parity%2){
					HAL_GPIO_WritePin(phy_tx_data_GPIO_Port,phy_tx_data_Pin,GPIO_PIN_SET);
				}
				else {
						HAL_GPIO_WritePin(phy_tx_data_GPIO_Port,phy_tx_data_Pin,GPIO_PIN_RESET);
				}		
				break;			
			
			
			case 11:
					HAL_GPIO_WritePin(phy_tx_data_GPIO_Port,phy_tx_data_Pin,GPIO_PIN_SET);
				break;			
			
			
			case 12:
					HAL_GPIO_WritePin(phy_tx_data_GPIO_Port,phy_tx_data_Pin,GPIO_PIN_SET);
					counter = 0;
					HAL_TIM_Base_Stop_IT(&htim3);
					phy_tx_busy = 0;	
					parity = 0;			
				break;			
				
		}	
	}		
}
void phy_RX(){
	static uint8_t parity = 0;
	static uint8_t buffer;
	static uint8_t throw_data_flag;
	static uint8_t counter = 0;
	static uint8_t clock4_flag = 0;
	Rx_value = HAL_GPIO_ReadPin(phy_rx_data_GPIO_Port,phy_rx_data_Pin);
	if (!Rx_value&&!clock4_flag){
		tim4->CNT = Counter_period;
		HAL_TIM_Base_Start_IT(&htim4);
		clock4_flag = 1;
	}
	counterperiod4 = tim4->CNT;
	//if (Phy_Rx_clock_status&&(counterperiod4>(Counter_period/4)&&counterperiod4<(0.75*Counter_period))){
		//for(uint8_t i =0;i<3;i++){
			//if (HAL_GPIO_ReadPin(phy_rx_data_GPIO_Port,phy_rx_data_Pin)){
			//one_counter++;
			//	i++;
			//}
		//}
		//sample_counter++;
	//}
	if (Phy_Rx_clock_status&&(counterperiod4>(Counter_period/4)&&counterperiod4<(0.75*Counter_period))&&(sample_counter<3)){
	 if (HAL_GPIO_ReadPin(phy_rx_data_GPIO_Port,phy_rx_data_Pin)){
			 one_counter++;
		}
	 sample_counter++;
	}
	//if (Phy_Rx_clock_status&&sample_counter==1){
	if (Phy_Rx_clock_status&&sample_counter==3){
		sample_counter = 0;
		Phy_Rx_clock_status = 0;
		counter ++;

		if(one_counter>=2){
			Rx_value = 1;
		}
		else{
			Rx_value = 0;
		}
		one_counter = 0;
		switch(counter){
			case 1:
				if (Rx_value){
				counter = 0;
				HAL_TIM_Base_Stop_IT(&htim4);		
				clock4_flag = 0;					
				}
			break;
				
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				if (Rx_value){
					buffer = buffer>>1;
					buffer = buffer | 0x80;
					parity++;
				}
				else{
					buffer = buffer>>1;
				}		
				break;
			
			case 10:
				if (Rx_value != (parity%2)){
					throw_data_flag = 1;
				}
				break;			
			case 11:
				if (Rx_value != 1){
					throw_data_flag = 1;
				}
				break;			
					
			case 12:
					if (Rx_value != 1){
					throw_data_flag = 1;
					}
					if(!throw_data_flag){
						phy_to_dll_rx_bus = buffer;
						phy_to_dll_rx_bus_valid = 1;
						counter = 0;
						throw_data_flag = 0 ;
						HAL_TIM_Base_Stop_IT(&htim4);		
						buffer = 0 ;			
						clock4_flag = 0;
						parity = 0;
					}
						else{
						counter = 0;
						throw_data_flag = 0 ;
						HAL_TIM_Base_Stop_IT(&htim4);		
						buffer = 0 ;			
						clock4_flag = 0;
						parity = 0;
						}
				break;						
		}	
	}		
}
void phy_layer()
{
	phy_TX();
	phy_RX();
}
void interface()
{
	uint16_t temp1;
	if(interface_clock_rising_edge){
		interface_clock_rising_edge = 0;
		
		if(phy_to_dll_rx_bus_valid){
			temp1 = gpiob->ODR;
			temp1 &= 0x00FF;
			temp1 |= ((uint16_t)phy_to_dll_rx_bus) << 8;
			gpiob->ODR = temp1;
			phy_to_dll_rx_bus_valid = 0;
			HAL_GPIO_WritePin(pin_phy_to_dll_rx_bus_valid_GPIO_Port,pin_phy_to_dll_rx_bus_valid_Pin,GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(pin_phy_to_dll_rx_bus_valid_GPIO_Port,pin_phy_to_dll_rx_bus_valid_Pin,GPIO_PIN_RESET);
		}
		if(phy_tx_busy){
			HAL_GPIO_WritePin(pin_phy_tx_busy_GPIO_Port,pin_phy_tx_busy_Pin,GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(pin_phy_tx_busy_GPIO_Port,pin_phy_tx_busy_Pin,GPIO_PIN_RESET);
		}
	}
	if(interface_clock_falling_edge){
		interface_clock_falling_edge = 0;
		dll_to_phy_tx_bus_valid = HAL_GPIO_ReadPin(pin_dll_to_phy_tx_bus_valid_GPIO_Port,pin_dll_to_phy_tx_bus_valid_Pin);
		if(dll_to_phy_tx_bus_valid&&!phy_tx_busy){
			dll_to_phy_tx_bus = gpioa->IDR;		
		}		
	}
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_GPIO_WritePin(pin_phy_alive_GPIO_Port,pin_phy_alive_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(phy_tx_data_GPIO_Port,phy_tx_data_Pin,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Tx_value = HAL_GPIO_ReadPin(phy_tx_data_GPIO_Port,phy_tx_data_Pin);
		//phy_alive = HAL_GPIO_ReadPin(pin_phy_alive_GPIO_Port,pin_phy_alive_Pin);
		//interface_clock  = HAL_GPIO_ReadPin(pin_interface_clock_GPIO_Port,pin_interface_clock_Pin);
		//phy_to_dll_rx_bus_valid_signal = HAL_GPIO_ReadPin(pin_phy_to_dll_rx_bus_valid_GPIO_Port,pin_phy_to_dll_rx_bus_valid_Pin);
		//dll_to_phy_tx_bus_valid_signal = HAL_GPIO_ReadPin(pin_dll_to_phy_tx_bus_valid_GPIO_Port,pin_dll_to_phy_tx_bus_valid_Pin);
		phy_layer();
		interface();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3750;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3750;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(phy_tx_data_GPIO_Port, phy_tx_data_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pin_phy_to_dll_rx_bus_Pin|pin_phy_to_dll_rx_busB11_Pin|pin_phy_to_dll_rx_busB12_Pin|pin_phy_to_dll_rx_busB13_Pin 
                          |pin_phy_to_dll_rx_busB14_Pin|pin_phy_to_dll_rx_busB15_Pin|pin_interface_clock_Pin|pin_phy_alive_Pin 
                          |pin_phy_tx_busy_Pin|pin_phy_to_dll_rx_bus_valid_Pin|pin_phy_to_dll_rx_busB8_Pin|pin_phy_to_dll_rx_busB9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : phy_tx_data_Pin */
  GPIO_InitStruct.Pin = phy_tx_data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(phy_tx_data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pin_dll_to_phy_tx_bus_Pin pin_dll_to_phy_tx_busA1_Pin pin_dll_to_phy_tx_busA2_Pin pin_dll_to_phy_tx_busA3_Pin 
                           pin_dll_to_phy_tx_busA4_Pin pin_dll_to_phy_tx_busA5_Pin pin_dll_to_phy_tx_busA6_Pin pin_dll_to_phy_tx_busA7_Pin */
  GPIO_InitStruct.Pin = pin_dll_to_phy_tx_bus_Pin|pin_dll_to_phy_tx_busA1_Pin|pin_dll_to_phy_tx_busA2_Pin|pin_dll_to_phy_tx_busA3_Pin 
                          |pin_dll_to_phy_tx_busA4_Pin|pin_dll_to_phy_tx_busA5_Pin|pin_dll_to_phy_tx_busA6_Pin|pin_dll_to_phy_tx_busA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : phy_rx_data_Pin pin_dll_to_phy_tx_bus_valid_Pin */
  GPIO_InitStruct.Pin = phy_rx_data_Pin|pin_dll_to_phy_tx_bus_valid_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : pin_phy_to_dll_rx_bus_Pin pin_phy_to_dll_rx_busB11_Pin pin_phy_to_dll_rx_busB12_Pin pin_phy_to_dll_rx_busB13_Pin 
                           pin_phy_to_dll_rx_busB14_Pin pin_phy_to_dll_rx_busB15_Pin pin_interface_clock_Pin pin_phy_alive_Pin 
                           pin_phy_tx_busy_Pin pin_phy_to_dll_rx_bus_valid_Pin pin_phy_to_dll_rx_busB8_Pin pin_phy_to_dll_rx_busB9_Pin */
  GPIO_InitStruct.Pin = pin_phy_to_dll_rx_bus_Pin|pin_phy_to_dll_rx_busB11_Pin|pin_phy_to_dll_rx_busB12_Pin|pin_phy_to_dll_rx_busB13_Pin 
                          |pin_phy_to_dll_rx_busB14_Pin|pin_phy_to_dll_rx_busB15_Pin|pin_interface_clock_Pin|pin_phy_alive_Pin 
                          |pin_phy_tx_busy_Pin|pin_phy_to_dll_rx_bus_valid_Pin|pin_phy_to_dll_rx_busB8_Pin|pin_phy_to_dll_rx_busB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

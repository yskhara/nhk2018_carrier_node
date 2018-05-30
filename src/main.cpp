
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"

#include "can.h"
#include "main.h"
#include "led.h"

#include "CarrierNode.hpp"

#include <array>


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


#define CAN_MTU 8

template<typename T>
union _Encapsulator
{
	T data;
	uint64_t i;
};

template <typename T>
static void can_unpack(const uint8_t (&buf) [CAN_MTU], T &data);
template<typename T>
static void can_pack(uint8_t (&buf) [CAN_MTU], const T data);

static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


static CarrierNode * const carrierNode = new CarrierNode();

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	//MX_CAN_Init();

    can_init();
    can_set_bitrate(CAN_BITRATE_500K);

    // turn on green LED
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // blink red LED for test
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    can_enable();

    // loop forever
    CanRxMsgTypeDef rx_msg;
    CanTxMsgTypeDef tx_msg;
    uint32_t status;

	static constexpr uint16_t id_handStatus	= 0x300;
	static constexpr uint16_t id_handCmd	= 0x301;

	uint32_t last_ctrl_time = HAL_GetTick();
	const uint32_t ctrl_interval = 1000 / 10;

	uint32_t last_stat_time = HAL_GetTick();
	const uint32_t stat_interval = 1000 / 2;

	NVIC_SetPriority(EXTI9_5_IRQn, 1);
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	//status = can_rx(&rx_msg, 3);

    while (1)
    {
    	if(HAL_GetTick() - last_ctrl_time > ctrl_interval)
    	{
			carrierNode->Control();
			last_ctrl_time = HAL_GetTick();
    	}

    	if(HAL_GetTick() - last_stat_time > stat_interval)
    	{
			const CarrierStatus status = carrierNode->GetStatus();
			can_pack(tx_msg.Data, static_cast<uint16_t>(status));
			tx_msg.StdId = id_handStatus;
			tx_msg.RTR = CAN_RTR_DATA;
			tx_msg.IDE = CAN_ID_STD;
			tx_msg.DLC = 2;
			can_tx(&tx_msg, 3);
			last_stat_time = HAL_GetTick();
    	}

		if (is_can_msg_pending(CAN_FIFO0))
		{
			status = can_rx(&rx_msg, 3);
			if (status == HAL_OK)
			{
				// received can frame
				if(rx_msg.StdId == id_handCmd)
				{
					uint16_t cmd;
					can_unpack(rx_msg.Data, cmd);
					carrierNode->SetCommand(static_cast<CarrierCommands>(cmd));
					carrierNode->Control();
					last_ctrl_time = HAL_GetTick();
				}
			}

			led_on();
		}

		led_process();
    }
}

extern "C" void EXTI9_5_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
		carrierNode->OnRightChuckSensorEXTInt();
	}

	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
		carrierNode->OnLeftChuckSensorEXTInt();
	}

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

template <typename T>
void can_unpack(const uint8_t (&buf) [CAN_MTU], T &data)
{
	_Encapsulator<T> _e;

	for(int i = 0; i < sizeof(T); i++)
	{
		_e.i = (_e.i << 8) | (uint64_t)(buf[i]);
	}

	data = _e.data;
}

template<typename T>
void can_pack(uint8_t (&buf) [CAN_MTU], const T data)
{
	_Encapsulator<T> _e;
	_e.data = data;

	for(int i = sizeof(T); i > 0;)
	{
		i--;
		buf[i] = _e.i & 0xff;
		_e.i >>= 8;
	}
}

/* CAN init function */
static void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_1TQ;
  hcan.Init.BS2 = CAN_BS2_1TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB12 PB13 
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PA0   ------> UART4_TX
     PA1   ------> UART4_RX
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PD7   ------> SPI1_MOSI
     PB3 (JTDO/TRACESWO)   ------> SPI1_SCK
     PB4 (NJTRST)   ------> SPI1_MISO
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RADIO_RST_Pin|RADIO_AUX_Pin|M0_Radio_Pin|M1_Radio_Pin
                          |CS3_Pin|CS2_Pin|CS1E11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS1_Pin|DROUGE_BACK_Pin|DROUGE_MAIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CS1D8_Pin|CS2D10_Pin|CS3D12_Pin|USR_LED_Pin
                          |CS_ISM_Pin|CS1D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RADIO_RST_Pin RADIO_AUX_Pin M0_Radio_Pin M1_Radio_Pin
                           CS3_Pin CS2_Pin CS1E11_Pin */
  GPIO_InitStruct.Pin = RADIO_RST_Pin|RADIO_AUX_Pin|M0_Radio_Pin|M1_Radio_Pin
                          |CS3_Pin|CS2_Pin|CS1E11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DET_Pin */
  GPIO_InitStruct.Pin = SD_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin DROUGE_BACK_Pin DROUGE_MAIN_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|DROUGE_BACK_Pin|DROUGE_MAIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INT3_Pin */
  GPIO_InitStruct.Pin = INT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT2_Pin INT1_Pin */
  GPIO_InitStruct.Pin = INT2_Pin|INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1D8_Pin CS2D10_Pin CS3D12_Pin USR_LED_Pin
                           CS_ISM_Pin CS1D4_Pin */
  GPIO_InitStruct.Pin = CS1D8_Pin|CS2D10_Pin|CS3D12_Pin|USR_LED_Pin
                          |CS_ISM_Pin|CS1D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BMP390_INT1_Pin BMP390_INT2_Pin BMP390_INT3_Pin */
  GPIO_InitStruct.Pin = BMP390_INT1_Pin|BMP390_INT2_Pin|BMP390_INT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USR_BUTTON_Pin */
  GPIO_InitStruct.Pin = USR_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USR_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAIN_Pin */
  GPIO_InitStruct.Pin = MAIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ISM_INT_Pin */
  GPIO_InitStruct.Pin = ISM_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ISM_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOSI1_Pin */
  GPIO_InitStruct.Pin = MOSI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(MOSI1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCK1_Pin MISO1_Pin */
  GPIO_InitStruct.Pin = SCK1_Pin|MISO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_RST_Pin */
  GPIO_InitStruct.Pin = GPS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_RST_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

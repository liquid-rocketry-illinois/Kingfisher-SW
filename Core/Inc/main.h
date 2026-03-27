/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
  #include "tim.h"
  #include "Buzzer.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RADIO_RST_Pin GPIO_PIN_2
#define RADIO_RST_GPIO_Port GPIOE
#define RADIO_AUX_Pin GPIO_PIN_3
#define RADIO_AUX_GPIO_Port GPIOE
#define M0_Radio_Pin GPIO_PIN_4
#define M0_Radio_GPIO_Port GPIOE
#define M1_Radio_Pin GPIO_PIN_5
#define M1_Radio_GPIO_Port GPIOE
#define SERVO1_ENC_Pin GPIO_PIN_2
#define SERVO1_ENC_GPIO_Port GPIOC
#define SERVO2_ENC_Pin GPIO_PIN_3
#define SERVO2_ENC_GPIO_Port GPIOC
#define SD_DET_Pin GPIO_PIN_3
#define SD_DET_GPIO_Port GPIOA
#define SD_CS1_Pin GPIO_PIN_4
#define SD_CS1_GPIO_Port GPIOA
#define SCK6_Pin GPIO_PIN_5
#define SCK6_GPIO_Port GPIOA
#define MISO6_Pin GPIO_PIN_6
#define MISO6_GPIO_Port GPIOA
#define MOSI6_Pin GPIO_PIN_7
#define MOSI6_GPIO_Port GPIOA
#define SIMU_INT3_Pin GPIO_PIN_2
#define SIMU_INT3_GPIO_Port GPIOB
#define SIMU_CS3_Pin GPIO_PIN_7
#define SIMU_CS3_GPIO_Port GPIOE
#define SIMU_INT2_Pin GPIO_PIN_8
#define SIMU_INT2_GPIO_Port GPIOE
#define SIMU_CS2_Pin GPIO_PIN_9
#define SIMU_CS2_GPIO_Port GPIOE
#define SIMU_INT1_Pin GPIO_PIN_10
#define SIMU_INT1_GPIO_Port GPIOE
#define SIMU_CS1_Pin GPIO_PIN_11
#define SIMU_CS1_GPIO_Port GPIOE
#define SCK4_Pin GPIO_PIN_12
#define SCK4_GPIO_Port GPIOE
#define MISO4_Pin GPIO_PIN_13
#define MISO4_GPIO_Port GPIOE
#define MOSI4_Pin GPIO_PIN_14
#define MOSI4_GPIO_Port GPIOE
#define SHT41_SCL_Pin GPIO_PIN_10
#define SHT41_SCL_GPIO_Port GPIOB
#define SHT41_SDA_Pin GPIO_PIN_11
#define SHT41_SDA_GPIO_Port GPIOB
#define SCK2_Pin GPIO_PIN_13
#define SCK2_GPIO_Port GPIOB
#define MISO2_Pin GPIO_PIN_14
#define MISO2_GPIO_Port GPIOB
#define MOSI2_Pin GPIO_PIN_15
#define MOSI2_GPIO_Port GPIOB
#define BMP390_CS1_Pin GPIO_PIN_8
#define BMP390_CS1_GPIO_Port GPIOD
#define BMP390_INT1_Pin GPIO_PIN_9
#define BMP390_INT1_GPIO_Port GPIOD
#define BMP390_CS2_Pin GPIO_PIN_10
#define BMP390_CS2_GPIO_Port GPIOD
#define BMP390_INT2_Pin GPIO_PIN_11
#define BMP390_INT2_GPIO_Port GPIOD
#define BMP390_CS3_Pin GPIO_PIN_12
#define BMP390_CS3_GPIO_Port GPIOD
#define BMP390_INT3_Pin GPIO_PIN_13
#define BMP390_INT3_GPIO_Port GPIOD
#define USR_LED_Pin GPIO_PIN_14
#define USR_LED_GPIO_Port GPIOD
#define USR_BUTTON_Pin GPIO_PIN_15
#define USR_BUTTON_GPIO_Port GPIOD
#define SERVO2_PWM_Pin GPIO_PIN_6
#define SERVO2_PWM_GPIO_Port GPIOC
#define SERVO1_PWM_Pin GPIO_PIN_7
#define SERVO1_PWM_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOC
#define MAIN_Pin GPIO_PIN_9
#define MAIN_GPIO_Port GPIOC
#define DROUGE_BACK_Pin GPIO_PIN_8
#define DROUGE_BACK_GPIO_Port GPIOA
#define DROUGE_MAIN_Pin GPIO_PIN_9
#define DROUGE_MAIN_GPIO_Port GPIOA
#define USB_D__Pin GPIO_PIN_11
#define USB_D__GPIO_Port GPIOA
#define USB_D_A12_Pin GPIO_PIN_12
#define USB_D_A12_GPIO_Port GPIOA
#define ISM_INT_Pin GPIO_PIN_15
#define ISM_INT_GPIO_Port GPIOA
#define SCK3_Pin GPIO_PIN_10
#define SCK3_GPIO_Port GPIOC
#define MISO3_Pin GPIO_PIN_11
#define MISO3_GPIO_Port GPIOC
#define MOSI3_Pin GPIO_PIN_12
#define MOSI3_GPIO_Port GPIOC
#define CS_ISM_Pin GPIO_PIN_0
#define CS_ISM_GPIO_Port GPIOD
#define CS1_Pin GPIO_PIN_4
#define CS1_GPIO_Port GPIOD
#define MOSI1_Pin GPIO_PIN_7
#define MOSI1_GPIO_Port GPIOD
#define SCK1_Pin GPIO_PIN_3
#define SCK1_GPIO_Port GPIOB
#define MISO1_Pin GPIO_PIN_4
#define MISO1_GPIO_Port GPIOB
#define GPS_RST_Pin GPIO_PIN_5
#define GPS_RST_GPIO_Port GPIOB
#define GPS_I2CSCL_Pin GPIO_PIN_8
#define GPS_I2CSCL_GPIO_Port GPIOB
#define GPS_I2CSDA_Pin GPIO_PIN_9
#define GPS_I2CSDA_GPIO_Port GPIOB
#define RADIO_TX_Pin GPIO_PIN_0
#define RADIO_TX_GPIO_Port GPIOE
#define RADIO_RX_Pin GPIO_PIN_1
#define RADIO_RX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

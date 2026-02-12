/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "tim.h"
#include "ism6hg256x_reg.h"
#include "usart.h"
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
/* USER CODE BEGIN Variables */
struct IMU_Data {
  int16_t linAX;
  int16_t linAY;
  int16_t linAZ;
  int16_t angAX;
  int16_t angAY;
  int16_t angAZ;
};

extern SPI_HandleTypeDef hspi3;
stmdev_ctx_t dev_ctx;
struct IMU_Data main_imu_data;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);
int8_t IMU_read_data(stmdev_ctx_t *ctx, struct IMU_Data *data);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */



  // TODO: Check if we are polling, DMA, or data interrupt
  // 1: Sensor initialisation
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hspi3;
  dev_ctx.mdelay = platform_delay;


  platform_init();
  osDelay(20);




  // 3: Check sensor identity
  uint8_t id;
  ism6hg256x_device_id_get(&dev_ctx, &id);
  if (id != ISM6HG256X_ID) {
    while (id != ISM6HG256X_ID) {
      // Retry
      ism6hg256x_device_id_get(&dev_ctx, &id);
      // Wrong SPI handle signal: flashing LED for 1 second
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
      osDelay(1000);
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
      osDelay(1000);
    }
  }

  // 4: Sensor correct, initiate sensor
  ism6hg256x_sh_reset_set(&dev_ctx, PROPERTY_ENABLE);
  osDelay(20);
  ism6hg256x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  //TODO: Check if this is actually 240 Hz or if we need something else
  ism6hg256x_xl_setup(&dev_ctx, ISM6HG256X_ODR_AT_240Hz, ISM6HG256X_XL_HIGH_PERFORMANCE_MD);
  ism6hg256x_gy_setup(&dev_ctx, ISM6HG256X_ODR_AT_240Hz, ISM6HG256X_GY_HIGH_PERFORMANCE_MD);

  TIM3->ARR = 57141L;
  TIM3->CCR2 = 28571L;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* Infinite loop */
  for(;;)
  {

  if (IMU_read_data(&dev_ctx, &main_imu_data) == 0) {
    // Run code that does things
    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
    osDelay(100);
  }





  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE BEGIN 4 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len){
  HAL_GPIO_WritePin(CS_ISM_GPIO_Port, CS_ISM_Pin, GPIO_PIN_RESET); // Chip selection!!
  HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, &reg, 1, 1000); // Send register address
  HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, (uint8_t*)bufp, len, 1000); // Send data
  HAL_GPIO_WritePin(CS_ISM_GPIO_Port, CS_ISM_Pin, GPIO_PIN_SET); // Unselect chip
  return 0;
}

static int32_t platform_read(void *handle,
                             uint8_t reg,
                             uint8_t *bufp,
                             uint16_t len)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef*)handle;

  // Set MSB to indicate read
  reg |= 0xC0;

  // Pull CS low to start transaction
  HAL_GPIO_WritePin(CS_ISM_GPIO_Port, CS_ISM_Pin, GPIO_PIN_RESET);

  // 1. Send the register address
  HAL_SPI_Transmit(hspi, &reg, 1, HAL_MAX_DELAY);

  // 2. Receive the data bytes
  HAL_SPI_Receive(hspi, bufp, len, HAL_MAX_DELAY);

  // Pull CS high to end transaction
  HAL_GPIO_WritePin(CS_ISM_GPIO_Port, CS_ISM_Pin, GPIO_PIN_SET);

  return 0;
}



void platform_init(void){
  HAL_GPIO_WritePin(CS_ISM_GPIO_Port, CS_ISM_Pin, GPIO_PIN_SET);
}

static void tx_com(uint8_t *tx_buffer, uint16_t len){
  HAL_UART_Transmit(&huart4, tx_buffer, len, 1000);
}

static void platform_delay(uint32_t ms){
  HAL_Delay(ms);
}

int8_t IMU_read_data(stmdev_ctx_t *ctx, struct IMU_Data *data){
  ism6hg256x_data_ready_t drdy;
  ism6hg256x_flag_data_ready_get(ctx, &drdy); // Check if both accelerometer and gyroscope have data
  if(drdy.drdy_xl && drdy.drdy_gy){
    ism6hg256x_acceleration_raw_get(ctx, (int16_t*)&data -> linAX); // Implicit data dump into ay and az
    ism6hg256x_angular_rate_raw_get(ctx, (int16_t*)&data -> angAX); // Same as above
    return 0; // Succcess
  }
  else{
    return 1; // Failure
  }

}
/* USER CODE END Application */


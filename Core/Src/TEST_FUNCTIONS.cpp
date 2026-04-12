//
// Created by dyrel on 2/23/2026.
//

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "TEST_FUNCTIONS.h"

#include <cstring>

#include "task.h"

TEST::TEST(){;}

//TODO
//Need to change code of PWM generation. verify correct pwm signal
int TEST::SERVO_TEST() {
    Servo_Axon_Mini_MKII servoSet;
    bool initState = servoSet.Init({0,0}, PRECISION::TENTH_DEGREE, true);
    vTaskDelay(500);

    // Track status of servos
    static int status = 0;
    if (status == 0 && initState) {
        while (1) {
            for (int8_t i = 0; i < 90; i++) {
                servoSet.Update(i*0.5F, i*0.5F);
                vTaskDelay(2);
            }
            for (int8_t i = 90; i > -90; i--) {
                servoSet.Update(i*0.5F, i*0.5F);
                vTaskDelay(2);
            }
            vTaskDelay(1000);
        }
    }

    return status;
}

int TEST::IMU_TEST() {
    IMUs IMU_ENGINE(true);
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    IMUsStatus sensorStatuses = IMU_ENGINE.Init();

    while (1) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        sensorStatuses = IMU_ENGINE.Update();
        vTaskDelay(1);
    }
}

int TEST::BARO_TEST()
{
    Baro_Unified BaroEngine;
    auto init = BaroEngine.Init(true);

    while (1)
    {
        auto status = BaroEngine.Update();
        auto data = BaroEngine.getData();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        //vTaskDelay(100); TODO: vTaskDelay causing hard fault- memory bad? overflow? div by 0?
        HAL_Delay(100);
    }
}

int TEST::LIS2_TEST()
{
    Magnetometer mag;
    LIS2_Data data;
    uint8_t status = 0;
    uint8_t init = mag.Init();

    while (1)
    {
        status = mag.Update();
        data = mag.getRawData();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(100);
    }
}

int TEST::RADIO_TEST() {
    Telemetry telem;
    telemetryData dataE22;
    GndStationData keepalive;
    uint8_t status = 0;
    uint8_t init = telem.Init();

    while (1) {
        status = telem.Update();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(100);
    }
}

int TEST::PYRO_TEST()
{

    vTaskDelay(pdMS_TO_TICKS(1000));

    HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_SET);
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    vTaskDelay(pdMS_TO_TICKS(5000));
    HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_RESET);

    while (1) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

int TEST::SD_TEST() {
    FATFS FatFs;
    FIL Fil;
    FRESULT FR_Status;
    FATFS *FS_Ptr;
    DWORD FreeClusters;
    UINT RWC, WWC;
    uint32_t TotalSize, FreeSpace;
    char RW_Buffer[200];

    {
    //------------------[ Mount The SD Card ]--------------------
    FR_Status = f_mount(&FatFs, "", 1);
    if (FR_Status != FR_OK)
    {
      while (1) {
          HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
          vTaskDelay(pdMS_TO_TICKS(1500));
      }
    }

    //------------------[ Get & Print The SD Card Size & Free Space ]--------------------
    f_getfree("", &FreeClusters, &FS_Ptr);
    TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
    FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);

    //------------------[ Open A Text File For Write & Write Data ]--------------------
    //Open the file
    FR_Status = f_open(&Fil, "TextFileWrite.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(FR_Status != FR_OK)
    {
        while (1) {
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
            vTaskDelay(pdMS_TO_TICKS(700));
        }
    }
    // (1) Write Data To The Text File [ Using f_puts() Function ]
    f_puts("1234567890\n", &Fil);
    // (2) Write Data To The Text File [ Using f_write() Function ]
    strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SPI, Using f_write()\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    // Close The File
    f_close(&Fil);
    //------------------[ Open A Text File For Read & Read Its Data ]--------------------
    // Open The File
    FR_Status = f_open(&Fil, "TextFileWrite.txt", FA_READ);
    if(FR_Status != FR_OK)
    {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // (1) Read The Text File's Data [ Using f_gets() Function ]
    f_gets(RW_Buffer, sizeof(RW_Buffer), &Fil);

    // (2) Read The Text File's Data [ Using f_read() Function ]
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);

    // Close The File
    f_close(&Fil);

    //------------------[ Open An Existing Text File, Update Its Content, Read It Back ]--------------------
    // (1) Open The Existing File For Write (Update)
    FR_Status = f_open(&Fil, "TextFileWrite.txt", FA_OPEN_EXISTING | FA_WRITE);
    FR_Status = f_lseek(&Fil, f_size(&Fil)); // Move The File Pointer To The EOF (End-Of-File)
    if(FR_Status != FR_OK)
    {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    // (2) Write New Line of Text Data To The File
    f_puts("This New Line Was Added During Update!\r\n", &Fil);
    f_close(&Fil);
    memset(RW_Buffer,'\0',sizeof(RW_Buffer)); // Clear The Buffer
    // (3) Read The Contents of The Text File After The Update
    FR_Status = f_open(&Fil, "TextFileWrite.txt", FA_READ); // Open The File For Read
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    f_close(&Fil);
    //------------------[ Delete The Text File ]--------------------
    // Delete The File
    /*
    FR_Status = f_unlink(TextFileWrite.txt);
    if (FR_Status != FR_OK){
        sprintf(TxBuffer, "Error! While Deleting The (TextFileWrite.txt) File.. \r\n");
        UART_Print(TxBuffer);
    }
    */
  } while(0);
  //------------------[ Test Complete! Unmount The SD Card ]--------------------
  FR_Status = f_mount(NULL, "", 0);
  if (FR_Status != FR_OK)
  {
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
  }
}


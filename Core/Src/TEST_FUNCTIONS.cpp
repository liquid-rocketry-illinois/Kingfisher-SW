//
// Created by dyrel on 2/23/2026.
//

#include "cmsis_os2.h"
#include "TEST_FUNCTIONS.h"

#include <cstring>

#include "fatfs.h"

TEST::TEST(){;}

//TODO
//Need to change code of PWM generation. verify correct pwm signal
int TEST::SERVO_TEST() {
    Servo_Axon_Mini_MKII servoSet;
    bool initState = servoSet.Init({0,0}, PRECISION::TENTH_DEGREE, true);
    osDelay(500);

    // Track status of servos
    static int status = 0;
    if (status == 0 && initState) {
        servoSet.Update(90, -90);
        osDelay(10);
    }
    // Error - Not initialized within tolerance.
    else if (!initState) {
        // Attempt init again
        initState = servoSet.Init({0,0}, PRECISION::TENTH_DEGREE, true);
        osDelay(500);
        return -2;
    }
    // Something else went wrong (This should never really happen
    // but it is there cause muscle memory)
    else return -1;

    return status;
}

int TEST::IMU_TEST() {
    IMU IMU_ENGINE(false);
    IMUsStatus sensorStatuses = IMU_ENGINE.Init();

    while (1) {
        sensorStatuses = IMU_ENGINE.Update();
        osDelay(1);
    }
}

void TEST::SD_TEST()
{
    HAL_Delay(1000);
    FATFS FatFs; // file system object
    FIL Fil; // file object
    FRESULT Status; // return from FATFs operations (FR_OK = good)
    FATFS *FS_Ptr; // Pointer to reference the file system object
    UINT RWC, WWC; // Read/Write Word Counter
    DWORD FreeClusters;
    uint32_t TotalSize, FreeSpace;

    char RW_Buffer[200]; // Buffer for reading and writing

    do
    {
        // MOUNT SD CARD

        // detect if there is a card
        GPIO_PinState SD_Det = HAL_GPIO_ReadPin(SD_DET_GPIO_Port, SD_DET_Pin);
        if (SD_Det == GPIO_PIN_RESET)
            HAL_GPIO_WritePin(SD_CS1_GPIO_Port, SD_CS1_Pin, GPIO_PIN_SET);
            Status = f_mount(&FatFs, "", 0); // attempt to mount the sd card on default drive
        if (Status != FR_OK) // mount failed
        {
            HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
            return;
        }
        // GETTING AND PRINTING SD CARD FREE SPACE
        Status = f_getfree("", &FreeClusters, &FS_Ptr); // Query the number of free clusters

        // Calculate total size: (total clusters) * (sectors per cluster) * (512 bytes / sector / 1025) * 0.512 (convert to kilobytes
        TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.512); // FAT reserves 2 sectors

        // Same for free clusters
        FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.512);


        // OPENING FILE:

        // Open file to read the data access
        Status = f_open(&Fil, "TextFileWrite.txt", FA_READ);
        if (Status != FR_OK) // failed to open file
        {
            while (1) {
                HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
                vTaskDelay(500);
            }
        }
        // use f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC) to read the rest of the data up to f_size() bytes. RWC recieves the number of bytes left.
        f_close(&Fil); // close file after reading



    } while (false); // use do{}while(false) since we only want it to run once but can use break to skip code

    // unmounting drive
    Status = f_mount(nullptr, " ", 0); // passing NULL unmounts the drive
    if (Status != FR_OK)
    {
        while (1) {
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
            vTaskDelay(100);
        }
    }

    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
}
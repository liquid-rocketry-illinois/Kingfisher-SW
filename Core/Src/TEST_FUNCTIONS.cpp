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

SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart4;
char TxBuffer[250];

void UART_Print(char* str)
{
    HAL_UART_Transmit(&huart4, (uint8_t *) str, strlen(str), 100);
}

void TEST::SD_TEST()
{
    FATFS FatFs; // file system object
    FIL Fil; // file object
    FRESULT FR_Status; // return from FATFs operations (FR_OK = good)
    FATFS *FS_Ptr; // Pointer to reference the file system object
    UINT RWC, WWC; // Read/Write Word Counter
    DWORD FreeClusters;
    uint32_t TotalSize, FreeSpace;

    char RW_Buffer[200]; // Buffer for reading and writing

    do
    {
        // MOUNT SD CARD

        FR_Status = f_mount(&FatFs, " ", 1); // attempt to mount the sd card on default drive
        if (FR_Status != FR_OK) // mount failed
        {
            sprintf(TxBuffer, "Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status); // print FATFs error code
            UART_Print(TxBuffer);
            break; // EXIT loop
        }
        sprintf(TxBuffer, "SD Card mounted successfully \r\n\n");
        UART_Print(TxBuffer);

        // GETTING AND PRINTING SD CARD FREE SPACE

        f_getfree("", &FreeClusters, &FS_Ptr); // Query the number of free clusters

        // Calculate total size: (total clusters) * (sectors per cluster) * (512 bytes / sector / 1025) * 0.512 (convert to kilobytes
        TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.512); // FAT reserves 2 sectors

        // Same for free clusters
        FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.512);
        sprintf(TxBuffer, "Total SD Card Size: %lu Bytes\r\n", TotalSize);
        UART_Print(TxBuffer);
        sprintf(TxBuffer, "Free SD Card Space: %lu Bytes\r\n\n", FreeSpace);
        UART_Print(TxBuffer);

        // OPEN A TEXT FILE TO READ AND WRITE DATA

        // Open file to read the data access
        FR_Status = f_open(&Fil, "TextFileWrite.txt", FA_READ);
        if (FR_Status != FR_OK) // failed to open file
        {
            sprintf(TxBuffer, "Error while opening A New Text File, Error Code: (%i)\r\n", FR_Status);
            UART_Print(TxBuffer);
            break; // File couldn't be opened, skip to unmount
        }

        sprintf(TxBuffer, "Text File Opened! Reading Data From The Text File..\r\n\n"); // file opened succesfully
        UART_Print(TxBuffer);

        f_gets(RW_Buffer, sizeof(RW_Buffer), &Fil); // reads one line and stops when reaching newline or buffer is full
        sprintf(TxBuffer, "Data Read From (TextFileWrite.txt) Using f_gets():%s", RW_Buffer);
        UART_Print(TxBuffer);

        // use f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC) to read the rest of the data up to f_size() bytes. RWC recieves the number of bytes left.
        f_close(&Fil); // close file after reading



    } while (false); // use do{}while(false) since we only want it to run once but can use break to skip code

    FR_Status = f_mount(nullptr, " ", 0); // passing NULL unmounts the drive
    if (FR_Status != FR_OK)
    {
        sprintf(TxBuffer, "Error! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
        UART_Print(TxBuffer);
    } else {
        sprintf(TxBuffer, "SD Card Un-mounted Successfully! \r\n");
        UART_Print(TxBuffer);
    }
}
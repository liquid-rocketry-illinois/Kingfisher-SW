//
// Created by dyrel on 4/16/2026.
//

#include "SDCard.h"
#include "fatfs.h"

static FIL  s_file;
static bool s_open = false;

int8_t SD_Init()
{
    if (s_open) return 0;

    /* if (f_mount(&USERFatFS, USERPath, 1) != FR_OK) return -1; */
    /* if (res != FR_OK) return -2; */

    // Guard: verify FATFS driver was linked by MX_FATFS_Init (retUSER == 0)
    extern uint8_t retUSER;
    if (retUSER != 0) return -3;

    // Expose raw FRESULT as negative value for debugger visibility
    FRESULT res = f_mount(&USERFatFS, USERPath, 1);
    if (res != FR_OK) return -(int8_t)res;

    res = f_open(&s_file, "HAL1_LOG.TXT", FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) return -(int8_t)res;

    f_lseek(&s_file, f_size(&s_file));   // seek to end for append behaviour
    s_open = true;
    return 0;
}

int8_t SD_LogNewline(const char* msg)
{
    if (!s_open) return -1;
    /* f_printf(&s_file, "%s\n", msg); */
    /* f_sync(&s_file); */
    /* return 0; */
    if (f_printf(&s_file, "%s\n", msg) < 0) return -2;
    if (f_sync(&s_file) != FR_OK)           return -3;
    return 0;
}

int8_t SD_LogInline(const char* msg)
{
    if (!s_open) return -1;
    /* f_printf(&s_file, "%s", msg); */
    /* f_sync(&s_file); */
    /* return 0; */
    if (f_printf(&s_file, "%s", msg) < 0) return -2;
    if (f_sync(&s_file) != FR_OK)         return -3;
    return 0;
}

int8_t SD_LogGPS(double lat, double lon, double alt,
                 uint8_t hour, uint8_t min, uint8_t sec)
{
    if (!s_open) return -1;
    if (f_printf(&s_file, "%.7f,%.7f,%.2f,%02u,%02u,%02u\n",
                 lat, lon, alt, hour, min, sec) < 0) return -2;
    if (f_sync(&s_file) != FR_OK) return -3;
    return 0;
}

int8_t SD_Close()
{
    if (!s_open) return 0;
    f_close(&s_file);
    f_mount(NULL, USERPath, 0);
    s_open = false;
    return 0;
}

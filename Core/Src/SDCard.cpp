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

    if (f_mount(&USERFatFS, USERPath, 1) != FR_OK) return -1;

    FRESULT res = f_open(&s_file, "HAL1_LOG.TXT", FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) return -2;

    f_lseek(&s_file, f_size(&s_file));   // seek to end for append behaviour
    s_open = true;
    return 0;
}

int8_t SD_LogNewline(const char* msg)
{
    if (!s_open) return -1;
    f_printf(&s_file, "%s\n", msg);
    f_sync(&s_file);
    return 0;
}

int8_t SD_LogInline(const char* msg)
{
    if (!s_open) return -1;
    f_printf(&s_file, "%s", msg);
    f_sync(&s_file);
    return 0;
}

//
// Created by dyrel on 4/16/2026.
//

#include "SDCard.h"

#include <cstdio>

#include "fatfs.h"

static FIL  s_file;
static bool s_open = false;

static FIL  s_dyn_file;
static bool s_dyn_open = false;

int8_t SD_Init()
{
    if (s_open) return 0;

    /* if (f_mount(&USERFatFS, USERPath, 1) != FR_OK) return -1; */
    /* if (res != FR_OK) return -2; */

    // Guard: verify FATFS driver was linked by MX_FATFS_Init (retUSER == 0).
    // Returns -20 (not a valid FRESULT) so it is distinguishable in the debugger.
    extern uint8_t retUSER;
    if (retUSER != 0) return -20;

    // f_mount failure: return -(FRESULT).  FR_NOT_READY=-3, FR_NO_FILESYSTEM=-13, etc.
    // Check sd_init_fail_step (sd_diskio.c) to see exactly where disk_initialize stopped.
    FRESULT res = f_mount(&USERFatFS, USERPath, 1);
    if (res != FR_OK) return -(int8_t)res;

    // f_open failure: offset FRESULT by -100 so mount vs open errors are distinguishable.
    res = f_open(&s_file, "HAL1_LOG.TXT", FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) return (int8_t)(-100 - (int8_t)res);

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
    // f_printf in FatFS R0.12c does not support %f — use snprintf (newlib) instead
    char buf[64];
    snprintf(buf, sizeof(buf), "%.7f,%.7f,%.2f,%02u,%02u,%02u\n",
             lat, lon, alt, (unsigned)hour, (unsigned)min, (unsigned)sec);
    if (f_puts(buf, &s_file) == EOF) return -2;
    if (f_sync(&s_file) != FR_OK)   return -3;
    return 0;
}

int8_t SD_DynInit()
{
    if (s_dyn_open) return 0;
    if (!s_open)    return -1;  // must call SD_Init() first (filesystem already mounted)

    FRESULT res = f_open(&s_dyn_file, "HAL1_DYN.TXT", FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) return (int8_t)(-100 - (int8_t)res);

    f_lseek(&s_dyn_file, f_size(&s_dyn_file));
    f_printf(&s_dyn_file, "t,alt_m,vz_ms,vh_ms,roll_rate_rads\n");
    f_sync(&s_dyn_file);
    s_dyn_open = true;
    return 0;
}

int8_t SD_LogDyn(const char* msg)
{
    if (!s_dyn_open) return -1;
    if (f_printf(&s_dyn_file, "%s\n", msg) < 0) return -2;
    if (f_sync(&s_dyn_file) != FR_OK)           return -3;
    return 0;
}

int8_t SD_Close()
{
    if (s_dyn_open) { f_close(&s_dyn_file); s_dyn_open = false; }
    if (!s_open) return 0;
    f_close(&s_file);
    f_mount(NULL, USERPath, 0);
    s_open = false;
    return 0;
}

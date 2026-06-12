//
// Created by dyrel on 4/16/2026.
//

#ifndef KINGFISHER_SW_SDCARD_H
#define KINGFISHER_SW_SDCARD_H

#include <stdint.h>

// Mount the filesystem and open the log file.
// Must be called once before any SD_Log* call.
// Returns:
//    0   success
//  -20   FatFs driver not linked (MX_FATFS_Init() not called before SD_Init())
//   -(FRESULT)   mount failure (e.g. -3 = FR_NOT_READY — check sd_init_fail_step)
//   -(100+FRESULT)  file-open failure after successful mount
int8_t SD_Init();

// Append msg + newline, then flush to disk.
int8_t SD_LogNewline(const char* msg);

// Append msg inline (no newline), then flush to disk.
int8_t SD_LogInline(const char* msg);

// Log one GPS fix as a CSV row: lat,lon,alt,hh,mm,ss
int8_t SD_LogGPS(double lat, double lon, double alt,
                 uint8_t hour, uint8_t min, uint8_t sec);

// Open HAL1_DYN.TXT for dynamics model logging. Call after SD_Init().
int8_t SD_DynInit();

// Append one dynamics line to HAL1_DYN.TXT.
int8_t SD_LogDyn(const char* msg);

// Flush, close the log file, and unmount. Call before power-down.
int8_t SD_Close();

#endif //KINGFISHER_SW_SDCARD_H

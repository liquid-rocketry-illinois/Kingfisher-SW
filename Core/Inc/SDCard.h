//
// Created by dyrel on 4/16/2026.
//

#ifndef KINGFISHER_SW_SDCARD_H
#define KINGFISHER_SW_SDCARD_H

#include <stdint.h>

// Mount the filesystem and open the log file.
// Must be called once before any SD_Log* call.
// Returns 0 on success, -1 on mount failure, -2 on file-open failure.
int8_t SD_Init();

// Append msg + newline, then flush to disk.
int8_t SD_LogNewline(const char* msg);

// Append msg inline (no newline), then flush to disk.
int8_t SD_LogInline(const char* msg);

// Log one GPS fix as a CSV row: lat,lon,alt,hh,mm,ss
int8_t SD_LogGPS(double lat, double lon, double alt,
                 uint8_t hour, uint8_t min, uint8_t sec);

// Flush, close the log file, and unmount. Call before power-down.
int8_t SD_Close();

#endif //KINGFISHER_SW_SDCARD_H

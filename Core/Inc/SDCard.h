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

#endif //KINGFISHER_SW_SDCARD_H

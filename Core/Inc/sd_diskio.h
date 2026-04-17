/*
 * sd_diskio.h
 *
 * SPI-mode SD/SDHC/SDXC disk I/O functions for FatFs, declared to match
 * Diskio_drvTypeDef so they drop directly into user_diskio.c.
 *
 * Hardware (STM32H753 / Kingfisher):
 *   SPI6  — PA5=SCK  PA6=MISO  PA7=MOSI
 *   CS    — PA4  (SD_CS1_Pin  / SD_CS1_GPIO_Port)
 *   DET   — PA3  (SD_DET_Pin  / SD_DET_GPIO_Port, LOW = card inserted)
 */

#ifndef SD_DISKIO_H
#define SD_DISKIO_H

#include "diskio.h"   /* DSTATUS, DRESULT, BYTE, DWORD, UINT, WORD */

#ifdef __cplusplus
extern "C" {
#endif

DSTATUS SD_disk_initialize(BYTE pdrv);
DSTATUS SD_disk_status    (BYTE pdrv);
DRESULT SD_disk_read      (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT SD_disk_write     (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl     (BYTE pdrv, BYTE cmd, void *buff);

#ifdef __cplusplus
}
#endif

#endif /* SD_DISKIO_H */
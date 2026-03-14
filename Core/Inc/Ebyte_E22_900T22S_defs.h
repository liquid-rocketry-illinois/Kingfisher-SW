//
// Created by dyrel on 3/7/2026.
//

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifndef KINGFISHER_SW_EBYTE_E22_900T22S_DEFS_H
#define KINGFISHER_SW_EBYTE_E22_900T22S_DEFS_H

// CH902 = 902 MHz, CH903 = 903 MHz, etc....
typedef enum {
    CH902 = 52,
    CH903,
    CH904,
    CH905,
    CH906,
    CH907,
    CH908,
    CH909,
    CH910,
    CH911,
    CH912,
    CH913,
    CH914,
    CH915,
    CH916,
    CH917,
    CH918,
    CH919,
    CH920,
    CH921,
    CH922,
    CH923,
    CH924,
    CH925,
    CH926,
    CH927,
    CH928
} E22Channel915;

typedef enum {
    TRANS,
    WOR,
    CONFIG,
    OFF
} EBYTE_MODE;

typedef enum
{
    E22_OK                = 0,
    E22_ERR_TIMEOUT       = -1,
    E22_ERR_UART          = -2,
    E22_ERR_BUSY          = -3,
    E22_ERR_INVALID_PARAM = -4,
    E22_ERR_NOT_INITIALIZED = -5
} E22_Status;

typedef enum
{
    E22_AIR_RATE_2_4K   = 0b010,  // 2.4 kbps
    E22_AIR_RATE_4_8K   = 0b011,  // 4.8 kbps
    E22_AIR_RATE_9_6K   = 0b100,  // 9.6 kbps
    E22_AIR_RATE_19_2K  = 0b101,   // 19.2 kbps
    E22_AIR_RATE_38_4K  = 0b110,  // 38.4 kbps
    E22_AIR_RATE_62_5K  = 0b111  // 62.5 kbps
} E22_AIR_DATA_RATE;

typedef enum
{
    E22_UART_BAUD_1200    = 0b000 << 3,
    E22_UART_BAUD_2400    = 0b001 << 3,
    E22_UART_BAUD_4800    = 0b010 << 3,
    E22_UART_BAUD_9600    = 0b011 << 3,
    E22_UART_BAUD_19200   = 0b100 << 3,
    E22_UART_BAUD_38400   = 0b101 << 3,
    E22_UART_BAUD_57600   = 0b110 << 3,
    E22_UART_BAUD_115200  = 0b111 << 3
} E22_UART_BAUD;

typedef enum
{
    E22_TX_POWER_22DBM = 0b00,
    E22_TX_POWER_17DBM = 0b01,
    E22_TX_POWER_13DBM = 0b10,
    E22_TX_POWER_10DBM = 0b11
} E22_TX_POWER;

typedef struct
{
    uint8_t head;
    uint8_t addh;
    uint8_t addl;
    uint8_t sped;
    uint8_t chan;
    uint8_t option;

    UART_HandleTypeDef *huart;

    GPIO_TypeDef *E22_M0_PORT;
    uint16_t E22_M0_PIN;

    GPIO_TypeDef *E22_M1_PORT;
    uint16_t E22_M1_PIN;

    GPIO_TypeDef *E22_AUX_PORT;
    uint16_t E22_AUX_PIN;
} config_e22_900t22s;

#endif //KINGFISHER_SW_EBYTE_E22_900T22S_DEFS_H
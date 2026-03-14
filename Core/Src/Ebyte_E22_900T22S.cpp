//
// Created by dyrel on 3/12/2026.
//

#include "Ebyte_E22_900T22S.h"
#include <string.h>

/* ===============================
   Internal driver state
   =============================== */

static config_e22_900t22s current_cfg;
static bool initialized = false;


/* ===============================
   Internal helpers
   =============================== */

static void uart_write(uint8_t *data, size_t len)
{
    HAL_UART_Transmit(current_cfg.huart, data, len, HAL_MAX_DELAY);
}

static void uart_read(uint8_t *data, size_t len)
{
    HAL_UART_Receive(current_cfg.huart, data, len, HAL_MAX_DELAY);
}

static void set_pin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

static GPIO_PinState read_pin(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin);
}


/* ===============================
   Initialization
   =============================== */

int8_t init_e22_900t22s(config_e22_900t22s *cfg)
{
    if(cfg == NULL)
        return -1;
    
    memcpy(&current_cfg, cfg, sizeof(config_e22_900t22s));

    changeMode(EBYTE_MODE::CONFIG);

    if(writeConfig_e22_900t22s(cfg, true) < 0)
        return -2;

    changeMode(EBYTE_MODE::TRANS);

    initialized = true;

    return 0;
}


/* ===============================
   Transmission
   =============================== */

int8_t transmit_e22_900t22s(uint8_t *data, size_t length)
{
    if(!initialized)
        return -1;

    uart_write(data, length);

    waitAux_e22_900t22s(100);

    return 0;
}


int8_t transmit_fixed_e22_900t22s(
    uint16_t address,
    uint8_t channel,
    uint8_t *data,
    size_t length)
{
    if(!initialized)
        return -1;

    uint8_t header[3];

    header[0] = (address >> 8) & 0xFF;
    header[1] = address & 0xFF;
    header[2] = channel;

    uart_write(header, 3);
    uart_write(data, length);

    waitAux_e22_900t22s(100);

    return 0;
}


/* ===============================
   Reception
   =============================== */

int16_t recieve_e22_900t22s(uint8_t *buffer, size_t max_length)
{
    if(!initialized)
        return -1;

    if(!e22_available())
        return -2;

    HAL_UART_Receive(current_cfg.huart, buffer, max_length, 10);

    return max_length;
}


bool e22_available(void)
{
    return (__HAL_UART_GET_FLAG(current_cfg.huart, UART_FLAG_RXNE) != RESET);
}


/* ===============================
   Mode control
   =============================== */

void changeMode(EBYTE_MODE mode)
{
    switch(mode)
    {
        case EBYTE_MODE::TRANS:
            set_pin(current_cfg.E22_M0_PORT, current_cfg.E22_M0_PIN, GPIO_PIN_RESET);
            set_pin(current_cfg.E22_M1_PORT, current_cfg.E22_M1_PIN, GPIO_PIN_RESET);
            break;

        case EBYTE_MODE::WOR:
            set_pin(current_cfg.E22_M0_PORT, current_cfg.E22_M0_PIN, GPIO_PIN_SET);
            set_pin(current_cfg.E22_M1_PORT, current_cfg.E22_M1_PIN, GPIO_PIN_RESET);
            break;

        case EBYTE_MODE::CONFIG:
            set_pin(current_cfg.E22_M0_PORT, current_cfg.E22_M0_PIN, GPIO_PIN_RESET);
            set_pin(current_cfg.E22_M1_PORT, current_cfg.E22_M1_PIN, GPIO_PIN_SET);
            break;

        case EBYTE_MODE::OFF:
            set_pin(current_cfg.E22_M0_PORT, current_cfg.E22_M0_PIN, GPIO_PIN_SET);
            set_pin(current_cfg.E22_M1_PORT, current_cfg.E22_M1_PIN, GPIO_PIN_SET);

        default:
            break;
    }

    HAL_Delay(2);
}


/* ===============================
   Frequency
   =============================== */

void changeOpFreq_e22_900t22s(E22Channel915 channel)
{
    current_cfg.chan = channel;

    changeMode(EBYTE_MODE::CONFIG);

    writeConfig_e22_900t22s(&current_cfg, true);

    changeMode(EBYTE_MODE::TRANS);
}


E22Channel915 getOpFreq_e22_900t22s(void)
{
    return (E22Channel915)current_cfg.chan;
}


/* ===============================
   Address
   =============================== */

void setAddress_e22_900t22s(uint16_t address)
{
    current_cfg.addh = (address >> 8) & 0xFF;
    current_cfg.addl = address & 0xFF;
}

uint16_t getAddress_e22_900t22s(void)
{
    return ((uint16_t)current_cfg.addh << 8) | current_cfg.addl;
}


/* ===============================
   Parameter setters
   =============================== */

void setAirDataRate_e22_900t22s(E22_AIR_DATA_RATE rate)
{
    current_cfg.sped &= 0xF8;
    current_cfg.sped |= rate;
}

void setUARTBaud_e22_900t22s(E22_UART_BAUD baud)
{
    current_cfg.sped &= 0xC7;
    current_cfg.sped |= baud;
}

void setTxPower_e22_900t22s(E22_TX_POWER power)
{
    current_cfg.option &= 0xFC;
    current_cfg.option |= power;
}


/* ===============================
   Config read/write
   =============================== */

int8_t readConfig_e22_900t22s(config_e22_900t22s *cfg)
{
    uint8_t cmd[3] = {0xC1, 0xC1, 0xC1};

    uart_write(cmd, 3);

    uint8_t response[6];

    uart_read(response, 6);

    memcpy(cfg, &response[1], 5);

    return 0;
}


int8_t writeConfig_e22_900t22s(
    const config_e22_900t22s *cfg,
    bool save_to_flash)
{
    uint8_t frame[6];

    frame[0] = save_to_flash ? 0xC0 : 0xC2;
    frame[1] = cfg->addh;
    frame[2] = cfg->addl;
    frame[3] = cfg->sped;
    frame[4] = cfg->chan;
    frame[5] = cfg->option;

    uart_write(frame, 6);

    waitAux_e22_900t22s(100);

    return 0;
}


/* ===============================
   AUX control
   =============================== */

int8_t waitAux_e22_900t22s(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while(read_pin(current_cfg.E22_AUX_PORT, current_cfg.E22_AUX_PIN) == GPIO_PIN_RESET)
    {
        if((HAL_GetTick() - start) > timeout_ms)
            return -1;
    }

    return 0;
}


bool e22_isBusy(void)
{
    return (read_pin(current_cfg.E22_AUX_PORT, current_cfg.E22_AUX_PIN) == GPIO_PIN_RESET);
}


/* ===============================
   Reset
   =============================== */

void reset_e22_900t22s(void)
{
    changeMode(EBYTE_MODE::CONFIG);
    HAL_Delay(10);
    changeMode(EBYTE_MODE::TRANS);
}


/* ===============================
   Driver state
   =============================== */

bool e22_initialized(void)
{
    return initialized;
}
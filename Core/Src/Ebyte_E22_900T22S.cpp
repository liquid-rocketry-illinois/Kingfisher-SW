#include "Ebyte_E22_900T22S.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#include "main.h"
#include "semphr.h"

static config_e22_900t22s e22_cfg;
static bool initialized = false;

/* Mutex for thread-safe radio access */

// Because we have multiple things that can't overlap,
// we can use FreeRTOS's mutex to handle everything.
// using the pair xSemaphoreTake() and xSemaphoreGive(),
// whatever code we put in between will not be run unless
// no other code is queued. This is useful because we
// don't want things like commands and data to overlap.
// If they did, it would cause bytes to be corrupted and
// give bad data. Here, the semaphore stuff prevents
// cases like config commands being sent, then in the
// middle of its execution, starting a data transfer.
static SemaphoreHandle_t e22_mutex = NULL;

/* ================= AUX HANDLING ================= */

// Use aux to detect transmitted data via wireless, or if
// data won't go through UART, or if modules are still
// initializing/etc.+
static inline bool auxHigh(void)
{
    return HAL_GPIO_ReadPin(
        e22_cfg.E22_AUX_PORT,
        e22_cfg.E22_AUX_PIN
    ) == GPIO_PIN_SET;
}

bool e22_isBusy(void)
{
    return !auxHigh();
}

int8_t waitAux_e22_900t22s(uint32_t timeout_ms)
{
    uint32_t start = xTaskGetTickCount();
    uint32_t p = xTaskGetTickCount();

    while(!auxHigh())
    {
        if((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
            return E22_ERR_TIMEOUT;
        }

        if ((xTaskGetTickCount() - p) > pdMS_TO_TICKS(300)) {
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
            p = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
    return E22_OK;
}

/* ================= MODE CONTROL ================= */

void changeMode(EBYTE_MODE mode)
{
    // Module will finish its current task (e.g. transmission) even if
    // the mode is switched. No delay is needed
    switch(mode)
    {
        case TRANS:
            HAL_GPIO_WritePin(e22_cfg.E22_M0_PORT, e22_cfg.E22_M0_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(e22_cfg.E22_M1_PORT, e22_cfg.E22_M1_PIN, GPIO_PIN_RESET);
            break;

        case WOR:
            HAL_GPIO_WritePin(e22_cfg.E22_M0_PORT, e22_cfg.E22_M0_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(e22_cfg.E22_M1_PORT, e22_cfg.E22_M1_PIN, GPIO_PIN_RESET);
            break;

        case CONFIG:
            HAL_GPIO_WritePin(e22_cfg.E22_M0_PORT, e22_cfg.E22_M0_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(e22_cfg.E22_M1_PORT, e22_cfg.E22_M1_PIN, GPIO_PIN_SET);
            break;

        case OFF:
            HAL_GPIO_WritePin(e22_cfg.E22_M0_PORT, e22_cfg.E22_M0_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(e22_cfg.E22_M1_PORT, e22_cfg.E22_M1_PIN, GPIO_PIN_SET);
            break;
    }

    // Wait until the module has processed the mode switch
    waitAux_e22_900t22s(200);
}

/* ================= INITIALIZATION ================= */

int8_t init_e22_900t22s(config_e22_900t22s *cfg)
{
    if (cfg == NULL)
        return E22_ERR_INVALID_PARAM;

    /* copy desired configuration locally */
    e22_cfg = *cfg;

    /* ensure mutex exists */
    if (e22_mutex == NULL)
        e22_mutex = xSemaphoreCreateMutex();

    if (e22_mutex == NULL)
        return E22_ERR_BUSY;

    // Pretty sure it's supposed to be active-low, so this should turn the module on.
    HAL_GPIO_WritePin(RADIO_RST_GPIO_Port, RADIO_RST_Pin, GPIO_PIN_SET);

    /* ensure radio is ready */
    if (waitAux_e22_900t22s(5000) != E22_OK)
        return E22_ERR_TIMEOUT;

    /* switch to configuration mode */
    changeMode(CONFIG);
    // for safety
    if (waitAux_e22_900t22s(1000) != E22_OK)
        return E22_ERR_TIMEOUT;

    /* read current radio configuration */
    config_e22_900t22s current_cfg;
    // TODO stuck here. uart timing issue? cubemx config issue? uart read result weird
    if (readConfig_e22_900t22s(&current_cfg) != E22_OK)
        return E22_ERR_UART;

    /* compare relevant fields */
    bool config_matches =
        (current_cfg.ADDH == cfg->ADDH)   &&
        (current_cfg.ADDL == cfg->ADDL)   &&
        (current_cfg.REG0 == cfg->REG0)   &&
        (current_cfg.REG2 == cfg->REG2)   &&
        (current_cfg.REG1 == cfg->REG1);

    /* only write if configuration differs */
    if (!config_matches)
    {
        if (writeConfig_e22_900t22s(cfg, true) != E22_OK)
            return E22_ERR_UART;

        if (waitAux_e22_900t22s(1000) != E22_OK)
            return E22_ERR_TIMEOUT;
    }

    /* return to normal transmit mode */
    changeMode(TRANS);

    if (waitAux_e22_900t22s(1000) != E22_OK)
        return E22_ERR_TIMEOUT;

    initialized = true;

    return E22_OK;
}

// Keep this
bool e22_initialized(void)
{
    return initialized;
}

/* ================= UART HELPERS ================= */

#include "usart.h"

static int8_t uartWrite(uint8_t *data, uint16_t len)
{
    if(HAL_UART_Transmit(
        &huart8,// e22_cfg.huart,
        data,
        len,
        5000) != HAL_OK)
        return E22_ERR_UART;

    return E22_OK;
}

static int8_t uartRead(uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_UART_Receive(
        &huart8, // e22_cfg.huart,
        data,
        len,
        5000); // Make a generous amount of time
    if(status != HAL_OK) return E22_ERR_UART;

    return E22_OK;
}

/* ================= CONFIGURATION ================= */

// save_to_flash overwrites the factory default settings.
// Set to TRUE when deploying finalized code!
int8_t writeConfig_e22_900t22s(
    const config_e22_900t22s *cfg,
    bool save_to_flash)
{
    uint8_t frame[9]; // Overcompensate? maybe set to 32 or 64

    // Head command byte
    frame[0] = save_to_flash ?
        COMMAND_BYTE_WRITE_CFG_SAVE_FLASH :
        COMMAND_BYTE_WRITE_CFG_NOSAVE_FLASH;
    frame[1] = 0x00;
    frame[2] = 0x06;
    // REG3 is for more advanced functions and is not included
    frame[3] = cfg->ADDH;
    frame[4] = cfg->ADDL;
    frame[5] = cfg->NETID;
    frame[6] = cfg->REG0;
    frame[7] = cfg->REG1;
    frame[8] = cfg->REG2;

    xSemaphoreTake(e22_mutex, portMAX_DELAY);

    changeMode(CONFIG);

    // Construct and write config commands
    int8_t status = uartWrite(frame, 10);
    if(status != E22_OK)
    {
        xSemaphoreGive(e22_mutex);
        return status;
    }

    waitAux_e22_900t22s(200);

    changeMode(TRANS);

    e22_cfg = *cfg;

    xSemaphoreGive(e22_mutex);

    return E22_OK;
}

int8_t readConfig_e22_900t22s(config_e22_900t22s *cfg)
{
    uint8_t cmd[3];
    cmd[0] = COMMAND_BYTE_READ_CFG;
    cmd[1] = 0x00; // Start from ADDH
    cmd[2] = 0x06; // Read all necessary registers
    uint8_t resp[9];
    // resp: {c1, 00, 06, addh, addl, netid, reg0, reg1, reg2}

    xSemaphoreTake(e22_mutex, portMAX_DELAY);

    changeMode(CONFIG);

    waitAux_e22_900t22s(1000);
    uartWrite(cmd,3);

    waitAux_e22_900t22s(1000);
    if(uartRead(resp,12) != E22_OK)
    {
        xSemaphoreGive(e22_mutex);
        return E22_ERR_UART;
    }

    changeMode(TRANS);

    // first three values are just repetition of sent data
    if (resp[0] != cmd[0] ||
        resp[1] != cmd[1] ||
        resp[2] != cmd[2])
        static int8_t E22_INIT_WARNING = E22_ERR_DATA_VERIFICATION; // sohws up in debugger if flagged

    cfg->ADDH   = resp[3];
    cfg->ADDL   = resp[4];
    cfg->NETID  = resp[5];
    cfg->REG0   = resp[6];
    cfg->REG1   = resp[7];
    cfg->REG2   = resp[8];

    xSemaphoreGive(e22_mutex);

    return E22_OK;
}

/* ================= DATA TRANSMISSION ================= */

int8_t transmit_e22_900t22s(uint8_t *data, size_t length)
{
    if(!initialized)
        return E22_ERR_NOT_INITIALIZED;

    xSemaphoreTake(e22_mutex, portMAX_DELAY);

    waitAux_e22_900t22s(200);

    int8_t status = uartWrite(data, length);

    waitAux_e22_900t22s(200);

    xSemaphoreGive(e22_mutex);

    return status;
}

/* ================= FIXED TRANSMISSION ================= */

int8_t transmit_fixed_e22_900t22s(
    uint16_t address,
    uint8_t channel,
    uint8_t *data,
    size_t length)
{
    if(!initialized)
        return E22_ERR_NOT_INITIALIZED;

    uint8_t frame[3 + length];

    frame[0] = (address >> 8);
    frame[1] = (address & 0xFF);
    frame[2] = channel;

    memcpy(&frame[3], data, length);

    return transmit_e22_900t22s(frame, length + 3);
}

/* ================= RECEIVE ================= */

bool e22_available(void)
{
    return !e22_isBusy();
}

int16_t recieve_e22_900t22s(uint8_t *buffer, size_t max_length)
{
    if(!initialized)
        return E22_ERR_NOT_INITIALIZED;

    if(e22_isBusy())
        return 0;

    if(HAL_UART_Receive(
        e22_cfg.huart,
        buffer,
        max_length,
        10) != HAL_OK)
        return E22_ERR_UART;

    return max_length;
}

/* ================= ADDRESS ================= */

void setAddress_e22_900t22s(uint16_t address)
{
    e22_cfg.ADDH = (address >> 8);
    e22_cfg.ADDL = (address & 0xFF);

    writeConfig_e22_900t22s(&e22_cfg,false);
}

uint16_t getAddress_e22_900t22s(void)
{
    return (e22_cfg.ADDH << 8) | e22_cfg.ADDL;
}

/* ================= CHANNEL ================= */

void changeOpFreq_e22_900t22s(R2_E22Channel915 channel)
{
    e22_cfg.REG2 = channel;
    writeConfig_e22_900t22s(&e22_cfg,false);
}

R2_E22Channel915 getOpFreq_e22_900t22s(void)
{
    return (R2_E22Channel915)e22_cfg.REG2;
}

/* ================= PARAMETER SETTERS ================= */

void setAirDataRate_e22_900t22s(R0_210_E22_AIR_DATA_RATE rate)
{
    e22_cfg.REG0 &= ~0x07;
    e22_cfg.REG0 |= rate;

    writeConfig_e22_900t22s(&e22_cfg,false);
}

void setUARTBaud_e22_900t22s(R0_765_E22_UART_BAUD baud)
{
    e22_cfg.REG0 &= ~(0b111 << 5);
    e22_cfg.REG0 |= baud;

    writeConfig_e22_900t22s(&e22_cfg,false);
}

void setTxPower_e22_900t22s(R1_10_E22_TX_POWER power)
{
    e22_cfg.REG1 &= ~0x03;
    e22_cfg.REG1 |= power;

    writeConfig_e22_900t22s(&e22_cfg,false);
}

/* ================= RESET ================= */

void reset_e22_900t22s(void)
{
    uint8_t cmd = COMMAND_BYTE_RESET_MODULE;

    changeMode(CONFIG);

    uartWrite(&cmd,1);

    waitAux_e22_900t22s(500);

    changeMode(TRANS);
}
#include "Telemetry.h"
#include "usart.h"
#include <cstring>
#include "task.h"

config_e22_900t22s Telemetry::des_cfg;

Telemetry::Telemetry(TelemetryMode mode) : mode(mode)
{
    lastSeq = 0;
}

uint8_t Telemetry::Init()
{
    des_cfg.huart = &huart8;

    des_cfg.E22_AUX_PIN  = RADIO_AUX_Pin;
    des_cfg.E22_AUX_PORT = RADIO_AUX_GPIO_Port;
    des_cfg.E22_M0_PIN   = M0_Radio_Pin;
    des_cfg.E22_M0_PORT  = M0_Radio_GPIO_Port;
    des_cfg.E22_M1_PIN   = M1_Radio_Pin;
    des_cfg.E22_M1_PORT  = M1_Radio_GPIO_Port;

    // address differs by mode so both can coexist on the same channel
    if(mode == TELEMETRY_MODE_FLIGHT)
    {
        des_cfg.ADDH = HAL1_RADIO_ADDRHIGH;
        des_cfg.ADDL = HAL1_RADIO_ADDRLOW;
    }
    else
    {
        des_cfg.ADDH = GND_RADIO_ADDRHIGH;
        des_cfg.ADDL = GND_RADIO_ADDRLOW;
    }

    des_cfg.NETID = HAL1_RADIO_NETID;

    des_cfg.REG0 =   R0_765_E22_UART_BAUD::E22_UART_BAUD_9600
                   | R0_43_SERIAL_PORT_PARITY_BIT::MODE_8N1
                   | R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_9_6K;

    des_cfg.REG1 =   R1_76_SUB_PACKET_SETTING::BYTES_240
                   | R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_DISABLE
                   | R1_2_SOFTWARE_MODE_SWITCHING_OFF
                   | R1_10_E22_TX_POWER::E22_TX_POWER_22DBM;

    des_cfg.REG2 = CH915;

    des_cfg.REG3 =   R3_7_RSSI_BYTE_DISABLE
                   | R3_6_TRANSFER_METHOD_FIXED_POINT
                   | R3_5_REPEATER_OFF
                   | R3_4_LBT_DISABLED
                   | R3_3_WOR_MODE_RECIEVER
                   | R3_210_WOR_CYCLE_TIME::_2000_ms;

    int8_t status = init_e22_900t22s(&des_cfg);
    if(status != 0)
        return 1;

    changeMode(TRANS);
    vTaskDelay(pdMS_TO_TICKS(400));
    return 0;
}

uint8_t Telemetry::Update()
{
    uint8_t status = 0;

    if(mode == TELEMETRY_MODE_FLIGHT)
    {
        // FC: receive commands from ground, send telemetry up
        status = receiveCommands(GNDOutData);

        // TODO: populate from sensors
        HALOutData.altitude   = 1.0F;
        HALOutData.longitude  = 2.0F;
        HALOutData.latitude   = 3.0F;
        HALOutData.GPSaltitude = 4.0F;
        HALOutData.mAccX = 10.0F; HALOutData.mAccY = 10.0F; HALOutData.mAccZ = 10.0F;
        HALOutData.bAccX = 10.0F; HALOutData.bAccY = 10.0F; HALOutData.bAccZ = 10.0F;
        HALOutData.pitch = 10.0F; HALOutData.yaw   = 10.0F; HALOutData.roll  = 10.0F;
        HALOutData.servoPos1 = 50.0F;
        HALOutData.servoPos2 = 60.0F;
        HALOutData.keepAliveStatus = GNDOutData.keepAliveIn;

        status = sendData(HALOutData);
    }
    else
    {
        // GND: receive telemetry from FC, send commands down
        status = receiveTelemetry(HALOutData);

        // TODO: populate GNDOutData from operator input
        // GNDOutData.keepAliveIn = ...; // not sending anything for now
        GNDOutData.pyroActivation[1] = PYRODROGUEBKP;

        status = sendCommands(GNDOutData);
    }

    return status;
}

// ---------------------------------------------------------------------------
// shared encode/send — works for any struct
// ---------------------------------------------------------------------------

template<typename T>
uint8_t Telemetry::encodeAndSend(const T &payload)
{
    static_assert(sizeof(T) <= TELEMETRY_MAX_PAYLOAD - 10, "payload exceeds TX buffer size");
    uint8_t payload_len = sizeof(T);

    // fixed mode header — module strips these before delivering to receiver
    if (mode == TELEMETRY_MODE_FLIGHT) {
        tx_buffer[0] = GND_RADIO_ADDRHIGH;
        tx_buffer[1] = GND_RADIO_ADDRLOW;
    }
    else{
        tx_buffer[0] = HAL1_RADIO_ADDRHIGH;
        tx_buffer[1] = HAL1_RADIO_ADDRLOW;
    }

    tx_buffer[2] = CH915;

    // your packet starts at offset 3
    tx_buffer[3] = TELEMETRY_SYNC1;
    tx_buffer[4] = TELEMETRY_SYNC2;
    tx_buffer[5] = payload_len;
    tx_buffer[6] = lastSeq & 0xFF;
    tx_buffer[7] = (lastSeq >> 8) & 0xFF;

    memcpy(&tx_buffer[8], &payload, payload_len);

    uint16_t crc = Checksum(&tx_buffer[3], 5 + payload_len);  // start at SYNC1
    tx_buffer[8 + payload_len] = crc & 0xFF;
    tx_buffer[9 + payload_len] = (crc >> 8) & 0xFF;

    // Not using transmit_fixed
    int8_t status = transmit_e22_900t22s(tx_buffer, 10 + payload_len);
    if(status != E22_OK)
        return status;

    lastSeq++;
    return 0;
}

// ---------------------------------------------------------------------------
// shared decode — works for any struct
// ---------------------------------------------------------------------------

template<typename T>
int8_t Telemetry::decodeData(T &payload)
{
    uint8_t s = rx_buffer[0];
    uint8_t s2 = rx_buffer[1];
    if(rx_buffer[0] != TELEMETRY_SYNC1) return -1;
    if(rx_buffer[1] != TELEMETRY_SYNC2) return -2;

    uint8_t payload_len = rx_buffer[2];

    if(payload_len != sizeof(T))              return -3;
    if((5 + payload_len + 2) > TELEMETRY_MAX_PAYLOAD) return -4;

    uint16_t seq_rx = rx_buffer[3] | (rx_buffer[4] << 8);

    uint16_t crc_rx   = rx_buffer[5 + payload_len] | (rx_buffer[6 + payload_len] << 8);
    uint16_t crc_calc = Checksum(rx_buffer, 5 + payload_len);

    if(crc_rx != crc_calc) return -5;

    memcpy(&payload, &rx_buffer[5], sizeof(T));

    lastSeq = seq_rx;
    return 0;
}

// ---------------------------------------------------------------------------
// FC side
// ---------------------------------------------------------------------------

uint8_t Telemetry::sendData(const telemetryData &data)
{
    uint8_t status = encodeAndSend(data);
    return status;
}

uint8_t Telemetry::receiveCommands(GndStationData &gnd)
{
    bool a = e22_available();
    if(!a)
        return E22_NO_DATA;

    int16_t len = recieve_e22_900t22s(rx_buffer, sizeof(rx_buffer));
    if(len <= 0)                return E22_RECEIVE_ERR;
    if(len < 7)                 return E22_BAD_LENGTH;

    int8_t status = decodeData(gnd);
    if(status != 0)             return (uint8_t)status;

    processKeepalive(gnd.keepAliveIn);
    processPyros(gnd.pyroActivation);
    return 0;
}

void Telemetry::processKeepalive(uint8_t keepAliveIn)
{
    GNDOutData.keepAliveIn = keepAliveIn;

    if(keepAliveIn == SHUTDOWN_KEEPALIVE)
    {
        if(shutdownHoldStart == 0)
            shutdownHoldStart = HAL_GetTick();

        if((HAL_GetTick() - shutdownHoldStart) >= SHUTDOWN_HOLDOFF_MS)
            shutdownFlag = true;
    }
    else
    {
        shutdownHoldStart = 0;
        shutdownFlag = false;
    }
}

void Telemetry::processPyros(uint32_t pyroActivation[3])
{
    if(pyroActivation[0] == PYROMAIN) {
        HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(2000));
        HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_RESET);
    }
    if(pyroActivation[1] == PYRODROGUEBKP) {
        HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(2000));
        HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_RESET);
    }
    if(pyroActivation[2] == PYRODROGUEMAIN) {
        HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(2000));
        HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_RESET);
    }
}

// ---------------------------------------------------------------------------
// GND side
// ---------------------------------------------------------------------------

uint8_t Telemetry::sendCommands(const GndStationData &gnd)
{
    return encodeAndSend(gnd);
}

uint8_t Telemetry::receiveTelemetry(telemetryData &data)
{
    if(!e22_available())        return E22_NO_DATA;

    int16_t len = recieve_e22_900t22s(rx_buffer, sizeof(rx_buffer));
    if(len <= 0)                return E22_RECEIVE_ERR;
    if(len < 7)                 return E22_BAD_LENGTH;

    int8_t status = decodeData(data);
    if(status != 0)
        return (uint8_t)status;

    return 0;
}

// ---------------------------------------------------------------------------
// shared utils
// ---------------------------------------------------------------------------

uint16_t Telemetry::Checksum(uint8_t *data, uint16_t length)
{
    uint16_t sum = 0;
    for(uint16_t i = 0; i < length; i++)
        sum += data[i];
    return sum;
}

void Telemetry::Reconfigure(const config_e22_900t22s *cfg_new)
{
    writeConfig_e22_900t22s(cfg_new, true);
    des_cfg = *cfg_new;
}
//
// Created by dyrel on 3/12/2026.
//

#include "Telemetry.h"
#include "usart.h"
#include <cstring>

//TODO include all sensor files

config_e22_900t22s Telemetry::des_cfg;

Telemetry::Telemetry()
{
    seq = 0;
}

uint8_t Telemetry::Init()
{
    des_cfg.huart = &huart8;

    des_cfg.E22_AUX_PIN  = RADIO_AUX_Pin;
    des_cfg.E22_AUX_PORT = RADIO_AUX_GPIO_Port;

    des_cfg.E22_M0_PIN = M0_Radio_Pin;
    des_cfg.E22_M0_PORT = M0_Radio_GPIO_Port;

    des_cfg.E22_M1_PIN = M1_Radio_Pin;
    des_cfg.E22_M1_PORT = M1_Radio_GPIO_Port;

    // These are default values, we can use functions
    // to change them later.
    // See pg. 15 of datasheet
    des_cfg.ADDH = 0b00;
    des_cfg.ADDL = 0b00;

    // REG0 -> 9600 baud, 8N1 parity, 9600 wireless data rate
    des_cfg.REG0 =     R0_765_E22_UART_BAUD::E22_UART_BAUD_115200
                    | R0_43_SERIAL_PORT_PARITY_BIT::MODE_8N1
                    | R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_9_6K;
    // REG2 -> channel
    des_cfg.REG2 = CH915;
    // REG1 -> 240 bytes sub packet, off, reserved, off, 22dBm
    des_cfg.REG1 =    R1_76_SUB_PACKET_SETTING::BYTES_240
                    | R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_DISABLE
                    | R1_2_SOFTWARE_MODE_SWITCHING_OFF
                    | R1_10_E22_TX_POWER::E22_TX_POWER_22DBM;

    // Examples of changing params using functions
    // changeOpFreq_e22_900t22s(R2_E22Channel915::CH915);
    // setAirDataRate_e22_900t22s(R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_9_6K);
    // setUARTBaud_e22_900t22s(R0_765_E22_UART_BAUD::E22_UART_BAUD_115200);
    // setTxPower_e22_900t22s(R1_10_E22_TX_POWER::E22_TX_POWER_22DBM);

    if(init_e22_900t22s(&des_cfg) != 0)
        return 1;

    changeMode(TRANS);
    return 0;
}

uint8_t Telemetry::Update()
{
    static telemetryData t;
    static GndStationData gnd;

    receiveData(gnd);

    // TODO replace all values with current sensor values
    t.mAccX = 0.0F;
    t.mAccY = 0.0F;
    t.mAccZ = 0.0F;

    t.bAccX = 0.0F;
    t.bAccY = 0.0F;
    t.bAccZ = 0.0F;

    t.pitch = 0.0F;
    t.yaw   = 0.0F;
    t.roll  = 0.0F;

    t.servoPos1 = 0.0F;
    t.servoPos2 = 0.0F;

    // TODO Integrate RCI and RCP
    sendData(t);
    return 0;
}

uint8_t Telemetry::sendData(const telemetryData &data)
{
    uint16_t index = 0;

    tx_buffer[index++] = TELEMETRY_SYNC1;
    tx_buffer[index++] = TELEMETRY_SYNC2;

    tx_buffer[index++] = sizeof(telemetryData);   // payload length

    tx_buffer[index++] = (uint8_t)(seq & 0xFF);
    tx_buffer[index++] = (uint8_t)(seq >> 8);

    memcpy(&tx_buffer[index], &data, sizeof(telemetryData));
    index += sizeof(telemetryData);

    uint16_t crc = Checksum(tx_buffer, index);

    tx_buffer[index++] = crc & 0xFF;
    tx_buffer[index++] = crc >> 8;

    int8_t status = transmit_e22_900t22s(tx_buffer, index);
    if(status != E22_OK)
        return status;

    seq++;

    return 0;
}

uint8_t Telemetry::receiveData(GndStationData &gnd)
{
    if(!e22_available())
        return 0;

    int16_t len = recieve_e22_900t22s(rx_buffer, sizeof(rx_buffer));

    if(len <= 0)
        return 1;

    return decodeData(gnd);
}

uint8_t Telemetry::decodeData(GndStationData &gnd)
{
    if(rx_buffer[0] != TELEMETRY_SYNC1)
        return -1;

    if(rx_buffer[1] != TELEMETRY_SYNC2)
        return -2;

    uint8_t payload_len = rx_buffer[2];

    uint16_t seq_rx =
        rx_buffer[3] |
        (rx_buffer[4] << 8);

    uint16_t crc_rx =
        rx_buffer[5 + payload_len] |
        (rx_buffer[6 + payload_len] << 8);

    uint16_t crc_calc = Checksum(rx_buffer, 5 + payload_len);

    if(crc_rx != crc_calc)
        return -3;

    memcpy(&gnd, &rx_buffer[5], sizeof(GndStationData));

    /* update internal telemetry state */
    lastSeq = seq_rx;

    return 0;
}

uint16_t Telemetry::Checksum(uint8_t *data, uint16_t length)
{
    uint16_t sum = 0;

    for(uint16_t i = 0; i < length; i++)
        sum += data[i];

    return sum;
}

void Telemetry::Reconfigure(const config_e22_900t22s* cfg_new)
{
    writeConfig_e22_900t22s(cfg_new, true);
    des_cfg = *cfg_new;
}
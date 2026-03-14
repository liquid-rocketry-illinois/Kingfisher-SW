//
// Created by dyrel on 3/12/2026.
//

#include "Telemetry.h"
#include "usart.h"
#include <cstring>

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

    // These are default config values, so we can use functions
    // to change them later.
    // See pg. 15 of datasheet
    des_cfg.addh = 0x00;
    des_cfg.addl = 0x00;
    des_cfg.head = 0xC0; // Write config to radio. C1 and C2 are read
                         // and write temporarily, respectively
    // REG0 -> 9600 baud, 8N1 parity, 9600 wireless data rate
    // using 0bXXXXX instead of 0xXXXXXX for readability (binary form)
    des_cfg.sped = 0b01100100; // baud rate (3), parity (2), air rate (3)
    // REG2 -> channel
    des_cfg.chan = CH915;
    // REG1 -> 240 bytes sub packet, off, reserved, off, 22dBm
    des_cfg.option = 0b0001000; // sub packet setting, RSSI toggle, reserve, Software M switch, TX Power
    //TODO write in REG3??
    if(init_e22_900t22s(&des_cfg) != 0)
        return 1;

    changeMode(TRANS);

    changeOpFreq_e22_900t22s(CH915);

    setAirDataRate_e22_900t22s(E22_AIR_RATE_9_6K);
    setUARTBaud_e22_900t22s(E22_UART_BAUD_115200);
    setTxPower_e22_900t22s(E22_TX_POWER_22DBM);

    return 0;
}

uint8_t Telemetry::Update()
{
    sendData();
    receiveData();

    return 0;
}

uint8_t Telemetry::sendData()
{
    uint8_t index = 0;

    tx_buffer[index++] = TELEMETRY_SYNC1;
    tx_buffer[index++] = TELEMETRY_SYNC2;

    tx_buffer[index++] = (uint8_t)(seq & 0xFF);
    tx_buffer[index++] = (uint8_t)(seq >> 8);

    // Example payload (replace with sensor data)
    int16_t testValue = 1234;

    memcpy(&tx_buffer[index], &testValue, sizeof(testValue));
    index += sizeof(testValue);

    uint16_t crc = Checksum(tx_buffer, index);

    tx_buffer[index++] = crc & 0xFF;
    tx_buffer[index++] = crc >> 8;

    transmit_e22_900t22s(tx_buffer, index);

    seq++;

    return 0;
}

uint8_t Telemetry::receiveData()
{
    if(!e22_available())
        return 0;

    int16_t len = recieve_e22_900t22s(rx_buffer, sizeof(rx_buffer));

    if(len <= 0)
        return 1;

    return decodeData();
}

uint8_t Telemetry::decodeData()
{
    if(rx_buffer[0] != TELEMETRY_SYNC1)
        return -1;

    if(rx_buffer[1] != TELEMETRY_SYNC2)
        return -2;

    uint16_t seq_rx =
        rx_buffer[2] |
        (rx_buffer[3] << 8);

    uint16_t crc_rx =
        rx_buffer[6] |
        (rx_buffer[7] << 8);

    uint16_t crc_calc = Checksum(rx_buffer, 6);

    // Checksum bad
    if(crc_rx != crc_calc)
        return -3;

    // payload example
    int16_t value;

    memcpy(&value, &rx_buffer[4], sizeof(value));

    // now you have decoded telemetry data

    return 0;
}

uint16_t Telemetry::Checksum(uint8_t *data, uint16_t length)
{
    uint16_t sum = 0;

    for(uint16_t i = 0; i < length; i++)
        sum += data[i];

    return sum;
}

uint8_t Telemetry::Reconfigure(config_e22_900t22s cfg_new)
{
    writeConfig_e22_900t22s(&cfg_new, true);

    des_cfg = cfg_new;

    return 0;
}
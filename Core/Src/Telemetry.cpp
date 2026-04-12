//
// Created by dyrel on 3/12/2026.
//

#include "Telemetry.h"
#include "usart.h"
#include <cstring>

#include "task.h"

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

    // See pg. 15 of datasheet
    des_cfg.ADDH = HAL1_RADIO_ADDRHIGH;
    des_cfg.ADDL = HAL1_RADIO_ADDRLOW;
    des_cfg.NETID = HAL1_RADIO_NETID;

    // REG0 -> 9600 baud, 8N1 parity, 9600 wireless data rate
    des_cfg.REG0 =     R0_765_E22_UART_BAUD::E22_UART_BAUD_9600
                    | R0_43_SERIAL_PORT_PARITY_BIT::MODE_8N1
                    | R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_9_6K;
    // REG1 -> 240 bytes sub packet, off, reserved, off, 22dBm
    des_cfg.REG1 =    R1_76_SUB_PACKET_SETTING::BYTES_240
                    | R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_DISABLE
    // bits 4, 3: reserved and not usable
                    | R1_2_SOFTWARE_MODE_SWITCHING_OFF
                    | R1_10_E22_TX_POWER::E22_TX_POWER_22DBM;
    // REG2 -> channel
    des_cfg.REG2 = CH915; // Change according to radio band list at IREC
    des_cfg.REG3 =      R3_7_RSSI_BYTE_ENABLE
                    |   R3_6_TRANSFER_METHOD_TRANSPARENT
                    |   R3_5_REPEATER_OFF
                    |   R3_4_LBT_DISABLED
                    |   R3_3_WOR_MODE_RECIEVER
                    |   R3_210_WOR_CYCLE_TIME::_2000_ms;

    // Examples of changing params using functions
    // changeOpFreq_e22_900t22s(R2_E22Channel915::CH915);
    // setAirDataRate_e22_900t22s(R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_9_6K);
    // setUARTBaud_e22_900t22s(R0_765_E22_UART_BAUD::E22_UART_BAUD_115200);
    // setTxPower_e22_900t22s(R1_10_E22_TX_POWER::E22_TX_POWER_22DBM);

    int8_t status = init_e22_900t22s(&des_cfg);
    if(status != 0)
        return 1;

    changeMode(TRANS);

    // Finish config and delay to init
    vTaskDelay(pdMS_TO_TICKS(400));
    return 0;
}

uint8_t Telemetry::Update()
{
    static telemetryData t;
    static GndStationData gnd;

    receiveData(gnd);

    // TODO replace all values with current sensor values
    t.altitude = 0.0F;

    t.longitude = 0.0F;
    t.latitude = 0.0F;
    t.GPSaltitude = 0.0F;

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

// Send a packet in the format of telemetryData. Each value inside of the
// 'data' object should be appended to the packet and sent.
uint8_t Telemetry::sendData(const telemetryData &data)
{
    static_assert(sizeof(telemetryData) <= TELEMETRY_MAX_PAYLOAD - 7, "telemetryData exceeds TX buffer size");

    uint8_t payload_len = sizeof(telemetryData);

    tx_buffer[0] = TELEMETRY_SYNC1;
    tx_buffer[1] = TELEMETRY_SYNC2;
    tx_buffer[2] = payload_len;
    tx_buffer[3] = lastSeq & 0xFF;         // seq_lo
    tx_buffer[4] = (lastSeq >> 8) & 0xFF;  // seq_hi

    memcpy(&tx_buffer[5], &data, payload_len);

    uint16_t crc = Checksum(tx_buffer, 5 + payload_len);
    tx_buffer[5 + payload_len] = crc & 0xFF;        // crc_lo
    tx_buffer[6 + payload_len] = (crc >> 8) & 0xFF; // crc_hi

    int8_t status = transmit_e22_900t22s(tx_buffer, 7 + payload_len);
    if(status != E22_OK)
        return status;

    lastSeq++;
    return 0;
}

uint8_t Telemetry::receiveData(GndStationData &gnd)
{
    if(!e22_available())
        return E22_NO_DATA;

    int16_t len = recieve_e22_900t22s(rx_buffer, sizeof(rx_buffer));

    if(len <= 0)
        return E22_RECEIVE_ERR;

    // sanity check: must have at least the 7 bytes of overhead
    if(len < 7)
        return E22_BAD_LENGTH;

    return decodeData(gnd);
}

int8_t Telemetry::decodeData(GndStationData &gnd)
{
    if(rx_buffer[0] != TELEMETRY_SYNC1)
        return -1;

    if(rx_buffer[1] != TELEMETRY_SYNC2)
        return -2;

    uint8_t payload_len = rx_buffer[2];

    // ensure payload won't read out of bounds
    if(payload_len != sizeof(GndStationData))
        return -3;

    if((5 + payload_len + 2) > RX_BUFFER_SIZE)
        return -4;

    uint16_t seq_rx =
        rx_buffer[3] |
        (rx_buffer[4] << 8);

    uint16_t crc_rx =
        rx_buffer[5 + payload_len] |
        (rx_buffer[6 + payload_len] << 8);

    uint16_t crc_calc = Checksum(rx_buffer, 5 + payload_len);

    if(crc_rx != crc_calc)
        return -5;

    memcpy(&gnd, &rx_buffer[5], sizeof(GndStationData));

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
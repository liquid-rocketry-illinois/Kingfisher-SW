#include "Telemetry.h"
#include "usart.h"
#include "task.h"
#include <cstring>

#include "constants.h"
#include "CTRLS_Controls.h"

Telemetry::Telemetry()
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

    des_cfg.ADDH = HAL1_RADIO_ADDRHIGH;
    des_cfg.ADDL = HAL1_RADIO_ADDRLOW;

    des_cfg.NETID = HAL1_RADIO_NETID;

    des_cfg.REG0 =   R0_765_E22_UART_BAUD::E22_UART_BAUD_38400
                   | R0_43_SERIAL_PORT_PARITY_BIT::MODE_8N1
                   | R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_9_6K;

    des_cfg.REG1 =   R1_76_SUB_PACKET_SETTING::BYTES_240
                   | R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_ENABLE
                   | R1_2_SOFTWARE_MODE_SWITCHING_OFF
                   | R1_10_E22_TX_POWER::E22_TX_POWER_22DBM;

    des_cfg.REG2 = CH915;

    des_cfg.REG3 =   R3_7_RSSI_BYTE_ENABLE
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
    // FC slave: receive GND command first, then respond only when asked.
    // REQUEST_DATA_BYTE    → send full telemetry packet
    // HANDSHAKE_GND_BYTE   → send (CommandResponseByte already set by processCmd)
    // SHUTDOWN_KEEPALIVE   → send telemetry so GND stays informed during abort
    // BYTE_DEFLECT_TEST    → send telemetry to confirm deflection test started
    // SERVO_OFFSET_CMD_BYTE→ send telemetry to confirm offset received
    // anything else        → receive only, no TX (GND is rate-limiting)
    uint8_t status = receiveCommands(GNDOutData);
    if (status == 0)
    {
        uint8_t cmd = GNDOutData.CommandByte;
        if (cmd == REQUEST_DATA_BYTE   ||
            cmd == HANDSHAKE_GND_BYTE  ||
            cmd == SHUTDOWN_KEEPALIVE  ||
            cmd == BYTE_DEFLECT_TEST   ||   // 0x0C -- was DEFLECT_TEST(150) in old switch
            cmd == SERVO_OFFSET_CMD_BYTE)
        {
            status = sendData(HALOutData);
        }
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
    tx_buffer[0] = GND_RADIO_ADDRHIGH;
    tx_buffer[1] = GND_RADIO_ADDRLOW;

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
  int8_t Telemetry::decodeData(T &payload, uint16_t buf_len)
{
    if (buf_len < 7u) return -1; // too short to contain any valid frame

    // Search only within bytes actually received — prevents false sync matches
    // against stale buffer content from prior receives.
    const uint16_t search_end = buf_len - 1u;

    int16_t sync_idx = -1;
    for(int16_t i = 0; i < (int16_t)search_end; i++)
    {
        if(rx_buffer[i] == TELEMETRY_SYNC1 && rx_buffer[i + 1] == TELEMETRY_SYNC2)
        {
            sync_idx = i;
            break;
        }
    }

    if(sync_idx == -1)
        return -1;  // sync bytes not found in received data

    if((uint16_t)(sync_idx + 5) >= buf_len)
        return -2;  // not enough room for header after sync

    uint8_t payload_len = rx_buffer[sync_idx + 2];

    if(payload_len != sizeof(T))
        return -3;

    if((uint16_t)(sync_idx + 5 + payload_len + 2) > buf_len)
        return -4;

    uint16_t seq_rx = rx_buffer[sync_idx + 3] | (rx_buffer[sync_idx + 4] << 8);

    uint16_t crc_rx =
        rx_buffer[sync_idx + 5 + payload_len] |
        (rx_buffer[sync_idx + 6 + payload_len] << 8);

    uint16_t crc_calc = Checksum(&rx_buffer[sync_idx], 5 + payload_len);

    if(crc_rx != crc_calc)
        return -5;

    memcpy(&payload, &rx_buffer[sync_idx + 5], sizeof(T));

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
    // No e22_available() gate — FC is slave and must always be in receive mode
    // so GND's bytes land in HAL_UART_Receive before the 16-byte FIFO overflows.
    int16_t len = recieve_e22_900t22s(rx_buffer, sizeof(GndStationData));
    if(len <= 0)                return E22_RECEIVE_ERR;
    if(len < 7)                 return E22_BAD_LENGTH;

    int8_t status = decodeData(gnd, len);

    if(status != 0)             return (uint8_t)status;

    lastRSSI = get_rssi_e22_900t22s();
    HALOutData.RSSI = lastRSSI;

    processCmd(gnd.CommandByte);
    processPyros(gnd.pyroActivation);
    return 0;
}

#include "CTRLS_Controls.h"

void Telemetry::processCmd(uint8_t cmd)
{
    if (cmd == SHUTDOWN_KEEPALIVE) {
        shutdownFlag    = true;
        g_ctrls_enabled = false;  // zero servo outputs immediately on abort
    }

    if (cmd == HANDSHAKE_GND_BYTE) HALOutData.CommandResponseByte = HANDSHAKE_FC_BYTE;

    // BYTE_DEFLECT_TEST (12): ground-commanded servo test — enable controls output.
    // Controls are otherwise managed by the flight state machine via g_ctrls_enabled.
    if (cmd == BYTE_DEFLECT_TEST) g_ctrls_enabled = true;
}

void Telemetry::processPyros(uint32_t pyroActivation)
{
    // Set pending bits — PyroTask fires the GPIO in its own context so the
    // Radio task is never blocked during the 2-second hold.
    if (pyroActivation == PYROMAIN)       g_pyroPending |= PYRO_MAIN_BIT;
    if (pyroActivation == PYRODROGUEBKP)  g_pyroPending |= PYRO_DROGUE_BKP_BIT;
    if (pyroActivation == PYRODROGUEMAIN) g_pyroPending |= PYRO_DROGUE_MAIN_BIT;
}


// UTILS

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
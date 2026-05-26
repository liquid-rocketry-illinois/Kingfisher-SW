#include "Telemetry.h"
#include "usart.h"
#include "task.h"
#include <cstring>

#include "constants.h"
#include "CTRLS_Controls.h"

// g_gndData is defined in FlightComputer_SENSORS.cpp; updated here after every
// successful receive so the CTRLs task always has current servo offsets.
extern GndStationData g_gndData;

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

// ---------------------------------------------------------------------------
// FC is the slave: receive GND command first, then always send telemetry back.
// GND transmits before it tries to receive; both sides must agree on this order
// or they deadlock waiting on each other's RX.
// ---------------------------------------------------------------------------
uint8_t Telemetry::Update()
{
    uint8_t rx_status = receiveCommands(GNDOutData);

    // Always send telemetry back.
    // GND has already transmitted and is waiting on our response.
    sendData(HALOutData);
    return rx_status;
}

// ---------------------------------------------------------------------------
// Encode + transmit — FC sends telemetryData, GND receives it.
// Frame layout (matching GND's decodeData exactly):
//   [0]   GND_RADIO_ADDRHIGH  \  fixed-point routing header —
//   [1]   GND_RADIO_ADDRLOW    > stripped by E22 module before delivery,
//   [2]   CH915               /  not seen by the remote MCU's UART
//   [3]   SYNC1 (0xAA)        \  start sync pair
//   [4]   SYNC2 (0x55)        /
//   [5]   payload_len
//   [6]   seq lo
//   [7]   seq hi
//   [8 .. 8+N-1]  payload (N bytes)
//   [8+N] CRC lo
//   [9+N] CRC hi
//   [10+N] SYNC2 (0x55)       \  end sync pair — SYNC2 first,
//   [11+N] SYNC1 (0xAA)       /  SYNC1 is the final byte of the packet
// ---------------------------------------------------------------------------
template<typename T>
uint8_t Telemetry::encodeAndSend(const T &payload)
{
    static_assert(sizeof(T) <= TELEMETRY_MAX_PAYLOAD - 12, "payload exceeds TX buffer size");
    uint8_t payload_len = sizeof(T);

    tx_buffer[0] = GND_RADIO_ADDRHIGH;
    tx_buffer[1] = GND_RADIO_ADDRLOW;
    tx_buffer[2] = CH915;

    tx_buffer[3] = TELEMETRY_SYNC1;
    tx_buffer[4] = TELEMETRY_SYNC2;
    tx_buffer[5] = payload_len;
    tx_buffer[6] = lastSeq & 0xFF;
    tx_buffer[7] = (lastSeq >> 8) & 0xFF;

    memcpy(&tx_buffer[8], &payload, payload_len);

    uint16_t crc = Checksum(&tx_buffer[3], 5 + payload_len);  // covers SYNC1 → end of payload
    tx_buffer[8 + payload_len]  = crc & 0xFF;
    tx_buffer[9 + payload_len]  = (crc >> 8) & 0xFF;

    // Ending sync pair: SYNC2 then SYNC1 (SYNC1 is the final byte)
    tx_buffer[10 + payload_len] = TELEMETRY_SYNC2;
    tx_buffer[11 + payload_len] = TELEMETRY_SYNC1;

    int8_t status = transmit_e22_900t22s(tx_buffer, 12 + payload_len);
    if(status != E22_OK)
        return status;

    lastSeq++;
    return 0;
}

// ---------------------------------------------------------------------------
// Decode a received frame from rx_buffer into payload.
// buf_len is the number of bytes actually returned by recieve_e22_900t22s(),
// bounding the sync search to valid data so stale buffer bytes cannot match.
// Matches GND's Radio::decodeData exactly — all checks enabled.
//
// Expected frame layout (offsets relative to sync_idx):
//   [+0]  SYNC1 (0xAA)        \  start sync pair
//   [+1]  SYNC2 (0x55)        /
//   [+2]  payload_len
//   [+3]  seq lo
//   [+4]  seq hi
//   [+5 .. +4+N]  payload (N bytes)
//   [+5+N] CRC lo
//   [+6+N] CRC hi
//   [+7+N] SYNC2 (0x55)       \  end sync pair — SYNC2 first,
//   [+8+N] SYNC1 (0xAA)       /  SYNC1 is the final byte of the packet
// ---------------------------------------------------------------------------
template<typename T>
int8_t Telemetry::decodeData(T &payload, uint16_t buf_len)
{
    // Minimum frame: 5-byte header + 0 payload + 2-byte CRC + 2-byte end sync = 9 bytes
    if (buf_len < 9u) return -1;

    const uint16_t search_end = buf_len - 1u;

    // Locate start sync pair (SYNC1 followed by SYNC2)
    int16_t sync_idx = -1;
    for(int16_t i = 0; i < static_cast<int16_t>(search_end); i++)
    {
        if(rx_buffer[i] == TELEMETRY_SYNC1 && rx_buffer[i + 1] == TELEMETRY_SYNC2)
        {
            sync_idx = i;
            break;
        }
    }

    if(sync_idx == -1)
        return -1;  // start sync bytes not found

    if((uint16_t)(sync_idx + 5) >= buf_len)
        return -2;  // not enough bytes for the 5-byte header after sync

    uint8_t payload_len = rx_buffer[sync_idx + 2];

    if(payload_len != sizeof(T))
        return -3;  // length mismatch — wrong packet type or struct size skew

    // Full packet (5-byte header + payload + 2-byte CRC + 2-byte end sync) must fit.
    if((uint16_t)(sync_idx + 9 + payload_len) > buf_len)
        return -4;  // truncated — not all bytes arrived

    // Validate end sync bytes at their deterministic positions.
    // End sync is SYNC2 then SYNC1; SYNC1 is the final byte of the packet.
    if(rx_buffer[sync_idx + 7 + payload_len] != TELEMETRY_SYNC2 ||
       rx_buffer[sync_idx + 8 + payload_len] != TELEMETRY_SYNC1)
        return -6;  // end sync bytes missing or mismatched

    uint16_t seq_rx = rx_buffer[sync_idx + 3] | (rx_buffer[sync_idx + 4] << 8);

    uint16_t crc_rx =
        rx_buffer[sync_idx + 5 + payload_len] |
        (rx_buffer[sync_idx + 6 + payload_len] << 8);

    uint16_t crc_calc = Checksum(&rx_buffer[sync_idx], 5 + payload_len);

    if(crc_rx != crc_calc)
        return -5;  // corrupt frame — discard

    memcpy(&payload, &rx_buffer[sync_idx + 5], sizeof(T));

    lastSeq = seq_rx;
    return 0;
}

// ---------------------------------------------------------------------------
// FC side — public wrappers
// ---------------------------------------------------------------------------

uint8_t Telemetry::sendData(const telemetryData &data)
{
    return encodeAndSend(data);
}

uint8_t Telemetry::receiveCommands(GndStationData &gnd)
{
    // Pass the full rx_buffer size so the entire framed packet is read.
    // The framing overhead is 10 bytes (3-byte E22 header + 5-byte frame header
    // + 2-byte CRC), so passing sizeof(GndStationData) would truncate the packet
    // and make CRC verification impossible.

    // reset rx_buffer to all zeroes in anticipation of new byte
    memset(rx_buffer, 0, sizeof(rx_buffer));
    int16_t len = recieve_e22_900t22s(rx_buffer, sizeof(GndStationData));
    if(len <= 0) return E22_RECEIVE_ERR;
    if(len < 7)  return E22_BAD_LENGTH;

    int8_t status = decodeData(gnd, (uint16_t)len);
    if(status != 0) return static_cast<uint8_t>(-status);  // positive non-zero error code

    HALOutData.RSSI = get_rssi_e22_900t22s();

    // Propagate servo offsets to g_gndData so the CTRLs task can apply them.
    // The CTRLs task reads g_gndData under g_ctrls_sensor_mutex; acquire it here
    // too so the two-float update is atomic from its perspective.
    if (osMutexAcquire(g_ctrls_sensor_mutex, 5) == osOK) {
        g_gndData.servoOffset1 = gnd.servoOffset1;
        g_gndData.servoOffset2 = gnd.servoOffset2;
        osMutexRelease(g_ctrls_sensor_mutex);
    }

    processCmd(gnd.CommandByte);
    processPyros(gnd.pyroActivation);
    return 0;
}

// ---------------------------------------------------------------------------
// Command handler — byte values must match GND's RADIO_DEFNS.h exactly.
// ---------------------------------------------------------------------------
void Telemetry::processCmd(uint8_t cmd)
{
    switch (cmd) {
        case SHUTDOWN_KEEPALIVE:      // 217 — BYTE_ABORT on GND
            shutdownFlag    = true;
            g_ctrls_enabled = false;  // zero servo outputs immediately on abort
            break;

        case HANDSHAKE_GND_BYTE:      // 0xA1 — BYTE_HANDSHAKE on GND
            HALOutData.CommandResponseByte = HANDSHAKE_FC_BYTE;  // 0xB2
            break;

        case BYTE_DEFLECT_TEST:       // 150 — servo deflection test, enable controls output
            g_ctrls_enabled = true;
            break;

        default:
            // SERVO_OFFSET_CMD_BYTE (0xD4): offsets already applied in receiveCommands().
            // BYTE_NO_CMD (0), BYTE_REQUEST_DATA (0xC3): no side effects needed.
            break;
    }
}

void Telemetry::processPyros(uint32_t pyroActivation)
{
    // Set pending bits — PyroTask fires the GPIO in its own context so the
    // Radio task is never blocked during the 2-second hold.
    if (pyroActivation == PYROMAIN)       g_pyroPending |= PYRO_MAIN_BIT;
    if (pyroActivation == PYRODROGUEBKP)  g_pyroPending |= PYRO_DROGUE_BKP_BIT;
    if (pyroActivation == PYRODROGUEMAIN) g_pyroPending |= PYRO_DROGUE_MAIN_BIT;
}

// ---------------------------------------------------------------------------
// Utilities
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

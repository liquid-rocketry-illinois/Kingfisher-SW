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
    lastSeq         = 0;
    tx_burst_count  = 0;
    lastRxGood      = false;
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
                   | R0_210_E22_AIR_DATA_RATE::E22_AIR_RATE_4_8K;

    des_cfg.REG1 =   R1_76_SUB_PACKET_SETTING::BYTES_240
                   | R1_5_RSSI_ENVIRONMENTAL_NOISE_MEASURE_ENABLE
                   | R1_2_SOFTWARE_MODE_SWITCHING_OFF
                   | R1_10_E22_TX_POWER::E22_TX_POWER_22DBM;

    des_cfg.REG2 = GLOBAL_RADIO_CHANNEL;

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
// State machine:
//
//  FC_MASTER (default) — FC transmits TX_BURST_SIZE telemetry packets, then
//    opens a short receive window.  If a valid GND packet is decoded the
//    state switches to GND_MASTER.
//
//  GND_MASTER — FC only receives; GND is the active transmitter.  Each
//    successful decode resets the 5-second deadline.  If the deadline
//    expires with no valid packet, FC reverts to FC_MASTER.
//
// Return value:
//   0              — valid GND packet decoded this cycle
//   E22_NO_DATA    — FC-master TX cycle; no receive attempted
//   E22_RECEIVE_ERR / E22_BAD_LENGTH / positive framing codes
//                  — receive attempted but failed
// --------------------------------------------------------------------------
uint8_t Telemetry::Update()
{
    // Every 5 s, open a 2-second receive window — poll AUX continuously so
    // any incoming GND packet is caught.  If the window expires with no packet,
    // fall through to the normal TX path.
    static TickType_t lastWindowTick = 0u;
    const TickType_t  nowTick        = xTaskGetTickCount();
    if (nowTick - lastWindowTick >= pdMS_TO_TICKS(5000u)) {
        lastWindowTick = nowTick;
        const TickType_t deadline = nowTick + pdMS_TO_TICKS(2000u);
        while (xTaskGetTickCount() < deadline) {
            // If AUX is low the E22 is clocking an incoming packet onto the UART line.
            // Receive it before transmitting so we don't overwrite the UART buffer.
            if (!e22_aux_high()) {
                uint8_t rx_status = receiveCommands(GNDOutData);
                lastRxGood = (rx_status == 0);
                if (rx_status == 0)
                    return 0;
            }
        }
    }

    // AUX high — module idle, transmit telemetry normally.
    lastRxGood = false;
    sendData(HALOutData);
    return E22_NO_DATA;
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
    tx_buffer[2] = GLOBAL_RADIO_CHANNEL;

    // What is actually transmitted
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
    if (buf_len < 7u) return -1; // too short to contain any valid frame
    // buf_len >= 7 here, so buf_len - 1u >= 6u — no underflow risk.

    int16_t sync_idx = -1;
    for(int16_t i = 0; i < buf_len; i++)
    {
        if(rx_buffer[i] == TELEMETRY_SYNC1 && rx_buffer[i + 1] == TELEMETRY_SYNC2)
        {
            sync_idx = i;
            break;
        }
    }

    if(sync_idx == -1)
        return -1;  // sync bytes not found in received data

    // Enough bytes after sync for the full 5-byte header?
    // [SYNC1][SYNC2][len][seq_lo][seq_hi] = 5 bytes before payload starts.
    if((uint16_t)(sync_idx + 5) >= buf_len)
        return -2;

    uint8_t payload_len = rx_buffer[sync_idx + 2];

    if(payload_len != sizeof(T))
        return -3;  // length mismatch — wrong packet type or struct size skew

    // Full packet (header + payload + CRC + trailing sync pair) must fit inside received bytes.
    // Layout after sync_idx: [S1][S2][len][seq_lo][seq_hi][payload x N][CRC_lo][CRC_hi][S2][S1]
    //                          0   1   2    3       4       5..4+N      5+N     6+N     7+N  8+N
    if((uint16_t)(sync_idx + 5 + payload_len + 4) > buf_len)
        return -4;

    uint16_t seq_rx = rx_buffer[sync_idx + 3] | (rx_buffer[sync_idx + 4] << 8);

    uint16_t crc_rx =
        rx_buffer[sync_idx + 5 + payload_len] |
        (rx_buffer[sync_idx + 6 + payload_len] << 8);

    uint16_t crc_calc = Checksum(&rx_buffer[sync_idx], 5 + payload_len);

    if(crc_rx != crc_calc)
        return -5;

    // Verify trailing sync bytes (mirror of header: SYNC2 then SYNC1)
    if(rx_buffer[sync_idx + 7 + payload_len] != TELEMETRY_SYNC2 ||
       rx_buffer[sync_idx + 8 + payload_len] != TELEMETRY_SYNC1)
        return -6;  // trailing sync mismatch — frame boundary corrupted

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
    vTaskSuspendAll();
    int16_t len = recieve_e22_900t22s(rx_buffer, sizeof(GndStationData));
    xTaskResumeAll();
    if(len <= 0) return E22_RECEIVE_ERR;
    if(len < 7)  return E22_BAD_LENGTH;

    int8_t status = decodeData(gnd, (uint16_t)len);
    if(status != 0) return static_cast<uint8_t>(-status);  // positive non-zero error code

    // Propagate servo offsets to g_gndData so the CTRLs task can apply them.
    // The CTRLs task reads g_gndData under g_ctrls_sensor_mutex; acquire it here
    // too so the two-float update is atomic from its perspective.
    if (osMutexAcquire(g_ctrls_sensor_mutex, 5) == osOK) {
        // Idk why but for RCI to display these properly i have to switch them
        // im tired boss
        g_gndData.servoOffset1 = gnd.servoOffset2;
        g_gndData.servoOffset2 = gnd.servoOffset1;
        osMutexRelease(g_ctrls_sensor_mutex);
    }

    processCmd(gnd.CommandByte);
    return 0;
}

// ---------------------------------------------------------------------------
// Command handler — byte values must match GND's RADIO_DEFNS.h exactly.
//
// Default behaviour: echo the received command byte back in CommandResponseByte
// so GND always gets positive confirmation that HAL received its instruction.
// CommandResponseByte is only written here; no other path touches it, so the
// echo-then-override pattern below is the single source of truth.
//
// Exception — HANDSHAKE (0xA1): GND expects the dedicated ACK byte 0xB2, not
// a mirror of its own request byte, so that one case overrides the default.
// ---------------------------------------------------------------------------
void Telemetry::processCmd(uint8_t cmd)
{
    // Echo by default — GND inspects this field to confirm delivery.
    HALOutData.CommandResponseByte = cmd;

    switch (cmd) {
        case SHUTDOWN_KEEPALIVE:          // 217  — BYTE_ABORT
            shutdownFlag    = true;
            g_ctrls_enabled = false;      // zero servo outputs immediately
            break;

        case HANDSHAKE_GND_BYTE:          // 0xA1 — BYTE_HANDSHAKE
            // Override echo: reply with the dedicated ACK byte so GND can
            // distinguish a handshake response from its own request byte.
            HALOutData.CommandResponseByte = HANDSHAKE_FC_BYTE;  // 0xB2
            break;

        case BYTE_DEFLECT_TEST:           // 150  — BYTE_DEFLECT_TEST
            g_ctrls_enabled = true;
            break;

        case REQUEST_DATA_BYTE:           // 0xC3 — BYTE_REQUEST_DATA
            // FC always sends telemetry; no extra action beyond the echo.
            break;

        case SERVO_OFFSET_CMD_BYTE:       // 0xD4 — BYTE_SERVO_TARE
            // Servo offsets already propagated in receiveCommands();
            // the echo confirms receipt to GND.
            break;

        case 0:                           // BYTE_NO_CMD — idle / no-op
            HALOutData.CommandResponseByte = 0;
            break;

        default:
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

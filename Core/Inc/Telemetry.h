#ifndef KINGFISHER_SW_TELEMETRY_H
#define KINGFISHER_SW_TELEMETRY_H

#include "Ebyte_E22_900T22S.h"
#include "main.h"

// ── Protocol constants ── must match GND RADIO_DEFNS.h exactly ────────────────
#define TELEMETRY_SYNC1         0xAA
#define TELEMETRY_SYNC2         0x55
#define TELEMETRY_MAX_PAYLOAD   240     // max payload bytes (E22 sub-packet limit)

// Radio addresses
#define HAL1_RADIO_NETID        0xE6
#define HAL1_RADIO_ADDRLOW      0x06
#define HAL1_RADIO_ADDRHIGH     0x07
#define GND_RADIO_ADDRLOW       0x4A
#define GND_RADIO_ADDRHIGH      0x4A

// ── GND → FC command bytes ── (values mirror RADIO_DEFNS.h on GND side) ───────
#define SHUTDOWN_KEEPALIVE      217u    // = BYTE_ABORT on GND: halt all operations
#define SERVO_OFFSET_CMD_BYTE   0xD4u   // = BYTE_SERVO_TARE on GND: apply servoOffset1/2

// ── E22 driver error codes ─────────────────────────────────────────────────────
#define E22_NO_DATA             2
#define E22_RECEIVE_ERR         3
#define E22_BAD_LENGTH          4

// ── EXTI flag (set by interrupt, polled by radio task) ────────────────────────
extern volatile bool e22_data_ready;

// ── Pyrotechnic keys ───────────────────────────────────────────────────────────
typedef enum {
    PYROMAIN        = 121734683,
    PYRODROGUEBKP   = 402746912,
    PYRODROGUEMAIN  = 243656272
} pyroActivateKeys;

// ── GND → FC payload ──────────────────────────────────────────────────────────
// Transmitted by GND every cycle; received and decoded by FC.
typedef struct
{
    float    servoOffset1      = 0.0f;  // S1 zero-point offset (degrees)
    float    servoOffset2      = 0.0f;  // S2 zero-point offset (degrees)
    uint32_t pyroActivation    = 0;
    uint8_t  CommandByte; // Byte sent to HAL
} GndStationData;

// ── FC → GND payload ──────────────────────────────────────────────────────────
// Fields are ordered to minimise struct padding.
// @attention Layout must be byte-for-byte identical to GND's telemetryData.
//            Any change here requires the same change in HAL-1-GND Radio.h.
typedef struct
{
    float altitude; // new
    float verticalVelocity;
    float longitude, latitude, GPSaltitude;
    float mAccX, mAccY, mAccZ; // imu stuff
    float mGyrX, mGyrY, mGyrZ; // imu stuff
    float Qx, Qy, Qz, Qw; // new (quaternions)
    float pitch, yaw, roll; // tait-bryan angles (new)
    float servoTarget1, servoTarget2; // simple float vals (new?) these are commands so gnd station -> hal
    float servoPos1, servoPos2; // motor encoder readings
    float temperature; // use averaged temperatures from BMP390L and BMI323 TMR
    uint8_t callsign[12] = {75, 69, 57, 69, 82, 73, 95, 65, 76, 69, 80, 72}; // (new)
    uint8_t CommandResponseByte; // (mainly for the radio ping command)
    // @attention Must be int8_t to match HAL's telemetryData (Telemetry.h).
    // HAL stores the raw E22 RSSI byte here; convert with: dBm = -(256 - raw) / 2.
    // Using float here causes a 3-byte struct size difference that breaks decoding.
    int8_t RSSI;
    bool pyroMainDrogueFired   = false; // return status of pyro
    bool pyroBackupDrogueFired = false;
    bool pyroMainChuteFired    = false;
} telemetryData;

inline bool operator==(const telemetryData& a, const telemetryData& b) {
    return a.altitude          == b.altitude          &&
           a.verticalVelocity  == b.verticalVelocity  &&
           a.longitude         == b.longitude         &&
           a.latitude          == b.latitude          &&
           a.GPSaltitude       == b.GPSaltitude       &&
           a.mAccX == b.mAccX && a.mAccY == b.mAccY && a.mAccZ == b.mAccZ &&
           a.mGyrX == b.mGyrX && a.mGyrY == b.mGyrY && a.mGyrZ == b.mGyrZ &&
           a.Qx == b.Qx && a.Qy == b.Qy && a.Qz == b.Qz && a.Qw == b.Qw &&
           a.pitch == b.pitch && a.yaw == b.yaw && a.roll == b.roll &&
           a.servoTarget1      == b.servoTarget1      &&
           a.servoTarget2      == b.servoTarget2      &&
           a.servoPos1         == b.servoPos1         &&
           a.servoPos2         == b.servoPos2         &&
           a.temperature       == b.temperature       &&
           a.CommandResponseByte == b.CommandResponseByte &&
           a.RSSI              == b.RSSI              &&
           a.pyroMainDrogueFired   == b.pyroMainDrogueFired   &&
           a.pyroBackupDrogueFired == b.pyroBackupDrogueFired &&
           a.pyroMainChuteFired    == b.pyroMainChuteFired;
}
inline bool operator!=(const telemetryData& a, const telemetryData& b) { return !(a == b); }

// ── Telemetry class ─────────────────────────────────────────────────────────────
// FC is the slave: receive GND command first, then always reply with telemetry.
class Telemetry {

    config_e22_900t22s des_cfg = {};

    uint8_t  tx_buffer[TELEMETRY_MAX_PAYLOAD]{};
    uint8_t  rx_buffer[TELEMETRY_MAX_PAYLOAD]{};
    uint16_t lastSeq = 0;

    // FC → GND data path
    uint8_t sendData(const telemetryData &data);

    // GND → FC data path
    uint8_t receiveCommands(GndStationData &gnd);
    void    processCmd(uint8_t cmd);
    void    processPyros(uint32_t pyroActivation);

    // Shared framing
    template<typename T> int8_t  decodeData(T &payload, uint16_t buf_len);
    template<typename T> uint8_t encodeAndSend(const T &payload);
    uint16_t Checksum(uint8_t *data, uint16_t length);

    void Reconfigure(const config_e22_900t22s *cfg_new);

public:
    Telemetry();

    uint8_t Init();

    // Call every Radio-task cycle.
    // Receives one GND packet, applies any commands, then sends telemetry back.
    uint8_t Update();

    // HALOutData: populated by sensor tasks; transmitted to GND each cycle.
    // GNDOutData: decoded from the most recent GND packet received.
    telemetryData  HALOutData = {};
    GndStationData GNDOutData = {};

    bool shutdownFlag = false;
};

#endif // KINGFISHER_SW_TELEMETRY_H
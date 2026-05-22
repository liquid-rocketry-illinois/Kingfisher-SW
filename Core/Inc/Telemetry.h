#ifndef KINGFISHER_SW_TELEMETRY_H
#define KINGFISHER_SW_TELEMETRY_H

#include "Ebyte_E22_900T22S.h"
#include "main.h"

#define TELEMETRY_SYNC1         0xAA
#define TELEMETRY_SYNC2         0x55
#define TELEMETRY_MAX_PAYLOAD 240 // set to maximum payload size
#define HAL1_RADIO_NETID 0xE6
#define HAL1_RADIO_ADDRLOW 0x06
#define HAL1_RADIO_ADDRHIGH 0x07

#define GND_RADIO_ADDRLOW 0x4A
#define GND_RADIO_ADDRHIGH 0x4A

#define SHUTDOWN_KEEPALIVE      217
#define DEFLECT_TEST            150

#define E22_NO_DATA             2
#define E22_RECEIVE_ERR         3
#define E22_BAD_LENGTH          4

// for exti catch
extern volatile bool e22_data_ready;

typedef enum {
    PYROMAIN        = 121734683,
    PYRODROGUEBKP   = 402746912,
    PYRODROGUEMAIN  = 243656272
} pyroActivateKeys;

#define SERVO_OFFSET_CMD_BYTE   0xD4u   // GND→FC: apply servoOffset1/2 as new zero points

typedef struct
{
    float    servoOffset1      = 0.0f;  // S1 zero-point offset (degrees)
    float    servoOffset2      = 0.0f;  // S2 zero-point offset (degrees)
    uint32_t pyroActivation    = 0;
    uint8_t  CommandByte; // Byte sent to HAL
} GndStationData;

// ordering to reduce padding total size
typedef struct
{
    float altitude; // new
    float verticalVelocity;
    float longitude, latitude, GPSaltitude; // your implementation used four iirc
    float mAccX, mAccY, mAccZ; // imu stuff
    float mGyrX, mGyrY, mGyrZ; // imu stuff
    float Qx, Qy, Qz, Qw; // new (quaternions)
    float pitch, yaw, roll; // tait-bryan angles (new)
    float servoTarget1, servoTarget2; // simple float vals (new?) these are commands so gnd station -> hal
    float servoPos1, servoPos2; // motor encoder readings
    float temperature; // use averaged temperatures from BMP390L and BMI323 TMR
    uint8_t callsign[12] = {75, 69, 57, 69, 82, 73, 95, 65, 76, 69, 80, 72}; // (new)
    uint8_t CommandResponseByte; // (mainly for the radio ping command)
    int8_t RSSI; // RSSI byte from radio, describes signal strength
    bool pyroMainDrogueFired   = false; // return status of pyro
    bool pyroBackupDrogueFired = false;
    bool pyroMainChuteFired    = false;
} telemetryData;

inline bool operator==(const telemetryData& a, const telemetryData& b) {
    return a.altitude == b.altitude &&
           a.longitude == b.longitude && a.latitude == b.latitude && a.GPSaltitude == b.GPSaltitude &&
           a.mAccX == b.mAccX && a.mAccY == b.mAccY && a.mAccZ == b.mAccZ &&
           a.mGyrX == b.mGyrX && a.mGyrY == b.mGyrY && a.mGyrZ == b.mGyrZ &&
           a.Qx == b.Qx && a.Qy == b.Qy && a.Qz == b.Qz && a.Qw == b.Qw &&
           a.pitch == b.pitch && a.yaw == b.yaw && a.roll == b.roll &&
           a.servoTarget1 == b.servoTarget1 && a.servoTarget2 == b.servoTarget2 &&
           a.servoPos1 == b.servoPos1 && a.servoPos2 == b.servoPos2 &&
           a.temperature == b.temperature &&
           a.CommandResponseByte == b.CommandResponseByte &&
           a.RSSI == b.RSSI &&
           a.pyroMainDrogueFired == b.pyroMainDrogueFired &&
           a.pyroBackupDrogueFired == b.pyroBackupDrogueFired &&
           a.pyroMainChuteFired == b.pyroMainChuteFired;
}
inline bool operator!=(const telemetryData& a, const telemetryData& b) { return !(a == b); }

class Telemetry {

    config_e22_900t22s des_cfg = {};

    uint8_t  tx_buffer[TELEMETRY_MAX_PAYLOAD]{};
    uint8_t  rx_buffer[TELEMETRY_MAX_PAYLOAD]{};
    uint16_t lastSeq = 0;

    uint32_t shutdownHoldStart = 0;

    // FC-side
    uint8_t sendData(const telemetryData &data);
    uint8_t receiveCommands(GndStationData &gnd);
    void    processCmd(uint8_t cmd);
    void    processPyros(uint32_t pyroActivation);

    // GND-side
    uint8_t sendCommands(const GndStationData &gnd);
    uint8_t receiveTelemetry(telemetryData &data);

    // shared
    template<typename T> int8_t decodeData(T &payload);
    template<typename T> uint8_t encodeAndSend(const T &payload);
    uint16_t Checksum(uint8_t *data, uint16_t length);

    void Reconfigure(const config_e22_900t22s *cfg_new);

public:
    Telemetry();

    uint8_t Init();
    uint8_t Update();

    // exposed so the application can read latest decoded data
    telemetryData   HALOutData = {};   // flight → populated by sensors (FC) or decoded RX (GND)
    GndStationData  GNDOutData = {};   // GND → populated by operator (GND) or decoded RX (FC)

    uint8_t lastRSSI    = 0;    // raw RSSI byte from last receive; actual dBm = -(256-raw)/2
    bool    shutdownFlag = false;
};

#endif
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

#define SHUTDOWN_KEEPALIVE      0xDE
#define SHUTDOWN_HOLDOFF_MS     5000

#define E22_NO_DATA             2
#define E22_RECEIVE_ERR         3
#define E22_BAD_LENGTH          4

// for exti catch
extern volatile bool e22_data_ready;

typedef enum {
    TELEMETRY_MODE_FLIGHT,   // rocket — sends telemetry, receives GND commands
    TELEMETRY_MODE_GROUND    // ground station — sends commands, receives telemetry
} TelemetryMode;

typedef enum {
    PYROMAIN        = 121734683,
    PYRODROGUEBKP   = 402746912,
    PYRODROGUEMAIN  = 243656272
} pyroActivateKeys;

typedef struct
{
    uint8_t keepAliveIn;
    uint32_t pyroActivation[3] = {0, 0, 0};
} GndStationData;

typedef struct
{
    const uint8_t callsign[12] = {75, 69, 57, 69, 82, 73, 95, 65, 76, 69, 80, 72};
    uint8_t keepAliveStatus;
    float altitude;
    float longitude, latitude, GPSaltitude;
    float mAccX, mAccY, mAccZ;
    float bAccX, bAccY, bAccZ;
    float Qx, Qy, Qz, Qw;
    float pitch, yaw, roll;
    float servoPos1, servoPos2;
} telemetryData;

class Telemetry {
    uint8_t mode;

    config_e22_900t22s des_cfg = {};

    uint8_t  tx_buffer[TELEMETRY_MAX_PAYLOAD]{};
    uint8_t  rx_buffer[TELEMETRY_MAX_PAYLOAD]{};
    uint16_t lastSeq = 0;

    uint32_t shutdownHoldStart = 0;

    // FC-side
    uint8_t sendData(const telemetryData &data);
    uint8_t receiveCommands(GndStationData &gnd);
    void    processKeepalive(uint8_t keepAliveIn);
    void    processPyros(uint32_t pyroActivation[3]);

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

    uint8_t Init(TelemetryMode Mode);
    uint8_t Update();

    // exposed so the application can read latest decoded data
    telemetryData   HALOutData = {};   // flight → populated by sensors (FC) or decoded RX (GND)
    GndStationData  GNDOutData = {};   // GND → populated by operator (GND) or decoded RX (FC)

    bool shutdownFlag = false;
};

#endif
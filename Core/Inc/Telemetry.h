#ifndef KINGFISHER_SW_TELEMETRY_H
#define KINGFISHER_SW_TELEMETRY_H

#include "Ebyte_E22_900T22S.h"
#include <cstdint>

#define TELEMETRY_SYNC1 0xAA
#define TELEMETRY_SYNC2 0x55
#define TELEMETRY_MAX_PAYLOAD 32

typedef struct
{
    uint8_t keepAliveStatus;
    float mAccX, mAccY, mAccZ;
    float bAccX, bAccY, bAccZ;
    float pitch, yaw, roll;
    float servoPos1, servoPos2;
} telemetryData;


typedef struct
{
    uint8_t keepAliveIn;
} GndStationData;

class Telemetry {

    static config_e22_900t22s des_cfg;

    uint16_t seq = 0;
    uint16_t lastSeq = 0;

    // buffers to transmit and recieve data
    uint8_t tx_buffer[64];
    uint8_t rx_buffer[64];

    uint8_t sendData(const telemetryData &data);
    uint8_t receiveData(GndStationData &gnd);
    uint8_t decodeData(GndStationData &gnd);
    uint16_t Checksum(uint8_t *data, uint16_t length);

public:
    Telemetry();

    uint8_t Init();
    uint8_t Update();
    void Reconfigure(const config_e22_900t22s* cfg_new);
};

#endif
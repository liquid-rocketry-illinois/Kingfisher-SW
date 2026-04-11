#ifndef KINGFISHER_SW_TELEMETRY_H
#define KINGFISHER_SW_TELEMETRY_H

#include "Ebyte_E22_900T22S.h"
#include <cstdint>

#define TELEMETRY_SYNC1 0xAA
#define TELEMETRY_SYNC2 0x55
#define TELEMETRY_MAX_PAYLOAD 240 // set to maximum payload size
#define HAL1_RADIO_NETID 0xE6
#define HAL1_RADIO_ADDRLOW 0x12
#define HAL1_RADIO_ADDRHIGH 0x34

// Data defs
#define HAL1_RADIO_STARTBIT         0x02        //
#define HAL1_RADIO_ENDBIT           0x03        //
#define HAL1_RADIO_SEPARATOR        0x7C        // |
#define HAL1_INDC_CALLSIGNC         0x43        // C
#define HAL1_INDC_CALLSIGNS         0x53        // S
#define HAL1_INDC_KEEPALIVEk        0x6b        // k
#define HAL1_INDC_KEEPALIVEA        0x41        // A
#define HAL1_INDC_LOWGRAVA          0x41        // A
#define HAL1_INDC_w                 0x77        // w
#define HAL1_INDC_x                 0x78        // x
#define HAL1_INDC_y                 0x79        // y
#define HAL1_INDC_z                 0x7A        // z
#define HAL1_INDC_HIGHGRAVACCH      0x48        // H
#define HAL1_INDC_HIGHGRAVACCG      0x47        // G
#define HAL1_INDC_QUATERNIONS       0x51        // Q
#define HAL1_INDC_PITCH             0x50        // P
#define HAL1_INDC_YAW               0x59        // Y
#define HAL1_INDC_ROLL              0x52        // R
#define HAL1_INDC_SERVO1ROTATIONS   0x53        // S
#define HAL1_INDC_SERVO2ROTATION1   0x31        // 1
#define HAL1_INDC_SERVO2ROTATION2   0x32        // 2

typedef struct
{
    // KE9ERI_ALEPH
    const uint8_t callsign[12] = {75, 69, 57, 69, 82, 73, 95, 65, 76, 69, 80, 72};
    // Repeat back keepalive received from ground station
    uint8_t keepAliveStatus;
    float altitude;
    float longitude, latitude, GPSaltitude;
    float mAccX, mAccY, mAccZ;
    float bAccX, bAccY, bAccZ;
    float Qx, Qy, Qz, Qw;
    float pitch, yaw, roll;
    float servoPos1, servoPos2;
} telemetryData;
// Transmission structure: data = {HAL1_RADIO_STARTBIT, telemetryData t, HAL1_RADIO_ENDBIT}

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
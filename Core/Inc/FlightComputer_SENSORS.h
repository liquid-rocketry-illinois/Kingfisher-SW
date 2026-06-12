//
// Created by bzhan on 5/15/2026.
//

#ifndef KINGFISHER_SW_FLIGHTCOMPUTER_H
#define KINGFISHER_SW_FLIGHTCOMPUTER_H

#include <stdint.h>
#include "Servo_Axon_Mini_MKII.h"
#include "IMUs.h"
#include "Barometer.h"
#include "LIS2MDL.h"
#include "Telemetry.h"
#include "MAXM10S.h"

// FLIGHT STAGE PROGRESSION
typedef enum {
    PREFLIGHT,
    ASCENT,
    CONTROLS_TEST,
    APOGEE_APPROACH,
    APOGEE,
    APOGEE_PASS,
    DESCENT,
    MAIN_APPROACH,
    MAIN,
    FINAL_DESCENT
} STAGE;

// STANDARD RETURN CODES
typedef enum {
    STATUS_ABORT_ACTIVE     = -4,
    STATUS_SENSOR_FAIL      = -3,
    STATUS_COMMS_FAILURE    = -2,
    STATUS_INIT_FAILURE     = -1,
    STATUS_OK               =  0,
    STATUS_TIMEOUT          =  1,
    STATUS_LOCAL_READ_ERR   =  2,
    STATUS_LOCAL_DATA_ERR   =  3,
    STATUS_REMOTE_READ_ERR  =  4,
    STATUS_REMOTE_DATA_ERR  =  5,
    STATUS_ABORT_TRIGGERED  = 127
} ProcedureStatus;

typedef struct {
    double  latitude;
    double  longitude;
    double  altitude;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t satellites;
    float   hdop;
} GPS_Data;

typedef struct {
    float currS1;
    float currS2;
    float targetS1;
    float targetS2;
} ServoAngles;

class Sensors {
    IMUs IMU;
    Baro_Unified Baro;
    MAXM10S GPS;
    Servo_Axon_Mini_MKII Servos;
    Magnetometer mag;

public:
    Sensors();
    int8_t Init();
    int8_t Update();
};

#endif //KINGFISHER_SW_FLIGHTCOMPUTER_H

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
#include "constants.h"
#include "SDCard.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
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

struct GPS_Data {
    double latitude;
    double longitude;
    double altitude;
};

struct ServoAngles {
    float currS1;
    float currS2;
    float targetS1;
    float targetS2;
};

// GLOBAL SENSOR DATA (DEFINED IN FlightComputer.cpp)
extern telemetryData     g_telemNow;
extern telemetryData     g_telemPrev;
extern GndStationData    g_gndData;
extern BMI_Data          g_BMI;
extern BMP_Data          g_BMP;
extern GPS_Data          g_GPS;
extern ServoAngles       g_servoAngles;



class FlightComputer {
private:
    // sensor declarations
public:
    // interfaces to interact with each sensor
};
/*
TODO: task UpdateSensors()
    init BMI, BMP, IMU with TMR on and the GPS
TODO: task UpdateData()
*/

// instead of while(1) use while(bool ex. initDone) and check initDone after each init run to ensure
// we don't move on to Update() while Init() hasn't finished yet

#endif //KINGFISHER_SW_FLIGHTCOMPUTER_H
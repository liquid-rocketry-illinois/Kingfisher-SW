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


// telemetryData object (telemNow & telemPrev)
// gndStation data object
// BMI_Data struct
// BMP_Data struct
// GPS_Data struct
// current servo angles struct
// target servo angles struct

struct BMI_TMR_Data {

};

struct BMP_TMR_Data {

};

struct GPS_Location {

};

struct ServoAngles {
  float currAngleOne;
  float currAngleTwo;
  float targetAngleOne;
  float targetAngleTwo;
};

extern BMI_TMR_Data BMI_Data;
extern BMP_TMR_Data BMP_Data;
extern GPS_Location GPS_Data;
extern ServoAngles servoState;

/*
TODO: task UpdateSensors()
    init BMI, BMP, IMU with TMR on and the GPS
TODO: task UpdateData()
*/

// instead of while(1) use while(bool ex. initDone) and check initDone after each init run to ensure
// we don't move on to Update() while Init() hasn't finished yet

#endif //KINGFISHER_SW_FLIGHTCOMPUTER_H
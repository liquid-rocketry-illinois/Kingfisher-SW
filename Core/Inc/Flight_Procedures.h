//
// Created by dyrel on 4/13/2026.
//

#ifndef KINGFISHER_SW_FLIGHT_PROCEDURES_H
#define KINGFISHER_SW_FLIGHT_PROCEDURES_H

#include <stdint.h>
#include "Servo_Axon_Mini_MKII.h"
#include "IMUs.h"
#include "Barometer.h"
#include "LIS2MDL.h"
#include "Telemetry.h"
#include "usart.h"
#include <cstring>
#include "task.h"

typedef enum : int8_t {
    STATUS_ABORT_ACTIVE     = -4,  // Abort already latched, redundant trigger
    STATUS_SENSOR_MISMATCH  = -3,  // GND/FC sensor data exceeds 5% tolerance
    STATUS_COMMS_FAILURE    = -2,  // Handshake or radio TX/RX failure
    STATUS_INIT_FAILURE     = -1,  // One or more devices failed to initialize
    STATUS_OK               =  0,  // Nominal
    STATUS_TIMEOUT          =  1,  // Operation exceeded allowed time window
    STATUS_LOCAL_READ_ERR   =  2,  // Local sensor read returned bad data
    STATUS_LOCAL_DATA_ERR   =  3,  // Local sensor data out of expected range
    STATUS_REMOTE_READ_ERR  =  4,  // Radio/FC read returned bad data
    STATUS_REMOTE_DATA_ERR  =  5,  // FC data out of expected range
    STATUS_ABORT_TRIGGERED  =  127 // Abort signal freshly activated (was 255, clamped to int8_t max)
} ProcedureStatus;

// Both ground and fc record the same data down
typedef struct {
    GndStationData dat_GND_Data;
    telemetryData dat_FC_Data;
    DATA_Axon_Mini_MKII dat_Servos;
    BMI_Data dat_BMI_IMUs;
    BARO_DATA dat_Barometers;
    // GPS data struct here
} Data;

// devices used on both gnd and flight computers. Gnd station records,
// flight computer sends and records
typedef struct {
    Telemetry dev_telemetry;
    Baro_Unified dev_BarometerEngine;
    Servo_Axon_Mini_MKII dev_servoSet;
    IMUs dev_IMU_Engine;
    // GPS device here
} Devices;

class GroundStation {
    Data    GNDData = {};
    Devices GNDDevices = {};

public:
    GroundStation();
    int8_t Init();
    int8_t Update(); // Send/Receive from RCP

    int8_t UpdateKeepAlive();
    int8_t UpdateLogging(); // TODO implement
};




/// FLIGHT COMPUTER

class FlightComputer {
    Data    FCData;
    Devices FCDevices;

public:
    FlightComputer();
    int8_t Init();
    int8_t Update(); // Send to RCI/gnd station
};
#endif //KINGFISHER_SW_FLIGHT_PROCEDURES_H
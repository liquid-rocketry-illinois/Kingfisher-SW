//
// Created by dyrel on 4/13/2026.
//

#include "Flight_Procedures.h"

GroundStation::GroundStation() : GNDDevices(), GNDData(){}

int8_t GroundStation::Init() {
    GNDDevices.dev_telemetry.Init(TELEMETRY_MODE_GROUND);
    GNDDevices.dev_BarometerEngine.Init();
    GNDDevices.dev_IMU_Engine.Init(true);
    GNDDevices.dev_servoSet.Init({0,0}, TENTH_DEGREE, false);

    return 0;
}


int8_t GroundStation::Update() {
    return 0;
}

int8_t GroundStation::UpdateKeepAlive() {
    return 0;
}

int8_t GroundStation::UpdateLogging() {
    return 0;
}


// FLIGHT COMPUTER

FlightComputer::FlightComputer() {

}

int8_t FlightComputer::Init() {
    return 0;
}

int8_t FlightComputer::Update() {
    return 0;
}

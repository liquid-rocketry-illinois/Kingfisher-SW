//
// Created by bzhan on 5/7/2026.
//

#import <Flight_Procedures.h>

int8_t UpdateState() {
    // first update the abort accumulator
    // second update verticalvelocity
    // third move through conops and check state
    return STATUS_OK;
}
// combine AbortAccumulator
// combine VerticalVelocity
// combine TrackCONOPS

/*
 * TODO: Refactor Flight_Procedures.h to separate classes for state, control, abort
 * TODO: Ensure that data from sensors (IMU, GPS, SERVO), (COMMAND_BYTE, PYRO_ACTIV) can still be accessed through state vectors
 * TODO: Combine AbortAccumulator(), VerticalVelocity(), TrackCONOPS() and broadcast sensor data to ground station
 */

// instead of having everything in main.cpp, each of those functions would just be another task in freeRTOS.
// it takes all those tasks and loops over in its own way, increasing speed
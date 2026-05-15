//
// Created by bzhan on 5/7/2026.
//
#include "Flight_Procedures.h"
#include "timing.h"
#include "cmsis_os2.h"
#include "main.h"
#include "i2c.h"
#include <cstdio>

// TODO: add this function and dependencies to Flight_Procedures.h
int8_t FlightComputer::UpdateStateCONOPS() {

    // UPDATE VERTICAL VELOCITY AND FEED INTO CONOPS
    // combine original VerticalVelocity() and TrackCONOPS() sequentially
    float rawAlt = FCData.dat_Barometers.Filtered.heightMeters;

    // Median-of-3 spike filter: store 3 samples, return the middle value.
    // A single-point baro glitch is completely rejected with no lag on valid data.
    // Cost: 3 comparisons + temp swaps — negligible on Cortex-M7.
    altFilterBuf[altFilterIdx] = rawAlt;
    altFilterIdx = (altFilterIdx + 1) % 3;

    float a = altFilterBuf[0], b = altFilterBuf[1], c = altFilterBuf[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    filteredAltM = b;  // median — feeds all altitude-based decisions

    uint32_t now = millis();
    uint32_t dt  = now - prevAltTimeMs;
    if (dt > 0) {
        vertVelocityMs = (filteredAltM - prevAltM) / (dt / 1000.0f);
    }
    prevAltM      = filteredAltM;
    prevAltTimeMs = now;

    // TRACK CONOPS STATE
    switch (currentStage) {

        case PREFLIGHT:
            //FCDevices.dev_servoSet.SetOffset({FCData.dat_GND_Data.servoOffset1, FCData.dat_GND_Data.servoOffset2});
            FCDevices.dev_servoSet.Update(0.0f, 0.0f);   // hold neutral so GND offset is live
            if (FCData.dat_BMI_IMUs.accel_linear.magnitude() > LIFTOFF_ACCEL_G) {
                if (liftoffDetectMs == 0) liftoffDetectMs = millis();
                if ((millis() - liftoffDetectMs > LIFTOFF_SUSTAIN_MS) &&
                    (filteredAltM > launchAltM + LIFTOFF_ALT_DELTA_M)) {
                    currentStage = ASCENT;
                    SD_LogNewline("STAGE: ASCENT");
                }
            } else {
                liftoffDetectMs = 0;
            }
            break;

        case ASCENT:
            // TODO This needs to account for drag!!!!!
            if (FCData.dat_BMI_IMUs.accel_linear.magnitude() < BURNOUT_ACCEL_G) {
                currentStage = CONTROLS_TEST;
                SD_LogNewline("STAGE: CONTROLS_TEST");
            }
            break;

        case CONTROLS_TEST:
        {
            // Non-blocking servo sequence: 8° → -2° → 5° → repeat, 1.5 s per step.
            // Runs until APOGEE_APPROACH transition, then servos zero.
            static const float    ctrlAngles[3] = { 8.0f, -2.0f, 5.0f };
            static const uint32_t ctrlHoldMs    = 1500;

            if (ctrlTestStepStartMs == 0)
                ctrlTestStepStartMs = millis();

            if (millis() - ctrlTestStepStartMs >= ctrlHoldMs) {
                ctrlTestStep        = (ctrlTestStep + 1) % 3;
                ctrlTestStepStartMs = millis();
            }

            FCDevices.dev_servoSet.Update(ctrlAngles[ctrlTestStep],
                                          ctrlAngles[ctrlTestStep]);

            if (vertVelocityMs < APOGEE_APPROACH_VEL_MS) {
                currentStage = APOGEE_APPROACH;
                FCDevices.dev_servoSet.Update(0.0f, 0.0f);
                SD_LogNewline("STAGE: APOGEE_APPROACH");
            }
            break;
        }

        case APOGEE_APPROACH:
            if (vertVelocityMs < APOGEE_VEL_MS &&
                filteredAltM > launchAltM + MIN_APOGEE_ALT_M) {
                currentStage = APOGEE;
                SD_LogNewline("STAGE: APOGEE");
            }
            break;

        case APOGEE:
            if (vertVelocityMs < APOGEE_PASS_VEL_MS) {
                currentStage = APOGEE_PASS;
                SD_LogNewline("STAGE: APOGEE_PASS");
            }
            break;

        case APOGEE_PASS:
            if (vertVelocityMs < DESCENT_VEL_MS) {
                currentStage = DESCENT;
                SD_LogNewline("STAGE: DESCENT");
            }
            break;

        case DESCENT:
            if (filteredAltM < launchAltM + MAX_MAIN_DEPLOY_ALT_M &&
                filteredAltM > launchAltM + MIN_MAIN_DEPLOY_ALT_M) {
                currentStage = MAIN_APPROACH;
                SD_LogNewline("STAGE: MAIN_APPROACH");
            }
            break;

        case MAIN_APPROACH:
            if (filteredAltM <= launchAltM + TARGET_MAIN_ALT_M) {
                currentStage = MAIN;
                SD_LogNewline("STAGE: MAIN");
            }
            break;

        case MAIN:
            if (vertVelocityMs > FINAL_DESCENT_VEL_MS) {
                currentStage = FINAL_DESCENT;
                SD_LogNewline("STAGE: FINAL_DESCENT");
            }
            break;

        case FINAL_DESCENT:
            break;
    }
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
// keep in mind that the control systems will live in a separate task later, so leave room for that

// REVISED: ONLY COMBINE VERTICAL VELOCITY AND TRACKCONOPS
// ABORT ACCUMULATOR LIVES ON ITS OWN IN A SEPARATE TASK
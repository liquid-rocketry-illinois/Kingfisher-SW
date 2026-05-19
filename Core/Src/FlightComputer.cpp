//
// Created by bzhan on 5/15/2026.
//
#include "FlightComputer.h"

#include <stdint.h>
#include "timing.h"
#include "cmsis_os2.h"
#include "main.h"
#include "i2c.h"
#include "CTRLS_Controls.h"
#include <cstdio>

// GLOBAL SENSOR DATA (DEFINED HERE) [We can just directly make these vars here]
telemetryData     g_telemNow;
telemetryData     g_telemPrev;
GndStationData    g_gndData;
BMI_Data          g_BMI;
BMP_Data          g_BMP;
GPS_Data          g_GPS;
ServoAngles       g_servoAngles;



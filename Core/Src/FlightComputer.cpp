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

BMI_TMR_Data BMI_Data;
BMP_TMR_Data BMP_Data;
GPS_Location GPS_Data;
ServoAngles servoState;

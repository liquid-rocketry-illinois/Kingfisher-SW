//
// Created by dyrel on 5/11/2026.
//

#ifndef KINGFISHER_SW_GLOBAL_SENSORUPDATE_H
#define KINGFISHER_SW_GLOBAL_SENSORUPDATE_H

#include "IMUs.h"
#include "Barometer.h"
#include "MAXM10S.h"
#include "Servo_Axon_Mini_MKII.h"
#include "Math/Vector3TripleModRedundancy.h"
#include "cmsis_os2.h"
#include "i2c.h"
#include "timing.h"

typedef struct {

} SensorData;

// global SensorData
extern SensorData g_sensorData;

extern g_ctrls_sensor_mutex;
extern g_ctrls_sensor;

extern "C" void UpdateData(void *argument);

#endif //KINGFISHER_SW_GLOBAL_SENSORUPDATE_H
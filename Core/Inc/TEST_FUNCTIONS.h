//
// Created by dyrel on 2/23/2026.
//

#ifndef HAL_1_SW_TEST_FUNCTIONS_H
#define HAL_1_SW_TEST_FUNCTIONS_H

#include <stdint.h>
#include "Servo_Axon_Mini_MKII.h"
#include "IMUs.h"
#include "Barometer.h"
#include "LIS2MDL.h"
#include "Telemetry.h"
#include "usart.h"
#include <cstring>
#include "task.h"
#include "SDCard.h"

class TEST{
public:
    TEST();
    int SERVO_TEST();
    int IMU_TEST();
    int BARO_TEST();
    int LIS2_TEST();
    int RADIO_TEST();
    int PYRO_TEST();
    int SD_TEST();
};

#endif //HAL_1_SW_TEST_FUNCTIONS_H
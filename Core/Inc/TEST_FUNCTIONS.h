//
// Created by dyrel on 2/23/2026.
//

#ifndef HAL_1_SW_TEST_FUNCTIONS_H
#define HAL_1_SW_TEST_FUNCTIONS_H

#include <stdint.h>
#include "Servo_Axon_Mini_MKII.h"
#include "IMUs.h"
#include "BMP390.h"

class TEST{
public:
    TEST();
    int SERVO_TEST();
    int IMU_TEST();
    int BMP_TEST();
};

#endif //HAL_1_SW_TEST_FUNCTIONS_H
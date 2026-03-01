//
// Created by dyrel on 2/23/2026.
//

#include "cmsis_os2.h"
#include "TEST_FUNCTIONS.h"

TEST::TEST(){;}

//TODO
//Need to change code of PWM generation. verify correct pwm signal
int TEST::SERVO_TEST() {
    Servo_Axon_Mini_MKII servoSet;
    bool initState = servoSet.Init({0,0}, PRECISION::TENTH_DEGREE, true);
    osDelay(500);

    // Track status of servos
    static int status = 0;
    if (status == 0 && initState) {
        servoSet.Update(90, -90);
        osDelay(10);
    }
    // Error - Not initialized within tolerance.
    else if (!initState) {
        // Attempt init again
        initState = servoSet.Init({0,0}, PRECISION::TENTH_DEGREE, true);
        osDelay(500);
        return -2;
    }
    // Something else went wrong (This should never really happen
    // but it is there cause muscle memory)
    else return -1;

    return status;
}

int TEST::ISM_TEST() {
    IMU_ISM6HGx ISM6;
    IMU_Data data;
    int status = ISM6.Init();

    if (status != 0)
    {
        Error_Handler();
    }
    while (1) {
        ISM6.Update();
        data = ISM6.GetData();
        osDelay(1);
    }
}
//
// Created by dyrel on 2/21/2026.
//

#ifndef KINGFISHER_SW_SERVO_AXON_MINI_MKII_H
#define KINGFISHER_SW_SERVO_AXON_MINI_MKII_H

/** @attention
 * This library is intended for use with the Axon Mini MK2 Servo ONLY.
 * operations are specialized for the specific use on Ctrl-Frk, but
 * users may find portions of the code written here useful for future
 * rocketry applications.
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "projdefs.h"

enum PRECISION {
    ONE_DEGREE,
    TENTH_DEGREE,
    HUNDREDTH_DEGREE,
    THOUSANTHS_DEGREE
};

template <typename T>
struct SAsym {
    T S1;
    T S2;
};

struct DATA_Axon_Mini_MKII{
    SAsym<float> targetAngle;
    SAsym<float> currentAngle;
    SAsym<float> trackedError;
    SAsym<float> trackedAngle;
    SAsym<bool> Nominal = {false, false};
    SAsym<float> smoothedCorrection = {0.0f, 0.0f};
};

struct CFG_Axon_Mini_MKII{
    uint8_t Precision;
    bool ENABLE_DEBUG;
    SAsym<float> AngleOffsetDEGREES;
};

class Servo_Axon_Mini_MKII{
private:
    CFG_Axon_Mini_MKII config = {}; // Both servos use same config
    DATA_Axon_Mini_MKII data = {};

    SAsym<float> readCurrentAngle(); // For async calls outside of Update()
    SAsym<float> calculateError();
    void Actuate(SAsym<float> input);

public:
    Servo_Axon_Mini_MKII();

    bool Init(  SAsym<float> AngOffset,
                PRECISION precision,
                bool debug);
    void Update(float S1_Target_Deg, float S2_Target_Deg);
    SAsym<float> getCurrentAngle();
    void SetOffset(SAsym<float> offset);   // update zero-point offsets at runtime (GND trim command)
    DATA_Axon_Mini_MKII getData() const;
};

#endif //KINGFISHER_SW_SERVO_AXON_MINI_MKII_H
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

enum PRECISION {
    ONE_DEGREE,
    TENTH_DEGREE,
    HUNDREDTH_DEGREE,
    THOUSANTHS_DEGREE
};

struct SAsym{
    float S1;
    float S2;
};

struct DATA_Axon_Mini_MKII{
    SAsym targetAngle;
    SAsym currentAngle;
    SAsym trackedError;
    SAsym trackedAngle;
    SAsym Nominal = {false, false};
};

struct CFG_Axon_Mini_MKII{
    uint8_t Precision;
    bool ENABLE_DEBUG;
    float AngleOffsetDEGREES;
};

class Servo_Axon_Mini_MKII{
private:
    CFG_Axon_Mini_MKII config; // Both servos use same config
    DATA_Axon_Mini_MKII data;

    SAsym readCurrentAngle(); // For async calls outside of Update()
    SAsym calculateError();
    void Actuate(SAsym input);

public:
    bool Init(  float AngOffset,
                PRECISION precision,
                bool debug);
    void Update(float S1_Target_Deg, float S2_Target_Deg);
};

#endif //KINGFISHER_SW_SERVO_AXON_MINI_MKII_H
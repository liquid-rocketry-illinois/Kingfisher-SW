//
// Created by dyrel on 2/21/2026.
//

#include "Servo_Axon_Mini_MKII.h"

#include "adc.h"
#include "main.h"
#include "tim.h"
#include "Math/Math.h"

// PRIVATE
/** @brief readCurrentAngle(void);
 *  Reads the current angle of the servo. This is basically just here for
 *  the init inputs.
 *
 *  @return Motor encoder output angle state. {Servo1 Angle, Servo2 Angle}
 */
SAsym<float> Servo_Axon_Mini_MKII::readCurrentAngle() {
    // Calculate other components of servo
    // Current angle is derived from the voltage reading from
    // the servo pin. 0 is 0deg, 1.65V is 180deg, 3.3V is 360deg.

    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
    uint32_t raw1 = HAL_ADC_GetValue(&hadc3);

    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
    uint32_t raw2 = HAL_ADC_GetValue(&hadc3);

    // Convert 12-bit ADC to voltage
    float reading1 = (3.3f * raw1) / 4095.0f; // read voltage from SERVO1_ENC_Pin / SERVO1_ENC_GPIO
    float reading2 = (3.3f * raw2) / 4095.0f; // vice versa

    SAsym<float> output = {
        MATHEMATICS::Map(reading1, 0.0f, 3.3f, 0.0f, 360.0f),
        MATHEMATICS::Map(reading2, 0.0f, 3.3f, 0.0f, 360.0f)
    };
    return output;
}

/** @brief calculateError(void);
 *  Here, we calculate the servo error in a cycle, as a
 *  percentage. This functions takes the detected angle and
 *  compares it to the expected angle.
 *
 * @return calculated error, as a scalar
 */
SAsym<float> Servo_Axon_Mini_MKII::calculateError() {
    // Expected angle is data.trackedAngle - We compare this
    // value to readCurrentAngle().
    SAsym<float> output = {
        (readCurrentAngle().S1 - data.trackedAngle.S1) / data.trackedAngle.S1,
        (readCurrentAngle().S2 - data.trackedAngle.S2) / data.trackedAngle.S2
    };
    return output;
}

/** @brief Actuate({Servo1 input rotation, Servo2 input rotation})
 *  Actuate servos to the specified positional input.
 */
void Servo_Axon_Mini_MKII::Actuate(SAsym<float> input) {
    // Calculate the required number of increments for CCR1 and CCR2.
    // Then, actuate to specified values as quickly as possible. We
    // assume that the increments will change little each time due
    // to the high frequency and CPU speed.

    float sf = 5.55555555556f; // µs per degree

    float us1 = 500.0f + (input.S1 + config.AngleOffsetDEGREES) * sf;
    float us2 = 500.0f + (input.S2 + config.AngleOffsetDEGREES) * sf;

    // Convert µs to timer counts (1 µs = 200 counts @ 200 MHz)
    uint32_t ccr1 = (uint32_t)(us1 * 200.0f);
    uint32_t ccr2 = (uint32_t)(us2 * 200.0f);

    TIM3->CCR1 = ccr1;
    TIM3->CCR2 = ccr2;
}

// PUBLIC
/** @brief Init({offset 1, offset 2} (degrees), PRECISION::xxx, true/false); \n
 * Return true if the rotated degree matches commanded degree.
 * Initializing servo outputs PWM to servo pin, then checks
 * the encoder voltage for rotated angle. Matching angle indicates
 * no fault in motor hardware.\n
 *
 * @return bool value:\n
 * return true = no hardware fault\n
 * return false = hardware fault or tolerance not nominal
 */
bool Servo_Axon_Mini_MKII::Init(SAsym<float> AngOffset,
                                PRECISION precision = PRECISION::HUNDREDTH_DEGREE,
                                bool debug = false) {
    CFG_Axon_Mini_MKII configDesired;
    configDesired.AngleOffsetDEGREES = AngOffset;
    configDesired.Precision = precision;
    configDesired.ENABLE_DEBUG = debug;
    config = configDesired;
    float targetErr = 0.02f; // 2% error margin
    float targetAng = 0.0f;
    float increment = 0.0f;
    float sf = 5.55555555556f; // microseconds of pwm per degree rotated

    // We'll work in degrees in Init() just for ease.
    switch (config.Precision){
        case PRECISION::ONE_DEGREE:
            targetAng = 5.0f;
            increment = 1.0f;
            break;

        case PRECISION::TENTH_DEGREE:
            targetAng = 5.5f;
            increment = 0.1f;
            break;

        case PRECISION::HUNDREDTH_DEGREE:
            targetAng = 5.55f;
            increment = 0.01f;
            break;

        case PRECISION::THOUSANTHS_DEGREE:
            targetAng = 5.555f;
            increment = 0.001f;
            break;

        default:
            targetAng = 5.0f;
            increment = 1.0f;
            break;
    }

    //Init TIM3 channels 1 and 2 which are the servos' channels.
    TIM3->ARR = 4000000 - 1;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    float CCR1_us = 500.0f + config.AngleOffsetDEGREES.S1 * sf;
    float CCR2_us = 500.0f + config.AngleOffsetDEGREES.S2 * sf;

    // Scaling factor will give us the precision we need, then we
    // truncate the decimals.
    TIM3->CCR1 = (uint32_t)(CCR1_us * 200.0f);
    TIM3->CCR2 = (uint32_t)(CCR2_us * 200.0f);
    HAL_Delay(10); //Give time to rotate motor

    float testAngle = targetAng;

    // PWM range is from 500us to 2500us. We assume that 0deg is
    // 500us and 360 is 2500us
    float test_us1 = 500.0f + (testAngle + config.AngleOffsetDEGREES.S1) * sf;
    uint32_t testCCR1 = (uint32_t)(test_us1 * 200.0f);
    float test_us2 = 500.0f + (testAngle + config.AngleOffsetDEGREES.S2) * sf;
    uint32_t testCCR2 = (uint32_t)(test_us2 * 200.0f);

    TIM3->CCR1 = testCCR1;
    TIM3->CCR2 = testCCR2;

    HAL_Delay(50);  // again


    data.currentAngle = readCurrentAngle();

    data.trackedError.S1 = fabsf(data.currentAngle.S1 - targetAng) / targetAng;
    data.trackedError.S2 = fabsf(data.currentAngle.S2 - targetAng) / targetAng;

    data.Nominal.S1 = (data.trackedError.S1 <= targetErr);
    data.Nominal.S2 = (data.trackedError.S2 <= targetErr);

    return data.Nominal.S1 && data.Nominal.S2;
}

/** @brief Servo_Axon_Mini_MKII::Update(<float, -180.0f to +180.0f>)\n
 * This function supports more values than 180.0 degrees, but
 * considering the purposes it is used for, more than 45 degrees
 * of rotation are probably unnecessary at all.\n
 *
 * 0.00 degrees corresponds to no angle of attack. Positive
 * angle corresponds to positive roll moment, and vice versa.
 */
void Servo_Axon_Mini_MKII::Update(float S1_Target_Deg, float S2_Target_Deg) {
    data.targetAngle = {S1_Target_Deg, S2_Target_Deg};
    data.currentAngle = readCurrentAngle();
    // Add the target angle to the tracked angle, which is
    // independent from read angle data. This serves as a
    // perfect-scenario angle state.
    data.trackedAngle.S1 += S1_Target_Deg;
    data.trackedAngle.S2 += S2_Target_Deg;

    // Separate actual code to a lower level private function
    Actuate(data.currentAngle);

    // Tracked error builds over time. When this value reaches a
    // certain threshold, we should correct the servo position if
    // possible. Ideally this remains low enough but let's shoot
    // for a max error of 2% deviation. Here we calculate the
    // tracked error, continuously adding on calculated error
    // (positive and negative errors cancel out so we just get the
    // overall error) from each update cycle.
    data.trackedError.S1 += calculateError().S1;
    data.trackedError.S2 += calculateError().S2;
}
//
// Created by dyrel on 2/21/2026.
//

#include "Servo_Axon_Mini_MKII.h"

#include "adc.h"
#include "cmsis_os2.h"
#include "main.h"
#include "tim.h"
#include "Math/Math.h"

static uint32_t Servo_ADC_ENC_Data[2] = {};

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

    // Configure channel with raw1 channel
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        return {};
    }
    /* Convert raw1 */
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100);
    uint32_t raw1 = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);

    // Swap to raw2 channel
    sConfig.Channel = ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        return {};
    }
    /* Convert raw1 */
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100);
    uint32_t raw2 = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);

    // Convert 16-bit ADC to voltage
    float reading1 = (3.3f * raw1) / 65535.0f; // read voltage from SERVO1_ENC_Pin / SERVO1_ENC_GPIO
    float reading2 = (3.3f * raw2) / 65535.0f; // vice versa

    // Linearly map readings to between 0 and 360 degree angles
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
    SAsym<float> value = readCurrentAngle();

    // Guard against divide by zero when trackedAngle is near 0
    float denom1 = fabsf(data.trackedAngle.S1) > 0.001f ? fabsf(data.trackedAngle.S1) : 1.0f;
    float denom2 = fabsf(data.trackedAngle.S2) > 0.001f ? fabsf(data.trackedAngle.S2) : 1.0f;

    SAsym<float> output = {
        (value.S1 - data.trackedAngle.S1) / denom1,
        (value.S2 - data.trackedAngle.S2) / denom2
    };
    return output;
}

/** @brief Actuate({Servo1 input rotation, Servo2 input rotation})
 *  Actuate servos to the specified positional input.
 */
void Servo_Axon_Mini_MKII::Actuate(SAsym<float> input) {
    float sf = 5.55555555556f; // µs per degree

    float us1 = 500.0f + (input.S1 + config.AngleOffsetDEGREES.S1) * sf;
    float us2 = 500.0f + (input.S2 + config.AngleOffsetDEGREES.S2) * sf;

    // 1µs = 2 counts @ 2MHz
    uint32_t ccr1 = (uint32_t)(us1 * 2.0f);
    uint32_t ccr2 = (uint32_t)(us2 * 2.0f);

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
                                PRECISION precision,
                                bool debug) {
    CFG_Axon_Mini_MKII configDesired;
    configDesired.AngleOffsetDEGREES = AngOffset;
    configDesired.Precision = precision;
    configDesired.ENABLE_DEBUG = debug;
    config = configDesired;

    // Calibrate ADC before any reads
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

    // Start TIM3 PWM channels
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // Tolerance based on precision setting
    float tolerance = 0.0f;
    switch (config.Precision) {
        case PRECISION::ONE_DEGREE:         tolerance = 1.0f;    break;
        case PRECISION::TENTH_DEGREE:       tolerance = 0.1f;    break;
        case PRECISION::HUNDREDTH_DEGREE:   tolerance = 0.01f;   break;
        case PRECISION::THOUSANTHS_DEGREE:  tolerance = 0.001f;  break;
        default:                            tolerance = 0.1f;    break;
    }

    // Target is 0deg + offset for both motors
    const float target1 = config.AngleOffsetDEGREES.S1;
    const float target2 = config.AngleOffsetDEGREES.S2;

    const TickType_t timeout   = pdMS_TO_TICKS(3000);  // 3s total
    const TickType_t poll_rate = pdMS_TO_TICKS(5);     // poll at 200Hz
    const TickType_t start     = xTaskGetTickCount();

    TickType_t last_wake = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < timeout) {
        data.currentAngle = readCurrentAngle();

        float err1 = fabsf(data.currentAngle.S1 - target1);
        float err2 = fabsf(data.currentAngle.S2 - target2);

        data.Nominal.S1 = (err1 <= tolerance);
        data.Nominal.S2 = (err2 <= tolerance);

        if (data.Nominal.S1 && data.Nominal.S2) {
            // Both motors at target — stop and return success
            Actuate({target1, target2});
            return true;
        }

        // Drive toward target
        Actuate({target1, target2});

        // Yield to RTOS scheduler, wake at precise interval
        vTaskDelayUntil(&last_wake, poll_rate);
    }

    // Timeout — record final error state
    data.currentAngle = readCurrentAngle();
    data.trackedError.S1 = fabsf(data.currentAngle.S1 - target1);
    data.trackedError.S2 = fabsf(data.currentAngle.S2 - target2);
    data.Nominal.S1 = (data.trackedError.S1 <= tolerance);
    data.Nominal.S2 = (data.trackedError.S2 <= tolerance);

    return data.Nominal.S1 && data.Nominal.S2;
}

/** @brief Servo_Axon_Mini_MKII::Update(<float, -180.0f to +180.0f>)\n
 * This function supports more values than 180.0 degrees, but
 * considering the purposes it is used for, more than 45 degrees
 * of rotation are probably unnecessary at all.\n
 *
 * 0.00 degrees corresponds to no angle of attack. Positive
 * angle corresponds to positive roll moment, and vice versa.
 *
 * @attention There are const values in this functions which should be tuned!!!!!
 */
void Servo_Axon_Mini_MKII::Update(float S1_Target_Deg, float S2_Target_Deg) {
    data.targetAngle = {S1_Target_Deg, S2_Target_Deg};
    data.currentAngle = readCurrentAngle();

    data.trackedAngle.S1 += S1_Target_Deg;
    data.trackedAngle.S2 += S2_Target_Deg;

    const float kP       = 0.3f;   // proportional gain
    const float deadband = 0.5f;   // degrees
    const float alpha    = 0.05f;  // smoothing factor — lower = smoother, higher = more responsive
    // 0.1-0.2 is a good starting range for a rocket servo

    float err1 = data.targetAngle.S1 - data.currentAngle.S1;
    float err2 = data.targetAngle.S2 - data.currentAngle.S2;

    float correction1 = fabsf(err1) > deadband ? kP * err1 : 0.0f;
    float correction2 = fabsf(err2) > deadband ? kP * err2 : 0.0f;

    // Low-pass filter on the correction — exponential moving average
    // smoothed = alpha * new_value + (1 - alpha) * previous_value
    data.smoothedCorrection.S1 = (alpha * correction1) + ((1.0f - alpha) * data.smoothedCorrection.S1);
    data.smoothedCorrection.S2 = (alpha * correction2) + ((1.0f - alpha) * data.smoothedCorrection.S2);

    SAsym<float> corrected = {
        data.targetAngle.S1 + data.smoothedCorrection.S1,
        data.targetAngle.S2 + data.smoothedCorrection.S2
    };

    Actuate(corrected);

    float denom1 = fabsf(data.trackedAngle.S1) > 0.001f ? fabsf(data.trackedAngle.S1) : 1.0f;
    float denom2 = fabsf(data.trackedAngle.S2) > 0.001f ? fabsf(data.trackedAngle.S2) : 1.0f;

    data.trackedError.S1 += fabsf(err1) / denom1;
    data.trackedError.S2 += fabsf(err2) / denom2;
}
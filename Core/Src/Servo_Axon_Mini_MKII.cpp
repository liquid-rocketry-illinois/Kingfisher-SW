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

SAsym<float> Servo_Axon_Mini_MKII::readRawVoltage() {
    // ADC3 is configured in scan mode (NbrOfConversion=2):
    //   Rank 1 = CH0 (PC2_C, S1 encoder)
    //   Rank 2 = CH1 (PC3_C, S2 encoder)
    // Start once, poll twice — no channel reconfiguration needed.
    HAL_ADC_Start(&hadc3);

    uint32_t raw1 = 0, raw2 = 0;
    if (HAL_ADC_PollForConversion(&hadc3, 5) == HAL_OK)
        raw1 = HAL_ADC_GetValue(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 5) == HAL_OK)
        raw2 = HAL_ADC_GetValue(&hadc3);

    HAL_ADC_Stop(&hadc3);

    return {
        (3.3f * static_cast<float>(raw1)) / 4095.0f,
        (3.3f * static_cast<float>(raw2)) / 4095.0f
    };
}

/** @brief readCurrentAngle(void);
 *  Returns angle relative to the zero-voltage reference captured during Init().
 *  0° = servo at calibrated neutral position.
 */
SAsym<float> Servo_Axon_Mini_MKII::readCurrentAngle() {
    SAsym<float> v = readRawVoltage();
    return {
        (v.S1 - _zeroVoltage.S1) * (360.0f / 3.3f),
        (v.S2 - _zeroVoltage.S2) * (360.0f / 3.3f)
    };
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

    float us1 = 1500.0f + (input.S1 + config.AngleOffsetDEGREES.S1) * sf;
    float us2 = 1500.0f + (input.S2 + config.AngleOffsetDEGREES.S2) * sf;

    // 1µs = 2 counts @ 2MHz
    uint32_t ccr1 = (uint32_t)(us1 * 2.0f);
    uint32_t ccr2 = (uint32_t)(us2 * 2.0f);

    TIM3->CCR1 = ccr1;
    TIM3->CCR2 = ccr2;
}

// PUBLIC

Servo_Axon_Mini_MKII::Servo_Axon_Mini_MKII() {

}


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
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    // Start TIM3 PWM channels
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // Drive to mechanical center, let servo settle, then capture zero reference.
    // _zeroVoltage defaults to 1.65V so readCurrentAngle() is valid during the settle wait.
    Actuate({0.0f, 0.0f});
    vTaskDelay(pdMS_TO_TICKS(200));
    SAsym<float> zv = readRawVoltage();
    _zeroVoltage.S1 = zv.S1;
    _zeroVoltage.S2 = zv.S2;

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
    float target1 = config.AngleOffsetDEGREES.S1;
    float target2 = config.AngleOffsetDEGREES.S2;

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
    data.targetAngle  = {S1_Target_Deg, S2_Target_Deg};
    data.trackedAngle = {S1_Target_Deg, S2_Target_Deg};

    // Open-loop: Actuate() maps angle→PWM directly via the 1500µs center formula.
    // Closed-loop correction is disabled because readCurrentAngle() returns
    // absolute encoder position in 0–360° while targets are deflection angles
    // in a different coordinate frame. Without encoder calibration the error
    // term is meaningless and drives large spurious corrections causing jitter.
    Actuate({S1_Target_Deg, S2_Target_Deg});

    data.currentAngle       = readCurrentAngle();
    data.smoothedCorrection = {0.0f, 0.0f};

    // trackedError: raw encoder deviation from target (informational only)
    data.trackedError.S1 = fabsf(data.currentAngle.S1 - S1_Target_Deg);
    data.trackedError.S2 = fabsf(data.currentAngle.S2 - S2_Target_Deg);
}

SAsym<float> Servo_Axon_Mini_MKII::getCurrentAngle()
{
    return readCurrentAngle();
};

void Servo_Axon_Mini_MKII::SetOffset(SAsym<float> offset) {
    config.AngleOffsetDEGREES = offset;
}

DATA_Axon_Mini_MKII Servo_Axon_Mini_MKII::getData() const {
    return data;
}

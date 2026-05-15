//
// Created by dyrel on 2/23/2026.
//

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "TEST_FUNCTIONS.h"

#include "task.h"
#include <cstdio>

#include "timing.h"

TEST::TEST(){;}

//TODO
//Need to change code of PWM generation. verify correct pwm signal
int TEST::SERVO_TEST() {
    Servo_Axon_Mini_MKII servoSet;
    bool initState = servoSet.Init({0,0}, PRECISION::TENTH_DEGREE, true);
    vTaskDelay(500);

    // Track status of servos
    static int status = 0;
    if (status == 0 && initState) {
        while (1) {
            servoSet.Update(90.0F, -33.0F);
            vTaskDelay(1000);
            servoSet.Update(-45.0F, 45.0F);
            vTaskDelay(1000);
            servoSet.Update(0.0F, 0.0F);
        }
    }

    return status;
}

int TEST::IMU_TEST() {
    IMUs IMU_ENGINE = IMUs();
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    IMUsStatus sensorStatuses = IMU_ENGINE.Init(true);

    while (1) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        sensorStatuses = IMU_ENGINE.Update();
        vTaskDelay(1);
    }
}

int TEST::BARO_TEST()
{
    Baro_Unified BaroEngine;
    auto init = BaroEngine.Init(true);

    while (1)
    {
        auto status = BaroEngine.Update();
        auto data = BaroEngine.getData();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        //vTaskDelay(100); TODO: vTaskDelay causing hard fault- memory bad? overflow? div by 0?
        HAL_Delay(100);
    }
}

int TEST::LIS2_TEST()
{
    Magnetometer mag;
    LIS2_Data data;
    uint8_t status = 0;
    uint8_t init = mag.Init();

    while (1)
    {
        status = mag.Update();
        data = mag.getRawData();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(100);
    }
}

int TEST::RADIO_TEST() {
    Telemetry telem = Telemetry(); // or TelemetryMode::TELEMETRY_MODE_FLIGHT
    uint8_t status = 0;
    uint8_t init = telem.Init(TelemetryMode::TELEMETRY_MODE_FLIGHT);

    Baro_Unified baro = Baro_Unified();
    baro.Init(true);

    while (1) {
        baro.Update();
        telem.HALOutData.altitude = baro.getData().Filtered.heightMeters;
        status = telem.Update();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(10);
    }
}

int TEST::PYRO_TEST()
{

    vTaskDelay(pdMS_TO_TICKS(1000));

    HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_SET);
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    vTaskDelay(pdMS_TO_TICKS(5000));
    HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_RESET);

    while (1) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

int TEST::SD_TEST()
{
    // Phase 1 — Init
    int8_t res = SD_Init();
    if (res != 0) {
        // Fast blink: init failed
        while (1) {
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    uint32_t count = 0;

    // Phase 2 — Write loop (10 iterations)
    while (count < 10) {

        // Write header on first pass only
        if (count == 0) {
            res = SD_LogNewline("=== SD TEST START ===");
            if (res != 0) {
                while (1) {
                    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
        }

        // Write a GPS row with fake but distinct values
        res = SD_LogGPS(
            43.4723000 + count * 0.0000001,   // lat
            -80.5449000 + count * 0.0000001,  // lon
            330.00 + count * 0.5,             // alt
            12,                               // hour
            0,                                // min
            (uint8_t)(count * 6)              // sec (0, 6, 12, ... 54)
        );
        if (res != 0) {
            while (1) {
                HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        // Write inline counter string to verify inline append
        char buf[32];
        snprintf(buf, sizeof(buf), "row:%lu", count);
        res = SD_LogInline(buf);
        if (res != 0) {
            while (1) {
                HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(pdMS_TO_TICKS(1000));
        count++;
    }

    // Phase 3 — Clean close
    SD_LogNewline("=== SD TEST END ===");
    SD_Close();

    // Slow blink: test completed cleanly, safe to remove card
    while (1) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    return 0;
}

void TEST::NoiseTest() {
    IMUs IMU_ENGINE = IMUs();
    int8_t status = 0;
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    status = SD_Init();
    IMUsStatus Sstatus = IMU_ENGINE.Init(true);

    SD_LogNewline("\n"); SD_LogNewline("\n");
    SD_LogNewline("BEGINNING NOISE DATA");

    while (1) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        Sstatus = IMU_ENGINE.Update();
        char buf[200];

        // IMU line: timestamp, stage, altitude, vertical velocity, acceleration, angular velocity
        snprintf(buf, sizeof(buf),
            "T=%lu AX=%.5f AY=%.5f AZ=%.5f GX=%.5f GY=%.5f GZ=%.5f",
            static_cast<unsigned long>(millis()),
            IMU_ENGINE.getVotedBMI().accel_linear.x,
            IMU_ENGINE.getVotedBMI().accel_linear.y,
            IMU_ENGINE.getVotedBMI().accel_linear.z,
            IMU_ENGINE.getVotedBMI().ang_vel.x,
            IMU_ENGINE.getVotedBMI().ang_vel.y,
            IMU_ENGINE.getVotedBMI().ang_vel.z);
        SD_LogNewline(buf);
    }
}


//
// Created by dyrel on 2/26/2026.
//

#ifndef KINGFISHER_SW_IMU_ISM6HGX_H
#define KINGFISHER_SW_IMU_ISM6HGX_H

#include "ism6hg256x_reg.h"
#include "main.h"
#include "Math/Vector3D.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"

struct IMU_Data {
    Vector3D<float> acceleration;
    Vector3D<float> angular_velocity;
};

class IMU_ISM6HGx {
    static IMU_Data data;

    static int16_t data_raw_motion[3];
    static int16_t data_raw_temperature;
    static float_t temperature_degC;
    static uint8_t whoamI;
    static uint8_t tx_buffer[1000];
    static ism6hg256x_filt_settling_mask_t filt_settling_mask;

    static ism6hg256x_pin_int_route_hg_t pin_int;
    static double_t lowg_xl_sum[3], hg_xl_sum[3], gyro_sum[3], temp_sum;
    static float_t acceleration_mg[3], angular_rate_mdps[3]; // Arrays for output data.
    static uint16_t lowg_xl_cnt, hg_xl_cnt, gyro_cnt, temp_cnt;

    static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
    static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                                 uint16_t len);
    static void platform_delay(uint32_t ms);
    static void platform_init();

    static   stmdev_ctx_t dev_ctx;
    static   uint8_t lg_xl_data_valid;
    static   uint8_t hg_xl_data_valid;
    static   uint8_t gyro_data_valid;
    static   uint8_t temp_data_valid;
    static   uint8_t thread_wake;

    static void cs_low();
    static void cs_high();

public:
    static void ism6hg256x_read_data_drdy_handler();
    static int Init();
    static void Update();
    IMU_Data GetData();
};

#endif //KINGFISHER_SW_IMU_ISM6HGX_H
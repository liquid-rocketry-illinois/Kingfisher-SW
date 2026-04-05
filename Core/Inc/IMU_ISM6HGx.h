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

struct ISM_Data {
    Vector3D<float> acceleration;
    Vector3D<float> accelerationHighG;
    Vector3D<float> angular_velocity;
    float temperature_degC;
};

class IMU_ISM6HGx {
    ISM_Data data;

    int16_t data_raw_motion[3];
    int16_t data_raw_temperature;
    float_t temperature_degC;
    uint8_t whoamI;
    uint8_t tx_buffer[1000];
    ism6hg256x_filt_settling_mask_t filt_settling_mask;

    ism6hg256x_pin_int_route_hg_t pin_int;
    double_t lowg_xl_sum[3], hg_xl_sum[3], gyro_sum[3], temp_sum;
    float_t acceleration_mg[3], angular_rate_mdps[3]; // Arrays for output data.
    uint16_t lowg_xl_cnt, hg_xl_cnt, gyro_cnt, temp_cnt;

  stmdev_ctx_t dev_ctx;
  uint8_t lg_xl_data_valid;
  uint8_t hg_xl_data_valid;
  uint8_t gyro_data_valid;
  uint8_t temp_data_valid;
  static volatile uint8_t thread_wake;

public:
    IMU_ISM6HGx();
    static void ism6hg256x_read_data_drdy_handler();
    int8_t Init();
    int8_t Update();
    ISM_Data GetData();
};

#endif //KINGFISHER_SW_IMU_ISM6HGX_H
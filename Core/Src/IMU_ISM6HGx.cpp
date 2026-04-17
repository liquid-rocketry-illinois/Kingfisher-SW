//
// Created by dyrel on 2/26/2026.
//

#include "ism6hg256x_reg.h"
#include "main.h"
#include "IMU_ISM6HGx.h"

#include <cstring>

#include "cmsis_os2.h"
#include "../Inc/ISM6_Platform.h"

#define CNT_FOR_OUTPUT 100
#define SENSOR_BUS hspi3
volatile uint8_t IMU_ISM6HGx::thread_wake = 0;

// PUBLIC

IMU_ISM6HGx::IMU_ISM6HGx() {}

void IMU_ISM6HGx::ism6hg256x_read_data_drdy_handler() {
    thread_wake = 1;
}

int8_t IMU_ISM6HGx::Init() {
    /* Initialize mems driver interface */
    dev_ctx.write_reg = ISM_SPI_Write;
    dev_ctx.read_reg  = ISM_SPI_Read;
    dev_ctx.mdelay    = ISM_platform_MSdelay;
    dev_ctx.handle    = &SENSOR_BUS;

    /* Init test platform */
    Platform_Init();

    /* Perform device power-on-reset */
    ism6hg256x_sw_por(&dev_ctx);

    /* Wait sensor boot time */
    ISM_platform_MSdelay(10);

    /* Check device ID */
    ism6hg256x_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != ISM6HG256X_ID)
        return -1;

    /* Enable Block Data Update */
    ism6hg256x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set Output Data Rate */
    ism6hg256x_xl_setup(&dev_ctx, ISM6HG256X_ODR_AT_60Hz, ISM6HG256X_XL_HIGH_PERFORMANCE_MD);
    ism6hg256x_hg_xl_data_rate_set(&dev_ctx, ISM6HG256X_HG_XL_ODR_AT_960Hz, 1);
    ism6hg256x_gy_setup(&dev_ctx, ISM6HG256X_ODR_AT_120Hz, ISM6HG256X_GY_HIGH_PERFORMANCE_MD);

    /* Set full scale */
    ism6hg256x_xl_full_scale_set(&dev_ctx, ISM6HG256X_2g);
    ism6hg256x_hg_xl_full_scale_set(&dev_ctx, ISM6HG256X_256g);
    ism6hg256x_gy_full_scale_set(&dev_ctx, ISM6HG256X_2000dps);

    /* Configure filtering chain */
    filt_settling_mask.drdy   = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g  = PROPERTY_ENABLE;
    ism6hg256x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
    ism6hg256x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    ism6hg256x_filt_gy_lp1_bandwidth_set(&dev_ctx, ISM6HG256X_GY_ULTRA_LIGHT);
    ism6hg256x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    ism6hg256x_filt_xl_lp2_bandwidth_set(&dev_ctx, ISM6HG256X_XL_STRONG);

    /* Init accumulators */
    lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0f;
    hg_xl_sum[0]   = hg_xl_sum[1]   = hg_xl_sum[2]   = 0.0f;
    gyro_sum[0]    = gyro_sum[1]    = gyro_sum[2]    = 0.0f;
    temp_sum       = 0.0f;

    lowg_xl_cnt = 0;
    hg_xl_cnt   = 0;
    gyro_cnt    = 0;
    temp_cnt    = 0;

    /* Enable interrupt on High-G XL DRDY -> INT1 */
    pin_int = {0};
    pin_int.drdy_hg_xl = PROPERTY_ENABLE;
    ism6hg256x_pin_int1_route_hg_set(&dev_ctx, &pin_int);

    return 0;
}

int8_t IMU_ISM6HGx::Update()
{
    // Consume pending interrupt flag
    if (thread_wake) {
        ism6hg256x_data_ready_t status;

        thread_wake = 0;

        ism6hg256x_flag_data_ready_get(&dev_ctx, &status);

        /* Low-G accelerometer */
        if (status.drdy_xl) {
            ism6hg256x_acceleration_raw_get(&dev_ctx, data_raw_motion);

            lowg_xl_sum[0] += ism6hg256x_from_fs2_to_mg(data_raw_motion[0]);
            lowg_xl_sum[1] += ism6hg256x_from_fs2_to_mg(data_raw_motion[1]);
            lowg_xl_sum[2] += ism6hg256x_from_fs2_to_mg(data_raw_motion[2]);
            lowg_xl_cnt++;
        }

        /* High-G accelerometer */
        if (status.drdy_hgxl) {
            ism6hg256x_hg_acceleration_raw_get(&dev_ctx, data_raw_motion);

            hg_xl_sum[0] += ism6hg256x_from_fs256_to_mg(data_raw_motion[0]);
            hg_xl_sum[1] += ism6hg256x_from_fs256_to_mg(data_raw_motion[1]);
            hg_xl_sum[2] += ism6hg256x_from_fs256_to_mg(data_raw_motion[2]);
            hg_xl_cnt++;
        }

        /* Gyroscope */
        if (status.drdy_gy) {
            ism6hg256x_angular_rate_raw_get(&dev_ctx, data_raw_motion);

            gyro_sum[0] += ism6hg256x_from_fs2000_to_mdps(data_raw_motion[0]);
            gyro_sum[1] += ism6hg256x_from_fs2000_to_mdps(data_raw_motion[1]);
            gyro_sum[2] += ism6hg256x_from_fs2000_to_mdps(data_raw_motion[2]);
            gyro_cnt++;
        }

        /* Temperature */
        if (status.drdy_temp) {
            ism6hg256x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
            temp_sum += ism6hg256x_from_lsb_to_celsius(data_raw_temperature);
            temp_cnt++;
        }
    }

    // Output averaged data once enough low-G samples have accumulated
    // (low-G is the lowest ODR at 60 Hz, so it is the pacing reference)
    if (lowg_xl_cnt >= CNT_FOR_OUTPUT) {

        // Low-G accelerometer average (mg)
        data.acceleration.x = lowg_xl_sum[0] / lowg_xl_cnt;
        data.acceleration.y = lowg_xl_sum[1] / lowg_xl_cnt;
        data.acceleration.z = lowg_xl_sum[2] / lowg_xl_cnt;
        lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0f;
        lowg_xl_cnt = 0;

        // High-G accelerometer average (mg)
        if (hg_xl_cnt > 0) {
            data.accelerationHighG.x = hg_xl_sum[0] / hg_xl_cnt;
            data.accelerationHighG.y = hg_xl_sum[1] / hg_xl_cnt;
            data.accelerationHighG.z = hg_xl_sum[2] / hg_xl_cnt;
            hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0f;
            hg_xl_cnt = 0;
        }

        // Gyroscope average (mdps)
        if (gyro_cnt > 0) {
            data.angular_velocity.x = gyro_sum[0] / gyro_cnt;
            data.angular_velocity.y = gyro_sum[1] / gyro_cnt;
            data.angular_velocity.z = gyro_sum[2] / gyro_cnt;
            gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0f;
            gyro_cnt = 0;
        }

        // Temperature average (°C)
        if (temp_cnt > 0) {
            data.temperature_degC = temp_sum / temp_cnt;
            temp_sum = 0.0f;
            temp_cnt = 0;
        }

        return 0;   // new averaged data is ready
    }

    return 1;       // still accumulating data
}

ISM_Data IMU_ISM6HGx::GetData() {
    return data;
}
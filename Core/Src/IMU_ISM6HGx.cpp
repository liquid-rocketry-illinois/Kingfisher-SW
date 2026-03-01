//
// Created by dyrel on 2/26/2026.
//

#include "ism6hg256x_reg.h"
#include "main.h"
#include "IMU_ISM6HGx.h"
#include "cmsis_os2.h"

#define CNT_FOR_OUTPUT 100
#define SENSOR_BUS hspi3

// PRIVATE

void IMU_ISM6HGx::cs_low()
{
    HAL_GPIO_WritePin(CS_ISM_GPIO_Port, CS_ISM_Pin, GPIO_PIN_RESET);
}

void IMU_ISM6HGx::cs_high()
{
    HAL_GPIO_WritePin(CS_ISM_GPIO_Port, CS_ISM_Pin, GPIO_PIN_SET);
}

void IMU_ISM6HGx::platform_init() {
    // Nothing required if SPI and GPIO already initialized in CubeMX.
    // This is here in case init code is needed in addition to everything
    // here but nothing usually is needed... Continuing to monitor.
}

int32_t IMU_ISM6HGx::platform_write(void *handle,
                                    uint8_t reg,
                                    const uint8_t *bufp,
                                    uint16_t len)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef*)handle;

    uint8_t tx_reg = reg & 0x7F;   // Write
    if (len > 1)
        tx_reg |= 0x40;            // Auto-increment

    cs_low();

    HAL_SPI_Transmit(hspi, &tx_reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(hspi, (uint8_t*)bufp, len, HAL_MAX_DELAY);

    cs_high();

    return 0;
}

int32_t IMU_ISM6HGx::platform_read(void *handle,
                                   uint8_t reg,
                                   uint8_t *bufp,
                                   uint16_t len)
{
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef*)handle;

    uint8_t tx_reg = reg | 0x80;   // Read
    if (len > 1)
        tx_reg |= 0x40;            // Auto-increment

    cs_low();

    HAL_SPI_Transmit(hspi, &tx_reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, bufp, len, HAL_MAX_DELAY);

    cs_high();

    return 0;
}

void IMU_ISM6HGx::platform_delay(uint32_t ms)
{
    osDelay(ms);
}

// PUBLIC

void IMU_ISM6HGx::ism6hg256x_read_data_drdy_handler() {
    thread_wake = 1;
}

int IMU_ISM6HGx::Init() {
    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &SENSOR_BUS;

    /* Init test platform */
    platform_init();

    /* Wait sensor boot time */
    platform_delay(10);

    /* Check device ID */
    ism6hg256x_device_id_get(&dev_ctx, &whoamI);

    if (whoamI != ISM6HG256X_ID)
        return -1;

    /* Perform device power-on-reset */
    ism6hg256x_sw_por(&dev_ctx);

    /* Enable Block Data Update */
    ism6hg256x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set Output Data Rate.
     * Selected data rate have to be equal or greater with respect
     * with MLC data rate.
     */
    ism6hg256x_xl_setup(&dev_ctx, ISM6HG256X_ODR_AT_60Hz, ISM6HG256X_XL_HIGH_PERFORMANCE_MD);
    ism6hg256x_hg_xl_data_rate_set(&dev_ctx, ISM6HG256X_HG_XL_ODR_AT_960Hz, 1);
    ism6hg256x_gy_setup(&dev_ctx, ISM6HG256X_ODR_AT_120Hz, ISM6HG256X_GY_HIGH_PERFORMANCE_MD);

    /* Set full scale */
    ism6hg256x_xl_full_scale_set(&dev_ctx, ISM6HG256X_2g);
    ism6hg256x_hg_xl_full_scale_set(&dev_ctx, ISM6HG256X_256g);
    ism6hg256x_gy_full_scale_set(&dev_ctx, ISM6HG256X_2000dps);

    /* Configure filtering chain */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    ism6hg256x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
    ism6hg256x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    ism6hg256x_filt_gy_lp1_bandwidth_set(&dev_ctx, ISM6HG256X_GY_ULTRA_LIGHT);
    ism6hg256x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    ism6hg256x_filt_xl_lp2_bandwidth_set(&dev_ctx, ISM6HG256X_XL_STRONG);

    // Init static data arrays
    lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
    hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
    gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
    temp_sum = 0.0;

    // Init static counters
    lowg_xl_cnt = 0; hg_xl_cnt = 0; gyro_cnt = 0; temp_cnt = 0;

    /* enable interrupt on High-G XL (sensor at highest frequency) */
    // Here we enable high g detection by setting drdy_hg_xl to ENABLE
    pin_int = {0};
    pin_int.drdy_hg_xl = PROPERTY_ENABLE;
    ism6hg256x_pin_int1_route_hg_set(&dev_ctx, &pin_int);
    //ism6hg256x_pin_int2_route_hg_set(&dev_ctx, &pin_int);

    return 0;
}

void IMU_ISM6HGx::Update()
{
    // Only attempt getting data if ready
    if (thread_wake) {
        ism6hg256x_data_ready_t status;

        thread_wake = 0;

        /* Read output only if new xl value is available */
        ism6hg256x_flag_data_ready_get(&dev_ctx, &status);

        hg_xl_data_valid = status.drdy_hgxl;
        lg_xl_data_valid = status.drdy_xl;
        gyro_data_valid = status.drdy_gy;
        temp_data_valid = status.drdy_temp;

        if (lg_xl_data_valid) {
            lg_xl_data_valid = 0;

            /* Read acceleration field data */
            ism6hg256x_acceleration_raw_get(&dev_ctx, data_raw_motion);
            data.acceleration.x = ism6hg256x_from_fs2_to_mg(data_raw_motion[0]);
            data.acceleration.y = ism6hg256x_from_fs2_to_mg(data_raw_motion[1]);
            data.acceleration.z = ism6hg256x_from_fs2_to_mg(data_raw_motion[2]);

            lowg_xl_sum[0] += data.acceleration.x;
            lowg_xl_sum[1] += data.acceleration.y;
            lowg_xl_sum[2] += data.acceleration.z;
            lowg_xl_cnt++;
            }

        if (hg_xl_data_valid) {
            hg_xl_data_valid = 0;

            /* Read acceleration field data */
            ism6hg256x_hg_acceleration_raw_get(&dev_ctx, data_raw_motion);
            acceleration_mg[0] = ism6hg256x_from_fs256_to_mg(data_raw_motion[0]);
            acceleration_mg[1] = ism6hg256x_from_fs256_to_mg(data_raw_motion[1]);
            acceleration_mg[2] = ism6hg256x_from_fs256_to_mg(data_raw_motion[2]);

            hg_xl_sum[0] += acceleration_mg[0];
            hg_xl_sum[1] += acceleration_mg[1];
            hg_xl_sum[2] += acceleration_mg[2];
            hg_xl_cnt++;
            }

        /* Read output only if new xl value is available */
        if (gyro_data_valid) {
            gyro_data_valid = 0;

            /* Read angular rate field data */
            ism6hg256x_angular_rate_raw_get(&dev_ctx, data_raw_motion);
            angular_rate_mdps[0] = ism6hg256x_from_fs2000_to_mdps(data_raw_motion[0]);
            angular_rate_mdps[1] = ism6hg256x_from_fs2000_to_mdps(data_raw_motion[1]);
            angular_rate_mdps[2] = ism6hg256x_from_fs2000_to_mdps(data_raw_motion[2]);

            gyro_sum[0] += angular_rate_mdps[0];
            gyro_sum[1] += angular_rate_mdps[1];
            gyro_sum[2] += angular_rate_mdps[2];
            gyro_cnt++;
            }

        if (temp_data_valid) {
            temp_data_valid = 0;

            /* Read temperature data */
            ism6hg256x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
            temperature_degC = ism6hg256x_from_lsb_to_celsius(data_raw_temperature);
            temp_sum += temperature_degC;
            temp_cnt++;
            }
        }

        if (lowg_xl_cnt >= CNT_FOR_OUTPUT) {
            // Set avg accel data low-g
            if (lowg_xl_cnt > 0) {
                acceleration_mg[0] = lowg_xl_sum[0] / lowg_xl_cnt;
                acceleration_mg[1] = lowg_xl_sum[1] / lowg_xl_cnt;
                acceleration_mg[2] = lowg_xl_sum[2] / lowg_xl_cnt;
            }

            // Reset for next run
            lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
            lowg_xl_cnt = 0;

            // Set avg accel high-g
            if (hg_xl_cnt > 0) {
                acceleration_mg[0] = hg_xl_sum[0] / hg_xl_cnt;
                acceleration_mg[1] = hg_xl_sum[1] / hg_xl_cnt;
                acceleration_mg[2] = hg_xl_sum[2] / hg_xl_cnt;
            }

            // Reset for next pass
            hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
            hg_xl_cnt = 0;

            // Set avg gyro data
            if (gyro_cnt > 0) {
                angular_rate_mdps[0] = gyro_sum[0] / gyro_cnt;
                angular_rate_mdps[1] = gyro_sum[1] / gyro_cnt;
                angular_rate_mdps[2] = gyro_sum[2] / gyro_cnt;
            }

            // Reset for next pass
            gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
            gyro_cnt = 0;

            // Set avg temperature data, reset for next pass
            temperature_degC = temp_sum / temp_cnt;
            temp_cnt = 0;
            temp_sum = 0.0;
        }
}

IMU_Data IMU_ISM6HGx::GetData() {
    return data;
};



// STATIC VARS

IMU_Data IMU_ISM6HGx::data;

int16_t IMU_ISM6HGx::data_raw_motion[3];
int16_t IMU_ISM6HGx::data_raw_temperature;
float_t IMU_ISM6HGx::temperature_degC;
uint8_t IMU_ISM6HGx::whoamI;
uint8_t IMU_ISM6HGx::tx_buffer[1000];

ism6hg256x_filt_settling_mask_t IMU_ISM6HGx::filt_settling_mask;
ism6hg256x_pin_int_route_hg_t IMU_ISM6HGx::pin_int;

double_t IMU_ISM6HGx::lowg_xl_sum[3];
double_t IMU_ISM6HGx::hg_xl_sum[3];
double_t IMU_ISM6HGx::gyro_sum[3];
double_t IMU_ISM6HGx::temp_sum;

float_t IMU_ISM6HGx::acceleration_mg[3];
float_t IMU_ISM6HGx::angular_rate_mdps[3];

uint16_t IMU_ISM6HGx::lowg_xl_cnt;
uint16_t IMU_ISM6HGx::hg_xl_cnt;
uint16_t IMU_ISM6HGx::gyro_cnt;
uint16_t IMU_ISM6HGx::temp_cnt;

stmdev_ctx_t IMU_ISM6HGx::dev_ctx;

uint8_t IMU_ISM6HGx::lg_xl_data_valid;
uint8_t IMU_ISM6HGx::hg_xl_data_valid;
uint8_t IMU_ISM6HGx::gyro_data_valid;
uint8_t IMU_ISM6HGx::temp_data_valid;
uint8_t IMU_ISM6HGx::thread_wake;
//
// Created by dyrel on 3/5/2026.
//

#include "SIMU_BMI323.h"

SIMU_BMI323::SIMU_BMI323()
{
    _sensor_active = false;

    _raw.accel_linear = {0,0,0};
    _raw.ang_vel = {0,0,0};
}

int SIMU_BMI323::Init(BMI_INDEX DeviceNum)
{
    _obj.index = DeviceNum;
    _obj.SPIbus = &hspi4;

    /* Map sensor index to chip select pins */
    switch(_obj.index)
    {
        case SENSOR1_I:
            _obj.cs_port = SIMU_CS1_GPIO_Port;
            _obj.cs_pin  = SIMU_CS1_Pin;
            break;

        case SENSOR2_I:
            _obj.cs_port = SIMU_CS2_GPIO_Port;
            _obj.cs_pin  = SIMU_CS2_Pin;
            break;

        case SENSOR3_I:
            _obj.cs_port = SIMU_CS3_GPIO_Port;
            _obj.cs_pin  = SIMU_CS3_Pin;
            break;

        default:
            return 1;
    }

    device.intf = BMI3_SPI_INTF;
    device.intf_ptr = &_obj;

    device.read = BMI3_SPI_Read;
    device.write = BMI3_SPI_Write;
    device.delay_us = BMI323_delay_us;

    if(bmi323_init(&device) != BMI3_OK)
    {
        return 2;
    }

    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_GYRO;

    if(bmi323_get_sensor_config(config, 2, &device) != BMI3_OK)
        return 3;

    map_int.acc_drdy_int = BMI3_INT1;
    map_int.gyr_drdy_int = BMI3_INT1;
    map_int.temp_drdy_int = BMI3_INT1;

    if (bmi323_map_interrupt(map_int, &device) != BMI3_OK) {
        return 4;
    }

    config[0].cfg.acc.odr = BMI3_ACC_ODR_50HZ;
    config[0].cfg.acc.range = BMI3_ACC_RANGE_16G;
    config[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;
    config[0].cfg.acc.avg_num = BMI3_ACC_AVG64;
    config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    config[1].cfg.gyr.odr = BMI3_GYR_ODR_50HZ;
    config[1].cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;
    config[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;
    config[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
    config[1].cfg.gyr.avg_num = BMI3_GYR_AVG1;

    if (bmi323_set_sensor_config(config, 2, &device) != BMI3_OK)
        return 5;

    _sensor_active = true;
    return 0;
}

int SIMU_BMI323::Update()
{
    int rslt = BMI323_OK;

    if(!_sensor_active)
    {
        return -999;
    }

    uint16_t limit = 10;
    bmi3_sensor_data sensor_data[3] = {0};
    uint16_t int_status = 0;
    float temperature_value; // use temp for sense calibration

    uint8_t indx = 0;
    float acc_x = 0, acc_y = 0, acc_z = 0;
    float gyr_x = 0, gyr_y = 0, gyr_z = 0;

    /* Select accel and gyro sensor. */
    sensor_data[0].type = BMI323_ACCEL;
    sensor_data[1].type = BMI323_GYRO;
    sensor_data[2].type = BMI323_TEMP;

    while (indx <= limit)
    {
        /* To get the status of accel data ready interrupt. */
        rslt = bmi323_get_int1_status(&int_status, &device);

        if ((int_status & BMI3_INT_STATUS_ACC_DRDY) && (int_status & BMI3_INT_STATUS_GYR_DRDY) &&
            (int_status & BMI3_INT_STATUS_TEMP_DRDY))
        {
            /* Get accelerometer data for x, y and z axis. */
            rslt = bmi323_get_sensor_data(sensor_data, 3, &device);

            /* Converting lsb to gravity for 16 bit accelerometer at 16G range. */
            acc_x = lsb_to_g(sensor_data[0].sens_data.acc.x, 16.0f, device.resolution);
            acc_y = lsb_to_g(sensor_data[0].sens_data.acc.y, 16.0f, device.resolution);
            acc_z = lsb_to_g(sensor_data[0].sens_data.acc.z, 16.0f, device.resolution);

            /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
            gyr_x = lsb_to_dps(sensor_data[1].sens_data.gyr.x, (float)2000, device.resolution);
            gyr_y = lsb_to_dps(sensor_data[1].sens_data.gyr.y, (float)2000, device.resolution);
            gyr_z = lsb_to_dps(sensor_data[1].sens_data.gyr.z, (float)2000, device.resolution);

            temperature_value =
                (float)((((float)((int16_t)sensor_data[2].sens_data.temp.temp_data)) / 512.0) + 23.0);

            indx++;
        }
    }

    _raw.accel_linear.x = acc_x;
    _raw.accel_linear.y = acc_y;
    _raw.accel_linear.z = acc_z;
    _raw.ang_vel.x = gyr_x;
    _raw.ang_vel.y = gyr_y;
    _raw.ang_vel.z = gyr_z;

    float tempForDBG = temperature_value;

    return rslt;
}

// Return the raw data calculated
BMI_Data SIMU_BMI323::getRawData()
{
    return _raw;
}



// HELPER FUNCS

float SIMU_BMI323::lsb_to_g(int16_t val, float g_range, uint8_t bit_width){
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (val * g_range) / half_scale;
}

float SIMU_BMI323::lsb_to_dps(int16_t val, float dps, uint8_t bit_width) {
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
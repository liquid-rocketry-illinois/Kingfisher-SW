//
// Created by dyrel on 2/28/2026.
//

#include "main.h"
#include "Sensors.h"

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ISM_INT_Pin)  // Replace with your actual INT pin
    {
        IMU_ISM6HGx::ism6hg256x_read_data_drdy_handler();
    }

    if (GPIO_Pin == INT1_Pin) {

    }

    if (GPIO_Pin == INT2_Pin) {

    }

    if (GPIO_Pin == INT3_Pin) {

    }

    if (GPIO_Pin == BMP390_INT1_Pin) {

    }

    if (GPIO_Pin == BMP390_INT2_Pin) {

    }

    if (GPIO_Pin == BMP390_INT3_Pin) {

    }
}
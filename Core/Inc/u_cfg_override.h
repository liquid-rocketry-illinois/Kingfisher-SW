//
// Created by dyrel on 2/15/2026.
//

#ifndef KINGFISHER_SW_U_CFG_OVERRIDE_H
#define KINGFISHER_SW_U_CFG_OVERRIDE_H

    #ifndef U_CFG_OVERRIDE_H
    #define U_CFG_OVERRIDE_H

    #define U_CFG_OS_TYPE U_OS_TYPE_FREERTOS
    #define U_CFG_PLATFORM U_PORT_PLATFORM_STM32_CUBE

    #define U_CFG_APP_NEED_GNSS 1
    #define U_CFG_APP_NEED_CELL 0
    #define U_CFG_APP_NEED_SHORT_RANGE 0
    #define U_CFG_APP_NEED_MQTT 0
    #define U_CFG_APP_NEED_WIFI 0
    #define U_CFG_APP_NEED_THREADX 0
    #define U_CFG_APP_NEED_BLE 0

    #endif

#endif //KINGFISHER_SW_U_CFG_OVERRIDE_H
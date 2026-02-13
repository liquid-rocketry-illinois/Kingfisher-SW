//
// Created by dyrel on 2/12/2026.
//
#define U_CFG_GNSS_ENABLE 1
#define U_CFG_ENABLE_LOGGING 0

#include "GPS.h"

extern "C" {
#include "u_device.h"
#include "u_gnss.h"
#include "u_port.h"
#include "u_port_i2c.h"
#include "u_error_common.h"
}

#define GNSS_I2C_INSTANCE   1
#define GNSS_I2C_CLOCK      400000

GnssSensor::GnssSensor()
    : gnssHandle(nullptr),
      initialized(false)
{
    currentData.longitude = 0.0;
    currentData.latitude  = 0.0;
}

GnssSensor::~GnssSensor()
{
    if (gnssHandle != nullptr)
    {
        uDeviceClose((uDeviceHandle_t)gnssHandle, false);
        gnssHandle = nullptr;
    }

    uDeviceDeinit();
    uPortDeinit();
}

bool GnssSensor::Init()
{
    if (uPortInit() != 0)
        return false;

    if (uDeviceInit() != 0)
        return false;

    uDeviceCfg_t devCfg = {};
    devCfg.deviceType = U_DEVICE_TYPE_GNSS;
    devCfg.deviceCfg.cfgGnss.moduleType = U_GNSS_MODULE_TYPE_M10;
    devCfg.deviceCfg.cfgGnss.transportType = U_GNSS_TRANSPORT_I2C;

    devCfg.deviceCfg.cfgGnss.transportCfg.cfgI2c.i2c = GNSS_I2C_INSTANCE;
    devCfg.deviceCfg.cfgGnss.transportCfg.cfgI2c.pinSda = -1;
    devCfg.deviceCfg.cfgGnss.transportCfg.cfgI2c.pinScl = -1;
    devCfg.deviceCfg.cfgGnss.transportCfg.cfgI2c.clockHertz = GNSS_I2C_CLOCK;
    devCfg.deviceCfg.cfgGnss.transportCfg.cfgI2c.alreadyOpen = false;

    if (uDeviceOpen(&devCfg, (uDeviceHandle_t*)&gnssHandle) != 0)
        return false;

    // Use UBX protocol only (reduces I2C load)
    uGnssCfgSetProtocolOut((uDeviceHandle_t)gnssHandle,
                           U_GNSS_PROTOCOL_NMEA,
                           false);

    uGnssCfgSetProtocolOut((uDeviceHandle_t)gnssHandle,
                           U_GNSS_PROTOCOL_UBX,
                           true);

    // Set 1 Hz update rate
    uGnssCfgSetRate((uDeviceHandle_t)gnssHandle,
                    1000,  // ms
                    1,
                    0);

    initialized = true;
    return true;
}

bool GnssSensor::Update()
{
    if (!initialized)
        return false;

    int32_t latX1e7 = 0;
    int32_t lonX1e7 = 0;
    int32_t alt = 0;
    int32_t radius = 0;

    int32_t result = uGnssPosGet((uDeviceHandle_t)gnssHandle,
                                 &latX1e7,
                                 &lonX1e7,
                                 &alt,
                                 &radius);

    if (result != 0)
        return false;

    currentData.latitude  = static_cast<double>(latX1e7) / 1e7;
    currentData.longitude = static_cast<double>(lonX1e7) / 1e7;

    return true;
}

GnssData GnssSensor::GetData() const
{
    return currentData;
}

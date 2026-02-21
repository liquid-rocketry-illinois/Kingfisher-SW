//
// Created by dyrel on 2/12/2026.
//
#define U_CFG_GNSS_ENABLE 1
#define U_CFG_ENABLE_LOGGING 0
#define U_CFG_OVERRIDE
#define GNSS_I2C_INSTANCE 3

#include "GPS.h"

extern "C" {
#include "ubxlib.h"
#include "common/device/api/u_device.h"
#include "gnss/api/u_gnss_module_type.h"
#include "gnss/api/u_gnss_type.h"
#include "gnss/api/u_gnss_cfg.h"
#include "gnss/api/u_gnss_pos.h"
}

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
    devCfg.transportType = uDeviceTransportType_t::U_DEVICE_TRANSPORT_TYPE_I2C;

    devCfg.transportCfg.cfgI2c.i2c = GNSS_I2C_INSTANCE;
    devCfg.transportCfg.cfgI2c.pinSda = -1;
    devCfg.transportCfg.cfgI2c.pinScl = -1;
    devCfg.transportCfg.cfgI2c.alreadyOpen = true;

    if (uDeviceOpen(&devCfg, (uDeviceHandle_t*)&gnssHandle) != 0)
        return false;

    // Use UBX protocol only (reduces I2C load)
    uGnssCfgSetProtocolOut((uDeviceHandle_t)gnssHandle,
                           U_GNSS_PROTOCOL_NMEA,
                           false);

    uGnssCfgSetProtocolOut((uDeviceHandle_t)gnssHandle,
                           U_GNSS_PROTOCOL_UBX,
                           true);

    // Set 0.5 Hz update rate. 2s in between readings is enough, doesn't need to be pinpoint accurate
    uGnssCfgSetRate((uDeviceHandle_t)gnssHandle,
                    2000,  // ms
                    -1,
                    uGnssTimeSystem_t::U_GNSS_TIME_SYSTEM_UTC);

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
    int32_t speedMMperS = 0;
    int32_t pSvc = 0;
    int64_t pTimeUTC = 0;

    int32_t result = uGnssPosGet((uDeviceHandle_t)gnssHandle,
                                 &latX1e7,&lonX1e7,
                                 &alt,
                                 &radius,
                                 &speedMMperS,
                                 &pSvc, &pTimeUTC,
                                 nullptr);

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

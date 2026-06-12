//
// Created by chris on 4/1/2026.
// Revised by Claude Code 2026-04-17.
//

#include "MAXM10S.h"
#include "cmsis_os2.h"
#include "i2c.h"

MAXM10S::MAXM10S() : _hi2c(nullptr) {}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

bool MAXM10S::sendUBX(const uint8_t *msg, uint16_t len)
{
    return HAL_I2C_Master_Transmit(_hi2c, I2C_ADDRESS,
                                   const_cast<uint8_t*>(msg), len, 50) == HAL_OK;
}

uint16_t MAXM10S::getAvailableBytes()
{
    availBuf[0] = availBuf[1] = 0;
    if (HAL_I2C_Mem_Read(_hi2c, I2C_ADDRESS, REG_BYTES_AVAIL,
                         I2C_MEMADD_SIZE_8BIT, availBuf, 2, 50) != HAL_OK) {
        // Recover stuck I2C peripheral — common on STM32H7 after NACK/timeout
        HAL_I2C_DeInit(_hi2c);
        HAL_I2C_Init(_hi2c);
        return 0;
    }
    uint16_t n = (uint16_t)((availBuf[0] << 8) | availBuf[1]);
    return (n == 0xFFFF) ? 0 : n;  // 0xFFFF = I2C FIFO stall; treat as 0
}

uint16_t MAXM10S::readStream(uint16_t numBytes)
{
    if (numBytes == 0) return 0;
    if (numBytes > RX_BUF_SIZE) numBytes = RX_BUF_SIZE;
    // At 400kHz I2C, 1024 bytes takes ~23ms; 50ms is a safe ceiling
    if (HAL_I2C_Mem_Read(_hi2c, I2C_ADDRESS, REG_DATA_STREAM,
                         I2C_MEMADD_SIZE_8BIT, rxBuf, numBytes, 50) != HAL_OK)
        return 0;
    return numBytes;
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

uint8_t MAXM10S::Init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;

    // Release reset and wait for module boot (~300 ms minimum)
    HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
    osDelay(500);

    // Verify module is reachable over I2C before sending config.
    uint8_t probe[2] = {0xFF, 0xFF};
    if (HAL_I2C_Mem_Read(_hi2c, I2C_ADDRESS, REG_BYTES_AVAIL,
                         I2C_MEMADD_SIZE_8BIT, probe, 2, 50) != HAL_OK)
        return 1;

    // UBX-CFG-VALSET: configure dynamic model and enable NMEA on DDC (I2C).
    //
    // NOTE: UBX-CFG-NAV5 (0x06/0x24) is NOT supported on the M10 platform —
    // the module silently NAKs it and keeps its default "portable" model.
    // M10 uses UBX-CFG-VALSET (0x06/0x8A) for all runtime configuration.
    //
    // Keys configured (all 1-byte values, stored little-endian):
    //   CFG-NAVSPG-DYNMODEL  0x20110021  = 8  (Airborne <4g; 7 = <2g)
    //   CFG-MSGOUT-NMEA_ID_GGA_I2C 0x209100BB = 1  (position, altitude, sats)
    //   CFG-MSGOUT-NMEA_ID_RMC_I2C 0x209100AC = 1  (lat/lon, speed, time)
    //   CFG-MSGOUT-NMEA_ID_GSA_I2C 0x209100C0 = 1  (HDOP, fix type)
    //
    // Payload = 4 (header) + 4×5 (key-value pairs) = 24 bytes.
    // Fletcher-8 checksum over class+ID+length+payload: CK_A=0x46, CK_B=0x64.
    static const uint8_t ubx_valset[] = {
        0xB5, 0x62,                          // sync chars
        0x06, 0x8A,                          // class=CFG, ID=VALSET
        0x18, 0x00,                          // payload length = 24 (LE)
        // payload ─────────────────────────────────────────────────────────
        0x00,                                // version
        0x07,                                // layers: RAM + BBR + Flash
        0x00, 0x00,                          // reserved
        0x21, 0x00, 0x11, 0x20, 0x08,       // CFG-NAVSPG-DYNMODEL = 8
        0xBB, 0x00, 0x91, 0x20, 0x01,       // CFG-MSGOUT-NMEA_ID_GGA_I2C = 1
        0xAC, 0x00, 0x91, 0x20, 0x01,       // CFG-MSGOUT-NMEA_ID_RMC_I2C = 1
        0xC0, 0x00, 0x91, 0x20, 0x01,       // CFG-MSGOUT-NMEA_ID_GSA_I2C = 1
        // checksum ────────────────────────────────────────────────────────
        0x46, 0x64                           // CK_A, CK_B
    };
    sendUBX(ubx_valset, sizeof(ubx_valset));

    return 0;
}

uint8_t MAXM10S::update()
{
    uint16_t available = getAvailableBytes();
    if (available == 0) return GPS_NO_BYTES;

    uint16_t bytesRead = readStream(available);
    if (bytesRead == 0) return GPS_READ_FAIL;

    bool newData = false;

    for (uint16_t i = 0; i < bytesRead; i++) {
        uint8_t b = rxBuf[i];
        if (b == 0xFF) continue;  // u-blox DDC filler byte — no data pending

        if (!gpsParser.encode(b)) continue;  // mid-sentence — keep feeding

        // A complete NMEA sentence was parsed. Update all fields unconditionally
        // so callers always see current satellite/quality state even without a fix.

        if (gpsParser.satellites.isValid())
            gpsDataStruct.satellites = (uint8_t)gpsParser.satellites.value();

        if (gpsParser.hdop.isValid())
            gpsDataStruct.hdop = (float)gpsParser.hdop.hdop();

        gpsDataStruct.isLocated   = gpsParser.location.isValid();
        gpsDataStruct.isAltituded = gpsParser.altitude.isValid();
        gpsDataStruct.isTimed     = gpsParser.time.isValid();
        gpsDataStruct.hasFix      = gpsDataStruct.isLocated && gpsDataStruct.isAltituded;

        if (gpsParser.location.isUpdated() && gpsDataStruct.isLocated) {
            gpsDataStruct.latitude  = gpsParser.location.lat();
            gpsDataStruct.longitude = gpsParser.location.lng();
            gpsDataStruct.lastFixMs = HAL_GetTick();
            newData = true;
        }

        if (gpsParser.altitude.isUpdated() && gpsDataStruct.isAltituded) {
            gpsDataStruct.altitude = gpsParser.altitude.meters();
            newData = true;
        }

        if (gpsParser.speed.isUpdated() && gpsParser.speed.isValid()) {
            gpsDataStruct.speedKmh = (float)gpsParser.speed.kmph();
            newData = true;
        }

        if (gpsParser.time.isUpdated() && gpsDataStruct.isTimed) {
            gpsDataStruct.hour   = gpsParser.time.hour();
            gpsDataStruct.minute = gpsParser.time.minute();
            gpsDataStruct.second = gpsParser.time.second();
            newData = true;
        }
    }

    return newData ? GPS_NEW_DATA : GPS_NO_NEW_SENTENCE;
}

MAXM10S::gpsData MAXM10S::getData() const
{
    return gpsDataStruct;
}

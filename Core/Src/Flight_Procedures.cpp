//
// Created by dyrel on 4/13/2026.
//

#include "Flight_Procedures.h"

GroundStation::GroundStation() : GNDDevices(), GNDData(){}

/* PSEUDOCODE — Init()
 *
 * Init telemetry, barometer engine, IMU engine, GPS, and servos.
 *
 *      If any device Init() returns != STATUS_OK,
 *          return STATUS_INIT_FAILURE
 *
 * Verify communication handshake:
 *
 *      Send keepAlive byte from GND to FC.
 *
 *      FC waits up to 2s to receive byte.
 *
 *          If received byte == sent byte,
 *              FC sends verify byte back to GND.
 *          Else,
 *              return STATUS_COMMS_FAILURE
 *
 *      GND waits up to 2s to receive verify byte.
 *
 *          If received byte == expected verify byte,
 *              begin bidirectional data stream.
 *          Else,
 *              return STATUS_COMMS_FAILURE
 *
 * Verify sensor accuracy:
 *
 *      Update all GND sensors 5 times.
 *
 *          If any update status != STATUS_OK,
 *              return STATUS_LOCAL_READ_ERR
 *
 *      Read and store GND sensor data into respective buffers.
 *
 *          If any buffered value is out of expected range,
 *              return STATUS_LOCAL_DATA_ERR
 *
 *      Wait up to 2s to receive FC sensor data.
 *
 *          If FC data received,
 *
 *              Compare GND buffers vs FC data (±5% tolerance).
 *
 *              If delta > 5%,
 *                  log warning.
 *                  return STATUS_SENSOR_MISMATCH
 *
 *              FC sends STATUS_OK to GND.
 *
 *          If 2s elapses with no FC data,
 *              flag no radio signal.
 *              return STATUS_TIMEOUT
 *
 * Verify servo accuracy:
 *
 *      Command servos to ±1, 3, 5, 10, 20, 30, 45 degrees
 *      in both directions. Read back actual angle each step.
 *
 *          If cumulative error > 2%,
 *              return STATUS_LOCAL_DATA_ERR  // reuse — servo angle is local sensor data
 *
 *          If cumulative error > 1% but <= 2%,
 *              log warning, continue.
 *
 *          If cumulative error <= 1%,
 *              return STATUS_OK
 */
int8_t GroundStation::Init() {
    GNDDevices.dev_telemetry.Init(TELEMETRY_MODE_GROUND);
    GNDDevices.dev_BarometerEngine.Init();
    GNDDevices.dev_IMU_Engine.Init(true);
    GNDDevices.dev_servoSet.Init({0,0}, TENTH_DEGREE, false);

    return 0;
}

/* PSEUDOCODE — Update()
 *
 * If init status != STATUS_OK,
 *      return STATUS_INIT_FAILURE  // Guard: do not run without successful init
 *
 * Begin loop timer.
 *
 * While loop timer < 3s:
 *
 *      Every 3s:
 *          Update local sensors.
 *          If any sensor update != STATUS_OK,
 *              log STATUS_LOCAL_READ_ERR warning, attempt to continue.
 *          Flag: UpdateLocalLog = true
 *
 *      Run UpdateKeepAlive()
 *
 *          If status == STATUS_ABORT_TRIGGERED,
 *              transmit abort signal to FC.
 *
 *          If status == STATUS_ABORT_ACTIVE,
 *              log warning: abort already latched, skip retransmit.
 *
 *      Update radio.
 *          If radio update != STATUS_OK,
 *              log STATUS_REMOTE_READ_ERR warning, attempt to continue.
 *
 *      Run UpdateLogging()
 *          If status != STATUS_OK,
 *              log warning, attempt to continue.
 *
 *      return STATUS_OK
 *
 * Log STATUS_TIMEOUT error.
 * return STATUS_TIMEOUT
 *
 * [Guard path]
 * return STATUS_INIT_FAILURE
 */
int8_t GroundStation::Update() {
    return 0;
}

/* PSEUDOCODE — UpdateKeepAlive()
 *
 * If keepAlive already == active,
 *      return STATUS_ABORT_ACTIVE     // Abort already latched
 *
 * Read USR_BUTTON pin.
 *
 *      If pin is active (TODO: confirm active HIGH or LOW),
 *          set AbortFlag = active.
 *
 * If AbortFlag == active:
 *
 *      Start/continue hold timer.
 *      Toggle abort LED (on↔off).
 *
 *      If hold timer >= 1.5s:
 *          Set keepAlive = active.
 *          Log and flag abort event.
 *          return STATUS_ABORT_TRIGGERED
 *
 * Else:
 *      Reset hold timer.   // Button released before threshold
 *
 * return STATUS_OK
 */
int8_t GroundStation::UpdateKeepAlive() {
    return 0;
}

/* PSEUDOCODE — UpdateLogging()
 *
 * Call radio Update().
 *
 *      If radio Update() returns STATUS_OK:
 *
 *          Pull FC sensor data from radio into remote data buffers.
 *
 *              If data pull fails or values out of range,
 *                  return STATUS_REMOTE_DATA_ERR
 *
 *          Log timestamp + remote data.
 *
 *          If UpdateLocalLog == true:
 *              Log timestamp + local sensor data.
 *              Set UpdateLocalLog = false.
 *
 *          return STATUS_OK
 *
 *      Else:
 *          return STATUS_REMOTE_READ_ERR
 */
int8_t GroundStation::UpdateLogging() {
    return 0;
}


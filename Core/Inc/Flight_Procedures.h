//
// Created by dyrel on 4/13/2026.
//

#ifndef KINGFISHER_SW_FLIGHT_PROCEDURES_H
#define KINGFISHER_SW_FLIGHT_PROCEDURES_H

#include <stdint.h>
#include "Servo_Axon_Mini_MKII.h"
#include "IMUs.h"
#include "Barometer.h"
#include "LIS2MDL.h"
#include "Telemetry.h"
#include "usart.h"
#include <cstring>
#include "task.h"

typedef enum : int8_t {
    STATUS_ABORT_ACTIVE     = -4,  // Abort already latched, redundant trigger
    STATUS_SENSOR_FAIL      = -3,  // GND/FC sensor data exceeds 5% tolerance
    STATUS_COMMS_FAILURE    = -2,  // Handshake or radio TX/RX failure
    STATUS_INIT_FAILURE     = -1,  // One or more devices failed to initialize
    STATUS_OK               =  0,  // Nominal
    STATUS_TIMEOUT          =  1,  // Operation exceeded allowed time window
    STATUS_LOCAL_READ_ERR   =  2,  // Local sensor read returned bad data
    STATUS_LOCAL_DATA_ERR   =  3,  // Local sensor data out of expected range
    STATUS_REMOTE_READ_ERR  =  4,  // Radio/FC read returned bad data
    STATUS_REMOTE_DATA_ERR  =  5,  // FC data out of expected range
    STATUS_ABORT_TRIGGERED  =  127 // Abort signal freshly activated
} ProcedureStatus;

// Both ground and fc record the same data down
typedef struct {
    GndStationData dat_GND_Data;
    telemetryData dat_FC_Data;
    DATA_Axon_Mini_MKII dat_Servos;
    BMI_Data dat_BMI_IMUs;
    BARO_DATA dat_Barometers;
    // GPS data struct here
} Data;

// devices used on both gnd and flight computers. Gnd station records,
// flight computer sends and records
typedef struct {
    Telemetry dev_telemetry;
    Baro_Unified dev_BarometerEngine;
    Servo_Axon_Mini_MKII dev_servoSet;
    IMUs dev_IMU_Engine;
    // GPS device here
} Devices;

class GroundStation {
    Data    GNDData = {};
    Devices GNDDevices = {};

public:
    GroundStation();

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
    int8_t Init();


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
    int8_t Update(); // Send/Receive from RCP



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
    int8_t UpdateKeepAlive();


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
    int8_t UpdateLogging();
};




/// FLIGHT COMPUTER

class FlightComputer {
    Data    FCData;
    Devices FCDevices;

public:
    FlightComputer();

    /* PSEUDOCODE:
     * Init all sensors.
     *
     * If any sensor init statuses aren't 0,
     *
     *      return STATUS_INIT_FAILURE
     *
     * Attempt initialization of radio.
     *
     * If initialization returns != 0,
     *
     *      return STATUS_SENSOR_FAILURE
     *
     * While waiting for 3 seconds,
     *
     *      attempt receive handshake.
     *
     * After 3s, if handshake not received,
     *
     *      return STATUS_COMMS_FAILURE
     *
     * Check received data, if received byte != expected handshake byte,
     *
     *      Send handshake response byte.
     *
     * Start timer.
     *
     * While timer lenth < 2s,
     *
     *      Update local sensor data
     *      Record sensor data.
     *      Transmit/Receive radio sensor calibration data.
     *      Check received radio data. If received,
     *          Compare calibration data against local sensor data
     *          If data is within 5% of each other,
     *              add a count to success counter
     *
     *      If success counter >= 10,
     *          return STATUS_OK
     * return STATUS_TIMEOUT
     */
    int8_t Init();


    /* PSEUDOCODE:
     *
     * Receive ground station data.
     *
     * // TODO verify this logic. Abort signal (button press) must be active for 1.5s in order to abort rocket.
     * // TODO if abort signal is lost for a tiny time (0.1-0.2s?), still count that as active. If button is
     * // TODO held down for 0.5s, then released for a second, then held again, the sequence resets. If it is
     * // TODO continuously active for 1s, then a brief cutout results in the button being "off" for a sliver of
     * // TODO time, still count that as added to the time. 1s active + 0.1s "not" active + 0.4s active = 1.5s active.
     * If abort signal active:
     *      If abort timer not already started,
     *          Start abort timer.
     *      If abort timer active not flagged,
     *          flag abort timer active
     *
     * If abort timer active:
     *      check current time - abort timer activation time.
     *      if dt > 1.5s,
     *          run Abort command
     *          run abort procedure infinite loop.
     *
     * Update sensors.
     * Update current time using GPS.
     * If GPS time unavailable,
     *      update using ticks from mcu.
     * Update servo state (Update motor sequence states based on time.)
     *
     * [IN FUTURE, CONTROL ALGORITHM WILL BE RUN HERE. FOR NOW NOTHING HERE]
     *
     * Record measured sensor data.
     * If any statuses of sensor updates != 0,
     *      flag error in radio data
     *
     * Transmit radio data.
     * If radio transmit errors,
     *      flag a warning on FC sd card log.
     *      add a count to error counter
     *      continue
     *
     * If error counter exceeds _____ //TODO use a value that makes sense
     *      raise comms error warning, but attempt to continue.
     *      // TODO alter radio timeout to create shorter time between calls
     *      // TODO change radio rate to 57.6K (higher if possible), air data rate to a higher rate (38.4K)
     */
    int8_t Update(); // Send to RCI/gnd station


    // reset servos to 0, run loop which keeps getting data
    // Flag warning of servo turn-off
    int8_t Abort();
};
#endif //KINGFISHER_SW_FLIGHT_PROCEDURES_H
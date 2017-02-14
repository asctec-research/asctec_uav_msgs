/*
 * Copyright (C) 2017 Intel Deutschland GmbH, Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "vector_types.h"
#include <stdint.h>

#ifdef __cplusplus
namespace asctec_uav_msgs {
#endif

enum {
  MESSAGE_ID_IMU = 0x01,
  MESSAGE_ID_IMU_NAV1 = 0x02,
  MESSAGE_ID_IMU_RAW = 0x03,
  MESSAGE_ID_FILTERED_SENSOR_DATA = 0x10,
  MESSAGE_ID_RC_DATA = 0x20,
  MESSAGE_ID_MOTOR_STATE = 0x30,
  MESSAGE_ID_VEHICLE_STATUS = 0x40,
  MESSAGE_ID_SYSTEM_UPTIME = 0x41,
  MESSAGE_ID_GPS_DATA = 0x50,
  MESSAGE_ID_COMMAND_MOTOR_SPEED = 0x10000,
  MESSAGE_ID_COMMAND_ROLL_PITCH_YAWRATE_THRUST = 0x10001,
  MESSAGE_ID_COMMAND_ROLL_PITCH_YAWRATE_CLIMBRATE = 0x10002,
  MESSAGE_ID_COMMAND_GPS_WAYPOINT = 0x10003,
  MESSAGE_ID_CONFIG_SET_MESSAGE_RATE_DIVISOR = 0x20000,
};

#pragma pack(push,1) // <== make sure all declarations are within this and "#pragma pack(pop)" below !!!

/**
 * \brief Imu message.
 * Axis Conventions:
 *   - x: forward
 *   - y: right
 *   - z: down
 */
typedef struct
{
  int64_t timestampUs; ///< Time since micro-controller startup in [us].
  Quaternion attitude; ///< Estimated attitude [quaternion].
  Vector3f angularVelocity; ///< Measured angular velocity in [rad/s]
  Vector3f linearAcceleration; ///< Measured linear acceleration [m/s^2]
} Imu;

typedef struct
{
  int64_t timestampUs; ///< Time since micro-controller startup in [us].
  uint8_t hasLock; ///< Indicates whether the receiver receives valid data from the RC. 0: no lock; 1: lock
  int16_t stickRoll; ///< Roll stick (right horizontal stick) input. 0 ... 2^14 (center) ... 2^15.
  int16_t stickPitch; ///< Pitch stick (right vertical stick) input. 0 ... 2^14 (center) ... 2^15.
  int16_t stickYaw; ///< Yaw stick (left horizontal stick) input. 0 ... 2^14 (center) ... 2^15.
  int16_t stickThrust; ///< Thrust stick (left vertical stick) input. 0 ... 2^14 (center) ... 2^15.
  int16_t switchMode; ///< Flight mode switch. 0: down; 2^15: up. On Jeti-D14: switch SA.
  int16_t switchPowerOnOff; ///< Power on/off switch. 0: down; 2^15: up. On Jeti-D14: switch SG.
  int16_t switchExternalCommand; ///< External command enable switch. 0: down; 2^15: up. On Jeti-D14: switch SH.
  int16_t aux; ///< Auxiliary switch. Used as safety pilot control switch on current systems. 0: away from pilot; 2^15: towards pilot. On Jeti-D14: switch SF.
} RcData;

typedef struct
{
  int64_t timestampUs; ///< Time since micro-controller startup in [us].
  int16_t commadedRpm[8]; ///< Motor speed that were commanded to the motor speed controllers [rpm].
  int16_t measuredRpm[8]; ///< Actual measured motor speeds [rpm].
} MotorState;

typedef struct
{
  int64_t timestampUs; ///< Time since micro-controller startup in [us].
  int32_t flightMode; ///< Current flight mode flags. \sa FlightMode
  int32_t flightTimeMs; ///< Accumulated flight time since startup.
  int16_t batteryVoltageMv; ///< Battery voltage in [ms]
  int16_t cpuLoadPerMill; ///< CPU load of the peripheral (user-programmable) processor. 0 ... 1000 ==> 0 ... 100%
  uint8_t vehicleType; ///< Indicates the type of the vehicle (e.g. Hummingbird, Firefly ...). \sa VehicleType.
  uint8_t safetyPilotState; ///< Indicates the state of the safety pilot. \sa SafteyPilotState.
} VehicleStatus;

typedef struct
{
  int32_t latitude;   ///< latitude [deg * 10^7]
  int32_t longitude;  ///< longitude [deg * 10^7]
  int32_t height;     ///< GPS height [mm]
  Vector3f speed;     ///< velocity measurements in ENU frame [m/s]
  float heading;      ///< GPS heading [deg]
  uint16_t horizontalAccuracy; ///< horizontal accuracy estimate [mm]
  uint16_t verticalAccuracy;   ///< vertical accuracy estimate [mm]
  uint16_t speedAccuracy;      ///< speed accuracy estimate [mm/s]
  uint16_t numSatellites;     ///< number of satellites used
  int32_t status;     ///< 0x01: dead reckoning only, 0x02: GPSFIX 2D, 0x03: GPSFIX 3D
  uint32_t timeOfWeek;  ///< time of week [ms]
  uint16_t week;      ///< week, 1..52
  uint16_t pDop;      ///< position DOP
  uint16_t hDop;      ///< horizontal DOP
  uint16_t vDop;      ///< vertical DOP
  uint16_t tDop;      ///< timing DOP
  uint16_t eDop;      ///< easting DOP
  uint16_t nDop;      ///< northing DOP
} GpsData;

typedef struct
{
  int64_t timestampUs;  ///< Time since micro-controller startup in [us].
  Vector3f acc;         ///< accelerometer measurements in vehicle frame [m/s^2]
  Vector3f gyro;        ///< gyroscope measurements in vehicle frame [rad/s]
  Vector3f mag;         ///< magnetometer measurements in vehicle frame [uT]
  float baroHeight;     ///< barometer height above starting point [m]
} FilteredSensorData;

enum FlightMode {
  FLIGHTMODE_ACC = 0x01, ///< Absolute angle control active (always active with standard firmware). Actually, the sticks command acceleration.
  FLIGHTMODE_POS = 0x02, ///< XY-position controller (usually GPS based) active.
  FLIGHTMODE_FLYING = 0x04, ///< Motors are running.
  FLIGHTMODE_EMERGENCY = 0x08, ///< RC signal lost.
  FLIGHTMODE_TRAJECTORY = 0x10, ///< Automatic navigation active (waypoints, trajectories, etc.).
  FLIGHTMODE_HEIGHT = 0x20, ///< Altitude control active.
  FLIGHTMODE_MOTOR_CURRENT_CALIB = 0x40, ///< Calibration running.
  FLIGHTMODE_AUTO_COMPASS_CALIB = 0x80, ///< Calibration running.
  FLIGHTMODE_HOVER_CALIB = 0x100 ///< Calibration running.
};

enum VehicleType{
  VEHICLE_TYPE_FIREFLY = 0x02,
  VEHICLE_TYPE_HUMMINGBIRD = 0x03,
  VEHICLE_TYPE_PELICAN = 0x04,
  VEHICLE_TYPE_NEO_6_9 = 0x05,
  VEHICLE_TYPE_NEO_6_11 = 0x06
};

enum SafetyPilotState{
  SAFETY_PILOT_STATE_ARMED = 0x01, ///< Safety pilot is prepared to take over.
  SAFETY_PILOT_STATE_ACTIVE = 0x02,  ///< Safety pilot is in control.
  SAFETY_PILOT_STATE_STICKS_ACTIVE = 0x04 ///< RC stick inputs are active, such that the MAV can be landed.
};

typedef struct
{
  int64_t timestampUs;
} SystemUpTime;

/**
 * \brief Roll and pitch angle commands, yaw-rate commands and thrust force.
 * Axis Conventions:
 *   - x: forward
 *   - y: right
 *   - z: down
 * Rotation convention (z-y'-x''):
 *   - yaw rotates around fixed frame's z axis
 *   - pitch rotates around new y-axis (y')
 *   - roll rotates around new x-axis (x'')
 * Equivalent: rotations in the order x-y-z, all in fixed frames' axes.
 */
typedef struct
{
  float roll; ///< Desired roll angle in [rad]
  float pitch; ///< Desired pitch angle in [rad]
  float yawRate; ///< Desired yaw turn-rate in [rad/s]
  float thrust; ///< Desired thrust force in [N]
} CommandRollPitchYawrateThrust;

typedef struct
{
  float roll; ///< Desired roll angle in [rad]
  float pitch; ///< Desired pitch angle in [rad]
  float yawRate; ///< Desired yaw turn-rate in [rad/s]
  float climbRate; ///< Desired climb rate in [m/s]. Up --> positive, down --> negative
} CommandRollPitchYawrateClimbRate;

typedef struct
{
  int16_t commandRpm[8];
} CommandMotorSpeed;

typedef struct
{
  int32_t latitude;   ///< latitude [deg * 10^7]
  int32_t longitude;  ///< longitude [deg * 10^7]
  float height;       ///< (Baro) height above starting altitude [m]
  float heading;      ///< GPS heading [deg]
  float maxSpeed;     ///< maximum approach speed [m/s]
  float maxAccel;     ///< maximum approach acceleration [m/s^2]
} CommandGpsWaypoint;

#pragma pack(pop)

#ifdef __cplusplus
} // end namespace asctec_uav_msgs
#endif

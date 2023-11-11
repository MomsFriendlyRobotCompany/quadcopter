
# MavLink C

- [mavlink c](https://mavlink.io/en/mavgen_c/)

## 1. Parsing Packets

```c
uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
```

Parameters:

- `uint8_t chan`: ID of the current channel.
- `uint8_t c`: The char to parse.
- `mavlink_message_t* r_message`: On success, the decoded message. NULL if the message couldn't be decoded.
- `mavlink_status_t* r_mavlink_status`: The channel statistics, including information about the current parse state.

Returns: 

- `0` if the packet decoding is incomplete
- `1` if the packet successfully decoded

```c
#include <mavlink/common/mavlink.h>

mavlink_status_t status;
mavlink_message_t msg;
int chan = MAVLINK_COMM_0;

while(serial.bytesAvailable > 0)
{
  uint8_t byte = serial.getNextByte();
  if (mavlink_parse_char(chan, byte, &msg, &status))
    {
    printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
    // ... DECODE THE MESSAGE PAYLOAD HERE ...
    }
}
```

## 2. Decoding Payload

In the decode section above, you could put:

```c
switch(msg.msgid) {
case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // ID for GLOBAL_POSITION_INT
  {
    // Get all fields in payload (into global_position)
    mavlink_msg_global_position_int_decode(&msg, &global_position);

  }
  break;
case MAVLINK_MSG_ID_GPS_STATUS:
  {
    // Get just one field from payload
    visible_sats = mavlink_msg_gps_status_get_satellites_visible(&msg);
  }
  break;
default:
  break;
}
```

For the GPS above, `mavlink.h` header for your dialect has:

```c
// Message ID number definition
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33

// Function to decode whole message into C struct
static inline void mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* global_position_int)

// C-struct with fields mapping to original message
MAVPACKED(
typedef struct __mavlink_global_position_int_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 int32_t lat; /*< [degE7] Latitude, expressed*/
 int32_t lon; /*< [degE7] Longitude, expressed*/
 int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
 int32_t relative_alt; /*< [mm] Altitude above ground*/
 int16_t vx; /*< [cm/s] Ground X Speed (Latitude, positive north)*/
 int16_t vy; /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
 int16_t vz; /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
 uint16_t hdg; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
}) mavlink_global_position_int_t;

// Function to get just a single field from the payload (in this case, the altitude).
static inline int32_t mavlink_msg_global_position_int_get_alt(const mavlink_message_t* msg)
```

## MavLink 1

```c
mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);
chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
```

```c
if (msg->magic == MAVLINK_STX_MAVLINK1) {
   printf("This is a MAVLink 1 message\n");
}
```

- `mavlink_set_position_target_local_ned_t`
- `mavlink_local_position_ned_t`

```c
typedef struct __mavlink_set_position_target_local_ned_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float x; /*< [m] X Position in NED frame*/
 float y; /*< [m] Y Position in NED frame*/
 float z; /*< [m] Z Position in NED frame (note, altitude is negative in NED)*/
 float vx; /*< [m/s] X velocity in NED frame*/
 float vy; /*< [m/s] Y velocity in NED frame*/
 float vz; /*< [m/s] Z velocity in NED frame*/
 float afx; /*< [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float afy; /*< [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float afz; /*< [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
 float yaw; /*< [rad] yaw setpoint*/
 float yaw_rate; /*< [rad/s] yaw rate setpoint*/
 uint16_t type_mask; /*<  Bitmap to indicate which dimensions should be ignored by the vehicle.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t coordinate_frame; /*<  Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9*/
} mavlink_set_position_target_local_ned_t;

 uint16_t mavlink_msg_set_position_target_local_ned_pack(
  uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate)

// these seem to be inverse, 0-set, 1-unset

//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     B(0000110111111000)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     B(0000110111000111)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION B(0000110000111111)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        B(0000111000111111)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    B(0000100111111111)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     B0000010111111111)

void
set_acceleration(..., mavlink_set_position_target_local_ned_t &sp) {
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION & // should this be | ?
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     &
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    &
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE &
    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;
	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;
	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
  sp.yaw  = yaw;
  sp.yaw_rate  = yaw_rate;
}
```

```c
 uint16_t mavlink_msg_scaled_imu3_pack(
  uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature)

typedef struct __mavlink_scaled_imu3_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 int16_t xacc; /*< [mG] X acceleration*/
 int16_t yacc; /*< [mG] Y acceleration*/
 int16_t zacc; /*< [mG] Z acceleration*/
 int16_t xgyro; /*< [mrad/s] Angular speed around X axis*/
 int16_t ygyro; /*< [mrad/s] Angular speed around Y axis*/
 int16_t zgyro; /*< [mrad/s] Angular speed around Z axis*/
 int16_t xmag; /*< [mgauss] X Magnetic field*/
 int16_t ymag; /*< [mgauss] Y Magnetic field*/
 int16_t zmag; /*< [mgauss] Z Magnetic field*/
 int16_t temperature; /*< [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).*/
} mavlink_scaled_imu3_t;
```

```c
MAVPACKED(
typedef struct __mavlink_raw_imu_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 int16_t xacc; /*<  X acceleration (raw)*/
 int16_t yacc; /*<  Y acceleration (raw)*/
 int16_t zacc; /*<  Z acceleration (raw)*/
 int16_t xgyro; /*<  Angular speed around X axis (raw)*/
 int16_t ygyro; /*<  Angular speed around Y axis (raw)*/
 int16_t zgyro; /*<  Angular speed around Z axis (raw)*/
 int16_t xmag; /*<  X Magnetic field (raw)*/
 int16_t ymag; /*<  Y Magnetic field (raw)*/
 int16_t zmag; /*<  Z Magnetic field (raw)*/
 uint8_t id; /*<  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)*/
 int16_t temperature; /*< [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).*/
}) mavlink_raw_imu_t;
```

```c
uint16_t mavlink_msg_command_int_pack(
  uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)

typedef struct __mavlink_command_int_t {
 float param1; /*<  PARAM1, see MAV_CMD enum*/
 float param2; /*<  PARAM2, see MAV_CMD enum*/
 float param3; /*<  PARAM3, see MAV_CMD enum*/
 float param4; /*<  PARAM4, see MAV_CMD enum*/
 int32_t x; /*<  PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7*/
 int32_t y; /*<  PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7*/
 float z; /*<  PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).*/
 uint16_t command; /*<  The scheduled action for the mission item.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t frame; /*<  The coordinate system of the COMMAND.*/
 uint8_t current; /*<  Not used.*/
 uint8_t autocontinue; /*<  Not used (set 0).*/
} mavlink_command_int_t;

uint16_t mavlink_msg_command_long_pack(
  uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)

typedef struct __mavlink_command_long_t {
 float param1; /*<  Parameter 1 (for the specific command).*/
 float param2; /*<  Parameter 2 (for the specific command).*/
 float param3; /*<  Parameter 3 (for the specific command).*/
 float param4; /*<  Parameter 4 (for the specific command).*/
 float param5; /*<  Parameter 5 (for the specific command).*/
 float param6; /*<  Parameter 6 (for the specific command).*/
 float param7; /*<  Parameter 7 (for the specific command).*/
 uint16_t command; /*<  Command ID (of command to send).*/
 uint8_t target_system; /*<  System which should execute the command*/
 uint8_t target_component; /*<  Component which should execute the command, 0 for all components*/
 uint8_t confirmation; /*<  0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)*/
} mavlink_command_long_t;
```

```c
uint16_t mavlink_msg_actuator_control_target_pack(
  uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t time_usec, uint8_t group_mlx, const float *controls)

typedef struct __mavlink_actuator_control_target_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float controls[8]; /*<  Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.*/
 uint8_t group_mlx; /*<  Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.*/
} mavlink_actuator_control_target_t;

typedef struct __mavlink_actuator_output_status_t {
 uint64_t time_usec; /*< [us] Timestamp (since system boot).*/
 uint32_t active; /*<  Active outputs*/
 float actuator[32]; /*<  Servo / motor output array values. Zero values indicate unused channels.*/
} mavlink_actuator_output_status_t;
```

```c
typedef struct __mavlink_scaled_pressure3_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float press_abs; /*< [hPa] Absolute pressure*/
 float press_diff; /*< [hPa] Differential pressure*/
 int16_t temperature; /*< [cdegC] Absolute pressure temperature*/
 int16_t temperature_press_diff; /*< [cdegC] Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.*/
} mavlink_scaled_pressure3_t;
```

```c
MAVPACKED(
typedef struct __mavlink_battery_status_t {
 int32_t current_consumed; /*< [mAh] Consumed charge, -1: autopilot does not provide consumption estimate*/
 int32_t energy_consumed; /*< [hJ] Consumed energy, -1: autopilot does not provide energy consumption estimate*/
 int16_t temperature; /*< [cdegC] Temperature of the battery. INT16_MAX for unknown temperature.*/
 uint16_t voltages[10]; /*< [mV] Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).*/
 int16_t current_battery; /*< [cA] Battery current, -1: autopilot does not measure the current*/
 uint8_t id; /*<  Battery ID*/
 uint8_t battery_function; /*<  Function of the battery*/
 uint8_t type; /*<  Type (chemistry) of the battery*/
 int8_t battery_remaining; /*< [%] Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.*/
 int32_t time_remaining; /*< [s] Remaining battery time, 0: autopilot does not provide remaining battery time estimate*/
 uint8_t charge_state; /*<  State for extent of discharge, provided by autopilot for warning or external reactions*/
 uint16_t voltages_ext[4]; /*< [mV] Battery voltages for cells 11 to 14. Cells above the valid cell count for this battery should have a value of 0, where zero indicates not supported (note, this is different than for the voltages field and allows empty byte truncation). If the measured value is 0 then 1 should be sent instead.*/
 uint8_t mode; /*<  Battery mode. Default (0) is that battery mode reporting is not supported or battery is in normal-use mode.*/
 uint32_t fault_bitmask; /*<  Fault/health indications. These should be set when charge_state is MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY (if not, fault reporting is not supported).*/
}) mavlink_battery_status_t;
```

```c
MAVPACKED(
typedef struct __mavlink_distance_sensor_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint16_t min_distance; /*< [cm] Minimum distance the sensor can measure*/
 uint16_t max_distance; /*< [cm] Maximum distance the sensor can measure*/
 uint16_t current_distance; /*< [cm] Current distance reading*/
 uint8_t type; /*<  Type of distance sensor.*/
 uint8_t id; /*<  Onboard ID of the sensor*/
 uint8_t orientation; /*<  Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270*/
 uint8_t covariance; /*< [cm^2] Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.*/
 float horizontal_fov; /*< [rad] Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.*/
 float vertical_fov; /*< [rad] Vertical Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.*/
 float quaternion[4]; /*<  Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid."*/
 uint8_t signal_quality; /*< [%] Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.*/
}) mavlink_distance_sensor_t;
```

```c
uint16_t mavlink_msg_heartbeat_pack(
  uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)

typedef struct __mavlink_heartbeat_t {
 uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
 uint8_t type; /*<  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.*/
 uint8_t autopilot; /*<  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.*/
 uint8_t base_mode; /*<  System mode bitmap.*/
 uint8_t system_status; /*<  System status flag.*/
 uint8_t mavlink_version; /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/
} mavlink_heartbeat_t;
```

```c
uint16_t mavlink_msg_attitude_pack(
  uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)

typedef struct __mavlink_attitude_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float roll; /*< [rad] Roll angle (-pi..+pi)*/
 float pitch; /*< [rad] Pitch angle (-pi..+pi)*/
 float yaw; /*< [rad] Yaw angle (-pi..+pi)*/
 float rollspeed; /*< [rad/s] Roll angular speed*/
 float pitchspeed; /*< [rad/s] Pitch angular speed*/
 float yawspeed; /*< [rad/s] Yaw angular speed*/
} mavlink_attitude_t;
```

```c
 uint16_t mavlink_msg_attitude_quaternion_pack(
  uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time_boot_ms, float q1, float q2, float q3, float q4, float rollspeed, float pitchspeed, float yawspeed, const float *repr_offset_q)

typedef struct __mavlink_attitude_quaternion_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float q1; /*<  Quaternion component 1, w (1 in null-rotation)*/
 float q2; /*<  Quaternion component 2, x (0 in null-rotation)*/
 float q3; /*<  Quaternion component 3, y (0 in null-rotation)*/
 float q4; /*<  Quaternion component 4, z (0 in null-rotation)*/
 float rollspeed; /*< [rad/s] Roll angular speed*/
 float pitchspeed; /*< [rad/s] Pitch angular speed*/
 float yawspeed; /*< [rad/s] Yaw angular speed*/
 float repr_offset_q[4]; /*<  Rotation offset by which the attitude quaternion and angular speed vector should be rotated for user display (quaternion with [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], send [0, 0, 0, 0] if field not supported). This field is intended for systems in which the reference attitude may change during flight. For example, tailsitters VTOLs rotate their reference attitude by 90 degrees between hover mode and fixed wing mode, thus repr_offset_q is equal to [1, 0, 0, 0] in hover mode and equal to [0.7071, 0, 0.7071, 0] in fixed wing mode.*/
} mavlink_attitude_quaternion_t;

typedef struct __mavlink_attitude_quaternion_cov_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float q[4]; /*<  Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)*/
 float rollspeed; /*< [rad/s] Roll angular speed*/
 float pitchspeed; /*< [rad/s] Pitch angular speed*/
 float yawspeed; /*< [rad/s] Yaw angular speed*/
 float covariance[9]; /*<  Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.*/
} mavlink_attitude_quaternion_cov_t;
```

```c
typedef struct __mavlink_local_position_ned_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float x; /*< [m] X Position*/
 float y; /*< [m] Y Position*/
 float z; /*< [m] Z Position*/
 float vx; /*< [m/s] X Speed*/
 float vy; /*< [m/s] Y Speed*/
 float vz; /*< [m/s] Z Speed*/
} mavlink_local_position_ned_t;
```

```c
 uint16_t mavlink_msg_highres_imu_pack(
  uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint16_t fields_updated, uint8_t id)

typedef struct __mavlink_highres_imu_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float xacc; /*< [m/s/s] X acceleration*/
 float yacc; /*< [m/s/s] Y acceleration*/
 float zacc; /*< [m/s/s] Z acceleration*/
 float xgyro; /*< [rad/s] Angular speed around X axis*/
 float ygyro; /*< [rad/s] Angular speed around Y axis*/
 float zgyro; /*< [rad/s] Angular speed around Z axis*/
 float xmag; /*< [gauss] X Magnetic field*/
 float ymag; /*< [gauss] Y Magnetic field*/
 float zmag; /*< [gauss] Z Magnetic field*/
 float abs_pressure; /*< [hPa] Absolute pressure*/
 float diff_pressure; /*< [hPa] Differential pressure*/
 float pressure_alt; /*<  Altitude calculated from pressure*/
 float temperature; /*< [degC] Temperature*/
 uint16_t fields_updated; /*<  Bitmap for fields that have updated since last message*/
 uint8_t id; /*<  Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)*/
} mavlink_highres_imu_t;
```

```c
// Store message sysid and compid.
// Note this doesn't handle multiple message sources.
current_messages.sysid  = message.sysid;
current_messages.compid = message.compid;

// Handle Message ID
switch (message.msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT: {
    //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
    mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
    current_messages.time_stamps.heartbeat = get_time_usec();
    this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
    break;
  }

  case MAVLINK_MSG_ID_SYS_STATUS: {
    //printf("MAVLINK_MSG_ID_SYS_STATUS\n");
    mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
    current_messages.time_stamps.sys_status = get_time_usec();
    this_timestamps.sys_status = current_messages.time_stamps.sys_status;
    break;
  }

  case MAVLINK_MSG_ID_BATTERY_STATUS: {
    //printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
    mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
    current_messages.time_stamps.battery_status = get_time_usec();
    this_timestamps.battery_status = current_messages.time_stamps.battery_status;
    break;
  }

  case MAVLINK_MSG_ID_RADIO_STATUS: {
    //printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
    mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
    current_messages.time_stamps.radio_status = get_time_usec();
    this_timestamps.radio_status = current_messages.time_stamps.radio_status;
    break;
  }

  case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
    //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
    mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
    current_messages.time_stamps.local_position_ned = get_time_usec();
    this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
    break;
  }

  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
    //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
    mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
    current_messages.time_stamps.global_position_int = get_time_usec();
    this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
    break;
  }

  case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: {
    //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
    mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
    current_messages.time_stamps.position_target_local_ned = get_time_usec();
    this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
    break;
  }

  case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: {
    //printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
    mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
    current_messages.time_stamps.position_target_global_int = get_time_usec();
    this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
    break;
  }

  case MAVLINK_MSG_ID_HIGHRES_IMU: {
    //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
    mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
    current_messages.time_stamps.highres_imu = get_time_usec();
    this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
    break;
  }

  case MAVLINK_MSG_ID_ATTITUDE: {
    //printf("MAVLINK_MSG_ID_ATTITUDE\n");
    mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
    current_messages.time_stamps.attitude = get_time_usec();
    this_timestamps.attitude = current_messages.time_stamps.attitude;
    break;
  }

  default: {
    // printf("Warning, did not handle message id %i\n",message.msgid);
    break;
  }


} // end: switch msgid
```


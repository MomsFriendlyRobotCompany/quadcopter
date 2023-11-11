

from enum import IntEnum

class SysStatus(IntEnum):
    GYRO=1
    ACCEL=2
    MAG=4
    ABS_PRESSURE=8
    DIFF_PRESSURE=16
    GPS=32
    AHRS=2097152
    BATTERY=33554432
    PROXIMITY=67108864
    PROPULSION=1073741824


class Status(IntEnum):
    UNINT=0       # unknown state
    BOOT=1        # just started up
    CALIBRATING=2 # not ready for flight
    STANDBY=3     # on ground, ready to launch
    ACTIVE=4      # might be airborne, motors engaged
    POWEROFF=7    # power-down sequence started, will shutdown

class MavType(IntEnum):
    GENERIC=0
    QUADROTOR=2
    GCS=6

class AutoPilot(IntEnum):
    GENERIC=0
    ARDUPILOTMEGA=3
    GENERIC_MISSION_FULL=7
    INVALID=8
    PX4=12

class BaseMode(IntEnum):
    CUSTOM_MODE=1
    TEST=2
    AUTO=4
    GUIDED=8
    STABILIZED=16
    HIL=32
    MANUAL_INPUT=64
    SAFETY_ARMED=128

class HighResImuFlags(IntEnum):
    NONE=0
    ACCEL=(1|2|4)
    GYRO=(8|16|32)
    MAG=(64|128|256)
    PT=(512|4096)
    ALT=2048
    ALL=65535

class Frame(IntEnum):
    GLOBAL=0 # WGS84
    LOCAL_NED=1
    MISSION=2
    GLOBAL_REL_ALT=3 # WGS84, origin at altitude
    LOCAL_ENU=4
    GLOBAL_INT=5 # WGS84 scaled (1E7)
    LOCAL_OFFSET_NED=7 # origin at vehicle
    BODY_FRD=12 # forward, right, down (aerospace frame)
    LOCAL_FRD=20
    LOCAL_FLU=21

class EscFailure(IntEnum):
    NONE=0

class DistanceSensor(IntEnum):
    LASER=0
    ULTRASOUND=1
    INFRARED=2
    RADAR=3
    UNKNOWN=4

class EstimatorType(IntEnum):
    UNKNOWN=0

class BatteryType(IntEnum):
    UNKNOWN=0
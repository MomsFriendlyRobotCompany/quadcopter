
# import numpy as np
# from numpy import sin, cos, pi
# from pymavlink import mavutil
# from pymavlink.dialects.v20 import common as mavlink2
# import socket
# import time
# # from squaternion import Quaternion
# from enum import IntEnum

# class Status(IntEnum):
#     UNINT=0       # unknown state
#     BOOT=1        # just started up
#     CALIBRATING=2 # not ready for flight
#     STANDBY=3     # on ground, ready to launch
#     ACTIVE=4      # might be airborne, motors engaged
#     POWEROFF=7    # power-down sequence started, will shutdown

# class MavType(IntEnum):
#     GENERIC=0
#     QUADROTOR=2
#     GCS=6

# class AutoPilot(IntEnum):
#     GENERIC=0
#     ARDUPILOTMEGA=3
#     GENERIC_MISSION_FULL=7
#     INVALID=8
#     PX4=12

# class BaseMode(IntEnum):
#     CUSTOM_MODE=1
#     TEST=2
#     AUTO=4
#     GUIDED=8
#     STABILIZED=16
#     HIL=32
#     MANUAL_INPUT=64
#     SAFETY_ARMED=128

# class HighResImuFlags(IntEnum):
#     NONE=0
#     ACCEL=(1|2|4)
#     GYRO=(8|16|32)
#     MAG=(64|128|256)
#     PT=(512|4096)
#     ALT=2048
#     ALL=65535
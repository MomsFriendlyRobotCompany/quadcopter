
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
# import socket
from .enums import *
from squaternion import Quaternion


class MavlinkConnection:
    def __init__(self):
        self.conn = None

    def open(self, path):
        self.conn = mavutil.mavlink_connection(path, protocol=mavlink2)
        # self.conn.setup_signing(
        #     bytearray(chr(42)*32, 'utf-8' ),  # was just: chr(42)*32
        #     sign_outgoing=True)
        self.mav = self.conn.mav

    def heartbeat(self, mode, status):
        self.conn.mav.heartbeat_send(
            MavType.QUADROTOR,
            AutoPilot.PX4,
            mode,
            0, # custom_mode for autopilot
            status)

    def state(self, state, boot):
        """ p,v,w,q = state """
        # q = Quaternion(*state[9:])
        q = Quaternion(1,1,1,1)
        q = q.normalize
        w = state[6:9]
        self.conn.mav.attitude_quaternion_send(
            boot,
            q.w,
            q.x,
            q.y,
            q.z,
            w[0],w[1],w[2],
            [1,0,0,0]
        )

        p = state[:3]
        v = state[3:6]
        self.conn.mav.local_position_ned_send(
            boot,
            p[0],p[1],p[2],
            v[0],v[1],v[2]
        )

        self.conn.mav.global_position_int_send(
            boot,
            350844000, # degE7
            -1066504000,
            100000, # altitude mm
            100, # above ground mm
            0,
            0,
            0,
            0 # heading cdeg
        )

        self.conn.mav.sim_state_send(
            q.w,
            q.x,
            q.y,
            q.z,
            0,0,0, # r p y
            0,0,0, # a
            0,0,0, # g
            35.0, -106.0, # lat/lon
            1000, # alt
            0,0,
            0,0,0
        )

        self.conn.mav.rc_channels_raw_send(
            boot,
            0,
            1500, # 1000 - 2000
            1500,
            1500,
            1500,
            0xFFFF, # unused
            0xFFFF,
            0xFFFF,
            0xFFFF,
            0xFF # rssi unknown
        )

        self.conn.mav.battery_status_send(
            0,
            1, # does everything
            1, # lipo
            2550, # cdeg
            [0]*10, # mV
            30000, # mA
            1000, # mAh consumed
            -1, # doesn't provide info
            80, # 80%
            # 0,
            # 0,
            # [0,0,0,0], # cell level not supported
            # 0, # discharging
            # 0 # no fault
        )

        # conn.mav.battery_status_send()
    def sensors(self, a, g, m, w, press, temp, epoch):
        self.conn.mav.highres_imu_send(
            epoch,
            a[0],a[1],a[2], # accel
            g[0],g[1],g[2], # gyro
            m[0],m[1],m[2], # mag
            press,
            0, # diff
            0, # alt
            temp,
            HighResImuFlags.ALL,
            # 0 # IMU ID
        )

    def sys_status(self):
        sensors = SysStatus.ACCEL
        sensors |= SysStatus.MAG
        sensors |= SysStatus.ABS_PRESSURE
        sensors |= SysStatus.GPS
        sensors |= SysStatus.AHRS
        sensors |= SysStatus.BATTERY
        sensors |= SysStatus.PROXIMITY
        sensors |= SysStatus.PROPULSION

        self.conn.mav.sys_status_send(
            sensors,
            sensors,
            sensors,
            500, # ??
            12000, # mV
            30000, # mA
            80, # % battery
            0, # comm drop rate
            0, # errors
            0,0,0,0 # misc errors
        )

    def autopilot_version(self):
        caps = 64 | 128
        self.conn.mav.autopilot_version_send(
            caps,
            1,
            1,
            1,
            11, # board ver
            [1]*8,
            [1]*8,
            [1]*8,
            1,
            1,
            1
        )


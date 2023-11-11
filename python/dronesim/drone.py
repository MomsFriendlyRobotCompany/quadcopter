
from .enums import *
from .dynamics import Drone
import numpy as np
import time
from pyrk import RK4
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import socket
from squaternion import Quaternion
from .mav_conn import MavlinkConnection
from .sensors import SensorNoise
from threading import Thread
from queue import Queue

"""
https://github.com/MishkaRogachev/JAGCS/tree/master


https://docs.python.org/3/library/sched.html
import sched, time

def do_something(scheduler):
    # schedule the next call first
    scheduler.enter(60, 1, do_something, (scheduler,))
    print("Doing stuff...")
    # then do your stuff

my_scheduler = sched.scheduler(time.time, time.sleep)
my_scheduler.enter(60, 1, do_something, (my_scheduler,))
my_scheduler.run()
"""

MSG_1_HZ   = 1000
MSG_2_HZ   = 500
MSG_5_HZ   = 200
MSG_10_HZ  = 100
MSG_25_HZ  = 40
MSG_50_HZ  = 20
MSG_100_HZ = 10

class Alarm:
    def __init__(self, alarm):
        self.alarm = alarm
        self.last_alarm = time.process_time_ns()
        self.dt = 0

    def check(self):
        now = time.process_time_ns()
        dt = (now - self.last_alarm) // 1E6 # msec
        if (self.alarm > dt): return False
        self.last_alarm = now
        self.dt = dt / 1000 # sec
        return True

class DroneSim:
    def __init__(self):
        self.autopilot = None

        self.state = np.array([
            0,0,0,  # p [0-2]
            0,0,0,  # v [3-5]
            0,0,0,  # w [6-8]
            1,0,0,0 # q [9-12]
        ])

        params = {
            'm': 0.329,
            'l': 0.1785,
            'J': [2.238e-3, 2.986e-3, 4.804e-3],
            'kf': 7.00e-7,
            'km': 2.423e-6,
            'tau': 4.718e-3,
            'nmax': 1047
        }
        self.dyn = Drone(params)
        self.rk = RK4(self.dyn)
        self.state = np.array([
            0,0,0,   # p
            0,0,0,   # v
            0,0,0,   # w
            1,0,0,0  # q
        ])

        self.alarm_health = Alarm(MSG_1_HZ)
        self.alarm_sensors = Alarm(MSG_100_HZ)
        self.alarm_state = Alarm(MSG_5_HZ)

        self.accels = SensorNoise([.1,-.1,0],0.55, 3)
        self.gyros = SensorNoise([.1,-.1,0],0.55, 3)
        self.mags = SensorNoise([.1,-.1,0],0.55, 3)
        self.gps = SensorNoise([.1,-.1,0],0.55, 3)
        self.bar = SensorNoise([.01,-.01],0.15, 2)

        # mcast: return mavmcast object
        # udp or udpbcast: returns mavudp object
        # self.conn = mavutil.mavlink_connection('udpout:localhost:14550', protocol=mavlink2)
        # self.conn.setup_signing(
        #     bytearray(chr(42)*32, 'utf-8' ),  # was just: chr(42)*32
        #     sign_outgoing=True)
        path = 'udpout:localhost:14550'
        self.conn = MavlinkConnection()
        self.conn.open(path)
        self.conn.autopilot_version()

        self.inqueue = Queue(maxsize=100)
        self.outqueue = Queue()

        def recv(q):
            path = 'udpin:localhost:14556'
            iconn = mavutil.mavlink_connection(path, protocol=mavlink2)

            print(f">> Starting mavlink connection: {path}")
            print(f">> Thread: {self.recv.name}")

            while self.run:
                resp = iconn.recv_match(blocking=False, timeout=1.0)
                if resp:
                    # print(f">> Got: {resp.get_type()}")
                    # print(f"{resp}")
                    q.put(resp)

        self.run = True
        self.recv = Thread(target=recv, args=(self.inqueue,), name="mavlink_udpin_t")
        self.recv.daemon = True
        self.recv.start()

    def __del__(self):
        self.run = False
        self.recv.join()

    def loop_once(self):
        # time.sleep(0.001)
        epoch = time.time_ns()//1000        # uint64_t usec
        boot = time.process_time_ns()//1000 # uint32_t usec

        # resp = self.iconn.recv_match(blocking=False, timeout=0.01)
        # if resp:
        #     print(f">> Got: {resp.get_type()}")
        #     print(f"{resp}")
        while (not self.inqueue.empty()):
            msg = self.inqueue.get()
            print(f">> Got: {msg}")

        if self.alarm_health.check():
            self.conn.heartbeat(BaseMode.MANUAL_INPUT, Status.ACTIVE)
            self.conn.sys_status()
            # pass
            # conn.mav.system_time_send(
            #     epoch,
            #     boot,
            #     force_mavlink1=False)

            # # conn.mav.battery_status_send()

        if self.alarm_state.check():
            self.conn.state(self.state, boot)

        if self.alarm_sensors.check():
            dt = self.alarm_sensors.dt
            sec = boot/1E6
            self.state = self.rk(sec, self.state, None, dt)

            q = self.state[10:]

            a = self.dyn(boot/1E6, self.state, [0,0,0,0])[3:6]
            a = self.accels.read(a)

            w = self.state[6:9]
            g = self.gyros.read(w)

            press, temp = self.bar.read([10100, 25.5])

            m = [0,0,0]

            self.conn.sensors(a,g,m,w,press,temp,epoch)

            # print(epoch)


#!/usr/bin/env python3
# complementary filter:
#   https://nbviewer.org/github/walchko/bearsnacks/blob/main/drone/filters/quaternion-complementary-filter/qcf.ipynb
# curses doc: https://docs.python.org/3/library/curses.html
#
# IMU namedtuple data:
# ImuAGMQPT(
#     ax=-0.0064697265625, ay=-0.011962890625, az=1.020751953125,
#     wx=0.48828125, wy=0.1220703125, wz=0.6103515625,
#     imu_temp=26.4375,
#     mx=-39.37445068359375, my=41.80064392089844, mz=-44.28529739379883,
#     qw=-1.427112340927124, qx=-1.4100708961486816, qy=-0.15619266033172607, qz=-1.7291849851608276,
#     pressure=83455.8515625, temperature=25.23807144165039,
#     ts=2026743)
#
from time import sleep
import curses as crs
from serial import Serial
from yivo import Yivo
from yivo_msgs import IMU, Range, MSG
from yivo_msgs import unpack
from yivo_parser import YivoParser
from squaternion import Quaternion
import sys
import numpy as np
from threading import Thread

yivo = Yivo()

run = True

scrx = 0
scry = 0

imu = IMU(*[0]*17)
lidar = Range(0,0,0,0,0,0)
euler = (0,0,0,)

outbuffer = None

def altitude(p):
    """
    Given a pressure (Pa), this calculates an altitude (m)
    """
    g0 = 9.80665 # m / s^2
    M = 0.0289644 # kg / mol
    R = 8.31446261815324 # Nm/(mol K)
    Lb = -0.0065 # K/m
    Tb = 288.15 # K
    Pb = 101325.0 # Pa
    return Tb/Lb*((p/Pb)**(-R*Lb/(g0*M)) - 1.0)

def printHUD(stdscr):
    line_width = 60
    if imu is None or euler is None:
        return

    row = 0

    heading = np.arctan2(imu.my,imu.mx) * 180.0 / np.pi

    stdscr.hline(row, 0,"=",line_width); row+=1
    stdscr.addstr(row,0,f"RAW IMU"); row+=1
    stdscr.addstr(row,0,f"  Accel: {imu.ax:9.3f}, {imu.ay:9.3f}, {imu.az:9.3f} g"); row+=1
    stdscr.addstr(row,0,f"   Gyro: {imu.gx:9.3f}, {imu.gy:9.3f}, {imu.gz:9.3f} rad/sec"); row+=1
    stdscr.addstr(row,0,f"    Mag: {imu.mx:9.3f}, {imu.my:9.3f}, {imu.mz:9.3f} uT"); row+=1
    # stdscr.addstr(row,0,f"  IMU Temperaure: {imu.t:.2f} C"); row+=1
    # row+=1
    stdscr.hline(row, 0,"-",line_width); row+=1

    # stdscr.addstr(row,0,f"RAW Press/Temp"); row+=1
    # stdscr.addstr(row,0,f"  Pressure: {imu.p:10.3f} Pa\tTemperaure: {imu.t:5.3f} C"); row+=1
    stdscr.addstr(row,0,f"  Lidar: {imu.lidar:5.1f} cm\tAltitude: {imu.altitude:5.3f} m"); row+=1

    stdscr.hline(row, 0,"=",line_width); row+=1
    stdscr.addstr(row,0,f"Refined State"); row+=1
    stdscr.addstr(row,0,f"  Position: {0.0:9.3f} {0.0:9.3f} {0.0:9.3f} m"); row+=1
    stdscr.addstr(row,0,f"  Attitude: {euler[0]:9.3f} {euler[1]:9.3f} {euler[2]:9.3f} deg"); row+=1
    stdscr.addstr(row,0,f"   Compass: {heading:10.3f} m"); row+=1
    # stdscr.addstr(row,0,f"  Altitude: {altitude(imu.p):10.3f} m"); row+=1

    stdscr.hline(row, 0,"-",line_width); row+=1
    stdscr.addstr(row,0,f"Distance: {lidar.distance} cm"); row+=1

    stdscr.hline(row, 0,"-",line_width); row+=1
    stdscr.addstr(row,0,f"Run Time"); row+=1
    # dy = imu.ts//(24*360000)
    hr = imu.ts//360000
    mn = (imu.ts-hr*360000)//60000
    sec = imu.ts-hr*360000-mn*60000
    stdscr.addstr(row,0,f"  timestamp: {imu.ts*0.001:.3f} sec    Up Time: {hr}:{mn:02d}:{sec*0.001:06.3f}"); row+=1

    stdscr.hline(row, 0,"-",line_width); row+=1

def keypress(stdscr, c):
    global outbuffer

    if c == 'a': # arm ESCs
        # ser.write(b'a')
        outbuffer = b'a'
        c = "*** Arming ESCs ***"
    elif c == 'g':
        # ser.write(b'g')
        c = "ping"
    # elif c[0] == 'p': # PWM command for m0, m1, m2, m3
    #     cmd = c.split()
    #     if len(cmd) == 5:
    #         m = b'p'
    #         for i in range(4):
    #             pwm[i] = limit(int(cmd[i+1]))
    #             m += pwm[i].to_bytes(2,'little')
    #         # m = b'p' + m
    #         # print(m)
    #         ser.write(m)
    #         c = m
    #     else:
    #         print("invalid command:", c)
    #         return
    # elif c == 'r': # ramp motors
    #     # ser.write(b'r')
    #     c = "ramp motors"
    elif c == 's': # stop motors
        # ser.write(b's')
        pwm = [1000]*4
        c = "stop motors"
    elif c == 't': # telemetry toggle
        # ser.write(b't')
        c = "toggle telemetry"
    elif c == 'T': # telemetry toggle
        # ser.write(b'T')
        outbuffer = b'T'
    # elif c == 'w': # increase motor speed
    #     incr = 10
    #     m = b''
    #     for i in range(4):
    #         pwm[i] = limit(pwm[i] + incr)
    #         m += pwm[i].to_bytes(2,'little')
    #     m = b'p' + m
    #     ser.write(m)
    #     c = m
    # elif c == 'x': # decrease motor speed
    #     decr = 10
    #     m = b''
    #     for i in range(4):
    #         pwm[i] = limit(pwm[i] - decr)
    #         m += pwm[i].to_bytes(2,'little')
    #     m = b'p' + m
    #     ser.write(m)
    #     c = m
    # else:
    #     print("*** Unknown input ***")


def serial_read(port):

    ser = Serial()
    ser.port = port #"/dev/tty.usbmodem14301"
    ser.baud = 1000000
    ser.timeout = 0.01
    ser.open()

    yp = YivoParser(yivo.header)

    global imu
    global euler
    global outbuffer
    global lidar
    # print(euler)

    while run:
        c = ser.read()
        while yp.parse(c):
            c = ser.read()

        data, msgid = yp.get_info()
        msg = unpack(data, msgid)
        if msg is None:
            continue

        if msgid == MSG.IMU_FULL:
            imu = msg
            q = Quaternion(msg.qw,msg.qx,msg.qy,msg.qz)
            euler = q.to_euler(degrees=True)
        elif msgid == MSG.DISTANCE:
            lidar = msg

        if outbuffer is not None:
            ser.write(outbuffer)
            outbuffer = None

    ser.close()


def main(stdscr):
    scry, scrx = stdscr.getmaxyx()
    stdscr.nodelay(True) # getch/getkey is non-blocking
    while run:
        c = stdscr.getch()

        if c != crs.ERR:
            c = chr(c) # convert int to str
            if c == 'q':
                break
            keypress(stdscr, c)

        stdscr.erase()
        printHUD(stdscr)
        stdscr.refresh()

        sleep(1/20)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: gui-curses.py /dev/serial.port")
        sys.exit(1)

    port = sys.argv[1]
    p = Thread(target=serial_read, args=(port,))
    p.start()

    try:
        crs.wrapper(main)
        # main(None)

    except KeyboardInterrupt:
        pass

    finally:
        run = False
        sleep(0.1)
        p.join(timeout=1.0)


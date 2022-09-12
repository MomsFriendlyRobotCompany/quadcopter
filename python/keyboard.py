#!/usr/bin/env python3
from serial import Serial
from telemetry import Telemetry
import time
from yivo import Yivo
import sys, select

# --------------------
# https://stackoverflow.com/questions/1335507/keyboard-input-with-timeout

###############################################################

# def limit(a, min=1000, max=2000):
#     a = a if a < max else max
#     a = a if a > min else min
#     return a
def limit(a,vmin=1000, vmax=2000):
    return min(max(vmin, a), vmax)

def main():
    ser = Serial()
    ser.port = "/dev/tty.usbmodem14601"
    ser.baud = 115200
    ser.timeout = 0.1
    ser.open()

    # tel = Telemetry()
    # js = JSUpdater()

    yivo = Yivo()

    # js = PS4Joystick()
    # if not js.valid:
    #     print("*** No joystick found ***")

    pwm = [1000]*4

    try:
        while True:
            # if s.in_waiting > 0:
            #     # msgID, d = tel.get(s)
            #     pass
            i,o,e = select.select([sys.stdin],[],[],0.01)

            if i:
                # print(">>", sys.stdin.readline().strip())
                c = sys.stdin.readline().strip()

                if len(c) == 0:
                    continue
                elif c == 'a': # arm ESCs
                    ser.write(b'a')
                    c = "*** Arming ESCs ***"
                elif c == 'g':
                    ser.write(b'g')
                    c = "ping"
                elif c[0] == 'p': # PWM command for m0, m1, m2, m3
                    cmd = c.split()
                    if len(cmd) == 5:
                        m = b''
                        for i in range(4):
                            pwm[i] = limit(int(cmd[i+1]))
                            m += pwm[i].to_bytes(2,'little')
                        m = b'p' + m
                        ser.write(m)
                        c = m
                    else:
                        print("invalid command:", c)
                        continue
                elif c == 'q': # quit keyboard program
                    print(">> Quitting Keyboard")
                    break
                elif c == 'r': # ramp motors
                    ser.write(b'r')
                    c = "ramp motors"
                elif c == 's': # stop motors
                    ser.write(b's')
                    pwm = [1000]*4
                    c = "stop motors"
                elif c == 't': # telemetry toggle
                    ser.write(b't')
                    c = "toggle telemetry"
                elif c == 'w': # increase motor speed
                    incr = 10
                    m = b''
                    for i in range(4):
                        pwm[i] = limit(pwm[i] + incr)
                        m += pwm[i].to_bytes(2,'little')
                    m = b'p' + m
                    ser.write(m)
                    c = m
                elif c == 'x': # decrease motor speed
                    decr = 10
                    m = b''
                    for i in range(4):
                        pwm[i] = limit(pwm[i] - decr)
                        m += pwm[i].to_bytes(2,'little')
                    m = b'p' + m
                    ser.write(m)
                    c = m

                print(">>", c)

            else:
            #     # print(".", end="", flush=True)
            #     pass
                if ser.in_waiting:
                    c = ser.read(32)
                    print(c)

            # time.sleep(0.05)

    except KeyboardInterrupt:
        print("ctrl-c")

    finally:
        ser.close()
        # tel.save()
        pass


if __name__ == "__main__":
    main()

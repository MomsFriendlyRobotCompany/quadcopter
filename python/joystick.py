#!/usr/bin/env python3
from serial import Serial
from telemetry import Telemetry
from clamps import PS4Joystick
import time


###############################################################

class JSUpdater(PS4Joystick):
    lx = 0
    ly = 0

    def __init__(self):
        super().__init__(0)

        self.time = time.monotonic()
        self.dt = 0.1

    def check(self):
        now = time.monotonic()
        if now >= self.time:
            self.time = now + self.dt
            return True
        return False

    def updateLeft(self, x, y, deadzone=0.1):
        ret = False
        if abs(self.lx - x) > deadzone:
            self.lx = x
            ret = True
        if abs(self.ly - y) > deadzone:
            self.ly = y
            ret = True
        return ret

def main():
    s = Serial()
    s.port = "/dev/tty.usbmodem14601"
    s.baud = 115200
    s.timeout = 1
    s.open()
    # s.write(b"t\n")
    # s.write(b"g\n")

    tel = Telemetry()
    js = JSUpdater()

    # js = PS4Joystick()
    if not js.valid:
        print("*** No joystick found ***")

    try:
        while True:
            if s.in_waiting > 0:
                msgID, d = tel.get(s)

            if js.valid and js.check():
                ps4 = js.get(raw=True)

                if ps4.buttons.square is True:
                    s.write(b"t\n")
                elif ps4.buttons.circle is True:
                    s.write(b"g\n")
                elif ps4.buttons.x is True:
                    s.write(b"a\n")
                elif ps4.buttons.share is True:
                    tel.save_data = not tel.save_data

                x,y = ps4.leftstick
                x = x >> 7
                y = y >> 7
                # print(f"left x: {x}   y: {y}")
                if js.updateLeft(x,y,deadzone=50):
                    delta = 1
                    x = int(delta*js.lx) >> 2
                    y = int(delta*js.ly) >> 2
                    print(f"left x: {x << 2}   y: {y << 2}")
                    if y > 0: s.write(b"w\n")
                    elif y < 0: s.write(b"x\n")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("ctrl-c")

    finally:
        s.close()
        js.close()
        tel.save()


if __name__ == "__main__":
    main()

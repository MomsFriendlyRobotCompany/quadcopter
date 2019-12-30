#!/usr/bin/env python

import serial

port = "/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_013950B1-if00-port0"

ser = serial.Serial(port, 115200)
if not ser.is_open:
    print("** Couldn't open", port)
    exit(1)
else:
    ser.timeout = 2
    print(ser)

ser.write("#\r\n".encode("utf8"))  # put into interactive mode
resp = ser.read(4096)
print(resp.decode("utf8"))

cmd = None

print("Enter command or help")
while cmd != "exit":
    cmd = input(">> ")
    ser.write((cmd + "\r\n").encode("utf8"))
    resp = ser.read(1024*10)

    if resp:
        if cmd == "dump":
            ans = input("-- Do you want to archive? [y/n]")
            if ans == "y":
                print("*** saving ***")

        r = resp.decode("utf8")
        print(r)
    else:
        print("[No Response]")

ser.close()

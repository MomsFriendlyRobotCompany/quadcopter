#!/usr/bin/env python3

import pygame
import numpy as np
import threading
import time

from messages import *

from serial import Serial
from serial.tools.list_ports import comports, grep

from yivo import Yivo
from squaternion import Quaternion


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

class Debug:
    def __init__(self, font_size=None):
        if font_size is None:
            font_size = 20

        self.font = pygame.font.SysFont('Courier', font_size) # monowidth font
        self.font_size = font_size
        self.row = 0
        self.border = 5
        pass

    def draw(self, screen):

        w, h = screen.get_size()

        self.draw_text(screen,  "RAW IMU", color="cyan")
        self.draw_text(screen, f"  Accel: {imu.a.x:9.3f}, {imu.a.y:9.3f}, {imu.a.z:9.3f} g")
        self.draw_text(screen, f"   Gyro: {imu.g.x:9.3f}, {imu.g.y:9.3f}, {imu.g.z:9.3f} rad/sec")
        self.draw_text(screen, f"    Mag: {imu.m.x:9.3f}, {imu.m.y:9.3f}, {imu.m.z:9.3f} uT")
        # self.draw_text(screen, f"   Temp: {imu.temperature:.2f} C", 6)

        pygame.draw.line(screen, "cyan", (0,self.font_size*(self.row+.5)), (w,self.font_size*(self.row + .5)))
        self.row +=1
        self.draw_text(screen, f"RAW Press/Temp", color="cyan")
        self.draw_text(screen, f"  Pressure: {imu.pressure:10.3f} Pa\t   Altitude: {altitude(imu.pressure):8.3f} m")
        self.draw_text(screen, f"  Temperaure: {imu.temperature:5.3f} C")

        pygame.draw.line(screen, "cyan", (0,self.font_size*(self.row+.5)), (w,self.font_size*(self.row + .5)))
        self.row +=1
        self.draw_text(screen,  "GPS", color="cyan")
        self.draw_text(screen, f"  Pos: {gps.lat:5.1f}, {gps.lon:5.1f} deg")
        self.draw_text(screen, f"  Alt: {gps.altitude:5.3f} m")
        self.draw_text(screen, f" Date: 123 ")
        self.draw_text(screen, f" Time: 123 ")

        pygame.draw.line(screen, "cyan", (0,self.font_size*(self.row+.5)), (w,self.font_size*(self.row + .5)))
        self.row +=1
        q = Quaternion(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        )
        roll, pitch, yaw = q.to_euler(degrees=True)
        self.draw_text(screen,  "Pose", color="cyan")
        self.draw_text(screen, f"   Pos: {pose.position.x:9.3f}, {pose.position.y:9.3f}, {pose.position.z:9.3f} m")
        self.draw_text(screen, f"   Vel: {pose.velocity.x:9.3f}, {pose.velocity.y:9.3f}, {pose.velocity.z:9.3f} m/sec")
        self.draw_text(screen, f"  Quat: {pose.orientation.w:6.3f}, {pose.orientation.x:6.3f}, {pose.orientation.y:6.3f}, {pose.orientation.z:6.3f} ")
        self.draw_text(screen, f"   RPY: {roll:6.1f}, {pitch:6.1f}, {yaw:6.1f} deg")

        if recording:
            self.draw_text(screen, " Recording ", color="white", bg="red", x=0, y=h-self.font_size*2)
        self.draw_text(screen, f" Timestamp: {time.monotonic():10.1f} msec  ", color="black", bg="white", x=0, y=h-self.font_size)
        # self.draw_text(screen, f" Timestamp: {imu.timestamp:10} msec  ", color="black", bg="white", x=0, y=h-self.font_size)

        self.row = 0

    def draw_text(self, screen, string, color="white", bg=None, center=False, x=None, y=None):
        if bg is not None:
            txt = self.font.render(string, True, color, bg)
        else:
            txt = self.font.render(string, True, color)

        tw, th = 0, 0

        if x is None:
            x = self.border
        if y is None:
            y = self.font_size * self.row
            self.row += 1

        if center:
            tw = txt.get_width()
            th = txt.get_height()

        screen.blit(txt,(x-tw//2,y-th//2))

# pygame setup
pygame.init()
screen = pygame.display.set_mode((600, 600), pygame.RESIZABLE)
clock = pygame.time.Clock()
running = True
recording = False

debug = Debug(14)
# imu = imu_t(
#     vec_t(0,0,0),
#     vec_t(0,0,0),
#     vec_t(0,0,0),
#     80000.0,
#     25.5,
#     123456
# )
imu = imu_t(
    0,0,0,
    0,0,0,
    0,0,0,
    80000.0,
    25.5,
    123456
)

gps = gps_t(10,20,30)

pose = pose_t(
    0,0,0,
    0,0,0,
    0,0,0,1
)

def read_serial():
    global running
    while running:
        time.sleep(1)

t = threading.Thread(target=read_serial)
t.daemon = True
t.start()

def main():
    global running
    global recording

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_r:
                    recording = not recording

        screen.fill("black") # clear screen

        debug.draw(screen)

        pygame.display.flip() # swap buffers
        clock.tick(30)  # limits FPS to 60

    pygame.quit()

main()
#!/usr/bin/env python3
import pygame
from numpy import pi, cos, sin, tan
import time

"""
Flight Director Attitude Indicator (FDAI): for spacecraft, typically blue sky and
    black ground (space). Also called a NavBall

Moving Horizontal Display (MH): aircraft symbol remains still while the horizon moves
Frequency Separated Display (FSD): aircraft symbol rolls to represent the current
    command by pilot. If the controls are level, the aircraft symbol is level in the
    display. The horizon moves like the MH.

Ref: https://www.researchgate.net/publication/341622913_Indication_of_Flight_Attitude_by_a_Frequency_Separated_versus_Moving_Horizon_Cockpit_Display_An_Experimental_Study_with_Flight_Novices
"""

class HUD:
    def __init__(self, x, y, size=400):
        self.roll = 0
        self.pitch = 0
        self.heading = 0
        self.font = pygame.font.SysFont("Monaco",20)

        self.x = x
        self.y = y
        self.size = size

        self.r = pygame.Surface((size,size))
        self.r.set_colorkey("black")
        pygame.draw.rect(self.r, "blue", pygame.Rect(0,0,size,size//2))
        pygame.draw.rect(self.r, (25,25,25), pygame.Rect(0,size//2,size,size//2))

    def text(self, surface, txt, x, y):
        txt_surface = self.font.render(
            txt,
            True,
            (255, 255, 255)
        )
        txt_rect = txt_surface.get_rect()
        txt_rect.center = (x,y)
        surface.blit(txt_surface, txt_rect)

    def render(self, screen):
        # pygame.time.wait(200)
        # self.roll += 1
        self.roll = self.roll % 360

        # self.pitch = 10
        if self.pitch > 90: self.pitch = -90
        p = self.pitch * (self.size//2) / 90
        r = self.roll * pi / 180

        cx, cy = screen.get_rect().center

        rr = pygame.transform.rotate(self.r, self.roll)
        rect = rr.get_rect()
        rect.center = (
            cx+p*sin(r),
            cy+p*cos(r)
        )

        w = self.size // 2
        h = self.size // 2
        screen.set_clip(pygame.Rect(cx-w//2, cy-h//2, w, h))
        screen.blit(rr, rect)

        self.text(screen, "hello", cx, cy)

class Frame:
    def __init__(self):
        pygame.init()
        self.widgets = []
        self.clock = pygame.time.Clock()

    def __del__(self):
        print("bye ...")

    def append(self, widget):
        self.widgets.append(widget)

    def run(self):
        self.screen = pygame.display.set_mode((640,480))
        # pygame.display.flip()
        running = True

        while running:
            self.clock.tick(30)
            self.screen.fill("black")
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        running = False
                    elif event.key == pygame.K_UP:
                        self.widgets[0].pitch += 5
                    elif event.key == pygame.K_DOWN:
                        self.widgets[0].pitch -= 5
                    elif event.key == pygame.K_LEFT:
                        self.widgets[0].roll += 5
                    elif event.key == pygame.K_RIGHT:
                        self.widgets[0].roll -= 5

            for w in self.widgets:
                w.render(self.screen)

            pygame.display.flip()

frame = Frame()
frame.append(HUD(100,100, 500))

frame.run()
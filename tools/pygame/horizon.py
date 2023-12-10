#!/usr/bin/env python3

import pygame
import numpy as np
import threading
import time

def wrap(angle):
    return angle if angle > 0 else 360 + angle


class Horizon:
    def __init__(self, font_size=None):
        if font_size is None:
            font_size = 20

        self.font = pygame.font.SysFont('timesnewroman', font_size)
        pass

    def draw(self, screen, angle):
        deg = u'\xb0'

        dc = pygame.Color(102,0,204)
        mc = pygame.Color(191,128,255)
        lc = pygame.Color(230,204,255)

        w, h = screen.get_size()

        rect = pygame.Rect(0,0,w,30)
        pygame.draw.rect(screen, dc, rect)

        size = 75
        ax = size*np.cos(angle)
        ay = size*np.sin(angle)

        # fill the screen with a color to wipe away anything from last frame
        rect = pygame.Rect(w//2-30, h//2, 2*30, 75)
        pygame.draw.rect(screen, dc, rect, border_radius=5) # pitch bumpout
        pygame.draw.circle(screen,"white",(w//2,h//2),50,) # full circle
        pygame.draw.circle(screen,dc,(w//2,h//2),35) # small circle
        pygame.draw.circle(screen,lc,(w//2,h//2),50,5) # outline

        pygame.draw.line(screen, mc,(w//2-ax,h//2+ay),(w//2+ax,h//2-ay),5) # wings
        pygame.draw.line(screen, mc,(w//2,h//2),(w//2-ay,h//2-ax),5) # tail

        roll = wrap(angle*180/3.14)
        roll = self.font.render(f"{roll:5.1f}{deg}", True, "white", dc)

        pitch = wrap(angle*180/3.14)
        pitch = self.font.render(f"{pitch:5.1f}{deg}", True, "white", dc)
        pw = pitch.get_width()
        screen.blit(pitch,(w//2-pw//2, h//2+50))

        heading = wrap(angle*180/3.14)
        heading = self.font.render(f"{heading:5.1f}{deg}", True, "white", dc)

        rw = roll.get_width()
        rh = roll.get_height()
        screen.blit(roll,(w//2-rw//2,h//2-rh//2))
        hw = heading.get_width()
        screen.blit(heading,(w//2-hw//2,0))


# pygame setup
pygame.init()
screen = pygame.display.set_mode((300, 300), pygame.RESIZABLE)
clock = pygame.time.Clock()
running = True
cnt = 0
horizon = Horizon()

def counter():
    global cnt
    while running:
        cnt += 10
        print(cnt)
        pygame.time.wait(100)

t = threading.Thread(target=counter)
t.daemon = True
t.start()

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    w, h = screen.get_size()

    angle = (cnt % (2*314)) / 100 - 3.14

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("black")

    horizon.draw(screen, angle)

    # flip() the display to put your work on screen
    pygame.display.flip()

    clock.tick(60)  # limits FPS to 60

pygame.quit()
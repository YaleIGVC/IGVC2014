import pygame
import sys

axes=[0,0,0,0,0,0]

pygame.init()

pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

while True:
    c = sys.stdin.read(1)
    if (c == 'q'):
        break
    if c == 'r':
        pygame.event.get()
        for i in range(6):
            axes[i] = joystick.get_axis(i)
        print axes

# Quit!

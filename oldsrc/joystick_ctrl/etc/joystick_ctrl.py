import pygame
import sys

axes=[0,0,0,0,0,0]

pygame.init()

pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print joystick.get_numaxes()

while True:
    c = sys.stdin.read(1)
    if (c == 'q'):
        break
    elif c == 'r':
        pygame.event.get()
        for i in range(6):
            axes[i] = joystick.get_axis(i)
        print axes
    elif c == 'h':
        pygame.event.get()
        print joystick.get_hat(0)

# Quit!

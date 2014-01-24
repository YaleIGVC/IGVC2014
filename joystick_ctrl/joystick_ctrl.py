import pygame
import sys
import curses

stdscr=curses.initscr()
stdscr.keypad(1)
stdscr.nodelay(1)
stdscr.leaveok(1)
curses.noecho()
curses.cbreak()

stdscr=curses.initscr()
stdscr.keypad(1)
stdscr.nodelay(1)
stdscr.leaveok(1)

pad = curses.newpad(200, 200)
pad.scrollok(True)

axes = [0,0,0,0,0,0]


pygame.init()

pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

while 1:
    if stdscr.getch()==ord('q'):
        break
    elif stdscr.getch()==ord('r'):
        for i in range(6):
            axes[i] = joystick.get_axis(i)
    pad.addstr(0,0, "0:");pad.addstr(0,50,str(axes[0])+"   ")
    pad.addstr(1,0, "1:");pad.addstr(0,50,str(axes[1])+"   ")
    pad.addstr(2,0, "2:");pad.addstr(0,50,str(axes[2])+"   ")
    pad.addstr(3,0, "3:");pad.addstr(0,50,str(axes[3])+"   ")
    pad.addstr(4,0, "4:");pad.addstr(0,50,str(axes[4])+"   ")
    pad.addstr(5,0, "5:");pad.addstr(0,50,str(axes[5])+"   ")
    stdscr.refresh()
    pad.refresh( 0,0, 0,0, 100,100)
    #print "RPM\t",rpm,"\tTemp\t",temp,"\tLow Battery\t",battlow,"\tFuelIn\t",fuelin,"\tThrottle\t",throttlein,"\tBMS Fault\t",bmsfault,"\tClutch\t",clutch,"\tAssist In\t",assist,"\tBrake In\t",brake

# Quit!
curses.nocbreak(); stdscr.keypad(0); curses.echo()
curses.endwin()


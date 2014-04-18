import sys
pipe = open('/dev/input/js0','r')
action = []
spacing = 0
while 1:
        for character in pipe.read(1):
                action += [character]
                if len(action) == 8:
                        for byte in action:
                                sys.stdout.write('%02X ' % ord(byte))
                        spacing += 1
                        if spacing == 2:
                                sys.stdout.write('\n')
                                spacing = 0
                        sys.stdout.write('\n')
                        sys.stdout.flush()
                        action = []

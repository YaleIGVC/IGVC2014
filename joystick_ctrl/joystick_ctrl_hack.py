mport sys
pipe = open('/dev/input/js1','r')
action = []
spacing = 0
while True:
	for char in pipe.read(1):
		action += [character]
		if len(action) == 8:
			for byte in action:
				sys.stdout.write('%02X ' % s

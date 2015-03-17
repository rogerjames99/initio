#!/usr/bin/env python
import sys, tty, termios
import getopt
import time
import pigpio

sonar_pulse_start = 0
sonar_pulse_finish = 0
sonar_state = 0 # Waiting for trigger

def getchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print 'char', ch
    return ch

def cbf(g, l, t):
    global sonar_state, sonar_pulse_start, sonar_pulse_finish
    print("gpio={} level={} tick={}".format(g, l, t))
    if sonar_state == 0: # waiting for trigger start
        if l == 1:
            sonar_state = 2 # waiting for trigger end
            print 'Moving to waiting for trigger end'
        else:
            print 'Bad level in waiting for trigger start'
    elif sonar_state == 2: # waiting for trigger end
        if l == 0:
            sonar_state = 3 # waiting for return pulse start
            print 'Moving to waiting for return pulse start'
        else:
            print 'Bad level in waiting for trigger end'
    elif sonar_state == 3: # waiting for return pulse start
        if l == 1:
            sonar_pulse_start = t
            sonar_state = 4 # waiting for return pulse end
            print 'Moving to waiting for return pulse end'
        else:
            print 'Bad level in waiting for return pulse start'
    elif sonar_state == 4: # waiting for return pulse end
        if l == 0:
            sonar_pulse_finish = t
            sonar_state = 5 # cycle complete
            print 'Moving to cycle complete'
        else:
            print 'Bad level in waiting for return pulse end'
    else:
        print 'Unexpected edge'
    print 'Handler complete'
        
   
# Process command line options
opts, args_proper = getopt.getopt(sys.argv[1:], '')

hostname = 'localhost'
if args_proper.__len__() > 0:
    hostname = args_proper[0]
sonar_gpio = 11
if args_proper.__len__() > 1:
    hostname = args_proper[1]

# Initialise pigpio
gpio = pigpio.pi(hostname)

script = (b'm %d w '
            'w %d 1 '
            'mics 10 '
            'w %d 0 '
            'm %d r '
            't '
            'sta p0' % (sonar_gpio, sonar_gpio, sonar_gpio, sonar_gpio))
            
cb = gpio.callback(sonar_gpio, pigpio.EITHER_EDGE, cbf)
                    
sid = gpio.store_script(script)
print 'Sid',sid

while getchar() != 'q':
    sonar_state = 0 # waiting for trigger
    print 'Run script returns', gpio.run_script(sid)
    status = 0
    while status != 1:
        time.sleep(.0001) # Range of about 17 metres
        (status, parameters) = gpio.script_status(sid)
        if status == 1:
            break
    print 'Tick from host', parameters[0] & 0xffffffffL
    while not sonar_state == 5:
        time.sleep(.1)
    print 'Duration in microseconds', sonar_pulse_finish - sonar_pulse_start
    print 'Duration in seconds', float(sonar_pulse_finish - sonar_pulse_start) / 1000000.
    print 'Range', int((float(sonar_pulse_finish - sonar_pulse_start) / 1000000.) * 34000) / 2

print 'Delete script returns', gpio.delete_script(sid)


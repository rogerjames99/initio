#!/usr/bin/env python

import threading
import pigpio
import time
import math
import logging
import sys
#import getopt
import argparse

# Import matplotlib stuff
import matplotlib
matplotlib.use('TkAgg')
from numpy import arange, sin, pi, random, linspace
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.transforms import Affine2D
from matplotlib.projections import PolarAxes
import  mpl_toolkits.axisartist.angle_helper as angle_helper
from mpl_toolkits.axisartist.grid_finder import FixedLocator, MaxNLocator, \
    DictFormatter
import mpl_toolkits.axisartist.floating_axes as floating_axes
from matplotlib.figure import Figure

from Tkinter import *

class App(object):

    def __init__(self, master):
         # Initialise logging
        logging.basicConfig(filename='barnaby.log', level=logging.DEBUG, filemode='w', \
                             format='%(thread)x %(funcName)s %(lineno)d %(levelname)s:%(message)s')
        
       # Some starting constants
        
        self.speed_of_sound_ms = 340.29 # speed of sound in m/s
        
        self.fastServoMoveThreshold = 10 # To avoid 5v rail dips do not move the servo too far in one go
        self.servoDelay = 50 # Delay in mS between servo moves
        
        self.sid = -1
        self.thetaSamples = 50 # Number of theta samples
        self.servoIncrement = 100 # Default amount to move the servo 
        self.wheelDiameter = 5.4 # cm
        self.ticksPer360 = 300 # Number of wheel pulses to turn 360 degrees
        defaultDriveSpeed = 100 # %
        defaultRotationRate = 100 # %
        defaultDriveDistance = 10 # cm
        defaultRotationAmount = 90 # degrees
        self.motorStateStopped = 1
        self.motorStateForward = 2
        self.motorStateReverse = 3
        self.motorStateSpinningLeft = 4
        self.motorStateSpinningRight = 5
        
        # Process command line options
        parser = argparse.ArgumentParser(description='GUI for initio robot')
        parser.add_argument('hostname', default='localhost', nargs='?',
                   help='The hostname of the robot')

        args = parser.parse_args()
        self.hostname = args.hostname
            
        # Initialise pigpio
        pigpio.exceptions = False
        print 'pigpiod host', self.hostname
        try:
            self.gpio = pigpio.pi(self.hostname)
            self.gpio_hardware_revision = self.gpio.get_hardware_revision()
        except:
            print 'Failed to intialise pigpio'
            print 'Are you running on a pi?'
            print 'If not then you can supply a hostname to connect to as a parameter'
            exit(-1)
        pigpio.exceptions = True
        
        # Set up the GPIO channel constants
        self.leftObstacleSensorGPIOChannel = 4
        self.rightMotorForwardGPIOChannel = 7
        self.rightMotorReverseGPIOChannel = 8
        self.leftMotorForwardGPIOChannel = 9
        self.leftMotorReverseGPIOChannel = 10
        self.sonarGPIOChannel = 14
        self.rightObstacleSensorGPIOChannel = 17
        self.leftLineSensorGPIOChannel = 18
        self.leftWheelSensorGPIOChannel = 22
        self.rightWheelSensorGPIOChannel = 23
        self.tiltServoGPIOChannel = 24
        self.panServoGPIOChannel = 25
        if self.gpio_hardware_revision < 4:
            self.rightLineSensorGPIOChannel = 21
        else:
            self.rightLineSensorGPIOChannel = 27
        
        """
        DMA channels (engines) used by this module
        ==========================================
        From 3.12 onwards rasbian kernels have the broadcom dma module built in. 
        This module manages access to dma resources for kernel device drivers
        To see what channels are currently free to use try 'cat /sys/module/dma/parameters/dmachans'.
        You should see the value 32565 returned, converted to hex this gives 0x7f35.
        This is a bit mask so a 0 bit means that the channel is currently reserved.
        You need to add the channels used by this module to the kernel parameter that
        is used to initialise this mask.
        
        To do this edit /boot/cmdline.txt and add dma.dmachans=0x3f15 to the end and reboot.
        If a non default dma.dmachans parameter is already present you will have to work out what channels are free
        and modify the allocations below to match before editing the value there.
        You can check if this has worked by looking at /sys/module/dma/parameters/dmachans again.
        Adding our channels to this list stops the kernel from grabbing them from underneath us!
        If you want to use different channels remember that dma channel 0 is a special channel
        and should not be used, it is not in the reserved list because various kernel subsystems need
        to use it from time to time and they reserve it on a ad-hoc basis. Channel 15 should never be used
        and is normally already in the default reserve list.
        """
        
        # Default dma channels for pigpio
        self.primaryDmaChannel = 14
        self.secondaryDmaChannel = 5

        # Various state variables
        self.leftWheelCount = -1
        self.rightWheelCount = -1
        self.leftObstacleSensorState = 0
        self.rightObstacleSensorState = 0
        self.leftLineSensorState = 0
        self.rightLineSensorState = 0
        self.currentPan = 1500 # centre
        self.currentTilt = 1500 # centre
        self.powerPercentage = int()
        self.rotationPowerPercentage = int()
        
        # Sonar callback
        self.sonar_limit = 300 
        self.sonar_state = 0
        self.sonar_pulse_start = 0
        self.sonar_pulse_finish = 0
        self.gpio.set_mode(self.sonarGPIOChannel, pigpio.OUTPUT) # Just in case
        self.gpio.write(self.sonarGPIOChannel, 0)
        self.gpio.callback(self.sonarGPIOChannel, pigpio.EITHER_EDGE, self.sonarCallback)
        
        # Sonar plot data
        self.theta = []
        self.radius = []
        self.panpos = []
        for theta in linspace(pi*0.1, pi*0.9, self.thetaSamples):
            self.theta.append(theta)
            self.radius.append(0.)
            self.panpos.append(500 + int((2000.0 * theta) / (pi * 10.0)) * 10)
            
        # Lock for global data
        self.dataLock = threading.RLock()
              
        # Event for sonar scan
        self.pingEvent = threading.Event()
        self.pingTerminate = False
        
        # Flag to notify that new sonar data is ready
        self.newSonarDataAvailable = False
        
        # Check if the dma channels are correctly reserved
        if self.hostname == 'localhost':
            try:
                dmamanager = open('/sys/module/dma/parameters/dmachans', mode='r')
                dmachans = int(dmamanager.readline())
                if (dmachans & (1 << self.primaryDmaChannel | 1 << self.secondaryDmaChannel)) != 0:
                    print 'Dma channels are not reserved please see code for help on what to do'
                    exit(-1)
                
            except IOError:
                print 'Cannot read dma manager so cannot check DMA reservations'
                print 'Probably not running on a pi'
                exit(-1)
                        
        # pigiod script for sonar ping
        script = (b'm %d w '
            'w %d 1 '
            'mics 10 '
            'w %d 0 '
            'm %d r '
            't '
            'sta p0' % (self.sonarGPIOChannel, self.sonarGPIOChannel, self.sonarGPIOChannel, self.sonarGPIOChannel))
        self.sid = self.gpio.store_script(script)
        if self.sid < 0:
            print 'Failed to store pigpiod sonar script', pigpio.error_text(self.sid)
            exit(-1)
        
        # Thread for sonar scan
        pingThread = threading.Thread(target = self.pingThreadHandler)
        pingThread.start()

        # Catch the destroy event
        master.protocol('WM_DELETE_WINDOW', self.wmDeleteWindowHandler)
        
        # set up the GUI
        motion = LabelFrame(master, text="Motion", padx=5, pady=5)
        motion.grid(row=0, column=0, padx=10, pady=10)
        button = Button(motion, text="Forward")
        button.grid(row=0, column=1)
        button.bind("<Button-1>", self.forwardCallback)
        button = Button(motion, text="Left")
        button.grid(row=1, column=0)
        button.bind("<Button-1>", self.leftCallback)
        button = Button(motion, text="Stop")
        button.grid(row=1, column=1)
        button.bind("<Button-1>", self.stopCallback)
        button = Button(motion, text="Right")
        button.grid(row=1, column=2)
        button.bind("<Button-1>", self.rightCallback)
        button = Button(motion, text="Back")
        button.grid(row=2, column=1)
        button.bind("<Button-1>", self.backCallback)

        speedframe = LabelFrame(master, text="Speed control", padx=5, pady=5)
        speedframe.grid(row=1, column=0, padx=10, pady=10)
        label = Label(speedframe, text="Drive style")
        label.grid(row=0, column=1)
        self.driveStyle = IntVar()
        self.driveStyle.set(1)
        button = Radiobutton(speedframe, text="Continuous", variable=self.driveStyle, value=1)
        button.grid(row=0, column=0)
        button = Radiobutton(speedframe, text="Incremental", variable=self.driveStyle, value=2)
        button.grid(row=0, column=1)
        button = Radiobutton(speedframe, text="Autonomous", variable=self.driveStyle, value=3)
        button.grid(row=0, column=2)
        label = Label(speedframe, text="Drive speed")
        label.grid(row=1, column=0)
        self.driveSpeed = IntVar()
        self.driveSpeed.set(defaultDriveSpeed)
        self.driveSpeed.trace('w', self.driveSpeedCallback)
        driveSpeed = Spinbox(speedframe, from_=1, to=100, increment=1, textvariable=self.driveSpeed)
        driveSpeed.grid(row=1, column=1)
        label = Label(speedframe, text="%")
        label.grid(row=1, column=2)
        label = Label(speedframe, text="Rotation rate")
        label.grid(row=2, column=0)
        self.rotationRate = IntVar()
        self.rotationRate.set(defaultRotationRate)
        self.rotationRate.trace('w', self.rotationRateCallback)
        spinbox = Spinbox(speedframe, from_=1, to=100, increment=1, textvariable=self.rotationRate)
        spinbox.grid(row=2, column=1)
        label = Label(speedframe, text="%")
        label.grid(row=2, column=2)
        label = Label(speedframe, text="Drive distance")
        label.grid(row=3, column=0)
        self.driveDistance = IntVar()
        self.driveDistance.set(defaultDriveDistance)
        driveDistance = Spinbox(speedframe, from_=1, to=5000, increment=10, textvariable=self.driveDistance)
        driveDistance.grid(row=3, column=1)
        label = Label(speedframe, text="centimetres")
        label.grid(row=3, column=2)
        label = Label(speedframe, text="Rotation amount")
        label.grid(row=4, column=0)
        self.rotationAmount = IntVar()
        self.rotationAmount.set(defaultRotationAmount)
        rotationAmount = Spinbox(speedframe, from_=1, to=360, increment=10, textvariable=self.rotationAmount)
        rotationAmount.grid(row=4, column=1)
        label = Label(speedframe, text="degrees")
        label.grid(row=4, column=2)

        pantilt = LabelFrame(master, text="Pan Tilt Head", padx=5, pady=5)
        pantilt.grid(row=2, column=0, padx=10, pady=10)
        button = Button(pantilt, text="Tilt Up")
        button.grid(row=0, column=1)
        button.bind("<Button-1>", self.ptUpCallback)
        button = Button(pantilt, text="Pan Left")
        button.grid(row=1, column=0)
        button.bind("<Button-1>", self.ptLeftCallback)
        button = Button(pantilt, text="Centre")
        button.grid(row=1, column=1)
        button.bind("<Button-1>", self.ptCentreCallback)
        button = Button(pantilt, text="Pan Right")
        button.grid(row=1, column=2)
        button.bind("<Button-1>", self.ptRightCallback)
        button = Button(pantilt, text="Tilt Down")
        button.grid(row=2, column=1)
        button.bind("<Button-1>", self.ptDownCallback)

        obstacle = LabelFrame(master, text="Obstacle sensors", padx=5, pady=5)
        obstacle.grid(row=3, column=0, padx=10, pady=10)
        label = Label(obstacle, text="Left ")
        label.grid(row=0, column=0)
        self.leftObstacleState = StringVar()
        self.leftObstacleLabel = Label(obstacle, textvariable=self.leftObstacleState)
        self.leftObstacleLabel.grid(row=0, column=1)
        label = Label(obstacle, text="Right ")
        label.grid(row=0, column=2)
        self.rightObstacleState = StringVar()
        self.rightObstacleLabel = Label(obstacle, textvariable=self.rightObstacleState)
        self.rightObstacleLabel.grid(row=0, column=3)

        line = LabelFrame(master, text="Line sensors", padx=5, pady=5)
        line.grid(row=4, column=0, padx=10, pady=10)
        label = Label(line, text="Left ")
        label.grid(row=0, column=0)
        self.leftLineState = StringVar()
        self.leftLineLabel = Label(line, textvariable=self.leftLineState)
        self.leftLineLabel.grid(row=0, column=1)
        label = Label(line, text="Right ")
        label.grid(row=0, column=2)
        self.rightLineState = StringVar()
        self.rightLineLabel = Label(line, textvariable=self.rightLineState)
        self.rightLineLabel.grid(row=0, column=3)

        sonar = LabelFrame(master, text="Sonar", padx=5, pady=5)
        sonar.grid(row=0, column=1, rowspan=5, padx=10, pady=10)

        figure = Figure(figsize=(5,4), dpi=100)
        self.setup_axes(figure, 111)
        self.canvas = FigureCanvasTkAgg(figure, master=sonar)
        self.canvas.show()
        self.canvas.get_tk_widget().grid(row=0, column=0)
        
        button = Button(sonar, text="Ping")
        button.grid(row=1, column=0)
        button.bind("<Button-1>", self.pingCallback)

        # Initialise the motor control
        self.gpio.set_mode(self.leftMotorForwardGPIOChannel, pigpio.OUTPUT)
        self.gpio.set_mode(self.leftMotorReverseGPIOChannel, pigpio.OUTPUT)
        self.gpio.set_mode(self.rightMotorForwardGPIOChannel, pigpio.OUTPUT)
        self.gpio.set_mode(self.rightMotorReverseGPIOChannel, pigpio.OUTPUT)
        self.gpio.set_PWM_range(self.leftMotorForwardGPIOChannel, 100)
        self.gpio.set_PWM_range(self.leftMotorReverseGPIOChannel, 100)
        self.gpio.set_PWM_range(self.rightMotorForwardGPIOChannel, 100)
        self.gpio.set_PWM_range(self.rightMotorReverseGPIOChannel, 100)
        self.gpio.set_PWM_dutycycle(self.leftMotorForwardGPIOChannel, 0)
        self.gpio.set_PWM_dutycycle(self.leftMotorReverseGPIOChannel, 0)
        self.gpio.set_PWM_dutycycle(self.rightMotorForwardGPIOChannel, 0)
        self.gpio.set_PWM_dutycycle(self.rightMotorReverseGPIOChannel, 0)

        # Initialise the servos
        self.gpio.set_mode(self.panServoGPIOChannel, pigpio.OUTPUT)
        self.gpio.set_mode(self.panServoGPIOChannel, pigpio.OUTPUT)
        self.gpio.set_servo_pulsewidth(self.panServoGPIOChannel, self.currentPan) # Centre
        self.gpio.set_servo_pulsewidth(self.tiltServoGPIOChannel, self.currentTilt) # Centre

        # Set up the wheel sensors
        self.gpio.set_mode(self.leftWheelSensorGPIOChannel, pigpio.INPUT)
        self.gpio.set_pull_up_down(self.leftWheelSensorGPIOChannel, pigpio.PUD_DOWN)
        self.gpio.callback(self.leftWheelSensorGPIOChannel, pigpio.EITHER_EDGE, self.leftWheelSensorCallback)
        self.gpio.set_mode(self.rightWheelSensorGPIOChannel, pigpio.INPUT)
        self.gpio.set_pull_up_down(self.rightWheelSensorGPIOChannel, pigpio.PUD_DOWN)
        self.gpio.callback(self.rightWheelSensorGPIOChannel, pigpio.EITHER_EDGE, self.rightWheelSensorCallback)

        # Set up obstacle sensors
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        self.gpio.set_mode(self.leftObstacleSensorGPIOChannel, pigpio.INPUT)
        self.gpio.set_pull_up_down(self.leftObstacleSensorGPIOChannel, pigpio.PUD_DOWN)
        self.gpio.callback(self.leftObstacleSensorGPIOChannel, pigpio.EITHER_EDGE, self.leftObstacleSensorCallback)
        self.gpio.set_mode(self.rightObstacleSensorGPIOChannel, pigpio.INPUT)
        self.gpio.set_pull_up_down(self.rightObstacleSensorGPIOChannel, pigpio.PUD_DOWN)
        self.gpio.callback(self.rightObstacleSensorGPIOChannel, pigpio.EITHER_EDGE, self.rightObstacleSensorCallback)
        self.leftObstacleSensorState = self.gpio.read(self.leftObstacleSensorGPIOChannel)
        if self.leftObstacleSensorState != 0:
            self.leftObstacleLabel.config(background='green')
            self.leftObstacleState.set('High')
        else:
            self.leftObstacleLabel.config(background='red')
            self.leftObstacleState.set('Low')
        self.rightObstacleSensorState = self.gpio.read(self.rightObstacleSensorGPIOChannel)
        if self.rightObstacleSensorState != 0:
            self.rightObstacleLabel.config(background='green')
            self.rightObstacleState.set('High')
        else:
            self.rightObstacleLabel.config(background='red')
            self.rightObstacleState.set('Low')
        self.dataLock.release()
        logging.debug('Released data lock')

        # Set up the line sensors
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        self.gpio.set_mode(self.leftLineSensorGPIOChannel, pigpio.INPUT)
        self.gpio.set_pull_up_down(self.leftLineSensorGPIOChannel, pigpio.PUD_DOWN)
        self.gpio.callback(self.leftLineSensorGPIOChannel, pigpio.EITHER_EDGE, self.leftLineSensorCallback)
        self.gpio.set_mode(self.rightLineSensorGPIOChannel, pigpio.INPUT)
        self.gpio.set_pull_up_down(self.rightLineSensorGPIOChannel, pigpio.PUD_DOWN)
        self.gpio.callback(self.rightLineSensorGPIOChannel, pigpio.EITHER_EDGE, self.rightLineSensorCallback)
        self.leftLineSensorState = self.gpio.read(self.leftLineSensorGPIOChannel)
        if self.leftLineSensorState != 0:
            self.leftLineLabel.config(background='green')
            self.leftLineState.set('High')
        else:
            self.leftLineLabel.config(background='red')
            self.leftLineState.set('Low')
        self.rightLineSensorState = self.gpio.read(self.rightLineSensorGPIOChannel)
        if self.rightLineSensorState != 0:
            self.rightLineLabel.config(background='green')
            self.rightLineState.set('High')
        else:
            self.rightLineLabel.config(background='red')
            self.rightLineState.set('Low')
        self.dataLock.release()
        logging.debug('Released data lock')
                
    def sonarCallback(self, g, l, t):
        logging.debug('gpio = %d level %d = tick = %d', g, l, t)
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        if self.sonar_state == 0: # waiting for trigger start
            if l == 1:
                self.sonar_state = 2 # waiting for trigger end
                logging.debug('Moving to waiting for trigger end')
            else:
                self.sonar_state = 6 # error
                logging.debug('Bad level in waiting for trigger start')
        elif self.sonar_state == 2: # waiting for trigger end
            if l == 0:
                self.sonar_state = 3 # waiting for return pulse start
                logging.debug('Moving to waiting for return pulse start')
            else:
                self.sonar_state = 6 # error
                logging.debug('Bad level in waiting for trigger end')
        elif self.sonar_state == 3: # waiting for return pulse start
            if l == 1:
                self.sonar_pulse_start = t
                self.sonar_state = 4 # waiting for return pulse end
                logging.debug('Moving to waiting for return pulse end')
            else:
                self.sonar_state = 6 # error
                logging.debug('Bad level in waiting for return pulse start')
        elif self.sonar_state == 4: # waiting for return pulse end
            if l == 0:
                self.sonar_pulse_finish = t
                self.sonar_state = 5 # cycle complete
                logging.debug('Moving to cycle complete')
            else:
                self.sonar_state = 6 # error
                logging.debug('Bad level in waiting for return pulse end')
        elif self.sonar_state == 5:
            logging.debug('Unexpected edge in cycle complete state - ignored')
        else:
            logging.debug('Unexpected edge in sonar error state - ignored')
        
        self.dataLock.release()
        logging.debug('Released data lock')
        logging.debug('Handler complete')

    def pingThreadHandler(self):
        global root
        gotLock = False
        while not self.pingTerminate:
            self.pingEvent.wait()
            self.pingEvent.clear()
            if not self.pingTerminate:
                index = int(0)
                for theta in self.theta:
                    # Pan the servo
                    servoOffset = abs(self.panpos[index] - self.currentPan)
                    self.slowServoMove(self.panServoGPIOChannel, self.panpos[index])
                    logging.debug('Sleeping for %.03f', ((2 + servoOffset / self.fastServoMoveThreshold) * self.servoDelay) / 1000.)
                    time.sleep(((2 + servoOffset / self.fastServoMoveThreshold) * self.servoDelay) / 1000.)
                    logging.debug('Acquire the data lock')
                    self.dataLock.acquire()
                    gotLock = True
                    self.sonar_state = 0 # waiting for trigger
                    logging.debug('Got the data lock')
                    while self.sonar_state != 5:
                        if self.sonar_state == 6:
                            self.sonar_state = 0 # waiting for trigger
                        self.dataLock.release()
                        gotLock = False
                        logging.debug('Released data lock')
                        logging.debug('Run script returns %d', self.gpio.run_script(self.sid))
                        status = 0
                        while status != 1:
                            time.sleep(40. / self.speed_of_sound_ms) # Range of about 20 metres
                            if self.pingTerminate:
                                break;
                            (status, parameters) = self.gpio.script_status(self.sid)
                            if status == 1:
                                break
                        if self.pingTerminate:
                            break;
                        logging.debug('Tick from host %d', parameters[0] & 0xffffffffL)
                        logging.debug('Acquire the data lock')
                        self.dataLock.acquire()
                        gotLock = True
                        logging.debug('Got the data lock')
                        while self.sonar_state != 5 and self.sonar_state != 6:
                            logging.debug('Sonar state %d', self.sonar_state)
                            self.dataLock.release()
                            gotLock = False
                            logging.debug('Released data lock')
                            time.sleep(.1)
                            logging.debug('Sonar gpio level = %d', self.gpio.read(self.sonarGPIOChannel))
                            if self.pingTerminate:
                                break;
                            logging.debug('Acquire the data lock')
                            self.dataLock.acquire()
                            gotLock = True
                            logging.debug('Got the data lock')
                            if self.pingTerminate:
                                break;
                            if self.sonar_state == 6:
                                logging.debug('Sonar Error')
                                self.dataLock.release()
                                gotLock = False
                                logging.debug('Released data lock')
                                time.sleep(.5)
                                if self.pingTerminate:
                                    break;
                                logging.debug('Acquire the data lock')
                                self.dataLock.acquire()
                                gotLock = True
                                logging.debug('Got the data lock')
                            if self.pingTerminate:
                                break;
                        if self.pingTerminate:
                            break;
                    if self.pingTerminate:
                        break;
                    self.dataLock.release()
                    gotLock = False
                    logging.debug('Released data lock')
                    # calculate range in centimetres
                    distance = int(((float(self.sonar_pulse_finish - self.sonar_pulse_start) / 1000000.) * self.speed_of_sound_ms / 2.) * 100.)
                    
                    #logging.debug('distance %d', distance)
                    print 'index', index, 'distance', distance
                    logging.debug('Acquire the data lock')
                    self.dataLock.acquire()
                    gotLock = True
                    logging.debug('Got the data lock')
                    self.radius[index] = distance             
                    self.dataLock.release()
                    gotLock = False
                    logging.debug('Released data lock')
                    index = index + 1
                    #break # just do one for debugging
                    time.sleep(.1)
                    if self.pingTerminate:
                        break;

                if self.pingTerminate:
                    break;
                logging.debug('Scan finished')   
                self.slowServoMove(self.panServoGPIOChannel, 1500)
                logging.debug('Acquire the data lock')
                self.dataLock.acquire()
                logging.debug('Got the data lock')
                self.newSonarDataAvailable = True
                self.dataLock.release()
                logging.debug('Released data lock')
                root.after_idle(self.sonarIdleCallback)
            if gotLock:
                self.dataLock.release()
                logging.debug('Released data lock')
            logging.debug('Ping thread exiting')


    def wmDeleteWindowHandler(self):
        global root
        logging.debug('Stopping ping thread')
        self.pingTerminate = True
        self.pingEvent.set()
        root.destroy()
            
    def pingCallback(self, event):
        logging.debug('pingCallback')
        self.pingEvent.set()
        
    def sonarIdleCallback(self):
        global root
        plotit = False
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        if self.newSonarDataAvailable:
            plotit = True
            theta = self.theta[:]
            radius = self.radius[:]
            selfNewSonarDataAvailable = False
        self.dataLock.release()
#        theta.insert(0, 0.)
#        theta.append(0.)
#        radius.insert(0, 0.)
#        radius.append(0.)
        if plotit:
            logging.debug('Plotting')
            self.lines.set_data(theta, radius)
            self.axes.adjust_axes_lim()
            self.axes.relim()
            self.axes.autoscale_view()
            self.auxiliary_axes.autoscale_view()
            self.canvas.draw()
    
    def driveSpeedCallback(self, a, b, c):
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        self.powerPercentage = self.driveSpeed.get()
        self.dataLock.release()
        logging.debug('Released the data lock')
        logging.debug('driveSpeedCallback %d', self.powerPercentage)
        
    def rotationRateCallback(self, a, b, c):
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        self.rotationPowerPercentage = self.rotationRate.get()
        self.dataLock.release()
        logging.debug('Released the data lock')
        logging.debug('rotationRateCallback %d', self.rotationPowerPercentage)
        
    def forwardCallback(self, event):
        if self.driveStyle.get() == 2:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.leftWheelCount = int(self.driveDistance.get() / (self.wheelDiameter * math.pi) * 18.0)
            self.forwardMotors(self.powerPercentage)
            self.dataLock.release()
            logging.debug('Released the data lock')
        elif self.driveStyle.get() == 1:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.forwardMotors(self.powerPercentage)
            self.dataLock.release()
            logging.debug('Released the data lock')
        else:
            self.autonomousDriving()

    def backCallback(self, event):
        if self.driveStyle.get() == 2:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.leftWheelCount = int(self.driveDistance.get() / (self.wheelDiameter * math.pi) * 18.0)
            self.reverseMotors(self.powerPercentage)
            self.dataLock.release()
            logging.debug('Released the data lock')
        elif self.driveStyle.get() == 1:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.reverseMotors(self.powerPercentage)
            self.dataLock.release()
            logging.debug('Released the data lock')
        else:
            self.autonomousDriving()

    def leftCallback(self, event):
        if self.driveStyle.get() == 2:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.leftWheelCount = int(self.ticksPer360 * self.rotationAmount.get() / 360.0)
            self.spinLeftMotors(self.rotationPowerPercentage)
            self.dataLock.release()
            logging.debug('Released the data lock')
        elif self.driveStyle.get() == 1:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.spinLeftMotors(self.rotationPowerPercentage)
            self.dataLock.release()
            logging.debug('Released the data lock')
        else:
            self.autonomousDriving()

    def rightCallback(self, event):
        if self.driveStyle.get() == 2:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.leftWheelCount = int(self.ticksPer360 * self.rotationAmount.get() / 360.0)
            self.spinRightMotors(self.rotationPowerPercentage)
            self.dataLock.release()
            logging.debug('Released the data lock')
        elif self.driveStyle.get() == 1:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.spinRightMotors(self.rotationPowerPercentage)
            self.dataLock.release()
            logging.debug('Released the data lock')
        else:
            self.autonomousDriving()

    def stopCallback(self, event):
        self.stopMotors()

    def stopMotors(self):
        self.gpio.set_PWM_dutycycle(self.leftMotorForwardGPIOChannel, 0)
        self.gpio.set_PWM_dutycycle(self.rightMotorForwardGPIOChannel, 0)
        self.gpio.set_PWM_dutycycle(self.leftMotorReverseGPIOChannel, 0)
        self.gpio.set_PWM_dutycycle(self.rightMotorReverseGPIOChannel, 0)
        logging.debug('Motors stopped')
        
    def setPowerPercentage(self, percent, gpio):
        self.gpio.set_PWM_dutycycle(gpio, percent)
        if gpio == self.leftMotorForwardGPIOChannel:
            pass
            logging.debug('Left forward %d %d', percent, gpio)
        elif gpio == self.rightMotorForwardGPIOChannel:
            pass
            logging.debug('Right forward %d %d', percent, gpio)
        elif gpio == self.leftMotorReverseGPIOChannel:
            pass
            logging.debug('Left reverse %d %d', percent, gpio)
        elif gpio == self.rightMotorReverseGPIOChannel:
            pass
            logging.debug('Right reverse %d %d', percent, gpio)
        else: 
            pass    
            logging.debug('Unknown %d', percent)

    def forwardMotors(self, percent):
        self.setPowerPercentage(percent, self.leftMotorForwardGPIOChannel)
        self.setPowerPercentage(0, self.leftMotorReverseGPIOChannel)
        self.setPowerPercentage(percent, self.rightMotorForwardGPIOChannel)
        self.setPowerPercentage(0, self.rightMotorReverseGPIOChannel)

    def reverseMotors(self, percent):
        self.setPowerPercentage(0, self.leftMotorForwardGPIOChannel)
        self.setPowerPercentage(percent, self.leftMotorReverseGPIOChannel)
        self.setPowerPercentage(0, self.rightMotorForwardGPIOChannel)
        self.setPowerPercentage(percent, self.rightMotorReverseGPIOChannel)

    def spinLeftMotors(self, percent):
        self.setPowerPercentage(0, self.leftMotorForwardGPIOChannel)
        self.setPowerPercentage(percent, self.leftMotorReverseGPIOChannel)
        self.setPowerPercentage(percent, self.rightMotorForwardGPIOChannel)
        self.setPowerPercentage(0, self.rightMotorReverseGPIOChannel)

    def spinRightMotors(self, percent):
        self.setPowerPercentage(percent, self.leftMotorForwardGPIOChannel)
        self.setPowerPercentage(0, self.leftMotorReverseGPIOChannel)
        self.setPowerPercentage(0, self.rightMotorForwardGPIOChannel)
        self.setPowerPercentage(percent, self.rightMotorReverseGPIOChannel)
        
    def slowServoMove(self, channel, targetPosition):
        if channel ==  self.panServoGPIOChannel:
            self.targetPanPosition = targetPosition
            self.panCallback()
        else:
            self.targetTiltPosition = targetPosition
            self.tiltCallback()

    def tiltCallback(self):
        if abs(self.targetTiltPosition - self.currentTilt) > self.fastServoMoveThreshold:
            if self.targetTiltPosition > self.currentTilt:
                newServoPosition = self.currentTilt + self.fastServoMoveThreshold
            else:
                newServoPosition = self.currentTilt - self.fastServoMoveThreshold
            self.gpio.set_servo_pulsewidth(self.tiltServoGPIOChannel, newServoPosition)
            self.currentTilt = newServoPosition
            logging.debug('New tilt servo position %d', newServoPosition)
            root.after(self.servoDelay, self.tiltCallback)
        else:
            self.gpio.set_servo_pulsewidth(self.tiltServoGPIOChannel, self.targetTiltPosition)
            self.currentTilt = self.targetTiltPosition
            logging.debug('Final tilt servo position %d', self.currentTilt)
       
    def panCallback(self):
        if abs(self.targetPanPosition - self.currentPan) > self.fastServoMoveThreshold:
            if self.targetPanPosition > self.currentPan:
                newServoPosition = self.currentPan + self.fastServoMoveThreshold
            else:
                newServoPosition = self.currentPan - self.fastServoMoveThreshold
            self.gpio.set_servo_pulsewidth(self.panServoGPIOChannel, newServoPosition)
            self.currentPan = newServoPosition
            logging.debug('New pan servo position %d', newServoPosition)
            root.after(self.servoDelay, self.panCallback)
        else:
            self.gpio.set_servo_pulsewidth(self.panServoGPIOChannel, self.targetPanPosition)
            self.currentPan = self.targetPanPosition
            logging.debug('Final pan servo position %d', self.currentTilt)
        
    def ptDownCallback(self, event):
        newServoPosition = self.currentTilt + self.servoIncrement
        if newServoPosition > 2500:
            newServoPosition = 2500
        if abs(newServoPosition - self.currentTilt) > self.fastServoMoveThreshold:
            self.slowServoMove(self.tiltServoGPIOChannel, newServoPosition)
        else:
            self.gpio.set_servo_pulsewidth(self.tiltServoGPIOChannel, newServoPosition)
            self.currentTilt = newServoPosition

    def ptUpCallback(self, event):
        newServoPosition = self.currentTilt - self.servoIncrement
        if newServoPosition < 500:
            newServoPosition = 500
        if abs(newServoPosition - self.currentTilt) > self.fastServoMoveThreshold:
            self.slowServoMove(self.tiltServoGPIOChannel, newServoPosition)
        else:
            self.gpio.set_servo_pulsewidth(self.tiltServoGPIOChannel, newServoPosition)
            self.currentTilt = newServoPosition

    def ptLeftCallback(self, event):
        newServoPosition = self.currentPan + self.servoIncrement
        if newServoPosition > 2500:
            newServoPosition = 2500
        if abs(newServoPosition - self.currentPan) > self.fastServoMoveThreshold:
            self.slowServoMove(self.panServoGPIOChannel, newServoPosition)
        else:
            self.gpio.set_servo_pulsewidth(self.panServoGPIOChannel, newServoPosition)
            self.currentPan = newServoPosition

    def ptRightCallback(self, event):
        newServoPosition = self.currentPan - self.servoIncrement
        if newServoPosition < 500:
            newServoPosition = 500
        if abs(newServoPosition - self.currentPan) > self.fastServoMoveThreshold:
            self.slowServoMove(self.panServoGPIOChannel, newServoPosition)
        else:
            self.gpio.set_servo_pulsewidth(self.panServoGPIOChannel, newServoPosition)
            self.currentPan = newServoPosition

    def ptCentreCallback(self, event):
        newServoPosition = 1500
        if abs(newServoPosition - self.currentPan) > self.fastServoMoveThreshold:
            self.slowServoMove(self.panServoGPIOChannel, newServoPosition)
        else:
            self.gpio.set_servo_pulsewidth(self.panServoGPIOChannel, newServoPosition)
            self.currentPan = newServoPosition
        if abs(newServoPosition - self.currentTilt) > self.fastServoMoveThreshold:
            self.slowServoMove(self.tiltServoGPIOChannel, newServoPosition)
        else:
            self.gpio.set_servo_pulsewidth(self.tiltServoGPIOChannel, newServoPosition)
            self.currentTilt = newServoPosition

    def leftWheelSensorCallback(self, gpio_id, value, tick):
        logging.debug('Left wheel sensor state change %d', value)
        if self.leftWheelCount > 0:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.leftWheelCount = self.leftWheelCount - 1
            if self.leftWheelCount == 0:
                self.stopMotors()
                self.leftWheelCount = -1
                self.rightWheelCount = -1
            self.dataLock.release()
            logging.debug('Released the data lock')

    def rightWheelSensorCallback(self, gpio_id, value, tick):
        logging.debug('Right wheel sensor state change %d', value)
        if self.rightWheelCount > 0:
            logging.debug('Acquire the data lock')
            self.dataLock.acquire()
            logging.debug('Got the data lock')
            self.rightWheelCount = self.rightWheelCount - 1
            if self.rightWheelCount == 0:
                self.stopMotors()
                self.leftWheelCount = -1
                self.rightWheelCount = -1
            self.dataLock.release()
            logging.debug('Released the data lock')

    def obstacleSensorStateChange(self, oldLeft, newLeft, oldRight, newRight):
        logging.debug('Obstacle sensor state change old left %d new left %d old right %d new right %d', oldLeft, newLeft, oldRight, newRight)
        if self.driveStyle.get() == 3:
            self.autonomousDriving()

    def leftObstacleSensorCallback(self, gpio_id, value, tick):
        newvalue = value
        logging.debug('Left obstacle sensor state change %d', newvalue)
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        oldvalue = self.leftObstacleSensorState
        if oldvalue != newvalue:
            self.leftObstacleSensorState = newvalue
            self.obstacleSensorStateChange(oldvalue, newvalue, self.rightObstacleSensorState, self.rightObstacleSensorState)
            self.dataLock.release()
            logging.debug('Released the data lock')
            if newvalue != 0:
                self.leftObstacleLabel.config(background='green')
                self.leftObstacleState.set('High')
            else:
                self.leftObstacleLabel.config(background='red')
                self.leftObstacleState.set('Low')
        else:
            self.dataLock.release()
            logging.debug('Released the data lock')

    def rightObstacleSensorCallback(self, gpio_id, value, tick):
        newvalue = value
        logging.debug('Right obstacle sensor state change %d', newvalue)
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        oldvalue = self.rightObstacleSensorState
        if oldvalue != newvalue:
            self.rightObstacleSensorState = newvalue
            self.obstacleSensorStateChange(self.leftObstacleSensorState, self.leftObstacleSensorState, oldvalue, newvalue)
            self.dataLock.release()
            logging.debug('Released the data lock')
            if newvalue != 0:
                self.rightObstacleLabel.config(background='green')
                self.rightObstacleState.set('High')
            else:
                self.rightObstacleLabel.config(background='red')
                self.rightObstacleState.set('Low')
        else:
            self.dataLock.release()
            logging.debug('Released the data lock')

    def lineSensorStateChange(self, oldLeft, newLeft, oldRight, newRight):
        logging.debug('Line sensor state change old Left %d new Left %d old Right %d new Right %d', oldLeft, newLeft, oldRight, newRight)
        return

    def leftLineSensorCallback(self, gpio_id, value, tick):
        newvalue = value
        logging.debug('Left line sensor state change %d', newvalue)
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        oldvalue = self.leftLineSensorState
        if oldvalue != newvalue:
            self.leftLineSensorState = newvalue
            self.lineSensorStateChange(oldvalue, newvalue, self.rightLineSensorState, self.rightLineSensorState)
            self.dataLock.release()
            logging.debug('Released the data lock')
            if newvalue != 0:
                self.leftLineLabel.config(background='green')
                self.leftLineState.set('High')
            else:
                self.leftLineLabel.config(background='red')
                self.leftLineState.set('Low')
        else:
            self.dataLock.release()
            logging.debug('Released the data lock')

    def rightLineSensorCallback(self, gpio_id, value, tick):
        newvalue = value
        logging.debug('Right line sensor state change %d', newvalue)
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        oldvalue = self.rightLineSensorState
        if oldvalue != newvalue:
            self.rightLineSensorState = newvalue
            self.lineSensorStateChange(self.leftLineSensorState, self.leftLineSensorState, oldvalue, newvalue)
            self.dataLock.release()
            logging.debug('Released the data lock')
            if newvalue != 0:
                self.rightLineLabel.config(background='green')
                self.rightLineState.set('High')
            else:
                self.rightLineLabel.config(background='red')
                self.rightLineState.set('Low')
        else:
            self.dataLock.release()
            logging.debug('Released the data lock')

    def setup_axes(self, fig, rect):
        """
        With custom locator and formatter.
        Note that the extreme values are swapped.
        """

        transform = PolarAxes.PolarTransform()

        angle_ticks = [(0, r"$Right$"),
                       (.5*pi, r"$Forward$"),
                       (pi, r"$Left$")]
        grid_locator1 = FixedLocator([v for v, s in angle_ticks])
        tick_formatter1 = DictFormatter(dict(angle_ticks))

        grid_locator2 = MaxNLocator(4)

        self.grid_helper = floating_axes.GridHelperCurveLinear(transform,
                                            extremes=(0, pi, self.sonar_limit, 0),
                                            grid_locator1=grid_locator1,
                                            grid_locator2=grid_locator2,
                                            tick_formatter1=tick_formatter1,
                                            tick_formatter2=None,
                                            )
        self.axes = floating_axes.FloatingSubplot(fig, rect, grid_helper=self.grid_helper)
        self.axes.axis["bottom"].major_ticklabels.set_rotation(180)
        self.axes.axis["left"].set_axis_direction("bottom")
        self.axes.grid(b=True, which='major', color='b', linestyle='-')
        fig.add_subplot(self.axes)

        # create a parasite axes whose transData in RA, cz
        self.auxiliary_axes = self.axes.get_aux_axes(transform)

        self.auxiliary_axes.patch = self.axes.patch # for auxiliary_axis to have a clip path as in ax
        self.axes.patch.zorder=0.9 # but this has a side effect that the patch is
                            # drawn twice, and possibly over some other
                            # artists. So, we decrease the zorder a bit to
                            # prevent this.

        self.lines, = self.auxiliary_axes.plot(self.theta, self.radius)

    def __del__(self):
        logging.debug('Shutting down')
        if self.sid != -1:
            s = self.gpio.delete_script(self.sid)
            if s < 0:
                logging.debug('Failed to delete script %s', pigpio.error_text(s))
        if hasattr(self, 'gpio'):
            self.gpio.stop()
        logging.shutdown()
        
    def autonomousDriving(self):
        # Autonomous driving using the obstacle sensors
        # This may be better as a pigpio script
        self.stopMotors()
        logging.debug('Acquire the data lock')
        self.dataLock.acquire()
        logging.debug('Got the data lock')
        logging.debug('Autonomous driving left OS %d right OS %d', self.leftObstacleSensorState, self.rightObstacleSensorState)
        if self.leftObstacleSensorState and self.rightObstacleSensorState:
            logging.debug('All clear go forward')
            self.forwardMotors(self.powerPercentage)
        elif not self.leftObstacleSensorState and self.rightObstacleSensorState:
            logging.debug('Obstruction on left go right')
            self.spinRightMotors(self.rotationPowerPercentage)
        elif self.leftObstacleSensorState and not self.rightObstacleSensorState:
            logging.debug('Obstruction on right go left')
            self.spinLeftMotors(self.rotationPowerPercentage)
        else:
            logging.debug('Obstruction in in front spin left')
            self.spinRightMotors(self.rotationPowerPercentage)
        self.dataLock.release()
        logging.debug('Released the data lock')
        
print 'Initialising can take a few seconds - be patient!'
root = Tk()

root.title("Barnaby")

app = App(root)

root.mainloop()

del app

print 'Exiting'

#!/usr/bin/env python

import threading
import time
import math
import RPIO
from RPIO import PWM

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

import inspect

class App:

    def lineno(this):
        """Returns the current line number in our program."""
        return inspect.currentframe().f_back.f_lineno

    def __init__(self, master):
        

        # Some starting constants
        self.thetaSamples = 50 # Number of theta samples
        self.servoIncrement = 100 # Default amount to move the servo 
        self.wheelDiameter = 5.4 # cm
        self.ticksPer360 = 300 # Number of wheel pulses to turn 360 degrees
        defaultDriveSpeed = 90 # %
        self.defaultMinimumPowerPercentage = 40 # %
        defaultRotationRate = 100 # %
        defaultDriveDistance = 10 # cm
        defaultRotationAmount = 90 # degrees
        self.pulseIncrementMicroseconds = int()
        self.subcycleWidthMicroseconds =  4000 # This is the lowest I can get to work on a rev 1 board
        self.motorStateStopped = 1
        self.motorStateForward = 2
        self.motorStateReverse = 3
        self.motorStateSpinningLeft = 4
        self.motorStateSpinningRight = 5

        # GPIO channels Broadcom numbering is used by default
        # The commented out settings are the standard 4 tronix ones
        # The current settings are for a Pirocon v2 with IBoost64 daughter board
        # Note the reversal of GPIO channels 22 and 23 and the move swapping
        # of channels for the wheel sensors and obstacle sensors to allow access
        # to JP2 header pins #04 and #17 (wrongly labelled #07 on the board)
        #self.leftObstacleSensorGPIOChannel = 4
        self.leftObstacleSensorGPIOChannel = 23
        self.rightMotorReverseGPIOChannel = 7
        self.rightMotorForwardGPIOChannel = 8
        self.leftMotorReverseGPIOChannel = 9
        self.leftMotorForwardGPIOChannel = 10
        #self.sonarGPIOChannel = 14 # Pirocon
        self.sonarGPIOChannel = 11 # Robohat
        #self.rightObstacleSensorGPIOChannel = 17
        self.rightObstacleSensorGPIOChannel = 22
        self.leftLineSensorGPIOChannel = 18
        #self.leftWheelSensorGPIOChannel = 22
        self.leftWheelSensorGPIOChannel = 4
        #self.rightWheelSensorGPIOChannel = 23
        self.rightWheelSensorGPIOChannel = 17
        self.tiltServoGPIOChannel = 24
        self.panServoGPIOChannel = 25
        if RPIO.RPI_REVISION == 1:
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
        
        To do this edit /boot/cmdline.txt and add dma.dmachans=0x7f05 to the end and reboot.
        If a non default dma.dmachans parameter is already present you will have to work out what channels are free
        and modify the allocations below to match before editing the value there.
        You can check if this has worked by looking at /sys/module/dma/parameters/dmachans again.
        Adding our channels to this list stops the kernel from grabbing them from underneath us!
        If you want to use different channels remember that dma channel 0 is a special channel
        and should not be used, it is not in the reserved list because various kernel subsystems need
        to use it from time to time and they reserve it on a ad-hoc basis. Channel 15 should never be used
        and is normally already in the default reserve list.
        """
        
        self.motorDmaChannel = 4
        self.servoDmaChannel = 5

        # Various state variables
        self.leftWheelCount = -1
        self.rightWheelCount = -1
        self.leftObstacleSensorState = RPIO.LOW
        self.rightObstacleSensorState = RPIO.LOW
        self.leftLineSensorState = RPIO.LOW
        self.rightLineSensorState = RPIO.LOW
        self.currentPan = 1500 # centre
        self.currentTilt = 1500 # centre
        self.powerPercentage = int()
        self.rotationPowerPercentage = int()
        
        # Sonar plot data
        #self.theta = [pi*0.1, pi*0.2, pi*0.3, pi*0.4, pi*0.5, pi*0.6, pi*0.7, pi*0.8, pi*0.9]
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
        try:
            dmamanager = open('/sys/module/dma/parameters/dmachans', mode='r')
            dmachans = int(dmamanager.readline())
            if (dmachans & (1 << self.motorDmaChannel | 1 << self.servoDmaChannel)) != 0:
                print 'Dma channels are not reserved please see code for help on what to do'
                exit(-1)
            
        except IOError:
            print 'Cannot read dma manager please see code for help on what to do'
            exit(-1)
        
        # Kick off idle processing
        # master.after_idle(self.idleCallback)

        
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
        axis, self.auxiliary_axis = self.setup_axes(figure, 111)
        self.canvas = FigureCanvasTkAgg(figure, master=sonar)
        self.canvas.show()
        self.canvas.get_tk_widget().grid(row=0, column=0)
        
        button = Button(sonar, text="Ping")
        button.grid(row=1, column=0)
        button.bind("<Button-1>", self.pingCallback)

        # Setup GPIO interrupt handling
        # Handle all GPIO interrupts in a separate thread so a not to block the GUI
        # TkInter says that all GUI updates should happen on the main thread. I do
        # not know if this applies to linked textvariables, they seem to work OK.
        RPIO.wait_for_interrupts(threaded=True)

        # Initialise the motor control
        PWM.setup(delay_hw=PWM.DELAY_VIA_PCM) # RPIO PWM does not work on multiple dma channels when used with the DELAY_VIA_PWM timing source
        self.pulseIncrementMicroseconds = PWM.get_pulse_incr_us()
        RPIO.setup(self.leftMotorForwardGPIOChannel, RPIO.OUT, initial=RPIO.LOW)
        RPIO.setup(self.leftMotorReverseGPIOChannel, RPIO.OUT, initial=RPIO.LOW)
        RPIO.setup(self.rightMotorForwardGPIOChannel, RPIO.OUT, initial=RPIO.LOW)
        RPIO.setup(self.rightMotorReverseGPIOChannel, RPIO.OUT, initial=RPIO.LOW)
        PWM.init_channel(self.motorDmaChannel, self.subcycleWidthMicroseconds)

        # Initialise the servos
        self.Servos = PWM.Servo(self.servoDmaChannel, 20000)
        self.Servos.set_servo(self.panServoGPIOChannel, self.currentPan) # Centre
        self.Servos.set_servo(self.tiltServoGPIOChannel, self.currentTilt) # Centre

        # Set PWM logging
        PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)

        # Set up the wheel sensors
        RPIO.setup(self.leftWheelSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.leftWheelSensorGPIOChannel, self.leftWheelSensorCallback)
        RPIO.setup(self.rightWheelSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.rightWheelSensorGPIOChannel, self.rightWheelSensorCallback)

        # Set up obstacle sensors
        self.dataLock.acquire()
        #print 'Got the data lock ', self.lineno()
        RPIO.setup(self.leftObstacleSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.leftObstacleSensorGPIOChannel, self.leftObstacleSensorCallback)
        RPIO.setup(self.rightObstacleSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.rightObstacleSensorGPIOChannel, self.rightObstacleSensorCallback)
        self.leftObstacleSensorState = RPIO.input(self.leftObstacleSensorGPIOChannel)
        if self.leftObstacleSensorState != RPIO.LOW:
            self.leftObstacleLabel.config(background='green')
            self.leftObstacleState.set('High')
        else:
            self.leftObstacleLabel.config(background='red')
            self.leftObstacleState.set('Low')
        self.rightObstacleSensorState = RPIO.input(self.rightObstacleSensorGPIOChannel)
        if self.rightObstacleSensorState != RPIO.LOW:
            self.rightObstacleLabel.config(background='green')
            self.rightObstacleState.set('High')
        else:
            self.rightObstacleLabel.config(background='red')
            self.rightObstacleState.set('Low')
        self.dataLock.release()
        #print 'Released data lock ', self.lineno()

        # Set up the line sensors
        self.dataLock.acquire()
        #print 'Got the data lock', self.lineno()
        RPIO.setup(self.leftLineSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.leftLineSensorGPIOChannel, self.leftLineSensorCallback)
        #print 'RLS ', self.rightLineSensorGPIOChannel
        RPIO.setup(self.rightLineSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.rightLineSensorGPIOChannel, self.rightLineSensorCallback)
        self.leftLineSensorState = RPIO.input(self.leftLineSensorGPIOChannel)
        #print 'LLSS ', self.leftLineSensorState
        if self.leftLineSensorState != RPIO.LOW:
            self.leftLineLabel.config(background='green')
            self.leftLineState.set('High')
        else:
            self.leftLineLabel.config(background='red')
            self.leftLineState.set('Low')
        self.rightLineSensorState = RPIO.input(self.rightLineSensorGPIOChannel)
        #print 'RLSS ', self.rightLineSensorState
        if self.rightLineSensorState != RPIO.LOW:
            self.rightLineLabel.config(background='green')
            self.rightLineState.set('High')
        else:
            self.rightLineLabel.config(background='red')
            self.rightLineState.set('Low')
        self.dataLock.release()
        #print 'Released data lock ', self.lineno()
        
    def pingThreadHandler(self):
        global root
        while not self.pingTerminate:
            self.pingEvent.wait()
            self.pingEvent.clear()
            if not self.pingTerminate:
                index = int(0)
                for theta in self.theta:
                    # Pan the servo
                    self.Servos.set_servo(self.panServoGPIOChannel, self.panpos[index])
                    time.sleep(0.5)
                    RPIO.setup(self.sonarGPIOChannel, RPIO.OUT)
                    # Send 10us pulse to trigger
                    RPIO.output(self.sonarGPIOChannel, RPIO.HIGH)
                    time.sleep(0.00001)
                    RPIO.output(self.sonarGPIOChannel, RPIO.LOW)
                    #print 'sent pulse'
                    start = time.time()
                    count=time.time()
                    RPIO.setup(self.sonarGPIOChannel, RPIO.IN)
                    while RPIO.input(self.sonarGPIOChannel) == 0 and time.time() - count < 0.1:
                        start = time.time()
                    count = time.time()
                    stop = count
                    while RPIO.input(self.sonarGPIOChannel) == 1 and time.time() - count <0.1:
                        stop = time.time()
                    # Calculate pulse length
                    elapsed = stop - start
                    #print 'Got echo elasped = ', elapsed
                    # Distance pulse travelled in that time is time
                    # multiplied by the speed of sound (cm/s)
                    # That was the distance there and back so halve the value
                    distance = int(elapsed * 34000) / 2
                    #print 'distance ', distance
                    self.dataLock.acquire()
                    #print 'Got the data lock', self.lineno()
                    self.radius[index] = distance             
                    self.dataLock.release()
                    #print 'Released data lock ', self.lineno()
                    index = index + 1
                self.Servos.set_servo(self.panServoGPIOChannel, self.currentPan)
                self.dataLock.acquire()
                #print 'Got the data lock', self.lineno()
                self.newSonarDataAvailable = True
                self.dataLock.release()
                #print 'Released data lock ', self.lineno()
                root.after_idle(self.idleCallback)

    def wmDeleteWindowHandler(self):
        global root
        self.pingTerminate = True
        self.pingEvent.set()
        root.destroy()
            
    def pingCallback(self, event):
        #print 'pingCallback'
        self.pingEvent.set()
        
    def idleCallback(self):
        global root
        plotit = False
        theta = []
        radius = []
        self.dataLock.acquire()
        if self.newSonarDataAvailable:
            plotit = True
            theta = self.theta
            radius = self.radius
            selfNewSonarDataAvailable = False
        self.dataLock.release()
        theta.append(0.)
        radius.append(0.)
        if plotit:
            #print 'Plotting'
            self.lines.set_data(theta, radius)
            self.canvas.draw()
    
    def driveSpeedCallback(self, a, b, c):
        self.dataLock.acquire()
        #print 'Got the data lock', self.lineno()
        self.powerPercentage = self.driveSpeedToPowerPercentage(self.driveSpeed.get())
        self.dataLock.release()
        #print 'Released data lock ', self.lineno()
        print 'driveSpeedCallback ', self.powerPercentage
        
    def rotationRateCallback(self, a, b, c):
        self.dataLock.acquire()
        #print 'Got the data lock', self.lineno()
        self.rotationPowerPercentage = self.driveSpeedToPowerPercentage(self.rotationRate.get())
        self.dataLock.release()
        #print 'Released data lock ', self.lineno()
        print 'rotationRateCallback ', self.rotationPowerPercentage
        
    def driveSpeedToPowerPercentage(self, driveSpeed):
        return self.defaultMinimumPowerPercentage + (100 - self.defaultMinimumPowerPercentage) * driveSpeed / 100
        
    def forwardCallback(self, event):
        self.stopMotors()
        if self.driveStyle.get() == 2:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.leftWheelCount = int(self.driveDistance.get() / (self.wheelDiameter * math.pi) * 18.0)
            self.forwardMotors(self.powerPercentage)
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()
        elif self.driveStyle.get() == 1:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.forwardMotors(self.powerPercentage)
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()
        else:
            self.autonomousDriving()

    def backCallback(self, event):
        self.stopMotors()
        if self.driveStyle.get() == 2:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.leftWheelCount = int(self.driveDistance.get() / (self.wheelDiameter * math.pi) * 18.0)
            self.reverseMotors(self.powerPercentage)
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()
        elif self.driveStyle.get() == 1:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.reverseMotors(self.powerPercentage)
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()
        else:
            self.autonomousDriving()

    def leftCallback(self, event):
        self.stopMotors()
        if self.driveStyle.get() == 2:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.leftWheelCount = int(self.ticksPer360 * self.rotationAmount.get() / 360.0)
            self.spinLeftMotors(self.rotationPowerPercentage)
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()
        elif self.driveStyle.get() == 1:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.spinLeftMotors(self.rotationPowerPercentage)
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()
        else:
            self.autonomousDriving()

    def rightCallback(self, event):
        self.stopMotors()
        if self.driveStyle.get() == 2:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.leftWheelCount = int(self.ticksPer360 * self.rotationAmount.get() / 360.0)
            self.spinRightMotors(self.rotationPowerPercentage)
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()
        elif self.driveStyle.get() == 1:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.spinRightMotors(self.rotationPowerPercentage)
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()
        else:
            self.autonomousDriving()

    def stopCallback(self, event):
        self.stopMotors()

    def stopMotors(self):
        sleeptime = self.subcycleWidthMicroseconds * 0.0000011 # Alllow some time for DMA to finish
        PWM.clear_channel(self.motorDmaChannel)
        time.sleep(sleeptime)
        RPIO.output(self.leftMotorForwardGPIOChannel, RPIO.LOW)
        RPIO.output(self.rightMotorForwardGPIOChannel, RPIO.LOW)
        RPIO.output(self.leftMotorReverseGPIOChannel, RPIO.LOW)
        RPIO.output(self.rightMotorReverseGPIOChannel, RPIO.LOW)
        print 'Motors stopped'
        
    def setPowerPercentage(self, percent, gpio):
        # Need to split subcycle into 200uS chunks to simulate 5KHz signal
        pulse_width_in_incrs = (200 / self.pulseIncrementMicroseconds) * int(percent) / 100
        print 'Pulse width in incrs ', pulse_width_in_incrs
        if pulse_width_in_incrs < 3:
            print 'Clamping pulse width to 3'
            pulse_width_in_incrs = 3
        if pulse_width_in_incrs > (200 / self.pulseIncrementMicroseconds) - 1:
            pulse_width_in_incrs = (200 / self.pulseIncrementMicroseconds) - 1
            print 'Clamping pulse width to ', pulse_width_in_incrs
        for offset in range(0, self.subcycleWidthMicroseconds / self.pulseIncrementMicroseconds, 200 / self.pulseIncrementMicroseconds):
            PWM.add_channel_pulse(self.motorDmaChannel, gpio, offset, pulse_width_in_incrs)
        if gpio == self.leftMotorForwardGPIOChannel:
            print 'Left forward ', percent, ' ', gpio
        elif gpio == self.rightMotorForwardGPIOChannel:
            print 'Right forward ', percent, ' ', gpio
        elif gpio == self.leftMotorReverseGPIOChannel:
            print 'Left reverse ', percent, ' ', gpio
        elif gpio == self.rightMotorReverseGPIOChannel:
            print 'Right reverse ', percent, ' ', gpio
        else:     
            print 'Unknown ', percent

    def forwardMotors(self, percent):
        #localPercent = percent
        #if localPercent == 100:
        #    localPercent = 99
        self.stopMotors() # Ensure consistent state
        #PWM.add_channel_pulse(self.motorDmaChannel, self.leftMotorForwardGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
        #PWM.add_channel_pulse(self.motorDmaChannel, self.rightMotorForwardGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
        self.setPowerPercentage(percent, self.leftMotorForwardGPIOChannel)
        #self.setPowerPercentage(percent, self.rightMotorForwardGPIOChannel)

    def reverseMotors(self, percent):
        #localPercent = percent
        #if localPercent == 100:
        #    localPercent = 99
        self.stopMotors() # Ensure consistent state
        #PWM.add_channel_pulse(self.motorDmaChannel, self.leftMotorReverseGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
        #PWM.add_channel_pulse(self.motorDmaChannel, self.rightMotorReverseGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
        self.setPowerPercentage(percent, self.leftMotorReverseGPIOChannel)
        self.setPowerPercentage(percent, self.rightMotorReverseGPIOChannel)

    def spinLeftMotors(self, percent):
        #localPercent = percent
        #if localPercent == 100:
        #    localPercent = 99
        self.stopMotors() # Ensure consistent state
        #PWM.add_channel_pulse(self.motorDmaChannel, self.leftMotorReverseGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
        #PWM.add_channel_pulse(self.motorDmaChannel, self.rightMotorForwardGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
        self.setPowerPercentage(percent, self.leftMotorReverseGPIOChannel)
        self.setPowerPercentage(percent, self.rightMotorForwardGPIOChannel)

    def spinRightMotors(self, percent):
        #localPercent = percent
        #if localPercent == 100:
        #    localPercent = 99
        self.stopMotors() # Ensure consistent state
        #PWM.add_channel_pulse(self.motorDmaChannel, self.leftMotorForwardGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
        #PWM.add_channel_pulse(self.motorDmaChannel, self.rightMotorReverseGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
        self.setPowerPercentage(percent, self.leftMotorForwardGPIOChannel)
        self.setPowerPercentage(percent, self.rightMotorReverseGPIOChannel)

    def ptDownCallback(self, event):
        self.currentTilt += self.servoIncrement
        if self.currentTilt > 2500:
            self.currentTilt = 2500
        self.Servos.set_servo(self.tiltServoGPIOChannel, self.currentTilt)

    def ptUpCallback(self, event):
        self.currentTilt -= self.servoIncrement
        if self.currentTilt < 500:
            self.currentTilt = 500
        self.Servos.set_servo(self.tiltServoGPIOChannel, self.currentTilt)

    def ptLeftCallback(self, event):
        self.currentPan += self.servoIncrement
        if self.currentPan > 2500:
            self.currentPan = 2500
        self.Servos.set_servo(self.panServoGPIOChannel, self.currentPan)

    def ptRightCallback(self, event):
        self.currentPan -= self.servoIncrement
        if self.currentPan < 500:
            self.currentPan = 500
        self.Servos.set_servo(self.panServoGPIOChannel, self.currentPan)

    def ptCentreCallback(self, event):
        self.currentPan = 1500
        self.currentTilt = 1500
        self.Servos.set_servo(self.panServoGPIOChannel, self.currentPan)
        self.Servos.set_servo(self.tiltServoGPIOChannel, self.currentTilt)

    def leftWheelSensorCallback(self, gpio_id, value):
        #print 'Left wheel sensor state change ', value
        if self.leftWheelCount > 0:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.leftWheelCount = self.leftWheelCount - 1
            if self.leftWheelCount == 0:
                self.stopMotors()
                self.leftWheelCount = -1
                self.rightWheelCount = -1
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()

    def rightWheelSensorCallback(self, gpio_id, value):
        #print 'Right wheel sensor state change ', value
        if self.rightWheelCount > 0:
            self.dataLock.acquire()
            #print 'Got the data lock', self.lineno()
            self.rightWheelCount = self.rightWheelCount - 1
            if self.rightWheelCount == 0:
                self.stopMotors()
                self.leftWheelCount = -1
                self.rightWheelCount = -1
            self.dataLock.release()
            #print 'Released data lock ', self.lineno()

    def obstacleSensorStateChange(self, oldLeft, newLeft, oldRight, newRight):
        print 'Obstacle sensor state change', oldLeft, ' ', newLeft, ' ', oldRight, ' ', newRight
        if self.driveStyle.get() == 3:
            self.autonomousDriving()

    def leftObstacleSensorCallback(self, gpio_id, value):
        newvalue = bool(value)
        #print 'Left obstacle sensor state change ', newvalue
        self.dataLock.acquire()
        #print 'Got the data lock', self.lineno()
        oldvalue = self.leftObstacleSensorState
        if oldvalue != newvalue:
            self.leftObstacleSensorState = newvalue
            self.obstacleSensorStateChange(oldvalue, newvalue, self.rightObstacleSensorState, self.rightObstacleSensorState)
            if newvalue != RPIO.LOW:
                self.leftObstacleLabel.config(background='green')
                self.leftObstacleState.set('High')
            else:
                self.leftObstacleLabel.config(background='red')
                self.leftObstacleState.set('Low')
        self.dataLock.release()
        #print 'Released data lock ', self.lineno()

    def rightObstacleSensorCallback(self, gpio_id, value):
        newvalue = bool(value)
        #print 'Right obstacle sensor state change ', newvalue
        self.dataLock.acquire()
        #print 'Got the data lock', self.lineno()
        oldvalue = self.rightObstacleSensorState
        if oldvalue != newvalue:
            self.rightObstacleSensorState = newvalue
            self.obstacleSensorStateChange(self.leftObstacleSensorState, self.leftObstacleSensorState, oldvalue, newvalue)
            if newvalue != RPIO.LOW:
                self.rightObstacleLabel.config(background='green')
                self.rightObstacleState.set('High')
            else:
                self.rightObstacleLabel.config(background='red')
                self.rightObstacleState.set('Low')
        self.dataLock.release()
        #print 'Released data lock ', self.lineno()

    def lineSensorStateChange(self, oldLeft, newLeft, oldRight, newRight):
        #print 'Line sensor state change', oldLeft, ' ', newLeft, ' ', oldRight, ' ', newRight
        return

    def leftLineSensorCallback(self, gpio_id, value):
        newvalue = bool(value)
        #print 'Left line sensor state change ', newvalue
        self.dataLock.acquire()
        #print 'Got the data lock', self.lineno()
        oldvalue = self.leftLineSensorState
        if oldvalue != newvalue:
            self.leftLineSensorState = newvalue
            self.lineSensorStateChange(oldvalue, newvalue, self.rightLineSensorState, self.rightLineSensorState)
            if newvalue != RPIO.LOW:
                self.leftLineLabel.config(background='green')
                self.leftLineState.set('High')
            else:
                self.leftLineLabel.config(background='red')
                self.leftLineState.set('Low')
        self.dataLock.release()
        #print 'Released data lock ', self.lineno()

    def rightLineSensorCallback(self, gpio_id, value):
        newvalue = bool(value)
        #print 'Right line sensor state change ', newvalue
        self.dataLock.acquire()
        #print 'Got the data lock', self.lineno()
        oldvalue = self.rightLineSensorState
        if oldvalue != newvalue:
            self.rightLineSensorState = newvalue
            self.lineSensorStateChange(self.leftLineSensorState, self.leftLineSensorState, oldvalue, newvalue)
            if newvalue != RPIO.LOW:
                self.rightLineLabel.config(background='green')
                self.rightLineState.set('High')
            else:
                self.rightLineLabel.config(background='red')
                self.rightLineState.set('Low')
        self.dataLock.release()
        #print 'Released data lock ', self.lineno()

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

        grid_helper = floating_axes.GridHelperCurveLinear(transform,
                                            extremes=(0, pi, 100, 0),
                                            grid_locator1=grid_locator1,
                                            grid_locator2=grid_locator2,
                                            tick_formatter1=tick_formatter1,
                                            tick_formatter2=None,
                                            )

        axis1 = floating_axes.FloatingSubplot(fig, rect, grid_helper=grid_helper)
        axis1.axis["bottom"].major_ticklabels.set_rotation(180)
        axis1.axis["left"].set_axis_direction("bottom")
        axis1.grid(b=True, which='major', color='b', linestyle='-')
        fig.add_subplot(axis1)

        # create a parasite axes whose transData in RA, cz
        auxiliary_axis = axis1.get_aux_axes(transform)

        auxiliary_axis.patch = axis1.patch # for auxiliary_axis to have a clip path as in ax
        axis1.patch.zorder=0.9 # but this has a side effect that the patch is
                            # drawn twice, and possibly over some other
                            # artists. So, we decrease the zorder a bit to
                            # prevent this.

        self.lines, = auxiliary_axis.plot(self.theta, self.radius)
        return axis1, auxiliary_axis

    def __del__(self):
        PWM.cleanup()
        
    def autonomousDriving(self):
        # Autonomous driving using the obstacle sensors
        self.stopMotors()
        self.dataLock.acquire()
        #print 'Got the data lock', self.lineno()
        print 'Autonomous driving left OS ', self.leftObstacleSensorState, ' right OS ', self.rightObstacleSensorState
        if self.leftObstacleSensorState and self.rightObstacleSensorState:
            print 'All clear go forward'
            self.forwardMotors(self.powerPercentage)
        elif not self.leftObstacleSensorState and self.rightObstacleSensorState:
            print 'Obstruction on left go right'
            self.spinRightMotors(self.rotationPowerPercentage)
        elif self.leftObstacleSensorState and not self.rightObstacleSensorState:
            print 'Obstruction on right go left'
            self.spinLeftMotors(self.rotationPowerPercentage)
        else:
            print 'Obstruction in in front spin left'
            self.spinRightMotors(self.rotationPowerPercentage)
        self.dataLock.release()
        #print 'Released data lock ', self.lineno()
        
print 'Initialising can take a few seconds - be patient!'
root = Tk()

root.title("Barnaby")

app = App(root)

root.mainloop()


#!/usr/bin/env python

import thread
import time
import math
import RPIO
from RPIO import PWM

# Import matplotlib stuff
import matplotlib
matplotlib.use('TkAgg')
from numpy import arange, sin, pi, random
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
# implement the default mpl key bindings
#from matplotlib.backend_bases import key_press_handler
from matplotlib.transforms import Affine2D
from matplotlib.projections import PolarAxes
import  mpl_toolkits.axisartist.angle_helper as angle_helper
from mpl_toolkits.axisartist.grid_finder import FixedLocator, MaxNLocator, \
     DictFormatter
import mpl_toolkits.axisartist.floating_axes as floating_axes
from matplotlib.figure import Figure

from Tkinter import *

class App:

    def __init__(self, master):
    
        # Some starting constants
        self.servoIncrement = 100
        self.wheelDiameter = 5.4 # cm
        self.ticksPer360 = 300 # Number of wheel pulses to turn 360 degrees
        defaultDriveSpeed = 30 # %
        defaultRotationRate = 60 # %
        defaultDriveDistance = 10 # cm
        defaultRotationAmount = 90 # degrees
        self.subcycleFrequency = 100 # Hz - I cannot seem to get this to work much above 125 Hz
        self.subcycleWidthMicroseconds = 1000000 / self.subcycleFrequency
        self.defaultPowerPercentage = 14  # 14% seems about as low as I can take it without the motor stalling at 100 Hz
        self.motorStateStopped = 1
        self.motorStateForward = 2
        self.motorStateReverse = 3
        self.motorStateSpinningLeft = 4
        self.motorStateSpinningRight = 5
        
        # GPIO channels Broadcom numbering is used by default
        self.leftObstacleSensorGPIOChannel = 4
        self.rightMotorReverseGPIOChannel = 7
        self.rightMotorForwardGPIOChannel = 8
        self.leftMotorReverseGPIOChannel = 9
        self.leftMotorForwardGPIOChannel = 10
        self.sonarGPIOChannel = 14
        self.rightObstacleSensorGPIOChannel = 17
        self.leftLineSensorGPIOChannel = 18
        self.leftWheelSensorGPIOChannel = 22
        self.rightWheelSensorGPIOChannel = 23
        self.tiltServoGPIOChannel = 24
        self.panServoGPIOChannel = 25
        self.rightLineSensorGPIOChannel = 27
        
        # DMA channels (engines) used by this module
        # ==========================================
        # From 3.12 onwards rasbian kernels have the broadcom dma module built in. 
        # This module manages access to dma resources for kernel device drivers
        # To see what channels are free to use try 'cat /sys/module/dma/parameters/dmachans'.
        # You should see the value 32565 returned, converted to hex this gives 0x7f35.
        # This is a bit mask so a 0 bit means that the channel is reserved.
        # You need to add the channels used by this module to this mask.
        # To do this edit /boot/cmdline.txt and add dma.dmachans=0x7f05 to the end and reboot.
        # If a non default dma.dmachans parameter is already present you will have to work out what channels are free
        # and modify the allocations below to match before editing the value there.
        # You can check if this has worked by looking at /sys/module/dma/parameters/dmachans again.
        # Adding our channels to this list stops the kernel from grabbing them from underneath us!
        # If you want to use different channels remember that dma channel 0 is a special channel
        # and should not be used, it is not in the reserved list because various kernel subsystems need
        # to use it from time to time and they reserve it on a ad-hoc basis. Channel 15 should never be used
        # and is normally already in the default reserve list.
        self.motorDmaChannel = 4
        self.servoDmaChannel = 5

        # Various state variables
        self.leftWheelCount = -1
        self.rightWheelCount = -1
        self.leftObstacleSensorState = RPIO.LOW
        self.rightObstacleSensorState = RPIO.LOW
        self.leftLineSensorState = RPIO.LOW
        self.rightLineSensorState = RPIO.LOW
        self.currentMotorState = self.motorStateStopped
        self.currentPan = 1500
        self.currentTilt = 1500
        
        # Lock for global data
        self.dataLock = thread.allocate_lock()

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
                        
        # set up the GUI
        frame = Frame(master)
        frame.grid()
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
        continuous = Radiobutton(speedframe, text="Continuous", variable=self.driveStyle, value=1)
        continuous.grid(row=0, column=1)
        incremental = Radiobutton(speedframe, text="Incremental", variable=self.driveStyle, value=2)
        incremental.grid(row=0, column=2)
        label = Label(speedframe, text="Drive speed")
        label.grid(row=1, column=0)
        self.driveSpeed = IntVar()
        self.driveSpeed.set(defaultDriveSpeed)
        driveSpeed = Spinbox(speedframe, from_=0, to=100, increment=10, textvariable=self.driveSpeed)
        driveSpeed.grid(row=1, column=1)
        label = Label(speedframe, text="%")
        label.grid(row=1, column=2)
        label = Label(speedframe, text="Rotation rate")
        label.grid(row=2, column=0)
        self.rotationRate = IntVar()
        self.rotationRate.set(defaultRotationRate)
        spinbox = Spinbox(speedframe, from_=0, to=100, increment=10, textvariable=self.rotationRate)
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
        label = Label(obstacle, textvariable=self.leftObstacleState)
        label.grid(row=0, column=1)
        label = Label(obstacle, text="Right ")
        label.grid(row=0, column=2)
        self.rightObstacleState = StringVar()
        label = Label(obstacle, textvariable=self.rightObstacleState)
        label.grid(row=0, column=3)

        line = LabelFrame(master, text="Line sensors", padx=5, pady=5)
        line.grid(row=4, column=0, padx=10, pady=10)
        label = Label(line, text="Left ")
        label.grid(row=0, column=0)
        self.leftLineState = StringVar()
        label = Label(line, textvariable=self.leftLineState)
        label.grid(row=0, column=1)
        label = Label(line, text="Right ")
        label.grid(row=0, column=2)
        self.rightLineState = StringVar()
        label = Label(line, textvariable=self.rightLineState)
        label.grid(row=0, column=3)

        sonar = LabelFrame(master, text="Sonar", padx=5, pady=5)
        sonar.grid(row=0, column=1, rowspan=5, padx=10, pady=10)
        
      
        f = Figure(figsize=(5,4), dpi=100)
        ax, aux_ax = self.setup_axes(f, 111)
        theta = [0, pi*0.1, pi*0.2, pi*0.3, pi*0.4, pi*0.5, pi*0.6, pi*0.7, pi*0.8, pi*0.9, pi] 
        radius = random.rand(11)*2.
        aux_ax.plot(theta, radius)
        canvas = FigureCanvasTkAgg(f, master=sonar)
        canvas.show()
        canvas.get_tk_widget().grid(row=0, column=0)
        
        button = Button(sonar, text="Ping")
        button.grid(row=1, column=0)
        button.bind("<Button-1>", self.pingCallback)
        label = Label(sonar, text="Range ")
        label.grid(row=0, column=1)
        self.range = IntVar()
        label = Label(sonar, textvariable=self.range)
        label.grid(row=0, column=2)

        # Setup GPIO interrupt handling
        # Handle all GPIO interrupts in a separate thread so a not to block the GUI
        # TkInter says that all GUI updates should happen on the main thread. I do
        # not know if this applies to linked textvariables, they seem to work OK.
        RPIO.wait_for_interrupts(threaded=True)

        # Initialise the motor control
        PWM.setup(delay_hw=PWM.DELAY_VIA_PCM) # RPIO PWM does not work on multiple dma channels when used with the DELAY_VIA_PWM timing source
        RPIO.setup(self.leftMotorForwardGPIOChannel, RPIO.OUT, initial=RPIO.LOW)
        RPIO.setup(self.leftMotorReverseGPIOChannel, RPIO.OUT, initial=RPIO.LOW)
        RPIO.setup(self.rightMotorForwardGPIOChannel, RPIO.OUT, initial=RPIO.LOW)
        RPIO.setup(self.rightMotorReverseGPIOChannel, RPIO.OUT, initial=RPIO.LOW)
        currentPulseIncrementMicroseconds = PWM.get_pulse_incr_us()
        print "currentPulseIncrementMicroseconds ", currentPulseIncrementMicroseconds 
        PWM.init_channel(self.motorDmaChannel, self.subcycleWidthMicroseconds)
        print "subcycleWidthMicroseconds ", self.subcycleWidthMicroseconds
        self.pulseIncrementsPercent = self.subcycleWidthMicroseconds / currentPulseIncrementMicroseconds / 100
        print "pulseIncrementPercent ", self.pulseIncrementsPercent

        # Initialise the servos
        self.Servos = PWM.Servo(self.servoDmaChannel, 20000)
        self.Servos.set_servo(self.panServoGPIOChannel, self.currentPan) # Centre
        self.Servos.set_servo(self.tiltServoGPIOChannel, self.currentTilt) # Centre

        # Set up the wheel sensors
        RPIO.setup(self.leftWheelSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.leftWheelSensorGPIOChannel, self.leftWheelSensorCallback)
        RPIO.setup(self.rightWheelSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.rightWheelSensorGPIOChannel, self.rightWheelSensorCallback)

        # Set up obstacle sensors
        self.dataLock.acquire()
        RPIO.setup(self.leftObstacleSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.leftObstacleSensorGPIOChannel, self.leftObstacleSensorCallback)
        RPIO.setup(self.rightObstacleSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.rightObstacleSensorGPIOChannel, self.rightObstacleSensorCallback)
        self.leftObstacleSensorState = RPIO.input(self.leftObstacleSensorGPIOChannel)
        if self.leftObstacleSensorState != RPIO.LOW:
            self.leftObstacleState.set('True')
        else:
            self.leftObstacleState.set('False')
        self.rightObstacleSensorState = RPIO.input(self.rightObstacleSensorGPIOChannel)
        if self.rightObstacleSensorState != RPIO.LOW:
            self.rightObstacleState.set('True')
        else:
            self.rightObstacleState.set('False')
        self.dataLock.release()
        
        # Set up the line sensors
        self.dataLock.acquire()
        RPIO.setup(self.leftLineSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.leftLineSensorGPIOChannel, self.leftLineSensorCallback)
        RPIO.setup(self.rightLineSensorGPIOChannel, RPIO.IN)
        RPIO.add_interrupt_callback(self.rightLineSensorGPIOChannel, self.rightLineSensorCallback)
        self.leftLineSensorState = RPIO.input(self.leftLineSensorGPIOChannel)
        print 'LLSS ', self.leftLineSensorState
        if self.leftLineSensorState != RPIO.LOW:
            self.leftLineState.set('True')
        else:
            self.leftLineState.set('False')
        self.rightLineSensorState = RPIO.input(self.rightLineSensorGPIOChannel)
        print 'RLSS ', self.rightLineSensorState
        if self.rightLineSensorState != RPIO.LOW:
            self.rightLineState.set('True')
        else:
            self.rightLineState.set('False')
	    self.dataLock.release()

    def pingCallback(self, event):
        print 'pingCallback'
        RPIO.setup(self.sonarGPIOChannel, RPIO.OUT)
        # Send 10us pulse to trigger
        RPIO.output(self.sonarGPIOChannel, RPIO.HIGH)
        time.sleep(0.00001)
        RPIO.output(self.sonarGPIOChannel, RPIO.LOW)
        print 'sent pulse'
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
        print 'Got echo elasped = ', elapsed
        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound (cm/s)
        distance = elapsed * 34000
        # That was the distance there and back so halve the value
        self.range.set(distance / 2)

    def forwardCallback(self, event):
	    self.stopMotors()
	    if self.driveStyle.get() == 2:
	        self.dataLock.acquire()
	        self.leftWheelCount = int(self.driveDistance.get() / (self.wheelDiameter * math.pi) * 18.0)
	        self.dataLock.release()
	    self.forwardMotors(self.driveSpeed.get())


    def backCallback(self, event):
	    self.stopMotors()
	    if self.driveStyle.get() == 2:
	        self.dataLock.acquire()
	        self.leftWheelCount = int(self.driveDistance.get() / (self.wheelDiameter * math.pi) * 18.0)
	        self.dataLock.release()
	    self.reverseMotors(self.driveSpeed.get())

    def leftCallback(self, event):
        self.stopMotors()
        if self.driveStyle.get() == 2:
            self.dataLock.acquire()
            self.leftWheelCount = int(self.ticksPer360 * self.rotationAmount.get() / 360.0)
            self.dataLock.release()
        self.spinLeftMotors(self.rotationRate.get())

    def rightCallback(self, event):
	    self.stopMotors()
	    if self.driveStyle.get() == 2:
	        self.dataLock.acquire()
	        self.leftWheelCount = int(self.ticksPer360 * self.rotationAmount.get() / 360.0)
	        self.dataLock.release()
	    self.spinRightMotors(self.rotationRate.get())

    def stopCallback(self, event):
	    self.stopMotors()

    def stopMotors(self):
	    self.dataLock.acquire()
	    if self.currentMotorState == self.motorStateForward:
	        # Going forward
	        PWM.clear_channel_gpio(self.motorDmaChannel, self.leftMotorForwardGPIOChannel) # This appears to set the output to 0, which is good!
	        PWM.clear_channel_gpio(self.motorDmaChannel, self.rightMotorForwardGPIOChannel) # This appears to set the output to 0, which is good!
	    elif self.currentMotorState == self.motorStateReverse:
	        # Reversing
	        PWM.clear_channel_gpio(self.motorDmaChannel, self.leftMotorReverseGPIOChannel) # This appears to set the output to 0, which is good!
	        PWM.clear_channel_gpio(self.motorDmaChannel, self.rightMotorReverseGPIOChannel) # This appears to set the output to 0, which is good!
	    elif self.currentMotorState == self.motorStateSpinningLeft:
	        # Spinning left
	        PWM.clear_channel_gpio(self.motorDmaChannel, self.leftMotorReverseGPIOChannel) # This appears to set the output to 0, which is good!
	        PWM.clear_channel_gpio(self.motorDmaChannel, self.rightMotorForwardGPIOChannel) # This appears to set the output to 0, which is good!
	    elif self.currentMotorState == self.motorStateSpinningRight:
	        # Spinning right
	        PWM.clear_channel_gpio(self.motorDmaChannel, self.leftMotorForwardGPIOChannel) # This appears to set the output to 0, which is good!
	        PWM.clear_channel_gpio(self.motorDmaChannel, self.rightMotorReverseGPIOChannel) # This appears to set the output to 0, which is good!
	    self.currentMotorState = self.motorStateStopped
	    self.dataLock.release()

    def forwardMotors(self, percent):
	    localPercent = percent
	    if localPercent == 100:
	        localPercent = 99
	    self.stopMotors() # Ensure consistent state
	    PWM.add_channel_pulse(self.motorDmaChannel, self.leftMotorForwardGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
	    PWM.add_channel_pulse(self.motorDmaChannel, self.rightMotorForwardGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
	    self.dataLock.acquire()
	    self.currentMotorState = self.motorStateForward
	    self.dataLock.release()

    def reverseMotors(self, percent):
	    localPercent = percent
	    if localPercent == 100:
	        localPercent = 99
	    self.stopMotors() # Ensure consistent state
	    PWM.add_channel_pulse(self.motorDmaChannel, self.leftMotorReverseGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
	    PWM.add_channel_pulse(self.motorDmaChannel, self.rightMotorReverseGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
	    self.dataLock.acquire()
	    self.currentMotorState = self.motorStateReverse
	    self.dataLock.release()

    def spinLeftMotors(self, percent):
	    localPercent = percent
	    if localPercent == 100:
	        localPercent = 99
	    self.stopMotors() # Ensure consistent state
	    PWM.add_channel_pulse(self.motorDmaChannel, self.leftMotorReverseGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
	    PWM.add_channel_pulse(self.motorDmaChannel, self.rightMotorForwardGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
	    self.dataLock.acquire()
	    self.currentMotorState = self.motorStateSpinningLeft
	    self.dataLock.release()

    def spinRightMotors(self, percent):
	    localPercent = percent
	    if localPercent == 100:
	        localPercent = 99
	    self.stopMotors() # Ensure consistent state
	    PWM.add_channel_pulse(self.motorDmaChannel, self.leftMotorForwardGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
	    PWM.add_channel_pulse(self.motorDmaChannel, self.rightMotorReverseGPIOChannel, 0, localPercent * self.pulseIncrementsPercent)
	    self.dataLock.acquire()
	    self.currentMotorState = self.motorStateSpinningRight
	    self.dataLock.release()

    def ptDownCallback(self, event):
	    self.currentTilt += 100
            if self.currentTilt > 2500:
                self.currentTilt = 2500
	    self.Servos.set_servo(self.tiltServoGPIOChannel, self.currentTilt)
    
    def ptUpCallback(self, event):
	    self.currentTilt -= 100
            if self.currentTilt < 500:
                self.currentTilt = 500
	    self.Servos.set_servo(self.tiltServoGPIOChannel, self.currentTilt)
    
    def ptLeftCallback(self, event):
	    self.currentPan += 100
            if self.currentPan > 2500:
                self.currentPan = 2500
	    self.Servos.set_servo(self.panServoGPIOChannel, self.currentPan)
    
    def ptRightCallback(self, event):
	    self.currentPan -= 100
            if self.currentPan < 500:
                self.currentPan = 500
	    self.Servos.set_servo(self.panServoGPIOChannel, self.currentPan)
    
    def ptCentreCallback(self, event):
	    self.currentPan = 1500
	    self.currentTilt = 1500
	    self.Servos.set_servo(self.panServoGPIOChannel, self.currentPan)
	    self.Servos.set_servo(self.tiltServoGPIOChannel, self.currentTilt)

    def leftWheelSensorCallback(self, gpio_id, value):
        if self.leftWheelCount > 0:
            self.dataLock.acquire()
            self.leftWheelCount = self.leftWheelCount - 1
            if self.leftWheelCount == 0:
                self.stopMotors()
            self.leftWheelCount = -1
            self.rightWheelCount = -1
            self.dataLock.release()

    def rightWheelSensorCallback(self, gpio_id, value):
	    if self.rightWheelCount > 0:
	        self.dataLock.acquire()
	        self.rightWheelCount = self.rightWheelCount - 1
	        if self.rightWheelCount == 0:
	        	self.stopMotors()
            self.leftWheelCount = -1
            self.rightWheelCount = -1
            self.dataLock.release()

    def obstacleSensorStateChange(self, oldLeft, newLeft, oldRight, newRight):
	    print 'Obstacle sensor state change', oldLeft, ' ', newLeft, ' ', oldRight, ' ', newRight

    def leftObstacleSensorCallback(self, gpio_id, value):
        newvalue = bool(value)
        print 'Left obstacle sensor interrupt', newvalue
        self.dataLock.acquire()
        oldvalue = self.leftObstacleSensorState
        if oldvalue != newvalue:
            self.leftObstacleSensorState = newvalue
            self.obstacleSensorStateChange(oldvalue, newvalue, self.rightObstacleSensorState, self.rightObstacleSensorState)
            if newvalue != RPIO.LOW:
                self.leftObstacleState.set('True')
            else:
                self.leftObstacleState.set('False')
        self.dataLock.release()
	 
    def rightObstacleSensorCallback(self, gpio_id, value):
        newvalue = bool(value)
        print 'Right obstacle sensor interrupt', newvalue
        self.dataLock.acquire()
        oldvalue = self.rightObstacleSensorState
        if oldvalue != newvalue:
            self.rightObstacleSensorState = newvalue
            self.obstacleSensorStateChange(self.leftObstacleSensorState, self.leftObstacleSensorState, oldvalue, newvalue)
            if newvalue != RPIO.LOW:
               self.rightObstacleState.set('True')
            else:
              self.rightObstacleState.set('False')
        self.dataLock.release()
	 
    def lineSensorStateChange(self, oldLeft, newLeft, oldRight, newRight):
	    print 'Line sensor state change', oldLeft, ' ', newLeft, ' ', oldRight, ' ', newRight

    def leftLineSensorCallback(self, gpio_id, value):
        newvalue = bool(value)
        print 'Left line sensor interrupt', newvalue
        self.dataLock.acquire()
        oldvalue = self.leftLineSensorState
        if oldvalue != newvalue:
            self.leftLineSensorState = newvalue
            self.lineSensorStateChange(oldvalue, newvalue, self.rightLineSensorState, self.rightLineSensorState)
            if newvalue != RPIO.LOW:
                self.leftLineState.set('True')
            else:
                self.leftLineState.set('False')
        self.dataLock.release()
	 
    def rightLineSensorCallback(self, gpio_id, value):
        newvalue = bool(value)
        print 'Right line sensor interrupt', newvalue
        self.dataLock.acquire()
        oldvalue = self.rightLineSensorState
        if oldvalue != newvalue:
            self.rightLineSensorState = newvalue
            self.lineSensorStateChange(self.leftLineSensorState, self.leftLineSensorState, oldvalue, newvalue)
            if newvalue != RPIO.LOW:
                self.rightLineState.set('True')
            else:
                self.rightLineState.set('False')
        self.dataLock.release()
	 
    def setup_axes(self, fig, rect):
        """
        With custom locator and formatter.
        Note that the extreme values are swapped.
        """

        tr = PolarAxes.PolarTransform()

        angle_ticks = [(0, r"$Right$"),
                       (.5*pi, r"$Forward$"),
                       (pi, r"$Left$")]
        grid_locator1 = FixedLocator([v for v, s in angle_ticks])
        tick_formatter1 = DictFormatter(dict(angle_ticks))

        grid_locator2 = MaxNLocator(2)

        grid_helper = floating_axes.GridHelperCurveLinear(tr,
                                            extremes=(pi, 0, 2, 0),
                                            grid_locator1=grid_locator1,
                                            grid_locator2=grid_locator2,
                                            tick_formatter1=tick_formatter1,
                                            tick_formatter2=None,
                                            )

        ax1 = floating_axes.FloatingSubplot(fig, rect, grid_helper=grid_helper)
        fig.add_subplot(ax1)

        # create a parasite axes whose transData in RA, cz
        aux_ax = ax1.get_aux_axes(tr)

        aux_ax.patch = ax1.patch # for aux_ax to have a clip path as in ax
        ax1.patch.zorder=0.9 # but this has a side effect that the patch is
                            # drawn twice, and possibly over some other
                            # artists. So, we decrease the zorder a bit to
                            # prevent this.

        return ax1, aux_ax

    def __del__(self):
	    self.stopMotors()
	    PWM.cleanup()

root = Tk()

root.title("Barnaby")

app = App(root)

root.mainloop()


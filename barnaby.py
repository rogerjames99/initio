#!/usr/bin/env python

import thread
import math
import RPIO
from RPIO import PWM

from Tkinter import *

class App:

    def __init__(self, master):

	# Some starting constants

	# Broadcom GPIO channel numbering is used by default
	self.leftWheelSensorGPIOChannel = 22
	self.rightWheelSensorGPIOChannel = 23
	self.leftMotorForwardGPIOChannel = 10
	self.leftMotorReverseGPIOChannel = 9
	self.rightMotorForwardGPIOChannel = 8
	self.rightMotorReverseGPIOChannel = 7
	self.leftObstacleSensorGPIOChannel = 4
	self.rightObstacleSensorGPIOChannel = 17
	self.leftLineSensorGPIOChannel = 18
	self.rightLineSensorGPIOChannel = 27
	self.panServo = 25
	self.tiltServo = 24

	self.wheelDiameter = 5.4 # cm
	self.ticksPer360 = 300 # Number of wheel pulses to turn 360 degrees
	self.leftWheelCount = -1
	self.rightWheelCount = -1
	defaultDriveSpeed = 30 # %
	defaultRotationRate = 60 # %
	defaultDriveDistance = 10 # cm
	defaultRotationAmount = 90 # degrees
	self.dataLock = thread.allocate_lock()

	# DMA channels (engines) used by this module
	# ==========================================
	# From 3.12 onwards rasbian kernels have the broadcom dma module built in. 
	# This module manages access to dma resources for kernel device drivers
	# To see what channels are free to use try 'cat /sys/module/dma/parameters/dmachans'.
	# You should see the value 32565 returned, converted to hex this gives 0x7f35.
	# This is a bit mask so a 0 bit means that the channel is reserved.
	# You need to add the channels used by this module to this mask.
	# To do this edit /boot/cmdline.txt and add dma.dmachans=0x7f05 to the end and reboot.
	# If a dma.dmachans parameter is already present you will have to work out what channels are free
	# and modify the allocations below to match before editing the value there.
	# You can check if this has worked by looking at /sys/module/dma/parameters/dmachans again.
	# Adding our channles to this list stop the kernel from grabbing them from underneath us!
	# If you want to use different channels remember that dma channel 0 is a special channel
	# and should not be used, it is not in the reserved list because various kernel subsystems need
	# to use it from time to time and they reserve it on a ad-hoc basis. Channel 15 should never be used
	# and is normally already in the default reserve list.
	self.motorDmaChannel = 4
	self.servoDmaChannel = 5

	self.subcycleFrequency = 100 # Hz - I cannot seem to get this to work much above 125 Hz
	self.subcycleWidthMicroseconds = 1000000 / self.subcycleFrequency
	self.defaultPowerPercentage = 14  # 14% seems about as low as I can take it without the motor stalling at 100 Hz
	self.motorStateStopped = 1
	self.motorStateForward = 2
	self.motorStateReverse = 3
	self.motorStateSpinningLeft = 4
	self.motorStateSpinningRight = 5
	self.currentMotorState = self.motorStateStopped

	self.servoIncrement = 100
	self.currentPan = 1500
	self.currentTilt = 1500

	try:
		dmamanager = open('/sys/module/dma/parameters/dmachans', mode='r')
		dmachans = int(dmamanager.readline())
		if (dmachans & (1 << self.motorDmaChannel | 1 << self.servoDmaChannel)) != 0:
			print 'Dma channels are not reserved please see code for help on what to do'
			exit(-1)
	except IOError:
		print 'Cannot read dma manager please see code for help on what to do'
		exit(-1)

        frame = Frame(master)
        frame.pack()

	motion = LabelFrame(master, text="Motion", padx=5, pady=5)
	motion.pack(padx=10, pady=10)

	self.forward = Button(motion, text="Forward")
        self.forward.grid(row=0, column=1)
	self.forward.bind("<Button-1>", self.forwardCallback)
        self.left = Button(motion, text="Left")
        self.left.grid(row=1, column=0)
	self.left.bind("<Button-1>", self.leftCallback)
        self.stop = Button(motion, text="Stop")
        self.stop.grid(row=1, column=1)
	self.stop.bind("<Button-1>", self.stopCallback)
        self.right = Button(motion, text="Right")
        self.right.grid(row=1, column=2)
	self.right.bind("<Button-1>", self.rightCallback)
        self.back = Button(motion, text="Back")
        self.back.grid(row=2, column=1)
	self.back.bind("<Button-1>", self.backCallback)
	speedframe = LabelFrame(master, text="Speed control", padx=5, pady=5)
	speedframe.pack(padx=10, pady=10)
	label = Label(speedframe, text="Drive style")
	label.grid(row=0, column=0)
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
	rotationRate = Spinbox(speedframe, from_=0, to=100, increment=10, textvariable=self.rotationRate)
	rotationRate.grid(row=2, column=1)
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
	pantilt.pack(padx=10, pady=10)

	self.ptUp = Button(pantilt, text="Tilt Up")
        self.ptUp.grid(row=0, column=1)
	self.ptUp.bind("<Button-1>", self.ptUpCallback)
        self.ptLeft = Button(pantilt, text="Pan Left")
        self.ptLeft.grid(row=1, column=0)
	self.ptLeft.bind("<Button-1>", self.ptLeftCallback)
        self.ptCentre = Button(pantilt, text="Centre")
        self.ptCentre.grid(row=1, column=1)
	self.ptCentre.bind("<Button-1>", self.ptCentreCallback)
        self.ptRight = Button(pantilt, text="Pan Right")
        self.ptRight.grid(row=1, column=2)
	self.ptRight.bind("<Button-1>", self.ptRightCallback)
        self.ptDown = Button(pantilt, text="Tilt Down")
        self.ptDown.grid(row=2, column=1)
	self.ptDown.bind("<Button-1>", self.ptDownCallback)

	sonar = LabelFrame(master, text="Sonar", padx=5, pady=5)
	sonar.pack(padx=10, pady=10)

	label = Label(sonar, text="Range ")
	label.grid(row=0, column=0)

	obstacle = LabelFrame(master, text="Obstacle sensors", padx=5, pady=5)
	obstacle.pack(padx=10, pady=10)

	label = Label(obstacle, text="Left ")
	label.grid(row=0, column=0)
	label = Label(obstacle, text="false")
	label.grid(row=0, column=1)
	label = Label(obstacle, text="Right ")
	label.grid(row=0, column=2)
	label = Label(obstacle, text="false")
	label.grid(row=0, column=3)

	# Initialise the the initio
	RPIO.wait_for_interrupts(threaded=True)

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

	# Set up the wheel sensors
	RPIO.setup(self.leftWheelSensorGPIOChannel, RPIO.IN)
	RPIO.add_interrupt_callback(self.leftWheelSensorGPIOChannel, self.leftWheelSensorCallback)
	RPIO.setup(self.rightWheelSensorGPIOChannel, RPIO.IN)
	RPIO.add_interrupt_callback(self.rightWheelSensorGPIOChannel, self.rightWheelSensorCallback)

	# Initialise the servos
	self.Servos = PWM.Servo(self.servoDmaChannel, 20000)
	self.Servos.set_servo(self.panServo, self.currentPan) # Centre
	self.Servos.set_servo(self.tiltServo, self.currentTilt) # Centre


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

    def leftWheelSensorCallback(self, channel):
	if self.leftWheelCount > 0:
	    self.dataLock.acquire()
	    self.leftWheelCount = self.leftWheelCount - 1
	    if self.leftWheelCount == 0:
	    	self.stopMotors()
		self.leftWheelCount = -1
	    self.dataLock.release()

    def rightWheelSensorCallback(self, channel):
	print "Rising edge on right wheel"

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
	self.Servos.set_servo(self.tiltServo, self.currentTilt)
    
    def ptUpCallback(self, event):
	self.currentTilt -= 100
        if self.currentTilt < 500:
            self.currentTilt = 500
	self.Servos.set_servo(self.tiltServo, self.currentTilt)
    
    def ptLeftCallback(self, event):
	self.currentPan += 100
        if self.currentPan > 2500:
            self.currentPan = 2500
	self.Servos.set_servo(self.panServo, self.currentPan)
    
    def ptRightCallback(self, event):
	self.currentPan -= 100
        if self.currentPan < 500:
            self.currentPan = 500
	self.Servos.set_servo(self.panServo, self.currentPan)
    
    def ptCentreCallback(self, event):
	self.currentPan = 1500
	self.currentTilt = 1500
	self.Servos.set_servo(self.panServo, self.currentPan)
	self.Servos.set_servo(self.tiltServo, self.currentTilt)
    

    def __del__(self):
	self.stopMotors()
	PWM.cleanup()

root = Tk()

root.title("Barnaby")

app = App(root)

root.mainloop()

#!/usr/bin/env python2
import time
import numpy as np
#import matplotlib.pyplot as plt

import spidev
# TODO: include spi setup in readme

# must first run 'sudo pigpiod'
#import pigpio
import RPi.GPIO as GPIO

PWM_PIN = 12 # raspberry pi pwm pin for dimmer
ZC_PIN = 15 # zero-crossing interrupt for dimmer

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(ZC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

class MAX31855K(object):
    # Object containing data and methods for MAX31855K thermocouple
    # See https://github.com/sparkfun/SparkFun_MAX31855K_Thermocouple_Breakout_Arduino_Library
    def __init__(self):
        self.spi = spidev.SpiDev()
        bus = 0
        device = 0
        self.spi.open(bus, device)

    def readTempC(self):
        n = 4
        data = self.spi.readbytes(n) # length 4 list of bytes (in decimal)

        data = (data[0] << 3*8) + (data[1] << 2*8) + (data[2] << 8) + data[3]

        if __debug__:
            if data == 0:
                print('Null data.')

            if (data & (1 << 0)) == 1:
                print('Open circuit fault.')

            if (data & (1 << 1)) == 1:
                print('Thermocouple short-circuited to GND.')

            if (data & (1 << 2)) == 1:
                print('Thermocouple short-circuited to Vcc.')

        temp = ( data >> 18)/4.0 # Shift off all but the temperature data, divide by 4

        return temp

    def close(self):
        self.spi.close()

class RBDDimmer(object):
    # Object containing data and methods for RobotDyn Dimmer
    # See https://github.com/RobotDynOfficial/RBDDimmer
    def __init__(self):
        GPIO.add_event_detect(ZC_PIN, GPIO.RISING, callback=zeroCrossEventHandler, bouncetime=100)
        #GPIO.add_event_callback(ZC_PIN, zeroCrossEventHandler, bouncetime=100)

#    def begin(self):


#    def setPower(self):

def zeroCrossEventHandler():
    print('zc')

def pid(err, ierr, derr, Kp, Ki, Kd):

    if err <= 0:
        return 0

    u = Kp*err + Ki*ierr + Kd*derr

    # output is in range [0, 100]
    if u < 0:
        u = 0
    elif u > 100:
        u = 100

    return u

if __name__ == "__main__":
    # initialize measurement object
    # initialize actuator object
    # initialize control loop object
    thermocouple = MAX31855K()
    dimmer = RBDDimmer()

    Kp = 10
    Ki = 5
    Kd = 5

    err = 0
    derr = 0
    ierr = 0

    setpoint = 27

    dt = 1.0

    try:
        while(True):
            # execute control loop
            # read temperature
            temp = thermocouple.readTempC()
            ierr = ierr + err*dt
            olderr = err
            err = setpoint - temp
            derr = (err - olderr)/dt

            print('temp = {}'.format(temp))
            print('err = {}'.format(err))
            print('int err = {}'.format(ierr))
            print('der err = {}'.format(derr))

            # compute output
            u = pid(err, derr, ierr, Kp, Ki, Kd)
            print('u = {}'.format(u))
            # dim
        
            # sleep for the rest of the loop dt
            time.sleep(dt)

    except KeyboardInterrupt:
        # TODO: turn off light
        thermocouple.close()
        GPIO.cleanup()

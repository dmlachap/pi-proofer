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

INTERRUPT_ENABLE = False
ZC_COUNT = 0
ZC_COUNT_ON = 0
ZC_COUNT_OFF = 100
PWM_ON = False
class RBDDimmer(object):
    # Object containing data and methods for RobotDyn Dimmer
    # See https://github.com/RobotDynOfficial/RBDDimmer
    def __init__(self, ac_freq=50):
        self.ac_freq = ac_freq
        GPIO.add_event_detect(ZC_PIN, GPIO.RISING, callback=zeroCrossEventHandler, bouncetime=100)
        #GPIO.add_event_callback(ZC_PIN, zeroCrossEventHandler, bouncetime=100)

    def begin(self):
        INTERRUPT_ENABLE = True

    def setPower(self, power):
        if power > 100:
            power = 100
        elif power < 0:
            power = 0

        pwm_period = 2*ac_freq
        ZC_COUNT_ON = round((power/100.0)*pwm_period)
        ZC_COUNT_OFF = pwm_period - ZC_COUNT_ON

    def close(self):
        self.setPower(0)

def zeroCrossEventHandler():
    if INTERRUPT_ENABLE:
        # print('zc')
        ZC_COUNT = ZC_COUNT + 1
        if PWM_ON and ZC_COUNT >= ZC_COUNT_ON:
            # turn off, reset count
            GPIO.output(PWM_PIN, GPIO.LOW)
            ZC_COUNT = 0
            PWM_ON = False

        if not PWM_ON and ZC_COUNT >= ZC_COUNT_OFF:
            # turn on, reset count
            GPIO.output(PWM_PIN, GPIO.HIGH)
            ZC_COUNT = 0
            PWM_ON = True


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
    dimmer.begin()

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
            dimmer.setPower(u)
            # sleep for the rest of the loop dt
            time.sleep(dt)

    except KeyboardInterrupt:
        dimmer.close()
        thermocouple.close()
        GPIO.cleanup()

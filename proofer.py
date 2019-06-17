import time
import numpy as np
import matplotlib.pyplot as plt

import spidev

# use pigpio for pwm

CHIP_SELECT_PIN = # raspberry pi chip select pin for temp sensor
VCC_PIN = # raspberry pi power pin (3.3v) for temp sensor
GND_PIN = # raspberry pi ground pin for temp sensor

#PWM_PIN = # raspberry pi pwm pin for dimmer
#ZC_PIN = # zero-crossing interrupt for dimmer

class MAX31855K(object):
    # Object containing data and methods for MAX31855K thermocouple
    # See https://github.com/sparkfun/SparkFun_MAX31855K_Thermocouple_Breakout_Arduino_Library
    def __init__():
        self.spi = spidev.SpiDev()
        bus = 0
        device = 1
        self.spi.open(bus, device)

    def readTempC():
        n = 4
        data = self.spi.readbytes(n)
        value = data >> 18 # Shift off all but the temperature data

        temp = value/4.0

        return temp

    def close():
        self.spi.close()

#class RBDDimmer(object):
    # Object containing data and methods for RobotDyn Dimmer
    # See https://github.com/RobotDynOfficial/RBDDimmer
#    def __init__():


#    def begin():


#    def setPower():

def pid(err, ierr, derr, Kp, Ki, Kd)

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

    dt = 1

    try:
        while(true):
        # execute control loop
        # read temperature
        temp =
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
#!/usr/bin/env python

from __future__ import division

import sys
import glob
import threading

import serial


class PWM(object):
    baud_1ms = {
                4000000: 258,
                2000000: 218,
                1000000: 87,
                500000: 45,
                115200: 11.3,
                0: 0
               }

    def __init__(self, port=None, period=1, baudrate=1000000, **kwargs):
        if port is None:
            for f in glob.iglob("/dev/serial/by-path/*"):
                try:
                    self.serial = serial.Serial(f, baudrate=baudrate, **kwargs)
                    break
                except serial.SerialException:
                    continue
            else:
                raise serial.SerialException("Could not find Serial Port.")
        else:
            self.serial = serial.Serial(port)
        self.period = period

    def transmit(self, width, duration=None):
        print width
        down = int(self.baud_1ms[self.serial.baudrate] * width)
        up = int(self.baud_1ms[self.serial.baudrate] * (self.period - width))
        #up = int(self.baud_1ms[self.serial.baudrate] *
                 #(self.period - self.range * width) / 2)
        #down = int(self.baud_1ms[self.serial.baudrate] *
                   #(self.period + self.range * width) / 2)
        i = 0
        if duration is None:
            duration = 1
            period = 0
        else:
            period = self.period
        while i < duration:
            self.serial.write('\xff' * up)
            self.serial.write('\x00' * down)
            i += period


if __name__ == '__main__':
    pwm = PWM(period=20)
    pwm.transmit(1.0, 3000)
    sys.stdin.readline()
    pwm.transmit(0.5, 3000)
    sys.stdin.readline()
    pwm.transmit(2.5, 3000)
    sys.stdin.readline()
    pwm.transmit(1.5, 3000)
    sys.stdin.readline()
    pwm.transmit(18.5, 3000)
    sys.exit(0)
    pwm.transmit(0.0, 2000)
    pwm.transmit(0.5, 2000)
    pwm.transmit(1.0, 2000)
    pwm.transmit(0.5, 2000)
    pwm.transmit(0.0, 2000)
    pwm.transmit(-0.5, 2000)
    pwm.transmit(-1.0, 2000)
    pwm.transmit(-0.5, 2000)
    pwm.transmit(-0.0, 2000)
    sys.exit(0)

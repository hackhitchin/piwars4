#!/usr/bin/python

import time
import VL53L0X as VL53L0X_module
import RPIO


def xshut(gpio_pin):
    """ Turn off all VL53L0X devices.

        Takes a single GPIO pin to the devices' XSHUT ports are connected to,
        and brings it to 0 volts (low) """

    # PNP transistors so GPIO 1 = sensor power off
    RPIO.setup(gpio_pin, RPIO.OUT, initial=1)
    time.sleep(0.5)


def create(gpio_pin, tof_lib, addr):
    """ Turn on a specific device (by GPIO pin which its XSHUT port is connected
        to), create a VL53L0X object for it, and start it in constant ranging
        mode.

        Unresets the chip by making the GPIO a high-impedence input, so that
        the pull-up resister on the LIDAR board can bring XSHUT high.

        Don't set the address to 0x29 if there are any more devices to do.
        Chaos will ensue. """
    RPIO.output(gpio_pin, 0)           # Set the pin low, sensor on
    time.sleep(0.2)                # Wait for chip to wake6

    # Create a VL53L0X object
    tof = VL53L0X_module.VL53L0X(tof_lib=tof_lib, address=addr)
    tof.start_ranging(VL53L0X_module.VL53L0X_LONG_RANGE_MODE)
    print("lidar enabled")

    return tof


def turnoff(gpio_pin):
    RPIO.output(gpio_pin, 1)

from __future__ import division
import time
import i2c_lidar
from enum import Enum

MOTOR_LEFT_PWM = 17
MOTOR_LEFT_A = 22
MOTOR_LEFT_B = 27

MOTOR_RIGHT_PWM = 18
MOTOR_RIGHT_A = 23
MOTOR_RIGHT_B = 24

MAX_SPEED = 90


class I2C_Lidar(Enum):
    # Enum listing each servo that we can control
    LIDAR_FRONT = 10
    # LIDAR_LEFT = 9
    # LIDAR_RIGHT = 11


class Core():
    """ Instantiate a 4WD drivetrain, utilising 2x H Bridges,
        controlled using a 2 axis (throttle, steering)
        system """

    def __init__(self, GPIO, tof_lib):
        """ Constructor """

        # Motors will be disabled by default.
        self.motors_enabled = False
        self.GPIO = GPIO

        # Configure motor pins with GPIO
        self.motor = dict()
        self.motor['left'] = self.setup_motor(
            MOTOR_LEFT_PWM,
            MOTOR_LEFT_A,
            MOTOR_LEFT_B
        )
        self.motor['right'] = self.setup_motor(
            MOTOR_RIGHT_PWM,
            MOTOR_RIGHT_A,
            MOTOR_RIGHT_B
        )

        # Speed Multiplier 1.0 == max
        self.speed_factor = 1.0

        # Create a list of I2C time of flight lidar sensors
        # Note: we need to dynamically alter each
        # tof lidar sensors i2c address on boot
        self.lidars = dict()

        # FIRST: we must turn off all of the i2c
        # devices that have the same initial address.
        for pin in I2C_Lidar:
            i2c_lidar.xshut(int(pin.value))

        # Now loop again and change each ones address individually.
        loop = 0
        for pin in I2C_Lidar:
            print(str(pin))
            self.lidars[str(pin)] = i2c_lidar.create(
                int(pin.value),
                tof_lib,
                0x2a + loop
            )
            loop += 1

        # DEBUG testing
        time.sleep(1.0)
        # start_ranging
        distance_front = self.read_sensor(str(I2C_Lidar.LIDAR_FRONT))
        print('######')
        print(distance_front)
        print('######')

    def increase_speed_factor(self):
        self.speed_factor += 0.1
        # Clamp speed factor to [0.1, 1.0]
        if self.speed_factor > 1.0:
            self.speed_factor = 1.0
        elif self.speed_factor < 0.1:
            self.speed_factor = 0.1

    def decrease_speed_factor(self):
        self.speed_factor -= 0.1
        # Clamp speed factor to [0.1, 1.0]
        if self.speed_factor > 1.0:
            self.speed_factor = 1.0
        elif self.speed_factor < 0.1:
            self.speed_factor = 0.1

    def cleanup(self):
        self.motor['left'].stop()  # stop the PWM output
        self.motor['right'].stop()  # stop the PWM output

        # Turn off i2c lidar tof sensors
        for pin in I2C_Lidar:
            i2c_lidar.turnoff(int(pin.value))

        self.GPIO.cleanup()  # clean up GPIO

    def setup_motor(self, pwm_pin, a, b, frequency=10000):
        """ Setup the GPIO for a single motor.

        Return: PWM controller for single motor.
        """
        self.GPIO.setup(pwm_pin, self.GPIO.OUT)
        self.GPIO.setup(a, self.GPIO.OUT)
        self.GPIO.setup(b, self.GPIO.OUT)

        # Initialise a and b pins to zero (neutral)
        self.GPIO.output(a, 0)
        self.GPIO.output(b, 0)

        # create object D2A for PWM
        D2A = self.GPIO.PWM(pwm_pin, frequency)
        D2A.start(0)  # Initialise the PWM with a 0 percent duty cycle (off)
        return D2A

    def set_neutral(self, braked=False):
        """ Send neutral to the motors IMEDIATELY. """

        # Setting MOTOR pins to LOW will make it free wheel.
        pin_value = 0
        if braked:
            pin_value = 1  # Setting to HIGH will do active braking.
        self.GPIO.output(MOTOR_LEFT_A, pin_value)
        self.GPIO.output(MOTOR_LEFT_B, pin_value)
        self.GPIO.output(MOTOR_RIGHT_A, pin_value)
        self.GPIO.output(MOTOR_RIGHT_B, pin_value)

        # Turn motors off by setting duty cycle back to zero.
        dutycycle = 0.0
        self.motor['right'].ChangeDutyCycle(dutycycle)

    def enable_motors(self, enable):
        """ Called when we want to enable/disable the motors.
            When disabled, will ignore any new motor commands. """

        self.motors_enabled = enable

        # Set motors in neutral if disabling.
        if not enable:
            self.set_neutral()

    def set_motor_speed(self, motor, a, b, speed=0.0):
        """ Change a motors speed.

        Method expects a value in the range of [-1.0, 1.0]
        """
        forward = True

        # If speed is < 0.0, we are driving in reverse.
        if speed < 0.0:
            speed = -speed
            forward = False

        speed *= self.speed_factor

        # Set motor directional pins
        if forward:
            self.GPIO.output(a, 1)
            self.GPIO.output(b, 0)
        else:
            self.GPIO.output(a, 0)
            self.GPIO.output(b, 1)

        # Convert speed into PWM duty cycle
        # and clamp values to min/max ranges.
        dutycycle = speed
        if dutycycle < 0.0:
            dutycycle = 0.0
        elif dutycycle > MAX_SPEED:
            dutycycle = MAX_SPEED

        print(dutycycle)

        # Change the PWM duty cycle based on fabs() of speed value.
        motor.ChangeDutyCycle(dutycycle)

    def throttle(
        self,
        left_speed,
        right_speed
    ):
        """ Send motors speed value in range [-1,1]
            where 0 = neutral """
        if self.motors_enabled:  # Ignore speed change if disabled.
            self.set_motor_speed(
                self.motor['left'],
                MOTOR_LEFT_A,
                MOTOR_LEFT_B,
                speed=left_speed
            )
            self.set_motor_speed(
                self.motor['right'],
                MOTOR_RIGHT_A,
                MOTOR_RIGHT_B,
                speed=right_speed
            )

    def read_sensor(self, sensor):
        """ Read an i2c lidar time of flight
        sensor value and return it. """
        try:
            sensor_value = self.lidars[sensor].get_distance()
        except KeyError:
            print("Key Error")
            print(sensor)
            sensor_value = -1
        return sensor_value


def main():
    """ Simple method used to test motor controller. """
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)

    print("Creating CORE object")
    core = Core(GPIO)

    # Enable motors and drive forwards for x seconds.
    print("Enabling Motors")
    core.enable_motors(True)

    print("Setting Full Throttle")
    core.throttle(0.1, 0.1)
    time.sleep(5)

    print("Disabling Motors")
    core.enable_motors(False)
    # Stop PWM's and clear up GPIO
    core.cleanup()
    print("Finished")


if __name__ == '__main__':
    main()

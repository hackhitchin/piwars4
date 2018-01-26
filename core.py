from __future__ import division
import time

MOTOR_LEFT_A = 17
MOTOR_LEFT_PWM = 27
MOTOR_LEFT_B = 22

MOTOR_RIGHT_A = 23
MOTOR_RIGHT_PWM = 18
MOTOR_RIGHT_B = 24


class Core():
    """ Instantiate a 4WD drivetrain, utilising 2x H Bridges,
        controlled using a 2 axis (throttle, steering)
        system """

    def __init__(self, GPIO):
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

    def cleanup(self):
        self.motor['left'].stop()  # stop the PWM output
        self.motor['right'].stop()  # stop the PWM output
        self.GPIO.cleanup()  # clean up GPIO

    def setup_motor(self, pwm_pin, a, b, frequency=900):
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

        # Set motor directional pins
        if forward:
            self.GPIO.output(a, 1)
            self.GPIO.output(b, 0)
        else:
            self.GPIO.output(a, 0)
            self.GPIO.output(b, 1)

        # Convert speed into PWM duty cycle
        # and clamp values to min/max ranges.
        dutycycle = speed * 100.0
        if speed < 0.0:
            speed = 0
        elif speed > 100.0:
            speed = 100.0

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

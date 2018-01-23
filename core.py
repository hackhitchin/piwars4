from __future__ import division
import RPi.GPIO as GPIO

MOTOR_LEFT_A = 17
MOTOR_LEFT_PWM = 27
MOTOR_LEFT_B = 22

MOTOR_RIGHT_A = 10
MOTOR_RIGHT_PWM = 9
MOTOR_RIGHT_B = 11


class Core():
    """ Instantiate a 4WD drivetrain, utilising 2x H Bridges,
        controlled using a 2 axis (throttle, steering)
        system """

    def __init__(self, tof_lib):
        """ Constructor """

        # Motors will be disabled by default.
        self.motors_enabled = False

        # Set pin number meaning
        GPIO.setmode(GPIO.BCM)

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

    def setup_motor(self, pwm_pin, a, b, frequency=900):
        """ Setup the GPIO for a single motor.

        Return: PWM controller for single motor.
        """
        GPIO.setup(pwm_pin, GPIO.OUT)
        GPIO.setup(a, GPIO.OUT)
        GPIO.setup(b, GPIO.OUT)

        # Initialise a and b pins to zero (neutral)
        GPIO.output(a, 0)
        GPIO.output(b, 0)

        # create object D2A for PWM
        D2A = GPIO.PWM(pwm_pin, frequency)
        D2A.start(0)  # Initialise the PWM with a 0 percent duty cycle (off)
        return D2A

    def set_neutral(self, braked=False):
        """ Send neutral to the motors IMEDIATELY. """

        # Setting MOTOR pins to LOW will make it free wheel.
        pin_value = 0
        if braked:
            pin_value = 1  # Setting to HIGH will do active braking.
        GPIO.output(MOTOR_LEFT_A, pin_value)
        GPIO.output(MOTOR_LEFT_B, pin_value)
        GPIO.output(MOTOR_RIGHT_A, pin_value)
        GPIO.output(MOTOR_RIGHT_B, pin_value)

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

     def throttle(self,
                 left_speed,
                 right_speed,
                 left_servo=ServoEnum.LEFT_MOTOR_ESC,
                 right_servo=ServoEnum.RIGHT_MOTOR_ESC):
        """ Send motors speed value in range [-1,1]
            where 0 = neutral """

        # Calculate microseconds from command speed
        left_micros = 0
        right_micros = 0
        try:
            if left_servo != ServoEnum.SERVO_NONE:
                left_micros = self.servos[left_servo][0].micros(left_speed)
            if right_servo != ServoEnum.SERVO_NONE:
                right_micros = self.servos[right_servo][0].micros(right_speed)
        except:
            print("Failed to get servo throttle micros")

        # Tell the Arduino to move to that speed (eventually)
        if self.arduino:
            self.arduino.throttle(left_micros, right_micros)
        else:
            if self.PWMservo:
                # TODO: make this ramp speeds using RPIO
                try:
                    if left_servo != ServoEnum.SERVO_NONE:
                        self.PWMservo.set_servo(
                            self.servos[left_servo][1],
                            left_micros)
                    if right_servo != ServoEnum.SERVO_NONE:
                        self.PWMservo.set_servo(
                            self.servos[right_servo][1],
                            right_micros)
                except:
                    print("Failed to set servo throttle micros")

    def read_sensor(self, pin):
        """ Read a sensor value and return it. """
        if self.arduino:
            sensor_voltage = self.arduino.read_sensor()
            sensor_value = self.prox.translate(sensor_voltage)
        else:
            sensor_value = self.lidars[pin].get_distance()
        return sensor_value

    def stop(self):
        self.set_neutral()
        for pin in range(0, 3):
            i2c_lidar.turnoff(LIDAR_PINS[pin])

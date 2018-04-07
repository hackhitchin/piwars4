from core import I2C_Lidar
import time
import PID


class Speed:

    def __init__(self, core_module, oled):
        """Class Constructor"""
        self.oled = oled
        self.killed = False
        self.core = core_module
        self.time_limit = 3  # How many seconds before auto cutoff
        self.pidc = PID.PID(0.5, 0.0, 0.1)
        self.control_mode = "LINEAR"
        self.deadband = 10  # size of deadband in mm

        self.pidc = PID.PID(1.0, 0.0, 0.0)
        self.threshold_side = 400.0
        self.threshold_front = 200.0

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def decide_speeds_linear(self, distance_offset):
        """ Use the linear method to decide motor speeds. """
        speed_max = 1.0
        arbitrary_offset = 100

        if (abs(distance_offset) <= self.deadband):
            # Within reasonable tolerance of centre, don't bother steering
            print("Deadband")
            leftspeed = speed_max
            rightspeed = speed_max
        else:
            # Clamp offset to 100mm (arbitrary value for now)
            if distance_offset > arbitrary_offset:
                distance_offset = arbitrary_offset
            elif distance_offset < -arbitrary_offset:
                distance_offset = -arbitrary_offset

            # Calculate how much to reduce speed by on ONE MOTOR ONLY
            speed_drop = (abs(distance_offset) / float(arbitrary_offset))
            print("DropSpeed = {}".format(speed_drop))
            # Reduce speed drop by factor to turning sensitivity/affect.
            speed_drop = speed_drop * 0.8

            if distance_offset < 0:
                leftspeed = speed_max - speed_drop
                rightspeed = speed_max
            elif distance_offset > 0:
                leftspeed = speed_max
                rightspeed = speed_max - speed_drop
            else:
                leftspeed = speed_max
                rightspeed = speed_max

        # Left motors are ever so slightly slower,
        # fudge the rign speed down a tad to balance
        rightspeed *= 0.8

        return leftspeed, rightspeed

    def decide_speeds_pid(self, distance_offset):
        """ Use the pid  method to decide motor speeds. """
        speed_mid = -0.14
        speed_range = -0.2
        distance_range = 50.0

        ignore_d = False
        self.pidc.update(distance_offset, ignore_d)

        deviation = self.pidc.output / distance_range
        c_deviation = max(-1.0, min(1.0, deviation))

        print("PID out: %f" % deviation)

        if self.follow_left:
            leftspeed = (speed_mid - (c_deviation * speed_range))
            rightspeed = (speed_mid + (c_deviation * speed_range))
        else:
            leftspeed = (speed_mid + (c_deviation * speed_range))
            rightspeed = (speed_mid - (c_deviation * speed_range))

        return leftspeed, rightspeed

    def decide_speeds(self, distance_offset):
        """ Set up return values at the start"""
        leftspeed = 0
        rightspeed = 0

        if self.control_mode == "LINEAR":
            leftspeed, rightspeed = self.decide_speeds_linear(
                distance_offset
            )
        elif self.control_mode == "PID":
            leftspeed, rightspeed = self.decide_speeds_pid(
                distance_offset
            )
        return leftspeed, rightspeed

    def show_state(self):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            if self.core.motors_enabled:
                message = "SPEED: %0.2f" % (self.core.speed_factor)
            else:
                message = "SPEED: NEUTRAL"

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def run(self):
        """Read a sensor and set motor speeds accordingly"""

        # Wait (and do nothing) until we enable
        # motors or kill the challenge thread.
        print("Waiting for motor enable")
        while not self.killed and not self.core.motors_enabled:
            time.sleep(0.5)

        # Stop processing if we have killed the thread
        if self.killed:
            return

        print("Starting speed run now...")

        # Reduce MAX speed to 80%, stops motor controller kicking off.
        self.core.speed_factor = 0.5

        start_time = time.time()
        time_delta = 0

        while not self.killed and time_delta < self.time_limit:
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_LEFT)
                ]
                distance_left = lidar_dev['device'].get_distance()
            except KeyError:
                distance_left = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_FRONT)
                ]
                distance_front = lidar_dev['device'].get_distance()
            except KeyError:
                distance_front = -1
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_RIGHT)
                ]
                distance_right = lidar_dev['device'].get_distance()
            except KeyError:
                distance_right = -1

            # Have we fallen out of the end of
            # the course or nearing obstruction?
            if ((distance_left > self.threshold_side and distance_right > self.threshold_side) or
               distance_front < self.threshold_front):
                print("Outside of speed run")
                self.killed = True
                break

            if not self.killed:
                # Report offset from centre
                distance_offset = distance_left - distance_right
                print("Offset is %d (%d : %d)" % (distance_offset, distance_left, distance_right))

                # Got too close, ensure motors are actually working
                if distance_left <= 80 or distance_right <= 80:
                    print("Resetting motors")
                    self.core.reset_motors()

                # Calculate motor speeds
                leftspeed, rightspeed = self.decide_speeds(distance_offset)

                # Send speeds to motors
                self.core.throttle(leftspeed*100.0, rightspeed*100.0)
                print("Motors %f, %f" % (leftspeed, rightspeed))

                time.sleep(0.1)
                current_time = time.time()
                time_delta = current_time - start_time
                print("{}".format(time_delta))

        # Turn motors off and set into neutral (stops the vehicle moving)
        print("Appling Brakes")
        self.core.throttle(-50.0, -50.0)
        time.sleep(0.25)
        print("Neutral")
        self.core.enable_motors(False)


def main():
    """ Method run when codule called separately. """
    import RPi.GPIO as GPIO
    import core

    # Initialise GPIO
    GPIO = GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # Instantiate CORE / Chassis module and store in the launcher.
    core_module = core.Core(GPIO)

    speed = Speed(core_module, None)

    # Manually enable motors in this mode as controller not hooked up
    core_module.enable_motors(True)
    speed.run()

if __name__ == '__main__':
    main()

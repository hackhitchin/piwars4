from core import I2C_Lidar
import time
import PID

class Speed:

    def __init__(self, core_module, oled):
        """Class Constructor"""
        self.oled = oled
        self.killed = False
        self.core = core_module
        self.time_limit = 16  # How many seconds before auto cutoff
        self.pidc = PID.PID(0.5, 0.0, 0.1)
        self.control_mode = "LINEAR"
        self.deadband = 20  # size of deadband in mm

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def decide_speeds_linear(self, distance_offset):
        """ Use the linear method to decide motor speeds. """
        speed_max = 1.0
        arbitrary_offset = 100

        if (distance_offset <= self.deadband):
            # Within reasonable tolerance of centre, don't bother steering
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
            # Reduce speed drop by factor to turning sensitivity/affect.
            speed_drop = speed_drop * 0.5

            if distance_offset > 0:
                leftspeed = speed_max - speed_drop
                rightspeed = speed_max
            elif distance_offset < 0:
                leftspeed = speed_max
                rightspeed = speed_max - speed_drop

        return leftspeed, rightspeed

    def decide_speeds_pid(self, distance_offset):
        """ Use the pid  method to decide motor speeds. """
        # speed_mid = -0.14
        # speed_range = -0.2

        # distance_midpoint = 200.0
        # distance_range = 150.0
        # error = (sensorvalue - distance_midpoint)
        # self.pidc.update(error, ignore_d)

        # deviation = self.pidc.output / distance_range
        # c_deviation = max(-1.0, min(1.0, deviation))

        # print("PID out: %f" % deviation)

        # if self.follow_left:
        #     leftspeed = (speed_mid - (c_deviation * speed_range))
        #     rightspeed = (speed_mid + (c_deviation * speed_range))
        # else:
        #     leftspeed = (speed_mid + (c_deviation * speed_range))
        #     rightspeed = (speed_mid - (c_deviation * speed_range))
        leftspeed = 0
        rightspeed = 0
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
            if self.core_module.motors_enabled:
                message = "SPEED: %0.2f" % (self.core_module.speed_factor)
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
        self.core.speed_factor = 0.8

        start_time = time.time()
        time_delta = 0

        while not self.killed and time_delta < self.time_limit:
            print("Reading LEFT")
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_LEFT)
                ]
                distance_left = lidar_dev['device'].get_distance()
            except KeyError:
                distance_left = -1
            print("Reading FRONT")
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_FRONT)
                ]
                distance_front = lidar_dev['device'].get_distance()
            except KeyError:
                distance_front = -1
            print("Reading RIGHT")
            try:
                lidar_dev = self.core.lidars[
                    str(I2C_Lidar.LIDAR_RIGHT)
                ]
                distance_right = lidar_dev['device'].get_distance()
            except KeyError:
                distance_right = -1

            # Have we fallen out of the end of
            # the course or nearing obstruction?
            if ((distance_left > 400 and distance_right > 400) or
               distance_front < 400):
                print("Outside of speed run")
                self.killed = True
                break

            # Report offset from centre
            distance_offset = distance_left - distance_right
            print("Distance is %d" % (distance_offset))

            # Calculate motor speeds
            leftspeed, rightspeed = self.decide_speeds(distance_offset)

            # Send speeds to motors
            self.core.throttle(leftspeed, rightspeed)
            print("Motors %f, %f" % (leftspeed, rightspeed))

            time.sleep(0.1)
            current_time = time.time()
            time_delta = current_time - start_time
            print("{}".format(time_delta))

        # Turn motors off and set into neutral (stops the vehicle moving)
        self.core.enable_motors(False)

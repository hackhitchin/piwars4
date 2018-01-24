import core
import time


class rc:
    def __init__(self, core_module, controller, oled):
        """Class Constructor"""
        self.killed = False
        self.core_module = core_module
        self.ticks = 0
        # Bind to any available joystick.
        self.controller = controller
        self.oled = oled

    def show_motor_speeds(self, left_motor, right_motor):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            message = "[L: %0.2f] [R: %0.2f]" % (left_motor, right_motor)

            self.oled.cls()  # Clear Screen
            self.oled.canvas.text((10, 10), message, fill=1)
            # Now show the mesasge on the screen
            self.oled.display()

    def stop(self):
        """Simple method to stop the RC loop"""
        self.killed = True

    def mixer(self, yaw, throttle, max_power=100):
        left = throttle + yaw
        right = throttle - yaw
        scale = float(max_power) / max(1, abs(left), abs(right))
        return int(left * scale), int(right * scale)

    def run(self):
        """ Main Challenge method. Has to exist and is the
            start point for the threaded challenge. """
        nTicksSinceLastMenuUpdate = -1
        nTicksBetweenMenuUpdates = 10  # 10*0.05 seconds = every half second

        # Allow user to increase or decrease stick sensitivity
        speed_factor = 1.0

        try:
            # Loop indefinitely, or until this thread is flagged as stopped.
            while self.controller.connected and not self.killed:
                # While in RC mode, get joystick
                # states and pass speeds to motors.

                # Test whether a button is pressed
                self.controller.check_presses()
                if self.controller.has_presses:
                    if 'r1' in self.controller.presses:
                        speed_factor += 0.1
                    if 'l1' in self.controller.presses:
                        speed_factor -= 0.1

                # Clamp speed factor to [0.1, 1.0]
                if speed_factor > 1.0:
                    speed_factor = 1.0
                elif speed_factor < 0.1:
                    speed_factor = 0.1

                # Get joystick values from the left analogue stick
                # x_axis, y_axis = self.controller['lx', 'ly']
                x_axis, y_axis = self.controller['rx', 'ly']
                # print("x,y %0.2f, %0.2f" % (x_axis, y_axis))

                x_axis *= speed_factor
                y_axis *= speed_factor

                l_throttle, r_throttle = self.mixer(x_axis, y_axis, max_power=100)

                if self.core_module:
                    self.core_module.throttle(l_throttle, r_throttle)
                print("Motors %0.2f, %0.2f" % (l_throttle, r_throttle))

                # Show motor speeds on LCD
                if (nTicksSinceLastMenuUpdate == -1 or
                   nTicksSinceLastMenuUpdate >= nTicksBetweenMenuUpdates):
                    self.show_motor_speeds(l_throttle, r_throttle)
                    nTicksSinceLastMenuUpdate = 0
                else:
                    nTicksSinceLastMenuUpdate = nTicksSinceLastMenuUpdate + 1

                # Sleep between loops to allow other stuff to
                # happen and not over burden Pi and Arduino.
                time.sleep(0.05)

        except IOError:
            logging.error(
                "Could not connect to "
                "controller. please try again"
            )

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

    def run(self):
        """ Main Challenge method. Has to exist and is the
            start point for the threaded challenge. """
        nTicksSinceLastMenuUpdate = -1
        nTicksBetweenMenuUpdates = 10  # 10*0.05 seconds = every half second

        # Grab original motor scale factors
        left_motor_esc = self.core_module.servos[ServoEnum.LEFT_MOTOR_ESC][0]
        right_motor_esc = self.core_module.servos[ServoEnum.RIGHT_MOTOR_ESC][0]
        left_motor_orig_scale_factor = left_motor_esc.scale_factor
        right_motor_orig_scale_factor = right_motor_esc.scale_factor

        # Change motors to 1/4 speed
        speed_factor = 0.25
        left_motor_esc.set_scale_factor(speed_factor)
        right_motor_esc.set_scale_factor(speed_factor)

        # Loop indefinitely, or until this thread is flagged as stopped.
        while self.wiimote and not self.killed:
            # While in RC mode, get joystick states and pass speeds to motors.
            try:
                l_joystick_state = \
                    self.wiimote.get_classic_joystick_state(True)
                r_joystick_state = \
                    self.wiimote.get_classic_joystick_state(False)
            except:
                print("Failed to get Joystick")

            # Annotate joystick states to screen
            # if l_joystick_state:
            #     print("l_joystick_state: {}".format(l_joystick_state))
            # if r_joystick_state:
            #     print("r_joystick_state: {}".format(r_joystick_state))

            # Grab normalised x,y / steering,throttle
            # from left and right joysticks.
            l_joystick_pos = l_joystick_state['state']['normalised']
            l_steering, l_throttle = l_joystick_pos
            r_joystick_pos = r_joystick_state['state']['normalised']
            r_steering, r_throttle = r_joystick_pos

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

        # Reset motors to previous speed
        left_motor_esc.set_scale_factor(left_motor_orig_scale_factor)
        right_motor_esc.set_scale_factor(right_motor_orig_scale_factor)

if __name__ == "__main__":
    core = core.Core()
    rc = rc(core)
    try:
        rc.run_auto()
    except (KeyboardInterrupt) as e:
        # except (Exception, KeyboardInterrupt) as e:
        # Stop any active threads before leaving
        rc.stop()
        core.set_neutral()
        print("Quitting")

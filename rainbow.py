#!/usr/bin/env python
# coding: Latin

# Load library functions we want
import time
# import os
import sys
# import ThunderBorg
# import io
import threading
import picamera
import picamera.array
import cv2
import numpy
import core
import RPi.GPIO as GPIO

print('Libraries loaded')

# Global values
global camera
global processor
global debug
global colour
global colourindex
global imageCentreX
global imageCentreY

debug = False
colours = ['blue', 'red', 'green', 'yellow']
colourindex = 0
colour = colours[colourindex]

# Camera settings
imageWidth = 320  # Camera image width
imageHeight = 240  # Camera image height
frameRate = 3  # Camera image capture frame rate

# Auto drive settings
autoMaxPower = 1.0  # Maximum output in automatic mode
autoMinPower = 0.2  # Minimum output in automatic mode
autoMinArea = 10  # Smallest target to move towards
autoMaxArea = 10000  # Largest target to move towards
autoFullSpeedArea = 300  # Max target size


# Image stream processing thread
class StreamProcessor(threading.Thread):

    def __init__(self, core_module):
        self.core_module = core_module
        super(StreamProcessor, self).__init__()
        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.begin = 0

    def run(self):
        # This method runs in a separate thread
        global colour
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    self.stream.seek(0)
                    if self.ProcessImage(self.stream.array, colour):
                        self.terminated = True
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()

    # Image processing function
    def ProcessImage(self, image, colour):
        # View the original image seen by the camera.
        # if debug:
        #    cv2.imshow('original', image)
        #    cv2.waitKey(0)

        # Blur the image
        image = cv2.medianBlur(image, 5)
        if debug:
            cv2.imshow('blur', image)
            cv2.waitKey

        # Convert the image from 'BGR' to HSV colour space
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # if debug:
        #    cv2.imshow('cvtColour', image)
        #    cv2.waitKey(0)

        if colour == "red":
            imrange = cv2.inRange(
                image,
                # numpy.array((113, 96, 64)),
                # numpy.array((125, 255, 255))
                numpy.array((160, 100, 50)),
                numpy.array((180, 255, 255))
            )
        elif colour == 'yellow':
            imrange = cv2.inRange(
                image,
                # numpy.array((15, 64, 64)),
                # numpy.array((35, 255, 255))
                numpy.array((20, 100, 75)),
                numpy.array((40, 255, 255))
            )
        elif colour == "green":
            imrange = cv2.inRange(
                image,
                # numpy.array((50, 96, 64)),
                # numpy.array((85, 255, 255))
                numpy.array((50, 100, 50)),
                numpy.array((80, 255, 255))
            )
        elif colour == 'blue':
            imrange = cv2.inRange(
                image,
                # numpy.array((0, 64, 64)),
                # numpy.array((15, 255, 255))
                numpy.array((90, 64, 64)),
                numpy.array((130, 255, 255))
            )

        # I used the following code to find
        # the approximate 'hue' of the ball in
        # front of the camera
        # for crange in range(100,114,2):
        #    imrange = cv2.inRange(image, numpy.array((crange, 64, 64)), numpy.array((crange+2, 255, 255)))
        #    print(crange)
        #    cv2.imshow('range',imrange)
        #    cv2.waitKey(0)

        # View the filtered image found by 'imrange'
        if debug:
            cv2.imshow('imrange', imrange)
            cv2.waitKey()

        # Find the contours
        contourimage, contours, hierarchy = cv2.findContours(
            imrange,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE
        )
        # if debug:
        #    cv2.imshow('contour', contourimage)
        #    cv2.waitKey(0)

        # Go through each contour
        ballsiness = -1
        x = -1
        y = -1
        area = 0
        for (idx, contour) in enumerate(contours):
            x, y, w, h = cv2.boundingRect(contour)
            # cx = x + (w / 2)
            # cy = y + (h / 2)
            area = w * h

            extent = float(area)/area
            aspect = float(w)/h

            cont_ballsiness = (1.0/aspect if aspect > 1 else aspect)
            cont_ballsiness *= (0.75/extent if extent > 0.75 else extent)
            if (cont_ballsiness > ballsiness):
                if (debug):
                    print("New ballsiest: %f" % ballsiness)
                    print("extent = " + str(extent))
                    print("aspect = " + str(aspect))
                ballsiness = cont_ballsiness
                # ballsiest_index = idx

        if area > 0:
            ball = [x, y, area]
        else:
            ball = None
        # Set drives or report ball status
        finished = self.SetSpeedFromBall(ball)
        return finished

    # Set the motor speed from the ball position
    def SetSpeedFromBall(self, ball):
        global TB
        global colour
        global colours
        global colourindex
        global imageCentreX
        global imageCentreY

        finished = false

        driveLeft = 0.0
        driveRight = 0.0
        if ball:
            x = ball[0]
            area = ball[2]
            if area < autoMinArea:
                print('Too small / far')
                driveLeft = autoMinPower
                driveRight = autoMinPower
            elif area > autoMaxArea:
                print('Close enough')
                colourindex = colourindex + 1
                if (colourindex >= len(colours)):
                    print('Donezo!')
                    finished = True
                else:
                    colour = colours[colourindex]
                    print('Now looking for %s ball' % (colour))
            else:
                if area < autoFullSpeedArea:
                    speed = 1.0
                else:
                    speed = 1.0 / (area / autoFullSpeedArea)
                speed *= autoMaxPower - autoMinPower
                speed += autoMinPower
                direction = (imageCentreX - x) / imageCentreX
                if direction < 0.0:
                    # Turn left
                    print('Turn left for %s' % colour)
                    driveLeft = speed * (1.0 + direction)
                    driveRight = speed
                else:
                    # Turn right
                    print('Turn right for %s' % colour)
                    driveLeft = speed
                    driveRight = speed * (1.0 - direction)
        else:
            print('No %s ball' % colour)
            driveLeft = 0.4
            driveRight = 0.0
        print('%.2f, %.2f' % (driveLeft, driveRight))
        self.core_module.throttle(driveLeft, driveRight)

        return finished


# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        global camera
        global processor
        print('Start the stream using the video port')
        camera.capture_sequence(
            self.TriggerStream(),
            format='bgr',
            use_video_port=True
        )
        print('Terminating camera processing...')
        processor.terminated = True
        processor.join()
        print('Processing terminated.')

    # Stream delegation loop
    def TriggerStream(self):
        global running
        while running:
            if processor.event.is_set():
                time.sleep(0.01)
            else:
                yield processor.stream
                processor.event.set()


class Rainbow:
    def __init__(self, core_module, oled):
        """Class Constructor"""
        self.killed = False
        self.core_module = core_module
        self.ticks = 0
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

    def show_state(self):
        """ Show motor/aux config on OLED display """
        if self.oled is not None:
            # Format the speed to 2dp
            if self.core_module.motors_enabled:
                message = "Rainbow: %0.2f" % (self.core_module.speed_factor)
            else:
                message = "Rainbow: NEUTRAL"

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
        global camera
        global processor
        global imageCentreX
        global imageCentreY

        # Limit motor speeds in AutoMode
        self.core_module.decrease_speed_factor()  # 90%
        self.core_module.decrease_speed_factor()  # 80%
        self.core_module.decrease_speed_factor()  # 70%
        self.core_module.decrease_speed_factor()  # 60%
        self.core_module.enable_motors(True)

        # Setup the camera
        print('Setup camera')
        camera = picamera.PiCamera()
        camera.resolution = (imageWidth, imageHeight)
        camera.framerate = frameRate
        camera.awb_mode = 'off'

        # Load the exposure calibration
        try:
            with open("rbgains.txt") as f:
                content = f.readlines()
        except:
            # Failed to read the file, use sensible defaults
            content = "R: 1.30\nB: 2.40\n"

        content = [x.strip() for x in content]
        redgain = float(content[0][2:])
        bluegain = float(content[1][2:])
        camera.awb_gains = (redgain, bluegain)

        imageCentreX = imageWidth / 2.0
        imageCentreY = imageHeight / 2.0

        # Create a stream processor instance
        processor = StreamProcessor(self.core_module)

        # Wait a couple of seconds...lets the camera start and expose
        time.sleep(2)
        captureThread = ImageCapture()

        try:
            print('Press CTRL+C to quit')
            # Loop indefinitely until we are no longer running
            while not self.killed:
                # Wait for the interval period
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("User shutdown\n")
        except:
            e = sys.exc_info()[0]
            print
            print(e)

        self.core_module.enable_motors(False)
        self.killed = True
        captureThread.join()
        processor.terminated = True
        processor.join()
        del camera
        print("Program terminated")


def main():

    # Startup sequence

    # Initialise GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # Instantiate CORE / Chassis module and store in the launcher.
    core_module = core.Core(GPIO)
    rb = Rainbow(core_module, None)
    rb.Run()

if __name__ == '__main__':
    main()

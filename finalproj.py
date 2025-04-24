import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

'''
The following function is inspired by:
    User raja_961, "Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV". 
    Instructables. URL: https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/
'''

SPEED_PIN = 18
STEERING_PIN = 19

freq = 50

def start_video():
    '''
    - Inputs: None
    - Outputs: 
            - video: a cv2 video object, can be thought of as 
              just the video captured from the camera at index 0
    - Effects: Allows us to start collecting video from the default camera 
             at 320x240 resolution
    - Notes: The website recommends we lower the reolution if we want to 
             increase the frame rate so if it is slow, change the function
    '''
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH,320) 
    video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

def look_at_video(video):
    '''
    - Inputs: 
            - video: a cv2 video object, can be thought of as 
              just the video captured
    - Outputs:
            - frame: a single frame of our video
    - Effects:
            - retrieves a frame of video for us to process
              also returns an error if video capture has failed
    '''
    if(~video.isOpened()):
        video.open()
    ret, frame = video.read()
    if(~ret):
        print("Video capture failed")
    return frame

def convert_to_HSV(frame):
  '''
    Inputs: 
        - frame: a raw frame returned by reading the video
    Ouputs: 
        - hsv: a hue, saturation, value color space, basically the
            frame with colors differentiated
    Effects: allows the computer to 'see' color
  '''
  return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

def detect_blue_edges(hsv):
    '''
    Inputs: 
        - hsv: a frame converted into a hue, saturation, value color space
    Outputs:
        - edges: the pixels of the frame that are deemed to be an edge
    Effects: gives us the edges of blue things in the frame
    '''
    lower_blue = np.array([90, 120, 0], dtype = "uint8") # lower limit of blue color
    upper_blue = np.array([150, 255, 255], dtype="uint8") # upper limit of blue color
    mask = cv2.inRange(hsv,lower_blue,upper_blue) # this mask will filter out everything but blue

    # detect edges
    edges = cv2.Canny(mask, 50, 100) 
    cv2.imshow("edges",edges)
    return edges

def detect_red_pix(hsv):
    '''
    Inputs: 
        - hsv: a frame converted into a hue, saturation, value color space
    Outputs:
        - a boolean indicating whether or not the camera detects 'bound' 
          number of red pixels
    Effects: gives us if we are close enough to a red stop sign
    '''
    bound = 30
    lower_red1 = np.array([  0, 40, 60], dtype="uint8")
    upper_red1 = np.array([ 10, 80, 100], dtype="uint8")
    lower_red2 = np.array([170, 40, 60], dtype="uint8")
    upper_red2 = np.array([179, 80, 100], dtype="uint8")
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Counts the number of 'red' pixels
    num_red_px = cv2.countNonZero(mask)
    if(num_red_px >= bound):
        return True
    else:
        return False

def GPIO_setup():
    """
    Effects: Sets up the GPIO pins specified at the top of the file
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SPEED_PIN, GPIO.OUT)
    GPIO.setup(STEERING_PIN, GPIO.OUT)

def calibrate_esc(pwm):
    """
    Input:
        - pwm: this should be our speed pwm
    Effects:
        - Calibrates the esc of the car
    """
    pwm.ChangeDutyCycle(10)
    time.sleep(2)
    pwm.ChangeDutyCycle(5)
    time.sleep(2)
    pwm.ChangeDutyCycle(7.5)
    time.sleep(2)

def set_servo_angle(angle, pwm):
    """
    Input: 
        - pwm: a pwm object created by the GPIO library, should be the servo pin's pwm
        - angle: the angle between 0 and 180 the servo should be at. Foward is 90 degrees
    Effects:
        - sets the servo to the requested angle
    """
    # Convert angle (0 to 180 degrees) to duty cycle (2.5% to 12.5%)
    duty_cycle = 2.5 + (angle / 18)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1)  # Allow time for the servo to reach the position

def set_speed(pwm, speed_percent):
    """
    Input: 
        - pwm: a pwm object created by the GPIO library should be the speed pin's pwm
        - speed_percent: an number between 0-100 representing the percent of max speed we go
    to be calibrated after seeing the car move
    """
    cycle = (speed_percent/10) +7.5
    pwm.ChangeDutyCycle(cycle)


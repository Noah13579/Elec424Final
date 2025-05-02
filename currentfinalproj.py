import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import math
import os
'''
The following function is inspired by:
    User raja_961, "Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV". 
    Instructables. URL: https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/
'''

os.environ["QT_QPA_PLATFORM"] = "offscreen"
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
    video = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    video.set(cv2.CAP_PROP_FRAME_WIDTH,320) 
    video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
    return video

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
    ret, frame = video.read()
    if(ret):
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

def region_of_interest(edges):
    height, width = edges.shape # extract the height and width of the edges frame
    mask = np.zeros_like(edges) # make an empty matrix with same dimensions of the edges frame

    # only focus lower half of the screen
    # specify the coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height), 
        (0,  height/2),
        (width , height/2),
        (width , height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255) # fill the polygon with blue color 
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10 
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=0)
    return line_segments

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0: 
        slope = 0.1    

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

def average_slope_intercept(frame, line_segments):
    lane_lines = []

    if line_segments is None:
        print("no line segment detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/3

    left_region_boundary = width * (1 - boundary) 
    right_region_boundary = width * boundary 

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity)")
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    # lane_lines is a 2-D array consisting the coordinates of the right and left lane lines
    # for example: lane_lines = [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    # where the left array is for left lane and the right array is for right lane 
    # all coordinate points are in pixels
    return lane_lines

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6): # line color (B,G,R)
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)  
    return line_image

def get_steering_angle(frame, lane_lines):
     height, width, _ = frame.shape

     if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0] # extract right x2 from lane_lines array
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)  

     elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

     elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)

     angle_to_mid_radian = math.atan(x_offset / y_offset)
     angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
     steering_angle = angle_to_mid_deg + 90 

     return steering_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):

    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

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
    cycle = (speed_percent/75) +7.5
    pwm.ChangeDutyCycle(cycle)

#Pin definitions
SPEED_PIN = 20
STEERING_PIN = 21

#PWM freq
freq = 50

lastTime = 0 
lastError = 0

speed = 10

# PD constants
kp = 0.4
kd = kp * 0.65

video = start_video()

GPIO_setup()

pwm_speed = GPIO.PWM(SPEED_PIN, freq)
pwm_steer = GPIO.PWM(STEERING_PIN, freq)

pwm_speed.start(7.5)
pwm_steer.start(7.5)
calibrate_esc(pwm_speed)

#Initial Start on seeing red box
frame = look_at_video(video)
hsv = convert_to_HSV(frame)
#while(not detect_red_pix(hsv)):
 #   pass
#Saves the time we started at
start_time = time.time()

while True:
    frame = look_at_video(video)
    hsv = convert_to_HSV(frame)
    edges = detect_blue_edges(hsv)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame,line_segments)
    lane_lines_image = display_lines(frame,lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    heading_image = display_heading_line(lane_lines_image,steering_angle)

    key = cv2.waitKey(1)
    if key == 27:
        break

    now = time.time() # current time variable
    dt = now - lastTime
    deviation = steering_angle - 45 # equivalent to angle_to_mid_deg variable
    error = abs(deviation) 

    if deviation < 5 and deviation > -5: # do not steer if there is a 10-degree error range
        deviation = 0
        error = 0
        set_servo_angle(pwm=pwm_steer, angle=90)

    elif deviation > 5: # steer right if the deviation is positive
        set_servo_angle(pwm=pwm_steer, angle=70)

    elif deviation < -5: # steer left if deviation is negative
        set_servo_angle(pwm=pwm_steer, angle=110)

    derivative = kd * (error - lastError) / dt 
    proportional = kp * error
    PD = int(speed + derivative + proportional)

    spd = abs(PD)
    if spd > 25:
       spd = 25

    set_speed(pwm=pwm_speed, speed_percent=spd)

    lastError = error
    lastTime = time.time()
    #Stops the car if we see the second stop sign
    if ((detect_red_pix(hsv)) and (time.time() - start_time > 20)):
        break
video.release()
cv2.destroyAllWindows()

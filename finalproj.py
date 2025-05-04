import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import math
import os
import matplotlib.pyplot as plt


'''
The following function is inspired by:
User raja_961, "Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV".
Instructables. URL: https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/
'''
# Environment variable, used to remove errors with displaying
os.environ["QT_QPA_PLATFORM"] = "offscreen"


#Pin definitions
SPEED_PIN = 20
STEERING_PIN = 21


# Picture Sizing
frame_width = 160
frame_length = 88


#PWM freq
freq = 50


# PD constants
kp = 0.09
kd = kp * 0.5


# Speeds for motor control in nanosec
max_speed = 750000000
min_speed = 900000000


# Servo and Motor Functions


def GPIO_setup():
'''
Effects: Sets up the GPIO pins specified at the top of the file
'''
GPIO.setmode(GPIO.BCM)
GPIO.setup(SPEED_PIN, GPIO.OUT)
GPIO.setup(STEERING_PIN, GPIO.OUT)


def calibrate_esc(pwm):
'''
Input:
    - pwm: this should be our speed pwm
Effects:
    - Calibrates the esc of the car
'''
pwm.ChangeDutyCycle(10)
time.sleep(2)
pwm.ChangeDutyCycle(5)
time.sleep(2)
pwm.ChangeDutyCycle(7.5)
time.sleep(2)


def set_servo_angle(angle, pwm):
'''
Input:
    - pwm: a pwm object created by the GPIO library, should be the servo pin's pwm
    - angle: the angle between 0 and 180 the servo should be at. Foward is 90 degrees
Effects:
    - sets the servo to the requested angle
'''
# Convert angle (0 to 180 degrees) to duty cycle (2.5% to 12.5%)
duty_cycle = 2.5 + (angle / 18)
pwm.ChangeDutyCycle(duty_cycle)
time.sleep(1)  # Allow time for the servo to reach the position


def set_speed(pwm, speed_percent):
'''
Input:
    - pwm: a pwm object created by the GPIO library should be the speed pin's pwm
    - speed_percent: an number between 0-100 representing the percent of max speed we go
to be calibrated after seeing the car move
'''
cycle = (speed_percent/75) + 7.5
pwm.ChangeDutyCycle(cycle)


# Computer Vision Functions


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
video.set(cv2.CAP_PROP_FRAME_WIDTH,frame_width)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,frame_length)
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
lower_blue = np.array([90, 50, 0], dtype = "uint8") # lower limit of blue color
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
bound_perc = 25
bound = (frame_length*frame_width)*(bound_perc/100)
lower_red1 = np.array([  0, 50, 60], dtype="uint8")
upper_red1 = np.array([ 10, 255, 255], dtype="uint8")
lower_red2 = np.array([170, 50, 60], dtype="uint8")
upper_red2 = np.array([180, 255, 255], dtype="uint8")
mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask = cv2.bitwise_or(mask1, mask2)


# Counts the number of 'red' pixels
num_red_px = cv2.countNonZero(mask)
if(num_red_px >= bound):
    print("Red seen!")
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
            # print("skipping vertical lines (slope = infinity)")
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
# print(slope)
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


def get_speed_change():
interval_str = open(interval_loc, "r")
interval = int(interval_str.readline())
interval_str.close()


speed_change = 0
if (interval <= max_speed):
    speed_change = -0.001
elif (interval >= min_speed):
    speed_change = 0.001
return speed_change


# Path to location of parameter from driver
interval_loc = "/sys/module/gpiod_driver/parameters/nl"


# Used to get times for PD calculation
lastTime = 0
lastError = 0
current = 7.8


# Used to keep track of what frame we are on
ctr = 0


# Used to keep track of which stop sign we are at
seen_stop = 0
start_time = 0


# Lists to keep track of values on a run for plotting
errors = []
derivatives = []
proportions = []
frames = []
turns = []
speeds = []


# Gets the GPIO pins ready
GPIO_setup()
pwm_speed = GPIO.PWM(SPEED_PIN, freq)
pwm_steer = GPIO.PWM(STEERING_PIN, freq)
pwm_speed.start(7.5)
pwm_steer.start(7.5)
calibrate_esc(pwm_speed)


# Starts the Camera
video = start_video()


try:
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

    cv2.imwrite("hsv.jpg", hsv)
    cv2.imwrite("edges.jpg", edges)
    cv2.imwrite("roi.jpg", roi)
    cv2.imwrite("heading_image.jpg", heading_image)
    print("images saved!")


    key = cv2.waitKey(1)
    if key == 27:
        break
    
    now = time.time() # current time variable
    dt = now - lastTime
    deviation = steering_angle - 90


    error = deviation
    base_turn = 0
    proportional = kp * error
    derivative = kd * (error - lastError)/dt


    turn = base_turn + proportional + derivative + 7.5
    if (turn < 0):
        turn = 0
    print("turn: ", turn)
    print("derivative: ", derivative)
    errors.append(error)
    derivatives.append(derivative)
    proportions.append(proportional)
    turns.append(turn)
    speeds.append(current)
    frames.append(ctr)
    ctr+=1

    pwm_steer.ChangeDutyCycle(turn)


    if ((ctr % 10) == 0):
        if(detect_red_pix(hsv) and ((time.time() - start_time) > 10)):
        if(seen_stop):
            break
        else:
            pwm_speed.ChangeDutyCycle(7.5)
            time.sleep(3)
            seen_stop = 1
            start_time = time.time()


    if ((ctr % 2) == 0):
        new_speed = current + get_speed_change()
        if (new_speed < 7.72):
            new_speed = 7.72
        pwm_speed.ChangeDutyCycle(new_speed)
        current = new_speed
        time.sleep(0.025)


finally:
    video.release()
    cv2.destroyAllWindows()
    # First figure
    # plt.figure()
    # plt.plot(frames, errors, label="errors")
    # plt.plot(frames, derivatives, label="derivatives")
    # plt.plot(frames, proportions, label="proportions")
    # plt.legend()
    # plt.savefig("PD_vals.png")
    # Plot the proportional, derivative and error
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(proportions))
    ax1.plot(t_ax, proportions, label="P values")
    ax1.plot(t_ax, derivatives, label="D values")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, errors, label="Error", color='red')

    ax1.set_xlabel("Frame Number")
    ax1.set_ylabel("PD Value")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    plt.title("P and D Values with Error")
    fig.legend()
    # fig.tight_layout()
    plt.savefig("PD_vals.png")

    # Second figure
    # plt.figure()
    # plt.plot(frames, errors, label="errors")
    # plt.plot(frames, speeds, label="speeds")
    # plt.plot(frames, turns, label="turns")
    # plt.legend()
    # plt.savefig("Movement_vals.png")
    # Plot the speed steering and the error
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(speeds))
    ax1.plot(t_ax, speeds, label="Speed")
    ax1.plot(t_ax, turns, label="Turns")
    ax2 = ax1.twinx()
    
    ax2.plot(t_ax, errors, label="Error", color='red')

    ax1.set_xlabel("Frame Number")
    ax1.set_ylabel("Speed and Steering Duty Cycle")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    plt.title("Speed and Steering Duty Cycle with Error")
    fig.legend()
    plt.savefig("Movement_vals.png")


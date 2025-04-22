import RPi.GPIO as GPIO
import time

SPEED_PIN = 18
STEERING_PIN = 19

freq = 50

def GPIO_setup():
    """
    Effects: Sets up the GPIO pins specified at the top of the file
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SPEED_PIN, GPIO.OUT)
    GPIO.setup(STEERING_PIN, GPIO.OUT)

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
    pwm.ChangeDutyCycle(speed_percent)

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

try:
    GPIO_setup()

    # Creates our pwm objects
    pwm_speed = GPIO.PWM(SPEED_PIN, freq)
    pwm_steer = GPIO.PWM(STEERING_PIN, freq)
    
    pwm_speed.start(7.5)
    pwm_steer.start(7.5)
    
    calibrate_esc(pwm_speed)

    #Performs the sequence of events described in the course
    set_speed(pwm_speed, 8)
    time.sleep(2)
    set_speed(pwm_speed, 7.5)
    time.sleep(2)
    set_servo_angle(pwm_steer, 45)
    time.sleep(2)
    set_servo_angle(pwm_steer, 135)
    time.sleep(2)
    set_servo_angle(pwm_steer, 90)
    time.sleep(2)
finally:
    pwm_speed.stop()
    pwm_steer.stop()
    GPIO.cleanup()

import RPi.GPIO as GPIO
import time
from smbus import SMBus

dc = 1
steer_pwm_pin = 33
steer_dir_pin = 35
left = GPIO.LOW
right = GPIO.HIGH
currentAngle = 0
def setup():
    global pwm
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(steer_pwm_pin, GPIO.OUT, initial=GPIO.LOW)

    pwm=GPIO.PWM(steer_pwm_pin,10000)
    pwm.start(0)

    GPIO.setup(steer_dir_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(steer_dir_pin, GPIO.LOW)
setup()

def rotate(direction,turns):
    global dc
    global currentAngle
    if direction == "left":
        for _ in range(turns):
            GPIO.output(steer_dir_pin, left)
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.08)
        print("turned left : {turns} times")
    else:
        for _ in range(turns):
            GPIO.output(steer_dir_pin, right)
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.1)
        print("turned left : {turns} times")
    stop_steering()
def steer_straight(turns = 0):
    global currentAngle
    if turns != 0 :
        currentAngle = turns
    if currentAngle > 0:
        rotate("left",currentAngle)
        currentAngle = 0
    else:
        rotate("right",currentAngle)
        currentAngle = 0
    print("steering Straight")
def stop_steering():
    print("stop steering")
    pwm.ChangeDutyCycle(0)
def set_speed(speed):
    bus.write_byte_data(address,0x40,speed)
def read_speed():
    return bus.read_byte(address)
bus = SMBus(1)
address = 0x48
speed = 100
rotate("left")
#set_speed(speed)
#print(read_speed())
#time.sleep(2)
#set_speed(0)
GPIO.cleanup()
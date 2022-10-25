import RPi.GPIO as GPIO
import time
from smbus import SMBus
import sys
from dronekit import connect
steer_pwm_pin = 33
steer_dir_pin = 35
left = GPIO.LOW
right = GPIO.HIGH
currentAngle = 0
address = 0x48
def setup():
    global pwm
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(steer_pwm_pin, GPIO.OUT, initial=GPIO.LOW)

    pwm=GPIO.PWM(steer_pwm_pin,10000)
    pwm.start(0)

    GPIO.setup(steer_dir_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(steer_dir_pin, GPIO.LOW)
setup()
connection_string = "/dev/ttyACM0" 
sys.stdout.write("Connecting Pixhawk\r")
vehicle = connect(connection_string, wait_ready=True)
sys.stdout.write("Connected Pixhawk")
def rotate(direction,turns,dc=1):
    global currentAngle
    currentAngle = currentAngle+turns if direction=="r" else currentAngle-turns
    time.sleep(0.2)
    if direction == "l":
        for _ in range(turns):
            #time.sleep(0.5)
            GPIO.output(steer_dir_pin, left)
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.005)
            pwm.ChangeDutyCycle(0)
        print(f"turned left : {turns} times")
    else:
        for _ in range(turns):
            #time.sleep(0.7)
            GPIO.output(steer_dir_pin, right)
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.05)
            pwm.ChangeDutyCycle(0)
        print(f"turned right : {turns} times")
def steer_straight(turns = 0):
    global currentAngle
    if turns != 0 :
        currentAngle = turns
    if currentAngle > 0:
        rotate("l",currentAngle)
        currentAngle = 0
    else:
        rotate("r",currentAngle)
        currentAngle = 0
    print("steering Straight")
def set_speed(speed):
    global address
    bus.write_byte_data(address,0x40,speed)
def read_speed():
    global address
    return bus.read_byte(address)

def heading_nav(heading,dest):
    diff = dest - heading
    print(diff)
    if diff <= 2 and diff >= -2:
        sys.stdout.write("\rHeading Correctly\n")
    elif diff < 0:
        rotate("l",1,abs(diff)/(dest*2))
    else:
        rotate("r",1,diff/(dest*2))
bus = SMBus(1)
speed = 100
set_speed(speed)
while True:
    try:
        heading = vehicle.heading
        print(heading)
        heading_nav(heading,190)
    except:
        set_speed(0)
#rotate("l",3)
#set_speed(speed)
#rotate("l",3)
#time.sleep(2)
#rotate("r",5)
#rotate("l",5)
#print(currentAngle)
#set_speed(0)
GPIO.cleanup()
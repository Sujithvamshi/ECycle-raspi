#!/usr/bin/env python
# -*- coding: utf-8 -*-

# test2_throttle_out_and_angle_in_PCF8591_steer_out_pwm_md30c_brake_relay.py


from __future__ import print_function

import argparse
import io
import logging
import math
import socketserver
import sys
import time
from http import server
from threading import Condition

import picamera
#import Adafruit_MCP4725
import RPi.GPIO as GPIO
#import Jetson.GPIO as GPIO
import smbus
from dronekit import (Command, LocationGlobal, LocationGlobalRelative,
                      VehicleMode, connect)
from pymavlink import mavutil

from direction_functions import *
from functions import *

#from picamera.array import PiRGBArray
#from picamera import PiCamera
#import cv2
#import numpy as np
#import math

# https://thedatafrog.com/en/articles/show-data-google-map-python/

#lat, lon = 17.602627, 78.1266067
#p = plot(lat, lon)
1
global bus
bus = smbus.SMBus(1) # for PCF8591

global address
address = 0x48

global steer_rate
steer_rate=5

steer_pwm_pin = 33
steer_dir_pin = 35

global dc
dc= 1
dc1=1
2
speed =  90

steering_center = 110
steering_limits = 3
center_threshold = 2

global previous_steering
previous_steering=0 

# 0 for left
# 1 for right

left = GPIO.LOW
right = GPIO.HIGH

GPIO.cleanup()


def fun_bearing_diff():

    global currentLocation, targetLocation, dc

    Current_Bearing =  vehicle.heading
    # # while True:
      #   print("Current Bearing: ", vehicle.heading)
        # time.sleep(1)
    
    Required_Bearing = get_bearing(currentLocation, targetLocation)
    bearing_diff = Current_Bearing - Required_Bearing
    print("Current Bearing: ", Current_Bearing)
    print("Required Bearing: ", Required_Bearing)
            
    print("                     bearing_diff: ", bearing_diff)

    if bearing_diff < -180:
        bearing_diff = (bearing_diff+360)
        print("             changed bearing_diff: ", bearing_diff)

    if bearing_diff > 180:
        bearing_diff = bearing_diff-360
        print("             changed bearing_diff: ", bearing_diff)
    dc= abs(bearing_diff/2)
    if abs(bearing_diff)>40:
        dc=20
    return(bearing_diff)





def setup():
    global pwm
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(steer_pwm_pin, GPIO.OUT, initial=GPIO.LOW)

    pwm=GPIO.PWM(steer_pwm_pin,10000)
    pwm.start(0) # Set the starting Duty Cycle

    GPIO.setup(steer_dir_pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(steer_dir_pin, GPIO.LOW)
    #write1(0)
    
#    pwm = GPIO.PWM(steer_pwm_pin, 10000) # Set Frequency to 1 KHz
#    pwm.start(0) # Set the starting Duty Cycle





def rotate_left():
    global dc
    if read_angle() > (steering_center - steering_limits):
        print("turn left")
        GPIO.output(steer_dir_pin, left)
        #dc=10
        pwm.ChangeDutyCycle(dc)
        print(dc)
        time.sleep(0.05)
        stop_steering()
	   # pwm.ChangeDutyCycle(0)
	#    time.sleep(1)
    if read_angle() < (steering_center - steering_limits):
        print("dont turn left")
        GPIO.output(steer_dir_pin, left)
	    #dc=10
        pwm.ChangeDutyCycle(0)
	    #print(dc)
	   # time.sleep(0.1)
	   # pwm.ChangeDutyCycle(0)
	#    time.sleep(1)

def rotate_right():
    global dc
    if read_angle() < (steering_center+steering_limits):
        print("turn right")
        GPIO.output(steer_dir_pin, right)
        print(dc)
	    #write1(0)
	   # dc=10
        pwm.ChangeDutyCycle(dc)
        time.sleep(0.05)
        stop_steering()
	#    pwm.ChangeDutyCycle(0)
	#    time.sleep(1)
    if read_angle() > (steering_center + steering_limits):
        print("dont turn right")
        GPIO.output(steer_dir_pin, left)
	    #dc=10
        pwm.ChangeDutyCycle(0)
	    #print(dc)
	   # time.sleep(0.1)
	   # pwm.ChangeDutyCycle(0)
	#    time.sleep(1)



def stop_steering():
    
    print("stop steering")
    pwm.ChangeDutyCycle(0)



def return_to_zero():
    while (abs(read_angle() - steering_center) > center_threshold):
    #while (True):    
        print("in return_to_zero function")
        print(read_angle())
        if (read_angle() > steering_center):
            #dc=10
            rotate_left()   
            time.sleep(0.51)
        if (read_angle() < steering_center):
            #dc=10
            rotate_right()   
            time.sleep(0.51)
            
        
    stop_steering()
    dc=dc1




print("Started:")
#print(Jetson.GPIO.VERSION)
setup()



#write(0)
#return_to_zero()

while 0:
    read_angle()
    read_act()
    time.sleep(1)
    

write(0)
return_to_zero()



while 0:
    read_angle()
    read_act()
    time.sleep(1)
    
# for streaming
# https://randomnerdtutorials.com/video-streaming-with-raspberry-pi-camera/


#while True:
#    print(read(0))
#    time.sleep(1)


read_angle()
time.sleep(1)
read_angle()
time.sleep(1)
read_angle()
time.sleep(1)



#wp = get_wp(LocationGlobal(17.00, 78.00, 0))
#wp = get_wp(LocationGlobal(17.580261, 78.121134, 0))
     
    
    
connection_string = "/dev/ttyACM0"
# Connect to the Vehicle. 
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print("\nConnecting to vehicle on: %s" % connection_string)

vehicle = connect(connection_string, wait_ready=True)

vehicle.wait_ready('autopilot_version')

# Get all vehicle attributes (state)
print("\nGet vehicle attribute values:")
print(" Autopilot Firmware version: %s" % vehicle.version)
print(" Autopilot capabilities")
print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
print(" Global Location: %s" % vehicle.location.global_frame)
print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print(" Local Location: %s" % vehicle.location.local_frame)
#print(" Attitude: %s" % vehicle.attitude)
#print(" Velocity: %s" % vehicle.velocity)
print(" GPS: %s" % vehicle.gps_0)
print(" EKF OK?: %s" % vehicle.ekf_ok)
print(" Heading: %s" % vehicle.heading)
#print(" Is Armable?: %s" % vehicle.is_armable)
#print(" System status: %s" % vehicle.system_status.state)
print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
#print(" Airspeed: %s" % vehicle.airspeed)    # settable
#print(" Mode: %s" % vehicle.mode.name)    # settable
#print(" Armed: %s" % vehicle.armed)    # settable`print(" Heading: %s" % vehicle.heading)
print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
time.sleep(5)








def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    
    global previous_steering
    global currentLocation, targetLocation

    currentLocation = vehicle.location.global_relative_frame
    print("Current Location: ", currentLocation)

    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    
    targetDistance = get_distance_metres(currentLocation, targetLocation)

    gotoFunction(targetLocation)

    print("Target Location: ", targetLocation)
    print("targetDistance : ", targetDistance)
    Required_Bearing = get_bearing(currentLocation, targetLocation)
    print("Required Bearing: ", get_bearing(currentLocation, targetLocation))
    remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
    print("Remaining Distance: ", remainingDistance)

    if remainingDistance>5:     
        print("Setting Throttle Analog Voltage:")
        x=speed
        write(x)
        print(x)
#        return_to_zero()

    while remainingDistance>3: #Stop action if we are no longer in guided mode.

        time.sleep(0.5)
        #print "DEBUG: mode: %s" % vehicle.mode.name
        currentLocation = vehicle.location.global_relative_frame
        print("Current Location: ", currentLocation)

        remainingDistance=get_distance_metres(currentLocation, targetLocation)
        print("Distance to target: ", remainingDistance)

        if (fun_bearing_diff())>(5): 
            rotate_left()

        if (fun_bearing_diff())<(-5): 
            rotate_right()

        if abs(fun_bearing_diff())<(5):
            return_to_zero()

        remainingDistance=get_distance_metres(currentLocation, targetLocation)
        print("Distance to target: ", remainingDistance)




def goto_gps(t, gotoFunction=vehicle.simple_goto):
    
    global previous_steering
    global currentLocation, targetLocation

    currentLocation = vehicle.location.global_relative_frame
    print("Current Location: ", currentLocation)

    #targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetLocation = t #get_location_metres(currentLocation, dNorth, dEast)
    
    targetDistance = get_distance_metres(currentLocation, targetLocation)

    gotoFunction(targetLocation)

    print("Target Location: ", targetLocation)
    print("targetDistance : ", targetDistance)
    Required_Bearing = get_bearing(currentLocation, targetLocation)
    print("Required Bearing: ", get_bearing(currentLocation, targetLocation))
    remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
    print("Remaining Distance: ", remainingDistance)

    if remainingDistance>5:     
        print("Setting Throttle Analog Voltage:")
        x=speed
        write(x)
        print(x)
#        return_to_zero()

    while remainingDistance>5: #Stop action if we are no longer in guided mode.

        time.sleep(0.5)
        #print "DEBUG: mode: %s" % vehicle.mode.name
        currentLocation = vehicle.location.global_relative_frame
        print("Current Location: ", currentLocation)

        remainingDistance=get_distance_metres(currentLocation, targetLocation)
        print("Distance to target: ", remainingDistance)

        if (fun_bearing_diff())>(5): 
            rotate_left()

        if (fun_bearing_diff())<(-5): 
            rotate_right()

        if abs(fun_bearing_diff())<(5):
            return_to_zero()

        remainingDistance=get_distance_metres(currentLocation, targetLocation)
        print("Distance to target: ", remainingDistance)
        #print(" Heading: %s" % vehicle.heading)
        print(" Groundspeed: %s" % vehicle.groundspeed)    # settable







#Confirm current value on vehicle by re-downloading commands
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()



time.sleep(5)





wp = [] #final path
wp = [[17.60189183896,78.12661039844],
[17.60190534185,78.12661037303],
[17.60191858316,78.12661031466],
[17.60193304980,78.12661031913],
[17.60195400764,78.12661035663],
[17.60195951012,78.12661062776],
[17.60196705220,78.12661184892],
[17.60198261145,78.12661888555],
[17.60199567808,78.12663161689],
[17.60199971221,78.12663903324],
[17.60200262315,78.12664739598],
[17.60200424655,78.12665588672],
[17.60200510178,78.12666517910],
[17.60200547007,78.12667141658],
[17.60200618544,78.12668730843],
[17.60200692834,78.12670482203],
[17.60200731990,78.12671727537],
[17.60200776881,78.12673457320],
[17.60200787978,78.12673956917],
[17.60200797845,78.12674701397],
[17.60200799245,78.12675679234],
[17.60200802153,78.12676778099],
[17.60200805196,78.12677393921],
[17.60200803758,78.12677881729],
[17.60200805472,78.12678788844],
[17.60200807982,78.12679555737],
[17.60200798917,78.12680488709],
[17.60200786975,78.12681714509],
[17.60200779445,78.12682440672],
[17.60200774930,78.12682731575],
[17.60200775063,78.12682727843],
[17.60200772928,78.12682731624],
[17.60200774329,78.12682728591],
[17.60200776543,78.12682721016],
[17.60200776473,78.12682722798],
[17.60200777280,78.12682723205],
[17.60200776647,78.12682722422],
[17.60200778065,78.12682723828],
[17.60200777450,78.12682724989],
[17.60200777569,78.12682723920],
[17.60200776623,78.12682724572],
[17.60200777087,78.12682724604],
[17.60200777507,78.12682724562],
[17.60200777221,78.12682724599],
[17.60200777468,78.12682724445],
[17.60200777626,78.12682724416],
[17.60200778006,78.12682724480],
[17.60200777950,78.12682724658],
[17.60200777597,78.12682725155],
[17.60200779089,78.12682725832],
[17.60200778801,78.12682727901],
[17.60200780094,78.12682727179],
[17.60200779816,78.12682728202],
[17.60200779112,78.12682728692],
[17.60200778579,78.12682729078],
[17.60200778504,78.12682728269],
[17.60200779333,78.12682728990],
[17.60200778944,78.12682728846],
[17.60200779233,78.12682733373]]

print(len(wp))

time.sleep(1)


#while True:
#    time.sleep(1)
    
for i in range(len(wp)):
    print("Step:", i+1, " out of ", len(wp)+1)
    goto_gps(LocationGlobal(wp[i][0], wp[i][1], 0))

print("Setting Throttle Analog Voltage:")
x=0
write(x)




while True:
    print("Finished")
    time.sleep(1)
   
   
   
#goto(20,0)
#sc start
goto_gps(LocationGlobal(17.602627, 78.1266067, 0))
#sc mid
goto_gps(LocationGlobal(17.6027549, 78.1269163, 0))
#sc end
goto_gps(LocationGlobal(17.6026623, 78.1271547, 0))
#t2
goto_gps(LocationGlobal(17.6021478, 78.1271145, 0))



print("Setting Throttle Analog Voltage:")
x=0
write(x)
print(x)

#time.sleep(5)


pwm.stop()
GPIO.output(steer_pwm_pin, GPIO.LOW)
GPIO.output(steer_dir_pin, GPIO.LOW)
GPIO.cleanup()

print("Finished")

#Close vehicle object before exiting script
print("\nClose vehicle object")
vehicle.close()
print("Completed")
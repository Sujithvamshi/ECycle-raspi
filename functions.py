from __future__ import print_function

from dronekit import connect, VehicleMode, Command, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time, sys, argparse, math

import smbus


# https://thedatafrog.com/en/articles/show-data-google-map-python/


#import pandas as pd
#from bokeh.io import output_notebook
#output_notebook()
bokeh_width, bokeh_height = 1000,500
import os

from bokeh.io import show
from bokeh.plotting import gmap
from bokeh.models import GMapOptions
maps_api_key = "AIzaSyA9-vAo4DNA9qhJhM230J8w74nG-igeFsk"

#lat, lon = 17.602627, 78.1266067
def plot(lat, lng, zoom=14, map_type='roadmap'):
    gmap_options = GMapOptions(lat=lat, lng=lng, 
                               map_type=map_type, zoom=zoom)
    p = gmap(maps_api_key, gmap_options, title='Title', 
             width=bokeh_width, height=bokeh_height)
    center = p.circle([lng+0.001], [lat+0.001], size=10, alpha=0.5, color='red')
    center = p.circle([lng+0.002], [lat+0.001], size=10, alpha=0.5, color='red')
    center = p.circle([lng+0.003], [lat+0.001], size=10, alpha=0.5, color='red')
    show(p)
    return p
     
def plot_wp(lat, lng, zoom=14, map_type='roadmap'):
    gmap_options = GMapOptions(lat=lat, lng=lng, 
                               map_type=map_type, zoom=zoom)
    p = gmap(maps_api_key, gmap_options, title='Title', 
             width=bokeh_width, height=bokeh_height)
    center = p.circle([lng+0.001], [lat+0.001], size=10, alpha=0.5, color='red')
    center = p.circle([lng+0.002], [lat+0.001], size=10, alpha=0.5, color='red')
    center = p.circle([lng+0.003], [lat+0.001], size=10, alpha=0.5, color='red')
    show(p)
    return p
     



global bus
bus = smbus.SMBus(1) # for PCF8591
global address
address = 0x48



def read(chn): #channel
    #try:
    if chn == 0:
        bus.write_byte(address,0x40)
    if chn == 1:
        bus.write_byte(address,0x41)
    if chn == 2:
        bus.write_byte(address,0x42)
    if chn == 3:
        bus.write_byte(address,0x43)
        bus.read_byte(address) # dummy read to start conversion
    #except Exception, e:
     #   print "Address: %s" % address
     #   print e
    return bus.read_byte(address)

def write(val):
#    try:
    temp = val # move string value to temp
    temp = int(temp) # change string to integer
# print temp to see on terminal else comment out

    bus.write_byte_data(address, 0x40, temp)


   # except Exception, e:
     #   print "Error: Device address: 0x%2X" % address
    #    print e

def write1(val):
#    try:
    temp = val # move string value to temp
    temp = int(temp) # change string to integer
# print temp to see on terminal else comment out

#    bus.write_byte_data(address, 0x40, temp)

   # except Exception, e:
     #   print "Error: Device address: 0x%2X" % address
    #    print e

def read_act():
    p2=read(2) #No of turns
    p2=read(2)
    print(p2)
    return(p2)


def read_angle(): 
    #p1=read(1) #steering angle
    #p1=read(1) 
    p3=read(3) #No of turns
    p3=read(3) 

    print("P3: %0.2f" % (p3))

    #print("P3: %0.2f" % (((p3*100 / 255)-0.0))
    total_a1 = p3
    print("Steering Sensor Angle: %0.2f" % total_a1)
    return total_a1




"""
Functions to make it easy to convert between the different frames-of-reference. In particular these
make it easy to navigate in terms of "metres from the current position" when using commands that take 
absolute positions in decimal degrees.

The methods are approximations only, and may be less accurate over longer distances, and when close 
to the Earth's poles.

Specifically, it provides:
* get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
* get_distance_metres - Get the distance between two LocationGlobal objects in metres
* get_bearing - Get the bearing in degrees to a LocationGlobal
"""

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude, `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.
    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;



"""
Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

The methods include:

* goto - A convenience function that can use Vehicle.simple_goto (default) or 
    goto_position_target_global_int to travel to a specific position in metres North and East from the current location. 
    This method reports distance to the destination.
"""


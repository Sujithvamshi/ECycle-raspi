from __future__ import print_function

import argparse
import json
import math
import pprint
import sys
import time
import urllib
import urllib.request

import RPi.GPIO as GPIO
import smbus
from dronekit import (Command, LocationGlobal, LocationGlobalRelative,
                      VehicleMode, connect)
from googlemaps import Client
from pymavlink import mavutil

bokeh_width, bokeh_height = 1000,500
import os

from bokeh.io import show
from bokeh.models import GMapOptions
from bokeh.plotting import gmap

maps_api_key = "AIzaSyA9-vAo4DNA9qhJhM230J8w74nG-igeFsk"

def get_destination():


    with urllib.request.urlopen("https://location-tracker-c1861-default-rtdb.firebaseio.com/.json") as url:
        s = url.read()
    strin = s.decode("utf-8")
    #print(strin)

    for i in range(len(strin)-10):
      if (strin[i] == 'l')&(strin[i+1]== 'a')&(strin[i+2]=='t')&(strin[i+3]=='i')&(strin[i+4]=='t')&(strin[i+5]=='u')&(strin[i+6]=='d')&(strin[i+7]=='e'):
        #print(strin[i+11:i+21])
        lat=(float(strin[i+11:i+21].replace(',','')))
      
      if (strin[i] == 'l')&(strin[i+1]== 'o')&(strin[i+2]=='n')&(strin[i+3]=='g')&(strin[i+4]=='i')&(strin[i+5]=='t')&(strin[i+6]=='u')&(strin[i+7]=='d'):
        #print(strin[i+12:i+22])
        lon=(float(strin[i+12:i+22].replace(',','')))
        break
       

    lat_destination=lat
    lon_destination=lon
    return LocationGlobal(lat, lon, 0)


    
#lat_origin =  17.595522
#lon_origin = 78.117670 
#lat_origin =  lat+0.0100000
#lon_origin =  lon+0.0100000


def get_wp(origin_location):

    print("Origin Lat Lon:")
    lat_origin=(origin_location.lat)
    lon_origin=(origin_location.lon)
    print(lat_origin)
    print(lon_origin)
    
    dest = get_destination()
    print("Destination Lat Lon:")
    #dest.lat = lat_origin+0.01
    #dest.lon = lon_origin+0.01
    #dest.lat = 17.59657086577856
    #dest.lon = 78.12331298614025
    print(dest.lat)
    print(dest.lon)
    #print(type(dest.lat))

    


    #origin="+str(lat_origin)+"+"+str(lon_origin)+"&destination="+str(lat)+"+"+str(lon)+"

    mapService = Client(key='AIzaSyDs6e2jZeLHec3D5JYXIUDSG5oEwRVda-0')

    #directions = mapService.directions('start', 'destination')
    #directions = mapService.directions("str(lat_origin)+"+"+str(lon_origin)", 'destination')
    #directions = mapService.directions("str(lat_origin)+"+"+str(lon_origin)", "str(lat_origin)+"+"+str(lon_origin)")
    #directions = mapService.directions(origin=[lat_origin, lon_origin, 0], destination=[lat_destination, lon_destination, 0],mode='driving')
    directions = mapService.directions(origin=[lat_origin, lon_origin, 0], destination=[dest.lat, dest.lon, 0],mode='driving')
    #directions = mapService.directions("str(lat_origin)+"+"+str(lon_origin)", "str(lat+1)+"+"+str(lon+1)")
    #print(directions)
    directions = directions[0]
    
    wp=[]
    wp1=[]
    #wp_lon=[]

    total_distance=0
    i=1
    for leg in directions['legs']:
        startAddress = leg['start_address']
        #print ("Start Address:", startAddress)
        endAddress = leg['end_address']
        #print ("End Address:", endAddress)
        for step in leg['steps']:
            print("Step:", i)
            #print("Start Location:",step['start_location'])
            #print("End Location:",step['end_location'])
            
            wp=[step['end_location']['lat'], step['end_location']['lng'], 0]
            #wp[i-1][1] = step['start_location']['lat']
            #wp[i-1][2] = step['start_location']['lon']
            wp1.append(wp)
            #wp[i-1].altitude = step['start_location']['lon']
            
            #print("Distance:",step['distance'])
            print("Distance:",step['distance']['value']," meters")
            total_distance = total_distance + step['distance']['value']
            
            html_instructions = step['html_instructions']
            print ("STEP {} {}".format(i ,html_instructions))
            #print(distance)
            i = i+1
            #print("")
    
           
    
    print("total_distance:", total_distance, "meters")
    
    
            
            
    return(wp1)        
    #print(step['start_location']['lat'])
    #print(type(step['start_location']['lat']))

    #print("total_distance:", total_distance, "meters")



#lat, lon = 17.602627, 78.1266067
#wp = get_wp(LocationGlobal(lat, lon, 0))
#print(len(wp))
#print(len(wp[0]))

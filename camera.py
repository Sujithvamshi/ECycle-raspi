import math
from time import sleep

import numpy as np
from dronekit import connect


def get_bearing(lat1,lon1,lat2,lon2):
    dLon = lon2 - lon1;
    y = math.sin(dLon) * math.cos(lat2);
    x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dLon);
    brng = np.rad2deg(math.atan2(y, x));
    if brng < 0: brng+= 360
    return brng

gps_port = "/dev/ttyACM0"
print(f"\nConnecting to GPS: {gps_port}")
vehicle = connect(gps_port, wait_ready=True)
pl,po = None,None
while True:
    lat,lon = vehicle.location.global_frame.lat,vehicle.location.global_frame.lon
    print(lat,lon)
    if pl == None:
        pl,po = lat,lon
    print()
    print(f'GPS Heading: {vehicle.heading}')
    print(f'Lat-Lon Heading: {get_bearing(pl,po,lat,lon)}')
    sleep(2)
    pl,po = lat,lon
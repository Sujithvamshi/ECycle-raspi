import requests
from dronekit import connect
import time 
gps_port = "/dev/ttyACM0"
print(f"\nConnecting to GPS: {gps_port}")
vehicle = connect(gps_port, wait_ready=True)
data1 = {
    "userId":"1",
    "cycle":[]
}
data2 = {
    "cycleId":"1"
}
update_url = 'https://ecycle.herokuapp.com/api/updateorder'
read_url = 'https://ecycle.herokuapp.com/api/readcycle'
while True:
    response = requests.post("https://ecycle.herokuapp.com/api/readcycle",data2).json()['order']
    if response['booked'] == "true":
        lat,lon = vehicle.location.global_frame.lat,vehicle.location.global_frame.lon
        print(lat,lon)
        data1['cycle'] = [lat,lon]
        response = requests.post(update_url,data1)
        if response['trackId'] == "1":
            exec(open('track1.py').read())
        if response["trackId"] == "2":
            exec(open('track2.py').read())
        if response["trackId"] == "3":
            exec(open('track3.py').read())
        if response["trackId"] == "4":
            exec(open('track4.py').read())
    else:
        print("idle")
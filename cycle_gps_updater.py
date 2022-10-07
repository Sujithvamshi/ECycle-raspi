from dronekit import connect
import time 
import requests
gps_port = "/dev/ttyACM0"
print(f"\nConnecting to GPS: {gps_port}")
vehicle = connect(gps_port, wait_ready=True)
data = {
    "userId":"1",
    "cycle":[]
}
update_url = 'https://ecycle.herokuapp.com/api/updateorder'
while True:
    lat,lon = vehicle.location.global_frame.lat,vehicle.location.global_frame.lon
    print(lat,lon)
    data['cycle'] = [lat,lon]
    response = requests.post(update_url,data)
    time.sleep(2)
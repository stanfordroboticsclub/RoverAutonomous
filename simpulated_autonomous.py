import time
from UDPComms import Publisher

# Create a UDP Publisher and set it to the correct data types
fields = "time sats lat lon alt error_lat error_lon error_alt"
format_ = "ii3f3f"
port = 8860
fields_out = "lat lon"
typ_out = "2f"
port_out = 8890
pub = Publisher(fields, format_, port)
pub_click = Publisher(fields_out, typ_out, port_out)
fields = "angle"
format_ = "f"
port = 8870
pub_angle = Publisher(fields, format_, port)

i = 0
point = (37.429, -122.170)

pt_click = (37.428, -122.171)

i = 0
angles = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45]

while True:
    lat,lon = point[0], point[1]
    lat_click, lon_click = pt_click
    angle = angles[i]
    print lat, lon
    print lat_click, lon_click
    print angle

    i = (i+1) % len(angles)
    pub.send(0, 4, lat, lon, 0, 0, 0, 0)
    pub_click.send(lat_click, lon_click)
    pub_angle.send(angle)

    time.sleep(3)

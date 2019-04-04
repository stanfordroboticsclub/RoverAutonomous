from UDPComms import Publisher, Subscriber, timeout
import math
import time


class Pursuit:

    port = 8830
    pub = Publisher(port, false)

    def __init__(self):

        self.imu = Subscriber(8220, timeout=1)
        self.gps = Subscriber(8280, timeout=1)
        self.auto_contorl = Subscriber(8310, timeout=1)

        self.cmd_vel = Subscriber(8830)
        time.sleep(3)

        self.start_point = {"lat":0, "lon":0}
        self.lookahead_radius = 50
        self.robot = None

        while True:
            self.update()
            time.sleep(0.1)

    def update(self):

        cmd = self.auto_control.get()
        if cmd['cmd'] == 'off':
            self.start_point['lat'] = cmd['lat']
            self.start_point['lon'] = cmd['lon']
        lookahead = self.find_lookahead(cmd)
        diff_angle = self.get_angle(lookahead)
        print(diff_angle)
        # self.send_velocities( 10* diff_angle)

    def find_lookahead(self,cmd):
        x_start, y_start = self.project(self.start_point['lat'], self.start_point['lon'])
        x_end, y_end = self.project(cmd['lat'], cmd['lon'])

        self.robot = self.gps.get()
        x_robot, y_robot = self.project(self.robot['lat'], self.robot['lon'])

        if (x_robot-x_end)**2 + (y_robot - y_end)**2 < self.lookahead_radius**2:
            return (x_end, y_end)

        a = (x_end - x_start)**2 + (y_end - y_start)**2
        b = 2*( (x_end-x_start)*(x_start-x_robot) + (y_end-y_start)*(y_start-y_robot))
        c = (x_start-x_robot)**2 + (y_start-y_robot)**2 - self.lookahead_radius**2

        if b**2 - 4*a*c <=0:
            t = -b/2*a
        else:
            t = (-b + math.sqrt(b**2 - 4*a*c))/2*a

        assert t<=1
        assert t>=0

        x_look = t * x_end + (1-t) * x_start
        y_look = t * y_end + (1-t) * y_start

        return (x_look, y_look)


    def get_angle(self, lookahead):
        heading_deg = self.imu.get()['angle'][0]
        head_rad = math.radians(heading_deg)

        x_robot, y_robot = self.project(self.robot['lat'], self.robot['lon'])
        x_look, y_look = lookahead

        target = math.atan2( x_look - x_robot, y_look - y_robot)

        return (target - head_rad)%2*math.pi



    def send_velocities(self, turn_rate):
        self.cmd_vel.send({"f": 100, "t": 100*turn_rate})

    def project(self, lat, lon):
        lat_orig = self.start_point['lat']
        lon_orig = self.start_point['lon']
        RADIUS = 6371 * 1000
        lon_per_deg = RADIUS * 2 * math.pi / 360
        lat_per_deg = lon_per_deg * math.cos(math.radians(lat_orig))

        x = (lon - lon_orig) * lon_per_deg
        y = (lat - lat_orig) * lat_per_deg
        return (x,y)



if __name__ == '__main__':
    a = Pursuit()

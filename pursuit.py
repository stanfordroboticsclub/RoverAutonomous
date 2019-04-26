#!/usr/bin/env python3


from UDPComms import Publisher, Subscriber, timeout
import math
import time


class Pursuit:

    def __init__(self):

        self.imu = Subscriber(8220, timeout=1)
        self.gps = Subscriber(8280, timeout=2)
        self.auto_control = Subscriber(8310, timeout=5)

        self.cmd_vel = Publisher(8830)
        time.sleep(3)

        self.start_point = {"lat":self.gps.get()[b'lat'] , "lon":self.gps.get()[b'lon']}
        self.lookahead_radius = 7
        self.robot = None

        while True:
            self.update()
            time.sleep(0.1)

    def update(self):

        self.robot = self.gps.get()
        cmd = self.auto_control.get()
        if cmd['command'] == 'off':
            print("off")
            self.start_point['lat'] = self.gps.get()[b'lat']
            self.start_point['lon'] = self.gps.get()[b'lon']
        elif( cmd['command'] == 'auto'):
            waypoints = [self.start_point] + cmd['waypoints']
            i = 0
            for p1, p2 in zip(waypoints[::-1][1:], waypoints[::-1][:-1]):
                lookahead = self.find_lookahead(p1,p2)
                if lookahead is not None:
                    print("point intersection", i)
                    break
                i+=1
            else:
                print("NO intersection found!")
                return

            diff_angle = self.get_angle(lookahead)
            self.send_velocities(diff_angle)
        else:
            raise ValueError

    def find_lookahead(self,p1, p2):
        x_start, y_start = self.project(p1['lat'], p1['lon'])
        x_end, y_end = self.project(p2['lat'], p2['lon'])

        self.robot = self.gps.get()
        x_robot, y_robot = self.project(self.robot[b'lat'], self.robot[b'lon'])

        if (x_robot-x_end)**2 + (y_robot - y_end)**2 < self.lookahead_radius**2:
            return (x_end, y_end)

        # print('start', x_start,y_start)
        # print('end', x_end,y_end)
        # print('robot', x_robot,y_robot)

        a = (x_end - x_start)**2 + (y_end - y_start)**2
        b = 2*( (x_end-x_start)*(x_start-x_robot) + (y_end-y_start)*(y_start-y_robot))
        c = (x_start-x_robot)**2 + (y_start-y_robot)**2 - self.lookahead_radius**2

        # print('a', a)
        # print('b', b)
        # print('c', c)

        if b**2 - 4*a*c <=0:
            return None
            print("too far")
            t = -b/(2*a)
        else:
            t = (-b + math.sqrt(b**2 - 4*a*c))/(2*a)

        # print('t',t)
        if not t<=1:
            return None
        if not t>=0:
            return None

        x_look = t * x_end + (1-t) * x_start
        y_look = t * y_end + (1-t) * y_start

        return (x_look, y_look)


    def get_angle(self, lookahead):
        heading_deg = self.imu.get()['angle'][0]
        head_rad = math.radians(heading_deg)

        x_robot, y_robot = self.project(self.robot[b'lat'], self.robot[b'lon'])
        x_look, y_look = lookahead

        target = math.atan2( x_look - x_robot, y_look - y_robot)

        # anti clockwise postive
        return ((target - head_rad + math.pi)%(2*math.pi) - math.pi)



    def send_velocities(self, angle):
        turn_rate = -150*math.tanh(1.5*angle)

        if math.fabs(angle) < math.radians(10):
            forward_rate = 130
        else if math.fabs(angle) < math.radians(60):
            forward_rate = 80
        else if math.fabs(angle) < math.radians(140):
            forward_rate = 30
        else:
            forward_rate = 0

        out = {"f": forward_rate, "t": turn_rate }
        print(out)
        self.cmd_vel.send(out)

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

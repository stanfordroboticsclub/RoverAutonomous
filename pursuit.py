#!/usr/bin/env python3


from UDPComms import Publisher, Subscriber, timeout
import math
import time
import random


class Pursuit:

    def __init__(self):

        self.imu = Subscriber(8220, timeout=1)
        self.gps = Subscriber(8280, timeout=2)
        self.auto_control = Subscriber(8310, timeout=5)

        self.cmd_vel = Publisher(8830)
        self.lights = Publisher(8590)
        self.tennis = Subscriber(9999, timeout=2)
        time.sleep(3)

        self.start_point = {"lat":self.gps.get()['lat'] , "lon":self.gps.get()['lon']}
        self.lookahead_radius = 6
        self.final_radius = 1.5    # how close should we need to get to last endpoint
        self.search_radius = 20    # how far should we look from the given endpoint
        self.reached_destination = False # switch modes into tennis ball seach
        self.robot = None
        self.guess = None # where are we driving towards
        self.guess_radius = 3 # if we are within this distance we move onto the next guess

        self.past_locations = []
        self.stuck_time = 0

        while True:
            self.update()
            time.sleep(0.1)

    def get_guess(self,endpoint):
        x, y = endpoint
        rand_x = (2*random.random()-1)*self.search_radius
        rand_y = (2*random.random()-1)*self.search_radius
        self.random_corrd = (rand_x, rand_y)
        print(rand_x, rand_y)
        return (x + rand_x, y + rand_y)



    def find_ball(self, cmd):
        if cmd['end_mode'] == 'none':
            print("REACHED TENNIS BALL")
            self.lights.send({'r':0, 'g':1, 'b':0})
            self.send_stop()

        elif cmd['end_mode'] == 'tennis':
            print("TODO PROGRAM SEARCH")

            # tennis_balls = self.tennis.get()
            tennis_balls = []

            last_waypoint = cmd['waypoints'][-1]
            endpoint = self.project(last_waypoint['lat'], last_waypoint['lon'])

            if tennis_balls == []:
                if self.guess == None:
                    self.guess = self.get_guess(endpoint)
                    print("NEW RANDOM")
                elif self.distance(self.guess) < self.guess_radius:
                    self.guess = self.get_guess(endpoint)
                    print("NEW RANDOM")

                print("random corrds",self.random_corrd)
                diff_angle = self.get_angle(self.guess)
                self.send_velocities(diff_angle)
            else:
                pass
                # get best tennis ball
                # drive towards it
                #follow the tennis ball!


        else:
            print("incorrect mode")

    def update(self):

        try:
            self.robot = self.gps.get()
            cmd = self.auto_control.get()
        except:
            print('lost control')
            return

        if cmd['command'] == 'off':
            print("off")
            self.reached_destination = False
            self.start_point['lat'] = self.gps.get()['lat']
            self.start_point['lon'] = self.gps.get()['lon']
            self.lights.send({'r':1, 'g':0, 'b':0})
        elif( cmd['command'] == 'auto'):
            last_waypoint = cmd['waypoints'][-1]

            if self.reached_destination:
                self.find_ball(cmd)

            elif( self.distance(self.project(last_waypoint['lat'], last_waypoint['lon'])) \
                    < self.final_radius ):
                self.reached_destination = True
                print("REACHED final End point")
            elif self.analyze_stuck():
                self.un_stick()
            else:
                lookahead = self.find_lookahead(cmd['waypoints'])
                diff_angle = self.get_angle(lookahead)
                self.send_velocities(diff_angle)
        else:
            raise ValueError

    def analyze_stuck(self):
        self.past_locations.append([self.project(self.robot['lat'], self.robot['lon']), self.imu.get()['angle'][0], time.time()])
        if len(self.past_locations) < 100:
            return False

        while len(self.past_locations) > 100:
            self.past_locations.pop(0)
        if (time.time() - self.stuck_time < 14):
            return False

        abs_angle = lambda x,y: min( abs(x-y + i*360) for i in [-1,0,1])
        # location, t = self.past_locations[0]
        # if self.distance(location)/(time.time() - t) < 0.1
        locations = [self.past_locations[10*i+5][0] for i in range(9)]
        angles    = [self.past_locations[10*i+5][1] for i in range(9)]

        max_loc = max([self.distance(loc) for loc in locations])
        max_angle = max([abs_angle(ang,self.imu.get()['angle'][0]) for ang in angles])

        print("we are", max_loc, "from stcuk and angle", max_angle)

        if max_loc< 1 and max_angle < 30:
            print("WE ARE STUCK")
            # self.stuck_location = location
            self.stuck_time = time.time()
            return True
        return False

    def un_stick(self):
        start_time = time.time()
        while (time.time() - start_time) < 4 and self.auto_control.get()['command'] == 'auto':
            self.analyze_stuck()
            out = {"f": -100, "t": -20 }
            print('unsticking', out)
            self.cmd_vel.send(out)
            time.sleep(0.1)

        start_time = time.time()
        start_angle = self.imu.get()['angle'][0]
        while (time.time() - start_time) < 1 and self.auto_control.get()['command'] == 'auto':
            self.analyze_stuck()
            out = {"f": 0, "t": -70 }
            print('unsticking part 2', out)
            self.cmd_vel.send(out)
            time.sleep(0.1)

        start_time = time.time()
        while (time.time() - start_time) < 2 and self.auto_control.get()['command'] == 'auto':
            self.analyze_stuck()
            out = {"f": 70, "t": 0 }
            print('unsticking part 2', out)
            self.cmd_vel.send(out)
            time.sleep(0.1)

    def find_lookahead(self,waypoints):
        start_waypoints = [self.start_point] + waypoints
        waypoint_pairs = zip(start_waypoints[::-1][1:], start_waypoints[::-1][:-1])
        i = 0
        for p1, p2 in waypoint_pairs:
            lookahead = self.lookahead_line(p1,p2)
            if lookahead is not None:
                print("point intersection", i)
                return lookahead
            i+=1
        else:
            print("NO intersection found!")

        distances = []
        for p1, p2 in waypoint_pairs:
            lookahead = self.lookahead_line(p1,p2,project = True)
            if lookahead is not None:
                distances.append( (self.distance(lookahead), lookahead) )

        for p1 in waypoints:
            point = self.project(p1['lat'], p1['lon'])
            distances.append( (self.distance(point), point) )

        return min(distances)[1]

    def distance(self, p1):
        x_start, y_start = p1
        # self.robot = self.gps.get()
        x_robot, y_robot = self.project(self.robot['lat'], self.robot['lon'])
        return math.sqrt((x_start - x_robot)**2 + (y_start - y_robot)**2)

    def lookahead_line(self,p1, p2, project=False):
        x_start, y_start = self.project(p1['lat'], p1['lon'])
        x_end, y_end = self.project(p2['lat'], p2['lon'])

        # self.robot = self.gps.get()
        x_robot, y_robot = self.project(self.robot['lat'], self.robot['lon'])

        if (x_robot-x_end)**2 + (y_robot - y_end)**2 < self.lookahead_radius**2:
            return (x_end, y_end)

        a = (x_end - x_start)**2 + (y_end - y_start)**2
        b = 2*( (x_end-x_start)*(x_start-x_robot) + (y_end-y_start)*(y_start-y_robot))
        c = (x_start-x_robot)**2 + (y_start-y_robot)**2 - self.lookahead_radius**2

        if b**2 - 4*a*c <=0:
            if project:
                t = -b/(2*a)
            else:
                return None
        else:
            t = (-b + math.sqrt(b**2 - 4*a*c))/(2*a)

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

        x_robot, y_robot = self.project(self.robot['lat'], self.robot['lon'])
        x_look, y_look = lookahead

        target = math.atan2( x_look - x_robot, y_look - y_robot)

        # anti clockwise positive
        return ((target - head_rad + math.pi)%(2*math.pi) - math.pi)

    def send_velocities(self, angle):
        # turn_rate = -100*math.tanh(1*angle)
        turn_rate = -200*angle/math.pi

        if math.fabs(angle) < math.radians(10):
            forward_rate = 130
        elif math.fabs(angle) < math.radians(60):
            forward_rate = 80
        elif math.fabs(angle) < math.radians(140):
            forward_rate = 30
        else:
            forward_rate = 0

        out = {"f": forward_rate, "t": turn_rate }

        # out = {"f": 70, "t": -150*angle/math.pi }
        print(out)
        self.cmd_vel.send(out)

    def send_stop(self):
        out = {"f": 0, "t": 0 }
        # out = {"f": 70, "t": -150*angle/math.pi }
        print('stoping', out)
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

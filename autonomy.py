#!/usr/bin/env python3


from UDPComms import Publisher, Subscriber, timeout
import math
import time
import random


import enum

from typing import Dict, Tuple, Sequence, List


class Situation(enum.Enum):
    ok = enum.auto()
    stuck = enum.auto()
    done = enum.auto()

class StateMachine:
    def __init__(self, parent, rover):
        self.parent = parent
        self.rover = rover

    def run(self) -> Situation:
        raise NotImplementedError 


class PostTask(StateMachine):

    def __init__(self,*args):
        super().__init__(*args)

        self.pathfollower = PathFollower(self, self.rover)
        self.finalapproach = FinalApproachPost(self, self.rover)

        self.state = self.pathfollower

    def run(self) -> Situation:
        ret = self.state.run()

        if self.finalapproach.


class PathFollower(StateMachine):

    def __init__(self,*args):
        super().__init__(*args)
        self.lookahead_radius = 6


    def run(self) -> Situation:
        self.rover.update()

        if ()
            return Situation.done

    def save_start_point(self):
        self.start_point = self.rover.get_pose()

    def find_lookahead(self,waypoints: List[Pose]) -> Pose:
        start_waypoints = [self.start_point] + waypoints
        waypoint_pairs = zip(start_waypoints[::-1][1:], start_waypoints[::-1][:-1])

        for p1, p2 in waypoint_pairs:
            lookahead = self.lookahead_line(p1,p2)
            if lookahead is not None:
                return lookahead
        else:
            print("NO intersection found!")

        distances = []
        for p1, p2 in waypoint_pairs:
            lookahead = self.lookahead_line(p1,p2,project = True)
            if lookahead is not None:
                distances.append( (self.rover.get_pose().dist(lookahead), lookahead) )

        for p1 in waypoints:
            distances.append( (self.rover.get_pose().dist(point), point) )

        return min(distances)[1]

    def lookahead_line(self, start: Pose, end: Pose, project=False) -> Pose:
        robot = self.rover.get_pose()

        if robot.dist(end) < self.lookahead_radius:
            return end

        a = (end.x - start.x)**2 + (end.y - start.y)**2
        b = 2*( (end.x-start.x)*(start.x-robot.x) + (end.y-start.y)*(start.y-robot.y))
        c = (start.x-robot.x)**2 + (start.y-robot.y)**2 - self.lookahead_radius**2

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

        x_look = t * end.x + (1-t) * start.x
        y_look = t * end.y + (1-t) * start.y

        return Pose(x_look, y_look)


    def get_angle(self, lookahead: Pose):
        pose = self.rover.get_pose()
        target = pose.bearing(lookahead)

        # anti clockwise positive
        return ((target - pose.a + math.pi)%(2*math.pi) - math.pi)


    def send_velocities(self, angle):
        turn_rate = -200*angle/math.pi

        if math.fabs(angle) < math.radians(10):
            forward_rate = 130
        elif math.fabs(angle) < math.radians(60):
            forward_rate = 80
        elif math.fabs(angle) < math.radians(140):
            forward_rate = 30
        else:
            forward_rate = 0

        self.rover.send_vel(forward_rate, turn_rate)


class FinalApproachGate(StateMachine):
    pass

class FinalApproachPost(StateMachine):
    pass

class Unstuck(StateMachine):
    pass

class Search(StateMachine):
    pass



class Pose:
    def __init__(self, x, y, angle=None):
        self.x = x
        self.y = y
        self.a = angle

    def dist(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def bearing(self,other):
        return math.atan2( other.x - self.x, other.y - self.y)


class Rover:

    def __init__(self):
        self.imu = Subscriber(8220, timeout=1)
        self.gps = Subscriber(8280, timeout=2)
        self.auto_control = Subscriber(8310, timeout=5)

        self.cmd_vel = Publisher(8830)
        self.lights = Publisher(8590)

        self.aruco = Subscriber(9021, timeout=2)
        time.sleep(3)

        self.start_gps = self.gps.recv()
        # self.start_point = {"lat":self.gps.get()['lat'] , "lon":self.gps.get()['lon']}

    def project(self, lat, lon):
        lat_orig = self.start_gps['lat']
        lon_orig = self.start_gps['lon']
        RADIUS = 6371 * 1000
        lon_per_deg = RADIUS * 2 * math.pi / 360
        lat_per_deg = lon_per_deg * math.cos(math.radians(lat_orig))

        x = (lon - lon_orig) * lon_per_deg
        y = (lat - lat_orig) * lat_per_deg
        return (x,y)

    def get_pose(self):
        gps = self.gps.get()
        heading = math.radians( self.imu.get()['angle'][0] )
        return Pose( *self.project( gps['lat'], gps['lon']), heading)

    def send_velocities(self, forward, twist):
        pass





class Pursuit:

    def __init__(self):

        self.final_radius = 1.5    # how close should we need to get to last endpoint
        self.search_radius = 20    # how far should we look from the given endpoint
        self.reached_destination = False # switch modes into tennis ball seach
        self.robot = None
        self.guess = None # where are we driving towards
        self.guess_radius = 3 # if we are within this distance we move onto the next guess
        self.guess_time = None

        self.last_tennis_ball = 0

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
            # self.lights.send({'r':0, 'g':1, 'b':0})
            self.servo.send({'pan':0,'tilt':90})
            self.send_stop()

        elif cmd['end_mode'] == 'tennis':
            print("Entering search program")

            # tennis_balls = self.tennis.get()
            # tennis_balls = []

            try:
                t1 = self.tennis1.get()
            except timeout:
                t1 = (None, float('inf'))

            try:
                t2 = self.tennis2.get()
            except timeout:
                t2 = (None, float('inf'))

            try:
                t3 = self.tennis3.get()
            except timeout:
                t3 = (None, float('inf'))

            dists = [(t1[1], 0), (t2[1], 1), (t3[1], 2)]
            best = [t1,t2,t3][min(dists)[1]]

            last_waypoint = cmd['waypoints'][-1]
            endpoint = self.project(last_waypoint['lat'], last_waypoint['lon'])

            if best[0] == None:

                if (time.time() - self.last_tennis_ball) < 2:
                    # if we saw a ball in the last 2 sec
                    out = {"f": 70, "t": 0 }
                    print('cont to prev seen ball', out)
                    self.cmd_vel.send(out)

                elif self.guess == None:
                    self.guess = self.get_guess(endpoint)
                    self.guess_time = time.time()
                    print("NEW RANDOM GUESS - first")
                elif self.distance(self.guess) < self.guess_radius:
                    self.guess = self.get_guess(endpoint)
                    print("NEW RANDOM GUESS - next")
                    self.guess_time = time.time()
                elif (time.time() - self.guess_time) > 40:
                    self.guess = self.get_guess(endpoint)
                    print("NEW RANDOM GUESS - timeout")
                    self.guess_time = time.time()
                    self.guess = self.get_guess(endpoint)

                print("random corrds",self.random_corrd)
                diff_angle = self.get_angle(self.guess)
                self.send_velocities(diff_angle)
            else:
                # self.guess_time = time.time() # will this matter?
                self.last_tennis_ball = time.time()
                if best[1] < 200:
                    print("REACHED TENNIS BALL")
                    # self.lights.send({'r':0, 'g':1, 'b':0})
                    self.servo.send({'pan':0,'tilt':90})
                    self.send_stop()
                else:
                    self.send_velocities_slow(best[0])
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
            # self.lights.send({'r':1, 'g':0, 'b':0})
            self.servo.send({'pan':0,'tilt':-90})
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





if __name__ == '__main__':
    a = Pursuit()

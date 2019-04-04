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

        while True:
            self.update()
            time.sleep(0.1)

    def update(self):

        cmd = self.auto_control.get()
        if cmd['cmd'] == 'off':
            self.start_point['lat'] = cmd['lat']
            self.start_point['lon'] = cmd['lon']

        lookahead = self.find_lookahead()
        diff_angle = self.get_angle(lookahead)

    def find_lookahead(self):
        pass

    def get_angle(self, lookahead):
        pass

    def send_velocities(self, turn_rate):
        self.cmd_vel.send({"f": 100, "t": 100*turn_rate})


if __name__ == '__main__':
    a = Pursuit()

from UDPComms import Publisher, Subscriber, timeout
import math
import time


class AutonomousPannel:

    fields = "forward twist"
    typ = "2f"
    port = 8900
    pub = Publisher(fields, typ, port)

    def __init__(self):

        self.gyro = Subscriber('angle', 'f', 8870, 2)
        self.initial_pos = Subscriber('time sats lat lon alt error_lat error_lon error_alt', 'ii3f3f', 8860)
        self.final_pos = Subscriber('lat lon', '2f', 8890, 2)
        self.vForward = 0
        self.vTwist = 0

    def calculate_velocities(self):
        while not self.intersect():
            self.vTwist = 150
            pub.send(0, self.vTwist)
            time.sleep(0.5)

        if self.intersect():
            self.vForward = 150
            pub.send(self.vForward, 0)

    def twist_velocity(self):
        while not self.intersect():
            self.vTwist = 150

    def intersect(self):
        ###     Determines if arrow created by gyro intersects with destination
        ###     Returns a boolean

        msg_gyro = self.gyro.recv()
        msg_initial = self.initial_pos.recv()
        msg_final = self.final_pos.recv()
        angle_gyro = msg_gyro.angle
        v_pts = [msg_final.lon - msg_initial.lon, msg_final.lat - msg_initial.lat]
        v_north = [0, 0.001]
        dot = (v_pts[0] * v_north[0]) + (v_pts[1] * v_north[1])
        v_pts_mag = math.sqrt(v_pts[0]**2 + v_pts[1]**2)
        v_north_mag = 0.001
        angle_pts = acos(dot / (v_pts_mag * v_north_mag))

        if abs(angle_gyro - math.degrees(angle_pts)) <= 5:
            return True
        else:
            return False
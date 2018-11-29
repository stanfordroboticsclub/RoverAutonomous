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
        self.turn_counterclockwise = True;
        self.vForward = 0
        self.vTwist = 0


        while True:
            self.msg_gyro = self.gyro.recv()
            self.msg_initial = self.initial_pos.recv()
            self.msg_final = self.final_pos.recv()
            self.angle_gyro = self.msg_gyro.angle
            self.v_pts = [self.msg_final.lon - self.msg_initial.lon, self.msg_final.lat - self.msg_initial.lat]
            self.v_north = [0, 0.001]
            self.twist_direction()
            self.calculate_velocities()
            time.sleep(0.2)

    def calculate_velocities(self):
        if self.msg_initial.lat == self.msg_final.lat and self.msg_initial.lon == self.msg_final.lon:
            self.vTwist = 0
            self.vForward = 0
            self.pub.send(self.vForward, self.vTwist)
            print self.vForward
            print self.vTwist
            return


        if not self.intersect() and self.turn_counterclockwise:
            self.vTwist = 155
            self.vForward = 0
            self.pub.send(self.vForward, self.vTwist)
            print self.vForward
            print self.vTwist
            return

        if not self.intersect() and not self.turn_counterclockwise:
            self.vTwist = -155
            self.vForward = 0
            self.pub.send(self.vForward, self.vTwist)
            print self.vForward
            print self.vTwist
            return

        if self.intersect():
            self.vTwist = 0
            self.vForward = 150
            self.pub.send(self.vForward, self.vTwist)
            print self.vForward
            print self.vTwist
            return



    def intersect(self):
        ###     Determines if arrow created by gyro intersects with destination
        ###     Returns a boolean

        dot = (self.v_pts[0] * self.v_north[0]) + (self.v_pts[1] * self.v_north[1])
        v_pts_mag = math.sqrt(self.v_pts[0]**2 + self.v_pts[1]**2)
        v_north_mag = 0.001
        angle_pts = math.acos(dot / (v_pts_mag * v_north_mag))

        if abs(self.angle_gyro - math.degrees(angle_pts)) <= 5:
            return True
        else:
            return False

    def twist_direction(self):
        # Changes value of turn_counterclockwise based on relationship between vectors.
        angle_from_x_axis = math.degrees(math.atan2(self.v_pts[1], self.v_pts[0])) + 180
        angle_reverse = 360 - angle_from_x_axis
        angle = self.angle_gyro - (angle_reverse + 90)

        if abs(angle) < 180:
            self.turn_counterclockwise = False


if __name__ == '__main__':
    a = AutonomousPannel()
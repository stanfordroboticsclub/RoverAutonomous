from UDPComms import Publisher, Subscriber, timeout
import math
import time


class AutonomousPannel:

    ### fields = "forward twist"
    ### typ = "2f"
    port = 8830
    pub = Publisher(port, false)

    def __init__(self):

        # import pdb; pdb.set_trace()
        self.gyro = Subscriber(8870, timeout=1)
        self.initial_pos = Subscriber(8280, 5)
        self.final_pos = Subscriber(8310, 2)
        self.turn_counterclockwise = True
        self.intersecting = False
        self.slow = False
        self.vForward = 0
        self.vTwist = 0

        time.sleep(3)

        while True:
            self.msg_gyro = self.gyro.get()
            self.msg_initial = self.initial_pos.get()
            self.msg_final = self.final_pos.get()
            self.angle_gyro = self.msg_gyro
            self.v_pts = [self.msg_final[1] - self.msg_initial[3], self.msg_final[0] - self.msg_initial[2]]
            self.v_north = [0, 0.001]
            self.twist_direction()
            self.calculate_velocities()
            time.sleep(0.2)

    def calculate_velocities(self):
        if self.msg_initial[2] == self.msg_final[0] and self.msg_initial[3] == self.msg_final[1]:
            self.vForward = 0
            self.vTwist = 0
            dataSend = [self.vForward, self.vTwist]
            self.pub.send(dataSend)
            print self.vForward
            print self.vTwist
            return


        if not self.intersecting and self.turn_counterclockwise:
            self.vForward = 0
            self.vTwist = 10
            dataSend = [self.vForward, self.vTwist]
            self.pub.send(dataSend)
            print self.vForward
            print self.vTwist
            return

        if not self.intersecting and not self.turn_counterclockwise:
            self.vForward = 0
            self.vTwist = -10
            dataSend = [self.vForward, self.vTwist]
            self.pub.send(dataSend)
            print self.vForward
            print self.vTwist
            return

        if self.intersecting:
            self.vForward = 45
            self.vTwist = 0
            dataSend = [self.vForward, self.vTwist]
            self.pub.send(dataSend)
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
        angle_from_x_axis = math.degrees(math.atan2(self.v_pts[1], self.v_pts[0]))
        print 'Angle from x axis: {}'.format(angle_from_x_axis)
        angle_reverse = (90 - angle_from_x_axis) % 360
        print 'Angle from y axis: {}'.format(angle_reverse)

        angle = self.angle_gyro - angle_reverse

        if angle < 0:
            self.turn_counterclockwise = False
        if angle > 0:
            self.turn_counterclockwise = True

        if math.fabs(angle) <= 10:
            self.intersecting = True
        else:
            self.intersecting = False

        print angle

if __name__ == '__main__':
    a = AutonomousPannel()
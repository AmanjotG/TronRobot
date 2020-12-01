#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int16
from sensor_msgs.msg import NavSatFix
import socket
import math

UDP_IP = "192.168.0.197"
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

rospy.init_node('listener', anonymous=True)
rate = rospy.Rate(1)  # 1Hz


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Navigation:
    def __init__(self):
        self.robot = Point(None, None)
        self.robot_heading = None
        self.small_location = Point(None, None)
        self.home_location = Point(None, None)
        self.big_location = Point(None, None)
        self.destinationList = []

    def calculateGoalHeading(self, goalPoint):
        # Calculates vector from centre of the robot to the centre of the triangle
        l1 = Point(goalPoint.x - self.robot.x, goalPoint.y - self.robot.y)
        # Creates vector direction along positive x-axis from centre of the robot
        l2 = Point(10, 0)
        # Calculates angle in radians
        angleRad = math.atan2(l1.x - l2.x, l1.y - l2.y)
        # Converts angle to degrees
        heading = math.degrees(angleRad)
        # Returns angle
        if heading < 0:
            return 360 - abs(heading), heading - self.robot_heading
        elif heading > 360:
            return heading - 360, heading - self.robot_heading
        else:
            return heading, heading - self.robot_heading

    def calculateDistance(self, point):
        return math.hypot(self.robot.x - point.x, self.robot.y - point.y)

    def driveToLocation(self, location):
        goal_heading, delta = self.calculateGoalHeading(location)
        print(f"heading: {goal_heading}, delta: {delta}")
        if delta > 0:
            while not abs(delta) <= 10:
                # turn_right
                break
        else:
            while not abs(delta) <= 10:
                # turn_left
                break


nav = Navigation()


def navCallback(data):
    nav.robot = Point(data.latitude, data.longitude)

    # sock.sendto(bytes(incoming, 'utf-8'), (UDP_IP, UDP_PORT))


def matInfoCallback(data):
    locations = data.data.split(",")
    nav.home_location = locations[0]
    nav.small_location = locations[1]
    nav.big_location = locations[2]


def destCallback(data):
    next_point = Point(data.latitude, data.longitude)
    nav.destinationList.append(next_point)
    nav.driveToLocation(next_point)


def headingCallBack(data):
    nav.robot_heading = data.data


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber("/location", NavSatFix, navCallback)
    rospy.Subscriber("/mat_info", String, matInfoCallback)
    rospy.Subscriber("/destination", NavSatFix, destCallback)
    rospy.Subscriber("/heading", Int16, headingCallBack)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

#!/usr/bin/env python
import json
import rospy
from std_msgs.msg import String, Int16
from sensor_msgs.msg import NavSatFix
import socket
import keyboard
import math
import requests

rospy.init_node('listener', anonymous=True)
rate = rospy.Rate(1)  # 1Hz

url = "http://10.10.60.15/"


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Navigation:
    def __init__(self):
        self.robot = Point(None, None)
        self.robot_heading = None
        self.goal_heading = 0
        self.delta = 0
        self.small_location = Point(None, None)
        self.home_location = Point(None, None)
        self.big_location = Point(None, None)
        self.destinationList = []
        self.useCam = True
        self.left_speed = 50
        self.right_speed = 50
        self.use = 0
        self.destination_reached = False
        self.return_home = False
        self.at_location = False
        self.travel = True
        self.distance = 0.0

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
        self.goal_heading, self.delta = self.calculateGoalHeading(location)
        self.distance = self.calculateDistance(location)
        print(f"heading: {self.goal_heading}, delta: {self.delta}, distance: {self.distance}")

        nav.destination_reached = True
        nav.useCam = False


nav = Navigation()


def sendMessage():
    message = {"a": f"{nav.use}", "l": {nav.left_speed}, "r": nav.right_speed}
    # message = f"?a=0&l=50&r=50"
    res = requests.post(url + "control", data=message)
    print(f"Sending...{res.text}")


def receiveMessage():
    # data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
    # print(f"received message: {data}")  # Prints information received
    res = requests.get(url + "cube")
    print(f"Receiving...{res.text}")


def navCallback(data):
    nav.robot = Point(data.latitude, data.longitude)
    sendMessage()
    rate.sleep()


def matInfoCallback(data):
    locations = data.data.split(",")
    nav.home_location = locations[0]
    nav.small_location = locations[1]
    nav.big_location = locations[2]


def destCallback(data):
    next_point = Point(data.latitude, data.longitude)
    nav.destinationList.append(next_point)


def headingCallBack(data):
    nav.robot_heading = data.data
    receiveMessage()
    rate.sleep()


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

    if len(nav.destinationList) is not 0:
        if not nav.destination_reached:
            nav.driveToLocation(nav.destinationList[0])
        else:
            nav.destinationList.pop(0)
            nav.destination_reached = False

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

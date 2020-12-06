#!/usr/bin/env python
import json

import rospy
from std_msgs.msg import String, Int16
from sensor_msgs.msg import NavSatFix
import socket
import math
import requests
import grequests as async

# UDP_IP = "192.168.0.197"
# UDP_PORT = 4210
#
# sock = socket.socket(socket.AF_INET,  # Internet
#                      socket.SOCK_DGRAM)  # UDP
# sock.bind((UDP_IP, UDP_PORT))

rospy.init_node('listener', anonymous=True)
rate = rospy.Rate(1)  # 1Hz

url = "http://10.10.60.15/info"


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
        if abs(self.delta) > 0:
            while not abs(self.delta) <= 5:
                # turn_right
                break

        nav.destination_reached = True
        nav.useCam = False


nav = Navigation()


# A simple task to do to each response object
def do_something(response):
    print(response.url)


# A list to hold our things to do via async
async_list = []
data = {'useCam': nav.useCam, 'distance': nav.distance, 'delta': nav.robot_heading}
headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
action_item = async.post(url, data=json.dumps(data), headers=headers)
async_list.append(action_item)

# def sendMessage():
#     data = {'useCam': nav.useCam, 'distance': nav.distance, 'delta': nav.robot_heading}
#     headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
#     r = requests.post(url, )
#
# message = "{\"useCam\":" + f"{nav.useCam}" + ",\"x\":" + f"{nav.robot.x}" + ",\"y\":" + f"{nav.robot.y}" + ",
# \"delta\":" + f"{nav.delta}" + "}\r " sock.sendto(bytes(message, "utf-8"), (UDP_IP, UDP_PORT))


# def receiveMessage():
#     data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
#     print(f"received message: {data}")  # Prints information received


def navCallback(data):
    nav.robot = Point(data.latitude, data.longitude)


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
    async.map(async_list)
    listener()

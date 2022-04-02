#!/usr/bin/env python3

# ROS Node that handles the CNC Code parsing and publishes the required commands such as velocity and temperature to print a component.

# Importations.
from robot_operation.srv import *
from std_srvs.srv import SetBool

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64
import numpy as np

import sys
import rospy
import time


class robot_server:
    def __init__(self):
        # Class constructor.
        # Initiliazise robot_control node with arguments from the command line.
        rospy.init_node('robot_server', anonymous=True, argv=sys.argv)

        # Set up publishers.
        # Velocity command for navigation.
        self.velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Extruder temperature command.
        self.tempPub = rospy.Publisher('/cmd_extTemp', Float32, queue_size=1)
        # Z axis position command.
        self.zaxisPub = rospy.Publisher('cmd_zAxisPos', Float32, queue_size=1)
        self.joint1_arm = rospy.Publisher(
            '/robot1/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2_arm = rospy.Publisher(
            '/robot1/joint2_position_controller/command', Float64, queue_size=10)

        # Set up subscriber.
        # Actual extruder temperature.
        self.tempSubs = rospy.Subscriber(
            '/extTemp', Float32, callback=self.tempCallback)

        # Set up service servers.
        self.switchPowerState_service = rospy.Service(
            "switchPowerState_service", powerState, self.switchPowerState_serviceHDLR)
        self.calibration_service = rospy.Service(
            "calibration_service", SetBool, self.calibration_serviceHDLR)
        self.linearMovement = rospy.Service(
            "linearMovement", linearMovement, self.linearMovementHDLR)
        self.armMovement = rospy.Service(
            "armMovement", armMovement, self.armMovementHDLR)
        self.stop_service = rospy.Service(
            "stop_service", SetBool, self.stop_serviceHDLR)
        self.sendTemp_service = rospy.Service("sendTemp_service", temperature, self.sendTemp_serviceHDLR)

        # Create message objects.
        self.velMsg = Twist()
        self.tempMsg = Float32()
        self.zAxisPosMsg = Float32()
        self.TargetReachedMsg = Float32()

        # Init class instances.
        self.vx = 0
        self.vy = 0
        self.a = 0
        self.b = 1
        self.offset = 0
        self.offset2 = 0
        self.J = 99

        self.l1arm = 280
        self.l2arm = 140

        self.dir = 0

    def switchPowerState_serviceHDLR(self, data):
        pass

    def calibration_serviceHDLR(self, data):
        pass

    def accesories_serviceHDLR(self, data):
        pass

    def microcontrollers_serviceHDLR(self, data):
        pass

    def tempCallback(self, data):
        # Temperature callback function.
        self.extTemp = data.data

    def tempPublisher(self, temp):
        # Temperature publisher.
        self.tempMsg = temp
        self.tempPub.publish(self.tempMsg)

    def zAxisPosPublisher(self, pos):
        # Z axis position publisher.
        self.zAxisPosMsg = pos
        self.zaxisPub.publish(self.zAxisPosMsg)

    def velPublisher(self, Vx, Vy):
        # Velocity publisher.
        self.velMsg.linear.x = Vx
        self.velMsg.linear.y = Vy
        self.velMsg.linear.z = 0

        self.velMsg.angular.x = 0
        self.velMsg.angular.y = 0
        self.velMsg.angular.z = 0

        self.velPub.publish(self.velMsg)

    def linearMovementHDLR(self, delta, velocity, coord, xaux, yaux):
        # Performs linear movements.

        if (xaux != 0 or yaux != 0):
            coorX = xaux
            coorY = yaux
            totalDist = np.sqrt(coorX**2+coorY**2)
            totalDist = totalDist.real
        else:
            coorX = delta.x
            coorY = delta.y
            distance = abs(delta)
            totalDist = distance.length()

        lin_time = totalDist/velocity
        velocity = velocity/1000

        if (coorX == 0):
            tetha = np.pi/2
        else:
            tetha = np.arctan(abs(coorY)/abs(coorX))

        self.vx = velocity*np.cos(tetha)
        self.vy = velocity*np.sin(tetha)

        if (coorX >= 0 and coorY >= 0):
            self.vx = self.vx
            self.vy = self.vy
        elif (coorX < 0 and coorY >= 0):
            self.vx = -self.vx
            self.vy = self.vy
        elif (coorX < 0 and coorY < 0):
            self.vx = -self.vx
            self.vy = -self.vy
        elif (coorX >= 0 and coorY < 0):
            self.vx = self.vx
            self.vy = -self.vy

        self.velMsg.linear.x = self.vx
        self.velMsg.linear.y = self.vy

        self.velPub.publish(self.velMsg)

        time.sleep(lin_time)

        self.velPublisher(0, 0)

        self.zAxisPosPublisher(coord.z)

        while (self.TargetReachedMsg.data != Float32(1.0)):

            print("Moviendo Eje Z")
            time.sleep(0.001)

    def armMovementHDLR(self, delta, coord, velocity, offset):

        distance = abs(delta)
        totalDist = distance.length()
        lin_time = totalDist/velocity

        px = coord.x+self.offset2
        py = coord.y-offset
        pz = coord.z

        l1 = self.l1arm
        l2 = self.l2arm

        print("px:", px)
        print("py:", py)
        print("pz:", pz)
        print("")

        tetha2 = np.arccos((px**2+py**2-l1**2-l2**2)/(2*l1*l2))
        if px == 0:
            tetha1 = np.pi/2 - \
                np.arctan((l2*np.sin(tetha2))/(l1+l2*np.cos(tetha2)))
        else:
            tetha1 = np.arctan(
                py/px)-np.arctan((l2*np.sin(tetha2))/(l1+l2*np.cos(tetha2)))

        self.joint1_arm.publish(tetha1.real)
        self.joint2_arm.publish(tetha2.real)
        self.zaxisPub.publish(pz)

        time.sleep(lin_time)

    def stop_serviceHDLR(self, data):
        pass

    def sendTemp_serviceHDLR(self, data):
        pass


if __name__ == "__main__":
    try:
        # Create an instance of the robot_control class.
        rc = robot_server()

        # Get arguments.
        args = rospy.myargv(argv=sys.argv)

        if len(args) < 2:
            print("\nERROR: no file provided.")
            sys.exit()

        file_path = args[1]
        sim = args[2]

        if (sim == 1) or (sim == "true"):
            print('\nSimulation only.')

        op = None
        while (op != "yes"):
            print("\n--------------------------------------")
            op = input("Are you ready to print? options: yes, no, exit: ")
            if (op == "exit"):
                sys.exit

        print("\nPrinting file on: " + file_path)
        file = open(file_path, 'r')
        rc.gcodeReader(file, sim)
        print("\nFile printed.")

        time.sleep(2)

    except rospy.ROSInterruptException:
        pass

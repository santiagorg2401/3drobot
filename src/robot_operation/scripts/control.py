#!/usr/bin/env python3

#ROS Node which publishes waypoints to make the 3drobot perform squares.
import numpy as np
import rospy
from geometry_msgs.msg import Twist

class control:
    def __init__(self):
        rospy.init_node('robotVelPub', anonymous=True)                      #Initialize ROS Node.
        self.velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)     #Publisher object.
        self.velMsg = Twist()                                               #Message object.

    def moveStraight(self, speed, distance, direction):
        #Direction: 0 -> Forward, 1 -> Backward, 2-> Right, 3-> Left

        if(direction == 0):
            self.velMsg.linear.x = speed
            self.velMsg.linear.y = 0
            msg = "Forward."

        elif(direction == 1):
            self.velMsg.linear.x = -speed
            self.velMsg.linear.y = 0
            msg = "Backward."

        elif(direction == 2):
            self.velMsg.linear.y = speed
            self.velMsg.linear.x = 0
            msg = "Right."

        elif(direction == 3):
            self.velMsg.linear.y = -speed
            self.velMsg.linear.x = 0
            msg = "Left."

        else:
            msg = "Not recognized."

        print("\nMoving straight.\nSpeed: " + str(speed) + " m/s." + "\nDistance: " + str(distance) + " m." + "\nGoing " + msg)

        self.velMsg.linear.z = 0
        self.velMsg.angular.x = 0
        self.velMsg.angular.y = 0
        self.velMsg.angular.z = 0

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            self.velPub.publish(self.velMsg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)

        #After the loop, stops the robot
        self.stop()

    def stop(self):
        self.velMsg.linear.x = 0
        self.velMsg.linear.y = 0
        self.velMsg.linear.z = 0
        self.velMsg.angular.x = 0
        self.velMsg.angular.y = 0
        self.velMsg.angular.z = 0

        self.velPub.publish(self.velMsg)

if __name__ == '__main__':
    try:
        c = control()
        i = np.array([0, 2, 1, 3])
        vel = 0.5
        dis = 2

        for j in i:
            c.moveStraight(vel, dis, j)

    except rospy.ROSInterruptException: 
        pass
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

class imuDATA:

    def __init__(self):
    
        rospy.init_node('ImuSubs', anonymous=True)
        rospy.Subscriber("/imu", Imu, self.adquireImuDataCallback)
        
        self.orientation_X=0
        self.orientation_Y=0
        self.orientation_Z=0

        self.VAngular_X=0
        self.VAngular_Y=0
        self.VAngular_Z=0

        self.acceleration_X=0
        self.acceleration_Y=0
        self.acceleration_Z=0
        print("Holaaa")

    def adquireImuDataCallback(self,imumsg):

        print("Holaaa3")
            
        self.orientation_X=imumsg.orientation.x
        self.orientation_Y=imumsg.orientation.y
        self.orientation_Z=imumsg.orientation.z
        

        self.VAngular_X=imumsg.angular_velocity.x
        self.VAngular_Y=imumsg.angular_velocity.y
        self.VAngular_Z=imumsg.angular_velocity.z


        self.acceleration_X=imumsg.linear_acceleration.x
        self.acceleration_Y=imumsg.linear_acceleration.y
        self.acceleration_Z=imumsg.linear_acceleration.z

        
        
        print(self.acceleration_X)
        print(self.acceleration_Y)


if __name__ == '__main__':

    imuDATA()
    rospy.spin()
#!/usr/bin/env python3

from std_msgs.msg import Float32, Float64
import rospy
import time

class calibration:
    def __init__(self):       
        self.TargetReachedSub = rospy.Subscriber('/TargetReached',Float32,callback=self.TargetReachedCB)
        
        self.zaxisPub = rospy.Publisher('/cmd_zAxisPos', Float32, queue_size = 1)
        self.joint1_arm = rospy.Publisher('/robot1/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2_arm = rospy.Publisher('/robot1/joint2_position_controller/command', Float64, queue_size=10) 
        

        self.TargetReachedMsg = Float32()
        self.zAxisPosMsg = Float32()
        self.xAxisPosMsg = Float64()
        self.yAxisPosMsg = Float64()

        self.calibrationOrder()

    def TargetReachedCB(self,data):
        self.TargetReachedMsg.data = data
        
    
    def calibrationOrder(self):
        time.sleep(4)
        self.TargetReachedMsg.data = 0
        self.zAxisPosMsg.data = 999
        self.zaxisPub.publish(self.zAxisPosMsg)
        while (self.TargetReachedMsg.data != Float32(1.0)):
            print("Calibrando eje Z")
            time.sleep(0.001)
 
        self.TargetReachedMsg.data = 0
        self.xAxisPosMsg.data = 999
        self.joint1_arm.publish(self.xAxisPosMsg)
        while (self.TargetReachedMsg.data != Float32(1.0)):
            print("Calibrando articulacion 1")
            time.sleep(0.001)
 
        self.TargetReachedMsg.data = 0
        self.yAxisPosMsg.data = 999
        self.joint2_arm.publish(self.yAxisPosMsg)
        while (self.TargetReachedMsg.data != Float32(1.0)):
            print("Calibrando articulacion 2")
            time.sleep(0.001)



if __name__ == "__main__":
    rospy.init_node('calibration',anonymous=True)
    c = calibration()
    print("Calibracion Exitosa")
    
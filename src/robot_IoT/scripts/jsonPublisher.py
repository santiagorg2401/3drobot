#!/usr/bin/env python3

# Importations.
from std_msgs.msg import Int16
from std_msgs.msg import String

import rospy
import json
import datetime

class robot_IoT:
    def __init__(self):
        # Class constructor.
        # Initialize the ROS Node.
        rospy.init_node('robot_IoT', anonymous=True)

        # Set up subscriber.
        self.tempSubs = rospy.Subscriber('/extTemp', Int16, callback=self.tempCallback)    # Actual extruder temperature.
        self.weightSubs = rospy.Subscriber('/weight', Int16, callback=self.weightCallback) # Filament weight.

        # Set up publisher.
        self.jsonMsg = rospy.Publisher('/jsonMsg', String, queue_size=10)

    def tempCallback(self, data):
        # Temperature callback function.
        self.extTemp = data.data

        self.publishJson()

    def weightCallback(self, data):
        # Temperature callback function.
        self.weight = data.data

        self.publishJson()

    def publishJson(self):
        crrt_time = datetime.datetime.now()

        x = {
        "Bot": 1,
        "Extruder temperature": self.extTemp,
        "Filament weight": self.weight,
        "Date" : str(crrt_time)
        }

        
        jsonMsg = json.dumps(x)
        self.jsonMsg.publish(jsonMsg)

if __name__=='__main__':
    riot = robot_IoT()
    rospy.spin()



#!/usr/bin/env python3

# Importations.
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Bool
import rospy
import json
import datetime
import requests


class robot_IoT:
    def __init__(self):
        # Class constructor.
        # Initialize the ROS Node.
        rospy.init_node('robot_IoT', anonymous=True)

        # Set up subscriber.
        # Actual extruder temperature.
        self.tempSubs = rospy.Subscriber(
            '/extTemp', Int16, callback=self.tempCallback)
        # Filament weight.
        self.weightSubs = rospy.Subscriber(
            '/weight', Int16, callback=self.weightCallback)
        self.powerStageSub = rospy.Subscriber(
            '/powerStage', Bool, callback=self.powerStageCallback)

        # Set up publisher.
        self.jsonMsg = rospy.Publisher('/jsonMsg', String, queue_size=10)

        self.api_url = "http://things.ubidots.com/api/v1.6/devices/3drobot/?token=BBFF-yB97DF1ZzLZbBnUR0RsVPbzqKEA3iH"

        self.powerStage = None
        self.weight = None
        self.extTemp = None

    def powerStageCallback(self, data):
        self.powerStage = data.data

        self.publishJson()

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
            "Power stage status": self.powerStage,
            "Date": str(crrt_time)
        }

        jsonMsg = json.dumps(x)

        response = requests.post(self.api_url, data=jsonMsg)

        self.jsonMsg.publish(jsonMsg)


if __name__ == '__main__':
    riot = robot_IoT()
    rospy.spin()

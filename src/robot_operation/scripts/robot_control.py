#!/usr/bin/env python3

# ROS Node that handles the CNC Code parsing and publishes the required commands such as velocity and temperature to print a component.

# Importations.
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from cnc.gcode import GCode, GCodeException
from cnc.coordinates import *

import sys
import rospy
import time

class robot_control:
    def __init__(self):
        # Class constructor.
        # Initiliazise robot_control node with arguments from the command line.
        rospy.init_node('robot_control', anonymous=True, argv=sys.argv)

        # Set up publishers.
        self.velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)            # Velocity command for navigation.
        self.tempPub = rospy.Publisher('/cmd_extTemp', Int16, queue_size = 1)     # Extruder temperature command.
        self.zaxisPub = rospy.Publisher('/cmd_zAxisPos', Int16, queue_size = 1)    # Z axis position command.
        self.powerStagePub = rospy.Publisher('/powerStage', Bool, queue_size = 1)

        # Set up subscriber.
        self.tempSubs = rospy.Subscriber('/extTemp', Int16, callback=self.tempCallback)   # Actual extruder temperature.
        
        # Create message objects.
        self.velMsg = Twist()
        self.tempMsg = Int16()
        self.zAxisPosMsg = Int16()
        self.powerStageMsg = Bool()

        # Init class instances.
        self._position = Coordinates(0, 0, 0, 0)                        # TODO Replace with state estimator.
        self.extTemp = 0                                                # Extruder temperature, °C.
        self._velocity = 0                                              # Nozzle velocity, mm/sec.
        self._local = Coordinates(0.0, 0.0, 0.0, 0.0)                   # CNC code, local coordinates.
        self._convertCoordinates = 1.0                                  # CNC code, milimeters by default.
        self._absoluteCoordinates = True                                # CNC code, absolute coordinates, enabled by default.
        self._plane = None                                              # CNC code, plane.
        self.linnu = 0                                                  # Line counter.

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

    def powerStagePublisher(self, state):
        # powerStage commands publisher.
        self.powerStageMsg = state
        self.powerStagePub.publish(self.powerStageMsg)

    def velPublisher(self, Vx, Vy):
        # Velocity publisher.
        self.velMsg.linear.x = Vx
        self.velMsg.linear.y = Vy
        self.velMsg.linear.z = 0

        self.velMsg.angular.x = 0
        self.velMsg.angular.y = 0
        self.velMsg.angular.z = 0

        self.velPub.publish(self.velMsg)

    def linearMovement(self, delta, velocity):
        # Performs linear movements.
        distance = abs(delta)
        totalDist = distance.length()
        lin_time = totalDist/velocity

        velocity = velocity/1000

        if (delta.x > 0):
            self.velMsg.linear.x = velocity
        elif (delta.x == 0):
            self.velMsg.linear.x = 0
        elif (delta.x < 0):
            self.velMsg.linear.x = -velocity

        if (delta.y > 0):
            self.velMsg.linear.y = velocity
        elif (delta.y == 0):
            self.velMsg.linear.y = 0
        elif (delta.y < 0):
            self.velMsg.linear.y = -velocity
            
        self.velPub.publish(self.velMsg)
        self.zAxisPosPublisher(delta.z)
        self._position += delta
        time.sleep(lin_time)

        self.velPublisher(0,0)

    def gcodeReader(self, file, sim):
        # Read and publish CNC code lines.
        try:
            for line in file:
                line = line.strip()

                self.linnu += 1
                print(self.linnu)
                if not self.gcodeParser(line, sim):
                    break
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            pass
        print("\r\nExiting...")

    def gcodeParser(self, line, sim):
        # Extract the data within the CNC code.
        try:
            gcode = GCode.parse_line(line)
            ans = self.gcodeCommands(gcode, sim)
        except (GCodeException) as e:
            print('ERROR ' + str(e))
            return False
        if ans is not None:
            print('OK ' + ans)
        else:
            print('OK')
        return True

    def gcodeCommands(self, gcode, sim):
        # Perform a CNC command, taken and modified from the method do_command of the GMachine class by Nikolay Khabarov.
        if gcode is None:
            return None
        answer = None

        # Read command.
        c = gcode.command()

        if c is None and gcode.has_coordinates():
            c = 'G1'

        # Read parameters.
        if self._absoluteCoordinates:
            coord = gcode.coordinates(self._position - self._local,
                                      self._convertCoordinates)
            coord = coord + self._local
            delta = coord - self._position

        else:
            delta = gcode.coordinates(Coordinates(0.0, 0.0, 0.0, 0.0),
                                      self._convertCoordinates)

        velocity = gcode.get('F', self._velocity)

        # Check that the velocity is within operation parameters.
        if velocity > 120:
            print("Maximum speed exceeded, maximum speed: 120 mm/sec.")
            self.velPublisher(0,0)
            sys.exit()

        radius = gcode.radius(Coordinates(0.0, 0.0, 0.0, 0.0),
                              self._convertCoordinates)

        if c == 'G0':  # Rapid move.
            self.linearMovement(delta, 120)

        elif c == 'G1':  # Linear interpolation.
            self.linearMovement(delta, velocity)
            
        elif c == 'G2':  # Circular interpolation, clockwise.
            # TODO Implement.
            pass

        elif c == 'G3':  # Circular interpolation, counterclockwise.
            # TODO Implement.
            pass

        elif c == 'G4':  # Delay in seconds.
            if not gcode.has('P'):
                print("P is not specified")
                self.velPublisher(0,0)
                sys.exit()

            pause = gcode.get('P', 0)
            if pause < 0:
                print("bad delay")
                self.velPublisher(0,0)
                sys.exit()

            time.sleep(pause)

        elif c == 'G20':  # Switch to inches.
            self._convertCoordinates = 25.4

        elif c == 'G21':  # Switch to mm.
            self._convertCoordinates = 1.0

        elif c == 'G28':  # Home.
            pass

        elif c == 'G53':  # Switch to machine coordinates.
            self._local = Coordinates(0.0, 0.0, 0.0, 0.0)

        elif c == 'G90':  # Switch to absolute coordinates.
            self._absoluteCoordinates = True

        elif c == 'G91':  # Switch to relative coordinates.
            self._absoluteCoordinates = False

        elif c == 'G92':  # Switch to local coordinates.
            if gcode.has_coordinates():
                self._local = self._position - gcode.coordinates(
                    Coordinates(self._position.x - self._local.x,
                                self._position.y - self._local.y,
                                self._position.z - self._local.z,
                                self._position.e - self._local.e),
                    self._convertCoordinates)
            else:
                self._local = self._position

        elif c == 'M84':  # Disable motors.
            self.velPublisher(0,0)

        elif c == 'M114':  # Get current position.
            p = self.position()
            answer = "X:{} Y:{} Z:{} E:{}".format(p.x, p.y, p.z, p.e)
        
        # Extruder control.
        elif c == 'M104' or c == 'M109':
            if (sim == 1) or (sim == "true"):
                pass
            elif (sim == 0) or (sim == "false"):
                if not gcode.has("S"):
                    print("temperature is not specified")
                    self.velPublisher(0,0)
                    sys.exit()

                temp = gcode.get('S', 0)
                self.tempPub.publish(temp)

                if c == 'M109':
                    print("Waiting for extruder to heat up.")
                    while(self.extTemp != temp):
                        pass                

        elif c is None:  # Command not specified(ie just F was passed)
            pass
        
        # Save parameters on success.
        self._velocity = velocity
        return answer

if __name__ == "__main__":
    try:
        #Create an instance of the robot_control class.
        rc = robot_control()

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

        rc.powerStagePublisher(False)
        print("\nPrinting file on: " + file_path) 
        file = open(file_path, 'r')
        rc.gcodeReader(file, sim)

        print("\nFile printed.")
        rc.powerStagePublisher(True)
        
    except rospy.ROSInterruptException: 
        pass
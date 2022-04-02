#!/usr/bin/env python3

import sys
import rospy
import serial
import time
import numpy as np
import serial.tools.list_ports

from cnc.coordinates import *
from cnc.gcode import GCode, GCodeException
from robot_operation.srv import *
from std_msgs.msg import Bool, Float32, Int8
from std_srvs.srv import SetBool


class Robot:
    def __init__(self):
        # Set up services.
        rospy.wait_for_service("calibration_service")
        self.calibration_service = rospy.ServiceProxy(
            "calibration_service", SetBool)
        rospy.wait_for_service("switchPowerState_service")
        self.switchPowerState_service = rospy.ServiceProxy(
            "switchPowerState_service", SetBool)

    def detectMicrocontrollers(self):
        '''
        Automatically browses through all available USB devices and detects whether or not,
        it is a supported device.

        Args: None
        Returns: List of available microcontrollers, type: serial.tools.list_ports_linux.SysFS object

        '''
        mics = ["platform", "accesoryBoard"]

        ports = serial.tools.list_ports.comports()
        microcontrollers = []

        for port, desc, hwid in sorted(ports):
            print("{}: {} [{}]".format(port, desc, hwid))

            device = serial.Serial(port, 9600)
            time.sleep(2.0)

            resp = device.readline()

            if resp in mics:
                microcontrollers.append(device)

        return microcontrollers

    def detectAccesories(self, accesoryBoard):
        '''
        Automatically detects connected accesories to main board.

        Args: None
        Returns: List of accesories connected to accesory board.
        '''

        # List of available accesories.
        avl_accesories = ["marker", "extruder"]
        accesories = []

        for accesory in avl_accesories:
            # TODO, document command abbreviations, F1 means READ_ACCESORIES.
            accesoryBoard.write('F1')
            time.sleep(1)
            resp = accesoryBoard.readline()

            if resp == accesory:
                accesories.append(accesory)

        return accesories

    def switchPowerState(self, states):
        resp = self.switchPowerState_service(states)

        return resp

    def calibrate(self):
        resp = self.calibration_service(Bool(True))

        return resp


class GCodeHandler:
    def __init__(self, path, sim):
        # TODO Replace with state estimator.
        self._position = Coordinates(0, 0, 0, 0)
        # Extruder temperature, °C.
        self.extTemp = 0
        # Nozzle velocity, mm/sec.
        self._velocity = 0
        # CNC code, local coordinates.
        self._local = Coordinates(0.0, 0.0, 0.0, 0.0)
        # CNC code, milimeters by default.
        self._convertCoordinates = 1.0
        # CNC code, absolute coordinates, enabled by default.
        self._absoluteCoordinates = True
        # CNC code, plane.
        self._plane = None
        # Line counter.
        self.linnu = 0

        # Set up services.
        rospy.wait_for_service("linearMovement")
        self.linearMovement = rospy.ServiceProxy(
            "linearMovement", linearMovement)
        rospy.wait_for_service("armMovement")
        self.armMovement = rospy.ServiceProxy("armMovement", armMovement)
        rospy.wait_for_service("stop_service")
        self.stop_service = rospy.ServiceProxy("stop_service", SetBool)
        rospy.wait_for_service("sendTemp_service")
        self.sendTemp_service = rospy.ServiceProxy("sendTemp_service", temperature)
        rospy.wait_for_service("waitTemp_service")
        self.waitTemp_service = rospy.ServiceProxy("waitTemp_service", temperature)

        # Set up subscribers.
        self.extTempSub = rospy.Subscriber("ext_temp", Int8, self.extTempCB)
        self.extTemp = 0

        file = self.readGCodeFile(path)
        self.gcodeReader(file, sim)

    def extTempCB(self, data):
        self.extTemp = data.data

    def readGCodeFile(self, path):
        file = open(path, 'r')
        return file

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
            self.velPublisher(0, 0)
            sys.exit()

        radius = gcode.radius(Coordinates(0.0, 0.0, 0.0, 0.0),
                              self._convertCoordinates)

        if c == 'G0':  # Rapid move.
            resp = self.linearMovement(delta, 120, coord, 0, 0)
            self._position += delta

        elif c == 'G1':  # Linear interpolation.
            # self.linearMovement(delta, velocity, coord, 0, 0)
            resp = self.armMovement(delta, coord, velocity, 0)
            self._position += delta

        elif c == 'G2':  # Circular interpolation, clockwise.
            # TODO Implement.
            pass

        elif c == 'G3':  # Circular interpolation, counterclockwise.
            # TODO Implement.
            pass

        elif c == 'G4':  # Delay in seconds.
            if not gcode.has('P'):
                print("P is not specified")
                self.velPublisher(0, 0)
                sys.exit()

            pause = gcode.get('P', 0)
            if pause < 0:
                print("bad delay")
                self.velPublisher(0, 0)
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
            self.stop_service(Bool(True))

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
                    self.stop_service(Bool(True))
                    sys.exit()

                temp = gcode.get('S', 0)
                self.sendTemp_service(Int8(temp))

                if c == 'M109':
                    print("Waiting for extruder to heat up.")
                    while(self.extTemp != temp):
                        pass

        elif c is None:  # Command not specified(ie just F was passed)
            pass

        # Save parameters on success.
        self._velocity = velocity
        return answer

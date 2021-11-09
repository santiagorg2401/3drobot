# Robot operation package.
Robot control implementation in Python in the robot_control node, where it was used the [PyCNC](https://github.com/Nikolay-Kha/PyCNC) library by [Nikolay Khabarov](https://github.com/Nikolay-Kha), for this project only the GCode parser was used with a few modifications.

## Launch file.
This package was designed to work with Gazebo simulation or a physical robot, if the simulation parameter is set, then it includes the robot_gazebo package to launch the world and spawn the robot, else, it will run the rosserial node to make connection with the Arduino board, finally, to initialize the robot_control.py node it is necessary to provide a valid path to a .gcode file.

```console
roslaunch robot_operation control.launch file_path:=_insert a valid file path_ sim:=_true or false_
```

file_path: path to .gcode file, example: /home/user/Downloads/test.gcode. Always required.
sim: 1, 0, true or false. Optional: enabled by default.

Note: temperature commands such as M104 or M109 ar not enabled when sim = true.
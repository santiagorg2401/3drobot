#!/bin/bash

#This is an automated tool to install related software to work on the 3drobot.
# Author: Santiago Restrepo Garcia.
# e-mail: santiago.restrepo_g@uao.edu.co
# Organization: RAS @ Universidad Autonoma de Occidente.
# Role: student.

echo "Welcome to the 3drobot configuration and installation automated program, this will install the required dependencies and build everything into your system."

echo ""
echo "Updating system packages."
sudo apt-get update
sudo apt-get upgrade

echo ""
echo "System packages updated."

echo ""
echo "Installing dependencies."

sudo apt install python3-pip
sudo apt-get install git
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial

/usr/bin/python3 -m pip install --upgrade pip
pip3 install numpy
pip3 install rospy

echo ""
echo "Dependencies installed."

echo ""
echo "Cloning & building repository."

git clone https://github.com/santiagorg2401/3drobot.git
cd 3drobot/
catkin_make
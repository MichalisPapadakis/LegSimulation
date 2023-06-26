#!/bin/bash

# Check if an argument was provided
if [ $# -eq 0 ]; then
  echo "No workspace name was provided!"
  echo "Provide a workspace name by excecuting the script as:"
  echo -e "\n ./install_packages.sh <workspace_name>"
  exit 1
elif [ $# -eq 1 ]; then 
    workspace_name=$1 
elif [ $# -ne 1 ]; then
  echo "Multiple arguments were provided!"
  echo "Provide only workspace name by excecuting the script as:"
  echo -e "\n ./install_packages.sh <workspace_name>"
  exit 1
fi


currentDir=$(pwd)
cd


#Instal catkin tools:
echo "#################"
echo "Installing Catkin tools:"
echo "#################"
sudo apt-get install python3-catkin-tools

#Eigen
echo "#################"
echo "Installing Eigen:"
echo "#################"
sudo apt install libeigen3-dev

#Instal tf2:
echo "#################"
echo "Installing xacro:"
echo "#################"
sudo apt-get install ros-noetic-xacro

#Instal tf2:
echo "#################"
echo "Installing tf2:"
echo "#################"
sudo apt-get install ros-noetic-tf2
sudo apt-get install ros-noetic-tf2-ros

#Instal gazebo_ros:
echo "#################"
echo "Installing gazebo_ros:"
echo "#################"
sudo apt-get install ros-noetic-gazebo-ros


#ros_control:
echo "#################"
echo "Installing ros controllers:"
echo "#################"
sudo apt-get install ros-noetic-position-controllers
sudo apt-get install ros-noetic-joint-trajectory-controller
sudo apt-get install ros-noetic-effort-controllers
sudo apt-get install ros-noetic-controller-manager
sudo apt-get install ros-noetic-ros-control
sudo apt-get install ros-noetic-control-msgs

#extra:
echo "#################"
echo "Installing rviz:"
echo "#################"
sudo apt-get install ros-noetic-rviz 

echo "#################"
echo "Installing robot state publisher:"
echo "#################"
sudo apt-get install ros-noetic-robot-state-publisher,

echo "#################"
echo "Installed All packages"
echo "#################"

echo "Creating workspace and symlinks"
echo "#################"

# Create workspace:
if !which catkin >/dev/null 2>&1; then
    echo "catkin_tools is not installed."
    echo "Try installing catkin tools seperately and restarting the terminal:"
    echo -e "Use: \n sudo apt-get install python3-catkin-tools"
    exit 1
fi

#source ros 
source /opt/ros/noetic/setup.bash

#create directories
cd
mkdir -p ~/$workspace_name/src
cd ~/$workspace_name

catkin init

#create symlink:
ln -s $currentDir/leg_description ~/$workspace_name/src/
ln -s $currentDir/leg_control ~/$workspace_name/src/

#build workspace
echo "#################"
echo "Building workspace"
echo "#################"

catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release


echo "#################"
echo "Sourcing workspace:"
echo "#################"

source devel/setup.bash

echo "#################"
echo "Ready to launch!"
echo "#################"



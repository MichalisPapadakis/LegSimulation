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

#build workspace
echo "Building workspace"
echo "#################"

cd $currentDir
cd ../
catkin build 

echo "#################"
echo "Sourcing workspace:"
echo "#################"

source devel/setup.bash

echo "#################"
echo "Ready to launch!"
echo "#################"



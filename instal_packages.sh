#Eigen
cd
echo "#################"
echo "Installing Eigen:"
echo "#################"
sudo apt install libeigen3-dev

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
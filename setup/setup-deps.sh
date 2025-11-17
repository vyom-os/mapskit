#!/bin/bash


sudo apt update -y && sudo apt install locales -y 
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common -y 
sudo add-apt-repository universe -y 

sudo apt update -y 
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update -y 

sudo apt install ros-humble-ros-base -y
sudo apt install ros-dev-tools -y 
sudo apt install ros-humble-ament-cmake -y 

sudo apt-get install ros-humble-octomap ros-humble-octomap-mapping ros-humble-octomap-msgs -y
sudo apt-get install libompl-dev -y
sudo apt-get install python3-dev cmake -y

pip install "pybind11[global]"


source /opt/ros/humble/setup.bash
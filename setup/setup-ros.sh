#!/bin/bash


sudo apt update -y && sudo apt install locales -y 
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common -y 
sudo add-apt-repository universe -y 

sudo apt install ros-humble-ros-base -y
sudo apt install ros-dev-tools -y 
sudo apt install ros-humble-ament-cmake -y 

source /opt/ros/humble/setup.bash
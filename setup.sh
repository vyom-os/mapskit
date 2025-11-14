#!/bin/bash

# This script runs setup scripts in a specific order.

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SETUP_DIR="$SCRIPT_DIR/setup"

echo "Installing ROS 2 dependencies..."

# Update package lists
sudo apt-get update

# Install ROS 2 build tools and dependencies
echo "Installing ament_cmake..."
sudo apt-get install -y ros-$ROS_DISTRO-ament-cmake

echo "Installing ament_cmake_python..."
sudo apt-get install -y ros-$ROS_DISTRO-ament-cmake-python

echo "Installing rclcpp..."
sudo apt-get install -y ros-$ROS_DISTRO-rclcpp

echo "Installing rclpy..."
sudo apt-get install -y ros-$ROS_DISTRO-rclpy

echo "Setup finished successfully."

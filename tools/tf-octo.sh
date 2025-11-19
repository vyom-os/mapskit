#!/bin/bash

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Warning: /opt/ros/humble/setup.bash not found. Ensure ROS 2 is sourced."
fi

# Function to cleanup background processes
cleanup() {
    echo ""
    echo "Caught signal, cleaning up..."
    # Kill all child processes in the same process group
    kill $(jobs -p) 2>/dev/null
    wait
    echo "Done."
    exit
}

# Trap SIGINT (Ctrl+C) and SIGTERM
trap cleanup SIGINT SIGTERM

echo "Starting static transforms..."

# odom -> base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
PID1=$!
echo "Started odom -> base_link (PID: $PID1)"

# base_link -> camera_link
ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link camera_link &
PID2=$!
echo "Started base_link -> camera_link (PID: $PID2)"

echo "Transforms are running. Press Ctrl+C to stop."

# Wait for processes
wait

#!/bin/bash

SESSION_NAME="mapskit"

if [ "$1" == "kill" ]; then
  # Send Ctrl+C to the panes running active processes to trigger graceful shutdown
  tmux send-keys -t $SESSION_NAME:init.0 C-c 2>/dev/null
  tmux send-keys -t $SESSION_NAME:init.1 C-c 2>/dev/null
  
  echo "Sent shutdown signals. Waiting for cleanup..."
  sleep 3
  
  tmux kill-session -t $SESSION_NAME 2>/dev/null
  echo "Session $SESSION_NAME killed."
  exit 0
fi

# Check if the session already exists
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
  # Create a new session and name the first window 'init'
  tmux new-session -d -s $SESSION_NAME -n init

  # Pane 1: ros2 launch
  tmux send-keys -t $SESSION_NAME:init.0 "clear" C-m
  tmux send-keys -t $SESSION_NAME:init.0 'ros2 launch orbbec_camera gemini_330_series.launch.py' C-m

  # Split window to create Pane 2
  tmux split-window -h -t $SESSION_NAME:init

  # Pane 2: wait 10s and run tf-octo.sh
  # Get absolute path to the directory containing this script
  SCRIPT_DIR=$(dirname "$(realpath "$0")")
  tmux send-keys -t $SESSION_NAME:init.1 "clear" C-m
  tmux send-keys -t $SESSION_NAME:init.1 "sleep 10 && $SCRIPT_DIR/tf-octo.sh" C-m

  # Create the second window 'runtime'
  tmux new-window -t $SESSION_NAME:2 -n runtime

  # Create the third window 'monitor'
  tmux new-window -t $SESSION_NAME:3 -n monitor
  tmux split-window -h -t $SESSION_NAME:monitor.0
  tmux split-window -v -t $SESSION_NAME:monitor.0
  tmux split-window -v -t $SESSION_NAME:monitor.2

  tmux send-keys -t $SESSION_NAME:monitor.0 "clear" C-m
  tmux send-keys -t $SESSION_NAME:monitor.1 "clear" C-m
  tmux send-keys -t $SESSION_NAME:monitor.2 "clear" C-m
  tmux send-keys -t $SESSION_NAME:monitor.3 "clear" C-m

  tmux send-keys -t $SESSION_NAME:monitor.0 "watch 'ros2 topic list | grep mapskit'" C-m
  tmux send-keys -t $SESSION_NAME:monitor.1 "ros2 topic echo /camera/depth/points" C-m
  tmux send-keys -t $SESSION_NAME:monitor.2 "ros2 topic echo /mapkskit/voxelmap_full" C-m
  tmux send-keys -t $SESSION_NAME:monitor.3 "ros2 topic echo /mapskit/test_changes" C-m

  # Select the first window
  tmux select-window -t $SESSION_NAME:monitor
fi

# Attach to the session
tmux attach-session -t $SESSION_NAME

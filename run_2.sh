#!/bin/bash

# Start a new tmux session named "ros_session"
tmux new-session -d -s ros_session -n roscore "roscore"

# Open a new gnome-terminal window for the Python script
gnome-terminal -- bash -c "python3 main.py; exec bash"

# Open another tmux window, navigate to the 'ui' directory, and run 'npm run dev'
tmux new-window -t ros_session:1 -n ui "cd ui && npm run dev"

# Attach to the tmux session
tmux attach-session -t ros_session
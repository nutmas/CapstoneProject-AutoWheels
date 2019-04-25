#!/bin/sh
tmux new-session -d -s ros 
tmux send-keys 'source devel/setup.bash' 'C-m'
tmux select-window -t ros:0
tmux split-window -h
tmux send-keys 'source devel/setup.bash' 'C-m'
tmux split-window -v
tmux send-keys 'source devel/setup.bash' 'C-m'
tmux attach-session -t ros

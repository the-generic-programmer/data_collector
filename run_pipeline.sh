#!/bin/zsh
# run_pipeline.sh - Build, source, and run the data_collector node, then start logging
# Usage: zsh run_pipeline.sh
set -e

# Build the package
colcon build --packages-select data_collector

# Source the workspace
source install/setup.zsh

echo "Starting data_collector node..."
ros2 run data_collector my_node &
NODE_PID=$!

# Wait a few seconds for the node to start
sleep 3

echo "Starting logging via service call..."
ros2 service call /start_logging std_srvs/srv/Trigger

echo "Pipeline running. Press Ctrl+C to stop."
TRAPINT() {
  echo 'Stopping logging and shutting down...'
  ros2 service call /stop_logging std_srvs/srv/Trigger
  kill $NODE_PID
  exit 0
}

# Wait for the node process
wait $NODE_PID

#!/bin/bash
# run_pipeline.sh - Build, source, and run the data_collector node, then start logging and prediction
set -e
cd ..

# Ensure logs directory exists
mkdir -p logs

# Build the package
colcon build --packages-select data_collector

# Source the workspace
source install/setup.bash

echo "Starting data_collector node in a new terminal..."
gnome-terminal -- bash -c "ros2 run data_collector my_node; exec bash" &
NODE_TERM_PID=$!

# Wait a few seconds for the node to start
sleep 5

echo "Starting logging via service call..."
ros2 service call /start_logging std_srvs/srv/Trigger

# Start prediction script in TCP mode in a new terminal
PREDICT_SCRIPT="$(pwd)/data_collector/ml_weather/predict.py"
echo "Starting predict.py in TCP mode in a new terminal..."
gnome-terminal -- bash -c "cd $(pwd); . ../.venv/bin/activate; python3 $PREDICT_SCRIPT --tcp; exec bash" &
PREDICT_TERM_PID=$!

echo "Pipeline running. Press Ctrl+C to stop."
trap cleanup INT
cleanup() {
  echo 'Stopping logging and shutting down...'
  ros2 service call /stop_logging std_srvs/srv/Trigger
  kill $NODE_TERM_PID $PREDICT_TERM_PID
  exit 0
}

# Wait for both terminals to close
wait $NODE_TERM_PID $PREDICT_TERM_PID

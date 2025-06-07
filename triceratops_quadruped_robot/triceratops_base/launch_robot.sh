#!/bin/bash

# --- Lock File --- 
LOCKFILE="/tmp/triceratops_robot.lock"

if [ -e "$LOCKFILE" ] && kill -0 $(cat "$LOCKFILE") 2>/dev/null; then
    echo "Already running."
    exit 1
fi

# Make sure the lockfile is removed when we exit and when we receive a signal
trap "rm -f $LOCKFILE; exit" INT TERM EXIT

# Create the lock file, storing our process ID
echo $$ > "$LOCKFILE"
# --- End Lock File ---

# Source ROS2 setup files
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=33
source /home/panda2/ros2_humble/install/setup.bash

# Go to the working directory
cd /home/panda2/code/quadruped_robot_12_DOF-DEV/triceratops_quadruped_robot/triceratops_base

# Function to clean up child processes on exit
cleanup() {
    echo "Cleaning up child processes..."
    # Kill all child processes
    if [ ! -z "$JOY_PID" ]; then
        kill $JOY_PID 2>/dev/null
    fi
    if [ ! -z "$JOYSTICK_PID" ]; then
        kill $JOYSTICK_PID 2>/dev/null
    fi
    if [ ! -z "$CONTROLLER_PID" ]; then
        kill $CONTROLLER_PID 2>/dev/null
    fi
    # Remove lock file before exiting
    rm -f "$LOCKFILE"
    exit 0
}

# Set up trap for signals
trap cleanup SIGINT SIGTERM

# Start ROS joy node and save PID
ros2 run joy joy_node &
JOY_PID=$!
echo "Started joy node with PID: $JOY_PID"

# Wait for joy node to initialize
sleep 2

# Start joystick controller and save PID
python3 Joy_controller.py &
JOYSTICK_PID=$!
echo "Started joystick controller with PID: $JOYSTICK_PID"

# Wait for joystick to initialize
sleep 2

# Add delay for serial device initialization
echo "Waiting for serial device..."
sleep 5

# Start robot controller
python3 Controller.py &
CONTROLLER_PID=$!
echo "Started robot controller with PID: $CONTROLLER_PID"

# Wait for any process to exit or for signal
echo "Waiting for processes to exit..."
wait -n
EXIT_CODE=$?
echo "A child process exited with code $EXIT_CODE. Cleaning up..."

# Call cleanup if a process exits or we receive a signal
cleanup
# Ensure lock file is removed even if cleanup wasn't called normally
rm -f "$LOCKFILE" 
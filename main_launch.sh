#!/bin/bash

# Default values
RADIUS=4.0
RNAME="b1"
WNAME="indoor1"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --radius)
            RADIUS="$2"
            shift 2
            ;;
        --rname)
            RNAME="$2"
            shift 2
            ;;
        --wname)
            WNAME="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--radius <value>] [--rname <robot_name>] [--wname <world_name>]"
            exit 1
            ;;
    esac
done

# Generate random position within radius
generate_random_position() {
    local radius=$1
    # Generate random angle (0 to 2π)
    local angle=$(python3 -c "import random, math; print(random.uniform(0, 2 * math.pi))")
    # Generate random radius (0 to max_radius)
    local r=$(python3 -c "import random; print(random.uniform(0, $radius))")
    # Convert to cartesian coordinates
    local x=$(python3 -c "import math; print($r * math.cos($angle))")
    local y=$(python3 -c "import math; print($r * math.sin($angle))")
    echo "$x $y"
}

# Generate random position
pos=$(generate_random_position $RADIUS)
x=$(echo $pos | cut -d' ' -f1)
y=$(echo $pos | cut -d' ' -f2)

echo "Spawning robot at position: x=$x, y=$y (radius=$RADIUS)"

# Launch the robot with random position arguments
echo "Launching Gazebo simulation..."
roslaunch unitree_move_base gazebo_move_base.launch rname:=$RNAME wname:=$WNAME spawn_x:=$x spawn_y:=$y &
LAUNCH_PID=$!

# Wait for Gazebo to fully load
echo "Waiting for Gazebo to initialize..."
sleep 5

# Step 1: Overwrite action_plan.txt with "z 0"
echo "Step 1: Writing 'z 0' to action_plan.txt..."
echo "z 0" > /home/shivayogiakki/action_plan.txt
echo "Action plan file updated."
sleep 2

# Step 2: Run junior_ctrl with sudo
echo "Step 2: Running junior_ctrl..."
echo "michigantechyogi" | sudo -S LD_LIBRARY_PATH=$LD_LIBRARY_PATH ./devel/lib/unitree_guide/junior_ctrl &
JUNIOR_PID=$!
echo "junior_ctrl started with PID: $JUNIOR_PID"
sleep 5

# Step 3: Run scan_to_json
echo "Step 3: Running scan_to_json..."
rosrun llm_control scan_to_json &
SCAN_PID=$!
echo "scan_to_json started with PID: $SCAN_PID"
sleep 2

# Step 4: Run the Python server request script
echo "Step 4: Running Python server request script..."
python3 "LLM server request/send_request_v3.py" &
PYTHON_PID=$!
echo "Python script started with PID: $PYTHON_PID"

# Function to cleanup processes on exit
cleanup() {
    echo "Cleaning up processes..."
    kill $LAUNCH_PID 2>/dev/null
    kill $JUNIOR_PID 2>/dev/null
    kill $SCAN_PID 2>/dev/null
    kill $PYTHON_PID 2>/dev/null
    sudo pkill -f junior_ctrl 2>/dev/null
    exit 0
}

# Set up signal handlers for cleanup
trap cleanup SIGINT SIGTERM

echo "All processes started successfully!"
echo "Press Ctrl+C to stop all processes and exit."

# Wait for user interrupt
wait
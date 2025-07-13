#!/bin/bash

# Default values
RADIUS=4.0
WNAME="indoor1"
LOOP_DURATION=240  # 4 minutes in seconds
TOTAL_ITERATIONS=108  # 36 iterations per robot (go2, go1, a1) across 3 maps each = 108 total
MAPS=("indoor1" "indoor2" "indoor4")  # Array of available maps

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --radius)
            RADIUS="$2"
            shift 2
            ;;
        --wname)
            WNAME="$2"
            shift 2
            ;;
        --duration)
            LOOP_DURATION="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--radius <value>] [--wname <world_name>] [--duration <seconds>]"
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

# Function to cleanup processes
cleanup_processes() {
    echo "Cleaning up processes..."
    
    # Kill specific PIDs if they exist
    if [ ! -z "$LAUNCH_PID" ]; then
        kill $LAUNCH_PID 2>/dev/null
    fi
    if [ ! -z "$JUNIOR_PID" ]; then
        kill $JUNIOR_PID 2>/dev/null
    fi
    if [ ! -z "$SCAN_PID" ]; then
        kill $SCAN_PID 2>/dev/null
    fi
    if [ ! -z "$PYTHON_PID" ]; then
        kill $PYTHON_PID 2>/dev/null
    fi
    
    # Kill any remaining processes
    sudo pkill -f junior_ctrl 2>/dev/null
    pkill -f gazebo 2>/dev/null
    pkill -f roslaunch 2>/dev/null
    pkill -f scan_to_json 2>/dev/null
    pkill -f "send_request_v3.py" 2>/dev/null
    
    # Wait a moment for processes to terminate
    sleep 2
    
    # Force kill if necessary
    sudo pkill -9 -f junior_ctrl 2>/dev/null
    pkill -9 -f gazebo 2>/dev/null
    
    echo "Cleanup completed."
}

# Function to get robot and map based on iteration
get_robot_and_map() {
    local iteration=$1
    local robot_cycle=$(( (iteration - 1) / 36 ))  # 0, 1, 2 for go2, go1, a1
    local map_cycle=$(( ((iteration - 1) % 36) / 12 ))  # 0, 1, 2 for indoor1, indoor2, indoor4
    
    case $robot_cycle in
        0) echo "go2 ${MAPS[$map_cycle]}" ;;
        1) echo "go1 ${MAPS[$map_cycle]}" ;;
        2) echo "a1 ${MAPS[$map_cycle]}" ;;
    esac
}

# Function to run single iteration
run_iteration() {
    local iteration=$1
    local robot_name=$2
    local map_name=$3
    
    echo "=========================================="
    echo "Starting iteration $iteration"
    echo "Robot: $robot_name"
    echo "Map: $map_name"
    echo "=========================================="
    
    # Generate random position
    pos=$(generate_random_position $RADIUS)
    x=$(echo $pos | cut -d' ' -f1)
    y=$(echo $pos | cut -d' ' -f2)
    
    echo "Spawning robot at position: x=$x, y=$y (radius=$RADIUS)"
    
    # Launch the robot with random position arguments
    echo "Launching Gazebo simulation..."
    roslaunch unitree_move_base gazebo_move_base.launch rname:=$robot_name wname:=$map_name spawn_x:=$x spawn_y:=$y &
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
    
    echo "All processes started successfully for iteration $iteration!"
    echo "Running for $LOOP_DURATION seconds..."
    
    # Wait for the specified duration
    sleep $LOOP_DURATION
    
    echo "Iteration $iteration completed. Cleaning up..."
    cleanup_processes
    
    # Wait a bit before starting next iteration
    sleep 3
}

# Main execution function
main() {
    echo "Starting looping robot simulation with multiple maps..."
    echo "Total iterations: $TOTAL_ITERATIONS"
    echo "Loop duration: $LOOP_DURATION seconds ($(($LOOP_DURATION/60)) minutes)"
    echo "Maps: ${MAPS[*]}"
    echo "Robot sequence per map: go2 (1-36), go1 (37-72), a1 (73-108)"
    echo "Map sequence per robot: indoor1 (1-12), indoor2 (13-24), indoor4 (25-36)"
    
    # Calculate total expected runtime
    total_time=$(( TOTAL_ITERATIONS * (LOOP_DURATION + 10) ))  # +10 for setup/cleanup time
    echo "Estimated total runtime: $(($total_time/3600)) hours $(($total_time%3600/60)) minutes"
    
    # Set up signal handlers for cleanup
    trap 'echo "Interrupt received. Cleaning up..."; cleanup_processes; exit 0' SIGINT SIGTERM
    
    for ((iteration=1; iteration<=TOTAL_ITERATIONS; iteration++)); do
        # Get robot name and map name based on iteration
        robot_map=$(get_robot_and_map $iteration)
        RNAME=$(echo $robot_map | cut -d' ' -f1)
        CURRENT_MAP=$(echo $robot_map | cut -d' ' -f2)
        
        echo ""
        echo "Progress: $iteration/$TOTAL_ITERATIONS iterations"
        
        # Calculate position within current robot-map cycle
        robot_iteration=$(( ((iteration - 1) % 36) + 1 ))
        map_iteration=$(( ((iteration - 1) % 12) + 1 ))
        
        echo "Robot cycle: $RNAME (iteration $robot_iteration/36)"
        echo "Map cycle: $CURRENT_MAP (iteration $map_iteration/12)"
        
        # Run the iteration
        run_iteration $iteration $RNAME $CURRENT_MAP
        
        # Check if we should continue
        if [ $iteration -lt $TOTAL_ITERATIONS ]; then
            echo "Preparing for next iteration..."
            sleep 2
        fi
    done
    
    echo ""
    echo "=========================================="
    echo "All $TOTAL_ITERATIONS iterations completed!"
    echo "Simulation sequence finished."
    echo "Final summary:"
    echo "- 3 robots (go2, go1, a1) tested"
    echo "- 3 maps (indoor1, indoor2, indoor4) tested"
    echo "- 12 iterations per robot per map"
    echo "- Total: 108 iterations"
    echo "=========================================="
}

# Run main function
main
#!/bin/bash

# Sequential Robot Simulation Runner
# This script runs main_launch_auto.sh followed by main_launch_auto_attack.sh

# Default values that can be passed to both scripts
RADIUS=4.0
WNAME="indoor1"
LOOP_DURATION=240

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
        --help)
            echo "Sequential Robot Simulation Runner"
            echo "Usage: $0 [--radius <value>] [--wname <world_name>] [--duration <seconds>]"
            echo ""
            echo "Options:"
            echo "  --radius <value>     Set spawn radius for robots (default: 4.0)"
            echo "  --wname <name>       Set world name (default: indoor1)"
            echo "  --duration <seconds> Set loop duration in seconds (default: 240)"
            echo "  --help              Show this help message"
            echo ""
            echo "This script will run:"
            echo "1. main_launch_auto.sh (108 iterations)"
            echo "2. main_launch_auto_attack.sh (108 iterations)"
            echo "Total: 216 iterations"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--radius <value>] [--wname <world_name>] [--duration <seconds>]"
            echo "Use --help for more information"
            exit 1
            ;;
    esac
done

# Function to check if script files exist
check_script_files() {
    local missing_files=()
    
    if [ ! -f "main_launch_auto.sh" ]; then
        missing_files+=("main_launch_auto.sh")
    fi
    
    if [ ! -f "main_launch_auto_attack.sh" ]; then
        missing_files+=("main_launch_auto_attack.sh")
    fi
    
    if [ ${#missing_files[@]} -gt 0 ]; then
        echo "Error: Missing required script files:"
        for file in "${missing_files[@]}"; do
            echo "  - $file"
        done
        echo "Please ensure both scripts are in the current directory."
        exit 1
    fi
}

# Function to make scripts executable
make_scripts_executable() {
    echo "Making scripts executable..."
    chmod +x main_launch_auto.sh
    chmod +x main_launch_auto_attack.sh
}

# Function to run a script with error handling
run_script() {
    local script_name=$1
    local script_description=$2
    
    echo "=========================================="
    echo "Starting $script_description"
    echo "Script: $script_name"
    echo "Parameters: --radius $RADIUS --wname $WNAME --duration $LOOP_DURATION"
    echo "=========================================="
    
    # Run the script with the specified parameters
    ./"$script_name" --radius "$RADIUS" --wname "$WNAME" --duration "$LOOP_DURATION"
    
    # Check exit status
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo "=========================================="
        echo "$script_description completed successfully!"
        echo "=========================================="
    else
        echo "=========================================="
        echo "Error: $script_description failed with exit code $exit_code"
        echo "=========================================="
        exit $exit_code
    fi
}

# Function to calculate total runtime
calculate_total_runtime() {
    local total_iterations=216  # 108 + 108
    local per_iteration_time=$(( LOOP_DURATION + 10 ))  # +10 for setup/cleanup
    local total_seconds=$(( total_iterations * per_iteration_time ))
    local total_hours=$(( total_seconds / 3600 ))
    local total_minutes=$(( (total_seconds % 3600) / 60 ))
    
    echo "Estimated total runtime: $total_hours hours $total_minutes minutes"
}

# Function to handle cleanup on interrupt
cleanup_on_interrupt() {
    echo ""
    echo "=========================================="
    echo "Interrupt received. Stopping sequential runner..."
    echo "Note: Individual scripts handle their own cleanup."
    echo "=========================================="
    exit 130
}

# Main execution function
main() {
    echo "=========================================="
    echo "Sequential Robot Simulation Runner"
    echo "=========================================="
    echo ""
    echo "Configuration:"
    echo "- Radius: $RADIUS"
    echo "- World Name: $WNAME"
    echo "- Loop Duration: $LOOP_DURATION seconds ($(($LOOP_DURATION/60)) minutes)"
    echo ""
    echo "Execution Plan:"
    echo "1. Run main_launch_auto.sh (108 iterations)"
    echo "2. Run main_launch_auto_attack.sh (108 iterations)"
    echo "Total: 216 iterations"
    echo ""
    
    # Calculate and display total runtime
    calculate_total_runtime
    echo ""
    
    # Check if script files exist
    check_script_files
    
    # Make scripts executable
    make_scripts_executable
    
    # Set up signal handler for cleanup
    trap cleanup_on_interrupt SIGINT SIGTERM
    
    # Record start time
    start_time=$(date +%s)
    echo "Sequential simulation started at: $(date)"
    echo ""
    
    # Run first script (main_launch_auto.sh)
    run_script "main_launch_auto.sh" "Normal Robot Simulation Phase"
    
    echo ""
    echo "=========================================="
    echo "Phase 1 Complete - Starting Phase 2"
    echo "Waiting 5 seconds before starting attack phase..."
    echo "=========================================="
    sleep 5
    
    # Run second script (main_launch_auto_attack.sh)
    run_script "main_launch_auto_attack.sh" "Attack Robot Simulation Phase"
    
    # Calculate and display completion time
    end_time=$(date +%s)
    total_runtime=$(( end_time - start_time ))
    runtime_hours=$(( total_runtime / 3600 ))
    runtime_minutes=$(( (total_runtime % 3600) / 60 ))
    runtime_seconds=$(( total_runtime % 60 ))
    
    echo ""
    echo "=========================================="
    echo "ALL SIMULATIONS COMPLETED SUCCESSFULLY!"
    echo "=========================================="
    echo ""
    echo "Final Summary:"
    echo "- Phase 1 (Normal): 108 iterations completed"
    echo "- Phase 2 (Attack): 108 iterations completed"
    echo "- Total iterations: 216"
    echo "- Total robots tested: 3 (go2, go1, a1)"
    echo "- Total maps tested: 3 (indoor1, indoor2, indoor4)"
    echo "- Iterations per robot per map: 12"
    echo ""
    echo "Timing:"
    echo "- Started: $(date -d @$start_time)"
    echo "- Completed: $(date -d @$end_time)"
    echo "- Total runtime: $runtime_hours hours $runtime_minutes minutes $runtime_seconds seconds"
    echo ""
    echo "Both simulation phases have been completed successfully!"
    echo "=========================================="
}

# Run main function
main "$@"
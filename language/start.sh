#!/bin/bash

# Check if tmux is installed
check_tmux_installed() {
    if command -v tmux &> /dev/null; then
        echo "tmux is already installed."
        return 0
    else
        echo "tmux is not installed."
        return 1
    fi
}

# Install tmux on macOS or Linux if it's not already installed
install_tmux() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        if check_tmux_installed; then
            return
        fi
        echo "Installing tmux on macOS..."
        brew install tmux
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Linux
        if check_tmux_installed; then
            return
        fi
        echo "Installing tmux on Linux..."
        if [[ -x "$(command -v apt-get)" ]]; then
            sudo apt-get update
            sudo apt-get install -y tmux
        elif [[ -x "$(command -v yum)" ]]; then
            sudo yum install -y tmux
        elif [[ -x "$(command -v dnf)" ]]; then
            sudo dnf install -y tmux
        else
            echo "Unsupported Linux distribution. Please install tmux manually."
            exit 1
        fi
    else
        echo "Unsupported operating system. Please install tmux manually."
        exit 1
    fi
}

# Call the install_tmux function
install_tmux

# This script exists to launch all three of the tcp server, dummy robot connecting
# to the tcp server and the voice daemon that makes calls to the tcp server, which 
# then transmits to the robot. it also contains a flag for the vision model, which 
# can optionally be run to see if there is enough free space in front of the robot 

# Function to check if a conda environment exists
check_conda_env() {
    conda env list | grep -E "^$1\s"
}

CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"

# Name of the environments to check
ENV1="python310"
ENV2="python3.10"

# Check for the existence of the environments and activate if found
if check_conda_env "$ENV1"; then
    echo "Activating environment: $ENV1"
    conda activate "$ENV1"
elif check_conda_env "$ENV2"; then
    echo "Activating environment: $ENV2"
    conda activate "$ENV2"
else
    echo "No suitable environment found. Please create either $ENV1 or $ENV2."
fi

# install the requirements if we need to
RUN_VISION=false
INSTALL_REQUIREMENTS=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --vision)
      RUN_VISION=true
      shift # past argument
      ;;
    --requirements)
      INSTALL_REQUIREMENTS=true
      shift # past argument
      ;;
    *)
      shift # past argument
      ;;
  esac
done

if [ "$INSTALL_REQUIREMENTS" = true ]; then
    echo "Installing all packages from requirements.txt"
    pip install -r requirements.txt
fi

run_with_prefix() {
  local prefix=$1
  shift
  "$@" &> >(while IFS= read -r line; do echo "[$prefix] $line"; done)
}

# Array to keep track of background process PIDs
PIDS=()

# Trap function to handle Ctrl+C
cleanup() {
  echo "Caught SIGINT, terminating all processes..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null
  done
  tmux kill-server
  wait
  exit 1
}

# Set the trap for SIGINT
trap cleanup SIGINT

# Example commands to run
run_with_prefix "TCP Server" sh -c 'tmux new-session -d "cd submodules/TcpServer && ./gradlew build &&  ./gradlew bootrun"' &
PIDS+=($!)
run_with_prefix "Dummy Robot" python3 -c "from src.dummy_robot import try_start_headless; try_start_headless()" &
PIDS+=($!)
run_with_prefix "Voice Daemon" python3 daemon.py &
PIDS+=($!)

# Check if we should run the vision model
if [ "$RUN_VISION" = true ]; then
  run_with_prefix "Vision Model" sh -c 'cd ../vision/ && sudo ./visionServer submodules/ORB_SLAM3/Vocabulary/ORBvoc.txt ./data/ORB_SLAM3/RealSense_D415.yaml' &
  PIDS+=($!)
fi

# Wait for all background jobs to finish
wait
#!/bin/bash

# This script exists to launch all three of the tcp server, dummy robot connecting
# to the tcp server and the voice daemon that makes calls to the tcp server, which 
# then transmits to the robot.

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
if [[ "$1" == "--requirements" ]]; then
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
  wait
  exit 1
}

# Set the trap for SIGINT
trap cleanup SIGINT

# Example commands to run
run_with_prefix "TCP Server" sh -c 'cd submodules/TcpServer && ./gradlew build && ./gradlew bootrun' &
PIDS+=($!)
run_with_prefix "Dummy Robot" python3 -c "from src.dummy_robot import try_start_headless; try_start_headless()" &
PIDS+=($!)
run_with_prefix "Voice Daemon" python3 daemon.py &
PIDS+=($!)

# Wait for all background jobs to finish
wait


#!/bin/bash

# Source the global bash profile and other relevant files
source /etc/profile  # System-wide profile settings
source /home/user/.bashrc  # User-specific profile settings

# Activate conda environment
source /home/user/miniconda3/etc/profile.d/conda.sh

# Activate the Conda environment
conda activate python310

# Run the Python script
exec python3 /home/user/morbius/daemon.py

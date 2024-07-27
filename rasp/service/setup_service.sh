#!/bin/bash

# Variables
SERVICE_NAME="robot"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME.service"
SCRIPT_PATH="/home/user/morbius/daemon.py"
USER="user"


# Check if script exists
if [ ! -f "$SCRIPT_PATH" ]; then
    echo "Error: Script $SCRIPT_PATH does not exist."
    exit 1
fi

# Check if the service is already running
if systemctl is-active --quiet $SERVICE_NAME; then
    echo "Stopping $SERVICE_NAME service..."
    sudo systemctl stop $SERVICE_NAME.service
fi


# Create the .service file
echo "Creating $SERVICE_FILE..."

sudo bash -c "cat > $SERVICE_FILE" <<EOL
[Unit]
Description=Keyword Listener Service
After=sound.target
Requires=alsa-state.service

[Service]
ExecStart=/home/user/morbius/rasp/run_service.sh
WorkingDirectory=$(dirname $SCRIPT_PATH)
StandardOutput=inherit
StandardError=inherit
Restart=always
User=$USER
Group=audio
Environment="CONDA_EXE=/home/user/miniconda3/bin/conda"
Environment="CONDA_PREFIX=/home/user/miniconda3/envs/python310"
Environment="CONDA_DEFAULT_ENV=python310"
Environment="PATH=/home/user/miniconda3/envs/python310/bin:/home/user/miniconda3/condabin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

[Install]
WantedBy=multi-user.target
EOL

# Reload systemd manager configuration
echo "Reloading systemd manager configuration..."
sudo systemctl daemon-reload

# Enable the service to start at boot
echo "Enabling $SERVICE_NAME service..."
sudo systemctl enable $SERVICE_NAME.service

# Start the service
echo "Starting $SERVICE_NAME service..."
sudo systemctl restart $SERVICE_NAME.service

# Check the status of the service
sudo systemctl status $SERVICE_NAME.service

echo "$SERVICE_NAME service setup complete."

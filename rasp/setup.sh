#!/bin/bash

# This is a setup script for raspberry pi 5. It will not work on a mac, windows,
# or the majority of linux machines, as it is installing for aarch64 

sudo apt-get update

# download  
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh

# allow exec
chmod +x Miniconda3-latest-Linux-aarch64.sh

# run install headless
bash Miniconda3-latest-Linux-aarch64.sh -b -p $HOME/miniconda3

# Initialize Miniconda
$HOME/miniconda3/bin/conda init

# Reload shell configuration (this example assumes bash)
source ~/.bashrc

# Verify the installation
conda --version

# create new conda environment for everything
conda create -n python310 python=3.10 -y

# activate
conda activate python310

# install all other apt packages
sudo apt-get install gcc -y
sudo apt-get install portaudio19-dev espeak automake libtool -y
sudo apt-get install libhdf5-dev -y

pip install -r requirements.txt
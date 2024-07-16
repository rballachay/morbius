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

# this is necessary for forwarding graphics over ssh
sudo apt-get install xorg openbox -y

pip install -r requirements.txt

# compile libfvad to be used in voice detection for record_audio.so
git submodule update --init submodules/libfvad/
cd submodules/libfvad/
autoreconf -i
./configure
make
sudo make install
cd ../../

# compile the actual file
cd src/language
gcc -shared -o record_audio.so -fPIC record_audio.c  -lportaudio -lfvad 
cd ../../

# compile the planeSegment
cd src/vision/rgbdSeg
sudo bash build.sh --include-deps
mv planeSegment ../../../
cd ../../../

# need to install Pangolin as an ORBSLAM dependency
cd ../
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git 
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build
cd ../morbius

# compile and prepare ORBSLAM
git submodule update --init submodules/ORB_SLAM3/
cd submodules/ORB_SLAM3
git checkout master 
git reset --hard origin/master
git pull
chmod +x build.sh
sed -i 's/++11/++14/g' CMakeLists.txt
./build.sh
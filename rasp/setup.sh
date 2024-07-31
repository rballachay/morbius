#!/bin/bash

# This is a setup script for raspberry pi 5. It will not work on a mac, windows,
# or the majority of linux machines, as it is installing for aarch64 

# Function to display usage
usage() {
  echo "Usage: $0 [--miniconda] [--audio] [--vision] [--slam]"
  exit 1
}

if [ $# -eq 0 ]; then
  usage
fi

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --miniconda)
      MINICONDA=true
      ;;
    --audio)
      AUDIO=true
      ;;
    --vision)
      VISION=true
      ;;
    --slam)
      SLAM=true
      ;;
    *)
      usage
      ;;
  esac
  shift
done

# Phase 1: Download and install Miniconda
if [ "$MINICONDA" = true ]; then
  echo "Starting Miniconda installation..."
  sudo apt-get update

  # download  
  wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh

  # allow exec
  chmod +x Miniconda3-latest-Linux-aarch64.sh

  # run install headless
  bash Miniconda3-latest-Linux-aarch64.sh -b -p $HOME/miniconda3

  # Initialize Miniconda
  $HOME/miniconda3/bin/conda init

  # Source the conda initialization script
  source $HOME/miniconda3/etc/profile.d/conda.sh

  # Verify the installation
  conda --version

  # create new conda environment for everything
  conda create -n python310 python=3.10 -y

  echo "Miniconda installation complete."
fi

# Phase 2: Install audio system
if [ "$AUDIO" = true ]; then
  $HOME/miniconda3/bin/conda init

  if [ $? -ne 0 ]; then
    echo "Failed to initialize conda. ensure to install miniconda first"
    exit 1
  fi

  source $HOME/miniconda3/etc/profile.d/conda.sh
  conda activate python310
  if [ $? -ne 0 ]; then
    echo "Failed to activate conda environment. Ensure the environment 'python310' exists."
    exit 1
  fi

  echo "Starting audio installation and dependencies installation..."

  cd language/

  # install all other apt packages
  sudo apt-get install gcc -y
  sudo apt-get install portaudio19-dev espeak automake libtool -y
  sudo apt-get install libhdf5-dev -y
  sudo apt install libpulse-dev pulseaudio apulse

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
  cd src/stt
  gcc -shared -o record_audio.so -fPIC record_audio.c  -lportaudio -lfvad 
  cd ../../

  echo "Finished installation of audio system"
fi

# Phase 3: Compile libraries and projects
if [ "$VISION" = true ]; then
  pwd
  echo "Starting installation and compilation of vision system"
  # compile the planeSegment
  git config --global --add safe.directory /home/user/morbius/vision/submodules/librealsense
  git submodule update --init vision/submodules/librealsense
  cd vision/src/rgbdSeg
  sudo bash build.sh --include-deps
  mv planeSegment ../
  cd ../../../
  echo "Finished install of vision system"
fi

# Phase 4: Compile slam
if [ "$SLAM" = true ]; then
  sudo apt-get install -y cmake
  # need to install Pangolin as an ORBSLAM dependency
  cd ../
  git clone --recursive https://github.com/stevenlovegrove/Pangolin.git 
  cd Pangolin
  ./scripts/install_prerequisites.sh recommended
  cmake -B build
  cmake --build build
  cd ../morbius

  # compile and prepare ORBSLAM
  git config --global --add safe.directory /home/user/morbius/vision/submodules/ORB_SLAM3
  git submodule update --init vision/submodules/ORB_SLAM3
  cd vision/submodules/ORB_SLAM3
  git checkout master 
  git reset --hard origin/master
  git pull
  chmod +x build.sh
  sed -i 's/++11/++14/g' CMakeLists.txt
  ./build.sh
  cd ../../
  bash ./build.sh
  echo "Compilation of slam system complete"
fi

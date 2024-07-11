#!/bin/sh

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check if OpenCV is installed
check_opencv() {
    if command_exists brew; then
        if brew list opencv@4 &>/dev/null; then
            echo "OpenCV is installed."
        else
            brew install opencv
        fi
    elif command_exists pkg-config; then
        if pkg-config --exists opencv4; then
            echo "OpenCV is installed."
        else
            apt-get install -y libopencv-dev
        fi
    else
        echo "Neither pkg-config nor brew found. Cannot check OpenCV installation."
    fi
}

# Function to check if librealsense (realsense2) is installed
check_realsense2() {
    if command_exists brew; then
        if brew list librealsense &>/dev/null; then
            echo "librealsense (realsense2) is installed."
        else
            brew install librealsense
        fi
    elif command_exists pkg-config; then
        if pkg-config --exists librealsense; then
            echo "librealsense (realsense2) is installed."
        else
            apt-get install xorg-dev -y
            apt-get install libusb-1.0-0-dev -y
            git clone https://github.com/IntelRealSense/librealsense
            cd librealsense
            mkdir  build  && cd build
            cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
            make -j1
            sudo make install
            cd ../../
        fi
    else
        echo "Neither pkg-config nor brew found. Cannot check librealsense (realsense2) installation."
    fi
}

# Function to check if Eigen3 is installed
check_eigen3() {
    if command_exists brew; then
        if brew list eigen &>/dev/null; then
            echo "Eigen3 is installed."
        else
            brew install eigen
        fi
    elif command_exists pkg-config; then
        if pkg-config --exists eigen3; then
            echo "Eigen3 is installed."
        else
            apt-get install libeigen3-dev -y
        fi
    else
        echo "Neither pkg-config nor brew found. Cannot check Eigen3 installation."
    fi
}

check_cmake() {
    if command_exists brew; then
        if brew list cmake &>/dev/null; then
            echo "Cmake is installed."
        else
            brew install cmake
        fi
    elif command_exists pkg-config; then
        if pkg-config --exists eigen3; then
            echo "cmake is installed."
        else
            apt-get install cmake-curses-gui -y
        fi
    else
        echo "Neither pkg-config nor brew found. Cannot check cmake installation."
    fi
}

check_pcl() {
    if command_exists brew; then
        if brew list pcl &>/dev/null; then
            echo "Cmake is installed."
        else
            brew install pcl
        fi
    elif command_exists pkg-config; then
        if pkg-config --exists pcl; then
            echo "cmake is installed."
        else
            apt-get install libpcl-dev -y
        fi
    else
        echo "Neither pkg-config nor brew found. Cannot check cmake installation."
    fi
}

INSTALL_DEPS=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --include-deps)
            INSTALL_DEPS=true
            shift
            ;;
        *)
            echo "Invalid option: $1"
            exit 1
            ;;
    esac
done

if $INSTALL_DEPS; then
    check_cmake
    check_eigen3
    check_opencv
    check_realsense2
fi

cd include/MRF2.2
make
cd ../../
mkdir build
cd build
cmake ..
make
mv planeSegment ../
cd ..

echo "Built executable: planeSegment, must run with sudo"
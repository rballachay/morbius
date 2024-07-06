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
        if pkg-config --exists librealsense2; then
            echo "librealsense (realsense2) is installed."
        else
            apt-get install librealsense
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
            apt-get install eigen
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
            apt-get install cmake-curses-gui
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
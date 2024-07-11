cmake_minimum_required(VERSION 3.1)
project(planeSegment)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE "Release")

# Find required packages
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(jsoncpp REQUIRED)  # Use find_package to find JsonCpp

# Add include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include/MRF2.2  # Add any necessary include directories
    include/plane
)

# Add definitions and link directories
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Gather source files
file(GLOB sources "*.cpp" "include/plane/plane_detection.cpp")

# Create executable
add_executable(planeSegment ${sources})

# Link libraries
target_link_libraries(planeSegment 
    ${OpenCV_LIBS} 
    ${realsense2_LIBRARY} 
    ${EIGEN3_LIBRARIES} 
    ${PCL_LIBRARIES} 
    jsoncpp_lib
)

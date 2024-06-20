cmake_minimum_required(VERSION 3.10)
project(MyCombinedProject)

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find Librealsense2
find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIR})

# Find ONNX Runtime
find_package(onnxruntime REQUIRED)
include_directories(${onnxruntime_INCLUDE_DIRS})

# Add executable
add_executable(MyCombinedProject realsense-example.cpp)

# Link libraries
target_link_libraries(MyCombinedProject ${OpenCV_LIBS} realsense2::realsense2 onnxruntime::onnxruntime)

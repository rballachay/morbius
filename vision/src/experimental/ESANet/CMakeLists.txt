cmake_minimum_required (VERSION 3.1)

project (esaNet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE "Release")

# put FindGLFW.cmake in the source directory of CMakeLists.txt
find_package(OpenCV 4.9.0 REQUIRED)
find_package(realsense2 REQUIRED)

set(onnxruntime_INCLUDE_DIR /usr/local/include/onnxruntime)
set(onnxruntime_LIB_DIR /usr/local/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${realsense2_INCLUDE_DIR})
include_directories(${onnxruntime_INCLUDE_DIR})
link_directories(${realsense2_LIB_DIR})
link_directories(${onnxruntime_LIB_DIR})

file(GLOB sources "*.cpp")

add_executable(esaNet ${sources})

include_directories(include ${realsense_INCLUDE_DIR})
target_include_directories(esaNet PRIVATE ${realsense_INCLUDE_DIR} ${onnxruntime_INCLUDE_DIR})
target_link_libraries(esaNet ${OpenCV_LIBS} ${realsense2_LIBRARY}  ${onnxruntime_LIB_DIR}/libonnxruntime.dylib)
cmake_minimum_required (VERSION 3.1)

project (planeSegment)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE "Release")

# put FindGLFW.cmake in the source directory of CMakeLists.txt
find_package(OpenCV 4.9.0 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} /usr/local/include/eigen3)
include_directories(${realsense2_INCLUDE_DIR})
include_directories(include/peac)
include_directories(include/MRF2.2)

link_directories(include/MRF2.2)
link_directories(${realsense2_LIB_DIR})

file(GLOB sources "*.cpp")

add_executable(planeSegment ${sources})

include_directories(include ${realsense_INCLUDE_DIR})
target_include_directories(planeSegment PRIVATE ${realsense_INCLUDE_DIR})
target_link_libraries(planeSegment ${OpenCV_LIBS} libMRF.a ${realsense2_LIBRARY})
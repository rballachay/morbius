cmake_minimum_required(VERSION 3.1)
project(VisionProject)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the source file
set(SOURCES vision.cc)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Pangolin REQUIRED)

# Add the include directories
include_directories(${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${realsense2_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/include
${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/include/CameraModels
${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/Thirdparty/Sophus
${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3)

# Add the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
add_executable(vision ${SOURCES})

# Link libraries
target_link_libraries(vision
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${realsense2_LIBRARY}
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.dylib
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.dylib
    /usr/local/Cellar/boost/1.85.0/lib/libboost_serialization.dylib
    /usr/local/opt/openssl/lib/libcrypto.dylib
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/lib/libORB_SLAM3.dylib
)
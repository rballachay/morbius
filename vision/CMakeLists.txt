cmake_minimum_required(VERSION 3.1)
project(VisionProject)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the source file
set(SOURCES vision.cc ${PROJECT_SOURCE_DIR}/src/vision/rgbdSeg/include/plane/plane_detection.cpp)

# Check if JsonCpp target does not exist and then create it manually
if(NOT TARGET JsonCpp::JsonCpp)
  # Attempt to find JsonCpp library - modify paths as needed
  find_library(JSONCPP_LIBRARY NAMES jsoncpp PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/lib /usr/local/lib)
  find_path(JSONCPP_INCLUDE_DIR NAMES json/json.h PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/include /usr/local/include)

  # Create an imported target
  add_library(JsonCpp::JsonCpp SHARED IMPORTED)
  set_target_properties(JsonCpp::JsonCpp PROPERTIES
    IMPORTED_LOCATION "${JSONCPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${JSONCPP_INCLUDE_DIR}")
endif()


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)

# Add the include directories
include_directories(${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${realsense2_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/include
${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/include/CameraModels
${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/Thirdparty/Sophus
${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3
${PROJECT_SOURCE_DIR}/src/vision/rgbdSeg
${PROJECT_SOURCE_DIR}/src/vision/rgbdSeg/include/plane
${PROJECT_SOURCE_DIR}/src/vision/rgbdSeg/include/MRF2.2
)

# Add definitions and link directories
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Add the executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
add_executable(vision ${SOURCES})

# Link libraries
target_link_libraries(vision
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${realsense2_LIBRARY}
    ${PCL_LIBRARIES} 
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.dylib
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.dylib
    /usr/local/Cellar/boost/1.85.0/lib/libboost_serialization.dylib
    /usr/local/opt/openssl/lib/libcrypto.dylib
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/lib/libORB_SLAM3.dylib
)
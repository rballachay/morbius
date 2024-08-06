cmake_minimum_required(VERSION 3.1)
project(visionServer)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the source files
set(SOURCES
    visionServer.cpp 
    ${PROJECT_SOURCE_DIR}/src/rgbdSeg/include/plane/plane_detection.cpp 
)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(Pistache REQUIRED IMPORTED_TARGET libpistache)

# JsonCpp - Create an imported target if necessary
if(NOT TARGET JsonCpp::JsonCpp)
    find_library(JSONCPP_LIBRARY NAMES jsoncpp PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/lib /usr/local/lib)
    find_path(JSONCPP_INCLUDE_DIR NAMES json/json.h PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/include /usr/local/include)
    add_library(JsonCpp::JsonCpp SHARED IMPORTED)
    set_target_properties(JsonCpp::JsonCpp PROPERTIES
        IMPORTED_LOCATION "${JSONCPP_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${JSONCPP_INCLUDE_DIR}")
endif()

# Include directories
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/include
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/include/CameraModels
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/Thirdparty/Sophus
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3
    ${PROJECT_SOURCE_DIR}/src/rgbdSeg
    ${PROJECT_SOURCE_DIR}/src/rgbdSeg/include/plane
    ${PROJECT_SOURCE_DIR}/src/rgbdSeg/include/MRF2.2
)

# Add the executable
add_executable(${PROJECT_NAME} ${SOURCES})

if(APPLE)
  set(LIB_EXT "dylib")
else()
  set(LIB_EXT "so")
endif()

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${realsense2_LIBRARY}
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    PkgConfig::Pistache
    JsonCpp::JsonCpp
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.${LIB_EXT}
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.${LIB_EXT}
    ${PROJECT_SOURCE_DIR}/submodules/ORB_SLAM3/lib/libORB_SLAM3.${LIB_EXT}
)

# Set runtime output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

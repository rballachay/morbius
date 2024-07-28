cmake_minimum_required(VERSION 3.1)
project(planeSegment)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE "Release")

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

# Find required packages
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)

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
)

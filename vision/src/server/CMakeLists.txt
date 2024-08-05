cmake_minimum_required(VERSION 3.6)
set(CMAKE_CXX_STANDARD 17)
project("MyPistacheProject")

find_package(PkgConfig)
pkg_check_modules(Pistache REQUIRED IMPORTED_TARGET libpistache)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PkgConfig::Pistache)
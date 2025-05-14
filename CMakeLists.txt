cmake_minimum_required(VERSION 3.10)
project(ExtendedKalmanFilter)

set(CMAKE_CXX_STANDARD 14)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add the executable
add_executable(ekf_demo main.cpp ekf.cpp)

# Install target
install(TARGETS ekf_demo DESTINATION bin)
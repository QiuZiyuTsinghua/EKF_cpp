cmake_minimum_required(VERSION 3.10)
project(ExtendedKalmanFilter)

set(CMAKE_CXX_STANDARD 14)

# Option to build shared libraries
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(BUILD_EXAMPLES "Build examples" OFF)

# Make sure CMAKE_INSTALL_PREFIX is set properly
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Default install path" FORCE)
endif()

# Platform-specific settings
if(WIN32)
  # Add Windows-specific compiler flags if needed
  add_definitions(-DEKF_EXPORTS)
  
  # Set runtime library for Windows
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# Handle Eigen3 dependency
if(DEFINED EIGEN3_INCLUDE_DIR)
  # Use the manually specified Eigen3 directory
  message(STATUS "Using provided Eigen3 at: ${EIGEN3_INCLUDE_DIR}")
  include_directories(${EIGEN3_INCLUDE_DIR})
else()
  # Try to find Eigen3 via find_package
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# Create EKF library
add_library(ekf ekf.cpp)
target_include_directories(ekf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Add the demo executable
add_executable(ekf_demo main.cpp)
target_link_libraries(ekf_demo ekf)

# Install targets
install(TARGETS ekf ekf_demo
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(FILES ekf.h DESTINATION include)

# Build Simulink S-Function example if requested
if(BUILD_EXAMPLES)
    # Create simulink directory if it doesn't exist
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/examples/simulink)
    
    # Create S-Function if it doesn't exist yet
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/simulink/ekf_sfun.cpp)
        configure_file(
            ${CMAKE_CURRENT_SOURCE_DIR}/examples/simulink/ekf_sfun_template.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/examples/simulink/ekf_sfun.cpp
            COPYONLY)
    endif()
endif()

# Print some diagnostic information
message(STATUS "Build directory: ${CMAKE_BINARY_DIR}")
message(STATUS "Source directory: ${CMAKE_SOURCE_DIR}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Shared libs: ${BUILD_SHARED_LIBS}")
message(STATUS "Building on Windows: ${WIN32}")
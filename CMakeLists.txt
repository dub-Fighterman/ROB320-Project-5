cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 20)

project(inverse-kinematics)

# Used to find GTest on CAEN
list(APPEND CMAKE_PREFIX_PATH "~/.local")

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

# Used to compile on Autograder (possibly unnecessary ...)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT DEFINED ARCH)
    # If we do not pre-define our architecture, run this command to infer it
    EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH)
    # Normalize name
    if(ARCH STREQUAL "arm64")
        set(ARCH "aarch64")
    endif()
endif()

if(NOT DEFINED OS)
    # If we do not pre-define our operating system, infer it
    if(APPLE)
        set(OS "macos")
    elseif(UNIX)
        set(OS "linux")
    else()
        message(FATAL_ERROR "Unsupported OS.")
    endif()
endif()

message(STATUS "Architecture: ${ARCH}")
message(STATUS "Operating System: ${OS}")

# Compile the Project 5 library
add_library(project5 
    src/rix/robot/kinematics_solver.cpp
)
target_include_directories(project5 PRIVATE include/)
target_link_libraries(project5 PUBLIC Threads::Threads Eigen3::Eigen nlohmann_json::nlohmann_json)

# Link against the proper Project 4 library
target_link_libraries(project5 PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/${OS}-${ARCH}/libproject5_core.a
    ${CMAKE_SOURCE_DIR}/lib/${OS}-${ARCH}/libproject4.a
    ${CMAKE_SOURCE_DIR}/lib/${OS}-${ARCH}/libproject3.a
    ${CMAKE_SOURCE_DIR}/lib/${OS}-${ARCH}/libproject2.a
    ${CMAKE_SOURCE_DIR}/lib/${OS}-${ARCH}/libproject1.a
)

# Add kinematics_solver test
add_executable(generated_kinematics_tests tests/generated_kinematics_tests.cpp)
target_include_directories(generated_kinematics_tests PRIVATE include/)
target_link_libraries(generated_kinematics_tests project5 GTest::Main)

add_executable(trajectory_service src/trajectory_service/trajectory_service.cpp src/trajectory_service/main.cpp)
target_include_directories(trajectory_service PRIVATE include/)
target_link_libraries(trajectory_service project5)

add_executable(setpoint_controller src/setpoint_controller/setpoint_controller.cpp src/setpoint_controller/main.cpp)
target_include_directories(setpoint_controller PRIVATE include/)
target_link_libraries(setpoint_controller project5)

add_executable(rixhub src/rixhub/main.cpp)
target_include_directories(rixhub PRIVATE include/)
target_link_libraries(rixhub project5)
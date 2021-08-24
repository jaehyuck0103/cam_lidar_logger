cmake_minimum_required(VERSION 3.15)
project(cam_lidar_logger)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_VERBOSE_MAKEFILE true)

# Fetch Contents
include(FetchContent)
FetchContent_Declare(
  toml11
  GIT_REPOSITORY https://github.com/ToruNiina/toml11.git
  GIT_TAG v3.7.0)
FetchContent_MakeAvailable(toml11)

add_subdirectory(third_party/HesaiLidar_General_SDK)

# Add Executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE toml11::toml11 PandarSDK tbb)

# Add Custom Target
add_custom_target(
  runMain
  COMMAND cam_lidar_logger
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

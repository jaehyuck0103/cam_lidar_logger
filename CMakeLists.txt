cmake_minimum_required(VERSION 3.15)
project(cam_lidar_logger)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_VERBOSE_MAKEFILE true)

include(FetchContent)
# Fetch toml11
FetchContent_Declare(
  toml11
  GIT_REPOSITORY https://github.com/ToruNiina/toml11.git
  GIT_TAG v3.7.0)
FetchContent_MakeAvailable(toml11)

# Fetch fmt
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 8.0.1)
FetchContent_MakeAvailable(fmt)

find_package(OpenCV REQUIRED)

add_subdirectory(third_party/HesaiLidar_General_SDK)

# Add Executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE toml11::toml11 fmt::fmt PandarSDK
                                              tbb ${OpenCV_LIBS})
if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
  target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
  target_compile_definitions(${PROJECT_NAME} PRIVATE OLD_COMPILER)
endif()

# Add Custom Target
add_custom_target(
  runMain
  COMMAND cam_lidar_logger
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

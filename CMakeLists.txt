cmake_minimum_required(VERSION 3.25)

project(camera_imu_calibrate)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -Wpedantic")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(thirdparty/matplotplusplus)
add_subdirectory(thirdparty/argparse)
add_subdirectory(thirdparty/rapidcsv)

file(GLOB_RECURSE SRC_FILES src/*.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE 
    Eigen3::Eigen 
    matplot 
    argparse
    rapidcsv
    ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME} PRIVATE 
    include
    ${OpenCV_INCLUDE_DIRS}
)
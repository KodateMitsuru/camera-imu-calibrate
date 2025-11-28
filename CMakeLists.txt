cmake_minimum_required(VERSION 3.28)
project(camera_imu_calibrate)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 23)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -flto=auto -march=native")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(FFTW REQUIRED fftw3 IMPORTED_TARGET)
pkg_search_module(Magick++ REQUIRED Magick++ IMPORTED_TARGET)

# Disable tests in third-party libraries
set(BUILD_TESTING OFF CACHE BOOL "Disable third-party tests" FORCE)

add_subdirectory(thirdparty/argparse)
add_subdirectory(thirdparty/matplotplusplus)
add_subdirectory(thirdparty/lunasvg)

# Schweizer-Messer libraries
add_subdirectory(thirdparty/Schweizer-Messer/sm_common)
add_subdirectory(thirdparty/Schweizer-Messer/sm_random)
add_subdirectory(thirdparty/Schweizer-Messer/sm_boost)
add_subdirectory(thirdparty/Schweizer-Messer/sm_eigen)
add_subdirectory(thirdparty/Schweizer-Messer/sm_timing)
add_subdirectory(thirdparty/Schweizer-Messer/sm_logging)
add_subdirectory(thirdparty/Schweizer-Messer/sm_kinematics)
add_subdirectory(thirdparty/Schweizer-Messer/sm_opencv)
add_subdirectory(thirdparty/Schweizer-Messer/sm_property_tree)

# Create namespace aliases for Schweizer-Messer libraries
add_library(sm_common::sm_common ALIAS sm_common)
add_library(sm_random::sm_random ALIAS sm_random)
add_library(sm_boost::sm_boost ALIAS sm_boost)
add_library(sm_eigen::sm_eigen ALIAS sm_eigen)
add_library(sm_timing::sm_timing ALIAS sm_timing)
add_library(sm_logging::sm_logging ALIAS sm_logging)
add_library(sm_kinematics::sm_kinematics ALIAS sm_kinematics)
add_library(sm_opencv::sm_opencv ALIAS sm_opencv)
add_library(sm_property_tree::sm_property_tree ALIAS sm_property_tree)

# aslam_optimizer
add_subdirectory(thirdparty/aslam_optimizer/sparse_block_matrix)
add_subdirectory(thirdparty/aslam_optimizer/aslam_backend)
add_subdirectory(thirdparty/aslam_optimizer/aslam_backend_expressions)

add_library(sparse_block_matrix::sparse_block_matrix ALIAS sparse_block_matrix)
add_library(aslam_backend::aslam_backend ALIAS aslam_backend)
add_library(aslam_backend_expressions::aslam_backend_expressions ALIAS aslam_backend_expressions)

# aslam_nonparametric_estimation
add_subdirectory(thirdparty/aslam_nonparametric_estimation/bsplines)
add_subdirectory(thirdparty/aslam_nonparametric_estimation/aslam_splines)

add_library(bsplines::bsplines ALIAS bsplines)
add_library(aslam_splines::aslam_splines ALIAS aslam_splines)

# aslam_incremental_calibration
add_subdirectory(thirdparty/aslam_incremental_calibration/incremental_calibration)

add_library(incremental_calibration::incremental_calibration ALIAS incremental_calibration)

# aslam_offline_calibration
add_subdirectory(thirdparty/aslam_offline_calibration/ethz_apriltag2)

add_library(ethz_apriltag2::ethz_apriltag2 ALIAS ethz_apriltag2)

# aslam_cv
add_subdirectory(thirdparty/aslam_cv/aslam_time)
add_subdirectory(thirdparty/aslam_cv/aslam_cameras)
add_subdirectory(thirdparty/aslam_cv/aslam_cameras_april)
add_subdirectory(thirdparty/aslam_cv/aslam_cv_backend)
add_subdirectory(thirdparty/aslam_cv/aslam_cv_error_terms)

add_library(aslam_time::aslam_time ALIAS aslam_time)
add_library(aslam_cameras::aslam_cameras ALIAS aslam_cameras)
add_library(aslam_cameras_april::aslam_cameras_april ALIAS aslam_cameras_april)
add_library(aslam_cv_backend::aslam_cv_backend ALIAS aslam_cv_backend)
add_library(aslam_cv_error_terms::aslam_cv_error_terms ALIAS aslam_cv_error_terms)



file(GLOB_RECURSE SRC_FILES 
    src/*.cpp 
    src/kalibr_imu_camera_calibration/*.cpp
    src/kalibr_errorterms/*.cpp
    src/kalibr_common/*.cpp
    src/kalibr_backend/*.cpp
    src/kalibr_camera_calibration/*.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE
    OpenMP::OpenMP_CXX
    Eigen3::Eigen
    Ceres::ceres
    PkgConfig::FFTW
    PkgConfig::Magick++
    yaml-cpp
    argparse
    matplot
    lunasvg
    aslam_backend::aslam_backend
    aslam_splines::aslam_splines
    incremental_calibration::incremental_calibration
    aslam_cameras::aslam_cameras
    aslam_cameras_april::aslam_cameras_april
    aslam_cv_backend::aslam_cv_backend
    aslam_time::aslam_time
    aslam_cv_error_terms::aslam_cv_error_terms
    sm_kinematics::sm_kinematics
    sm_timing::sm_timing
    sm_logging::sm_logging
    ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include
    PkgConfig::FFTW
    ${OpenCV_INCLUDE_DIRS}
)

# Re-enable testing for our own tests only
set(BUILD_TESTING ON CACHE BOOL "Enable our own tests" FORCE)
enable_testing()

add_subdirectory(tests)


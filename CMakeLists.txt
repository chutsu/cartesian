cmake_minimum_required(VERSION 3.0.0)
project(cartesian)

###############################################################################
# COMPILATION SETTINGS
###############################################################################
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(USE_ADDRESS_SANITIZER OFF)
add_compile_options(-Wall -Wextra)

# Address Sanitizer
if (USE_ADDRESS_SANITIZER)
  message(AUTHOR_WARNING "Building with ASAN - Executables will be slower!")
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  # link_libraries(-fsanitize=address -static-libasan) # g++
  link_libraries(-fsanitize=address -static-libsan) # clang
endif()

###############################################################################
# DEPENDENCIES
###############################################################################

# Clang emits __kmpc_* symbols with -fopenmp; link libomp to resolve them
find_package(OpenMP REQUIRED)
find_library(OMP_LIB omp PATHS /usr/lib/llvm-14/lib /usr/lib/x86_64-linux-gnu)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)
find_package(rerun_sdk REQUIRED)

set(DEPS
    yaml-cpp::yaml-cpp
    rerun_sdk
    Eigen3::Eigen
    ${OpenCV_LIBS}
    apriltag::apriltag
    ethz_apriltag
    Ceres::ceres
    ${PCL_LIBRARIES}
    ${OMP_LIB})

###############################################################################
# BUILD LIBRARY
###############################################################################

link_directories(${CMAKE_PREFIX_PATH}/lib)
include_directories(lib)
add_library(
  ${PROJECT_NAME}
  STATIC
  # calib
  lib/calib/AprilGrid.cpp
  lib/calib/AprilGridConfig.cpp
  lib/calib/AprilGridDetector.cpp
  lib/calib/CalibCamera.cpp
  lib/calib/CalibCameraImu.cpp
  lib/calib/CalibInit.cpp
  lib/calib/CalibProblem.cpp
  lib/calib/CalibTarget.cpp
  lib/calib/CalibTargetChain.cpp
  lib/calib/CalibTargetGeometry.cpp
  lib/calib/CameraChain.cpp
  lib/calib/LissajousTrajectory.cpp
  lib/calib/SolvePnp.cpp
  # camera
  lib/camera/BrownConrady4.cpp
  lib/camera/CameraGeometry.cpp
  lib/camera/KannalaBrandt4.cpp
  lib/camera/Pinhole.cpp
  # ceres
  lib/ceres/ImuError.cpp
  lib/ceres/ParamBlock.cpp
  lib/ceres/PoseManifold.cpp
  lib/ceres/ReprojError.cpp
  lib/ceres/CalibCameraError.cpp
  lib/ceres/CalibCameraImuError.cpp
  lib/ceres/CalibCameraImuError.cpp
  lib/ceres/ResidualBlock.cpp
  # core
  lib/core/Colormap.cpp
  lib/core/Core.cpp
  lib/core/Logger.cpp
  lib/core/Plane.cpp
  lib/core/SO3.cpp
  # cv
  lib/cv/Feature.cpp
  lib/cv/FeatureGrid.cpp
  lib/cv/GridDetector.cpp
  lib/cv/Ransac.cpp
  lib/cv/Sfm.cpp
  # dataset
  lib/dataset/Euroc.cpp
  lib/dataset/Kitti.cpp
  # imu
  lib/imu/ImuBuffer.cpp
  lib/imu/ImuGeometry.cpp
  lib/imu/ImuPreintegrate.cpp
  lib/imu/ImuState.cpp
  # sim
  lib/sim/SimCalib.cpp
  lib/sim/SimCircle.cpp
  lib/sim/SimImu.cpp
  # lib/sim/SimVio.cpp
  # timeline
  lib/timeline/CalibTargetEvent.cpp
  lib/timeline/CameraEvent.cpp
  lib/timeline/ImuEvent.cpp
  lib/timeline/Timeline.cpp
  lib/timeline/TimelineEvent.cpp
)
target_link_libraries(${PROJECT_NAME} ${DEPS})

###############################################################################
# Build UNIT-TESTS
###############################################################################

set(TEST_DEPS GTest::gtest_main ${PROJECT_NAME} ${DEPS})
set(TEST_DATA ${CMAKE_CURRENT_LIST_DIR}/lib/test_data)

add_executable(unittests
  # calib
  lib/calib/TestAprilGrid.cpp
  lib/calib/TestAprilGridDetector.cpp
  lib/calib/TestCalibCamera.cpp
  lib/calib/TestCalibCameraImu.cpp
  lib/calib/TestCalibCameraLidar.cpp
  lib/calib/TestCalibInit.cpp
  lib/calib/TestCalibProblem.cpp
  lib/calib/TestCameraChain.cpp
  lib/calib/TestLissajousTrajectory.cpp
  # camera
  lib/camera/TestBrownConrady4.cpp
  lib/camera/TestKannalaBrandt4.cpp
  lib/camera/TestPinhole.cpp
  # cv
  lib/cv/TestFeatureGrid.cpp
  lib/cv/TestGridDetector.cpp
  lib/cv/TestSfm.cpp
  # dataset
  lib/dataset/TestEuroc.cpp
  lib/dataset/TestKitti.cpp
  # imu
  lib/imu/TestImuBuffer.cpp
  # ceres
  lib/ceres/TestCalibCameraError.cpp
  lib/ceres/TestCalibCameraImuError.cpp
  lib/ceres/TestCalibCameraImuError.cpp
  lib/ceres/TestImuError.cpp
  lib/ceres/TestReprojError.cpp
  # core
  lib/core/TestLogger.cpp
  lib/core/TestPlane.cpp
  # sim
  lib/sim/TestSimCalib.cpp
  lib/sim/TestSimCircle.cpp
  lib/sim/TestSimImu.cpp
  lib/sim/TestSimVio.cpp
  # timeline
  lib/timeline/TestTimeline.cpp
)
add_compile_definitions(unittests PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(unittests ${TEST_DEPS})

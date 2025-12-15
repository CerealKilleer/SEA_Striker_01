# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(ENCODER_CALIBRATION n)
set(ESCS_CALIBRATION n)
set(TEST_WHEELS n)
set(TEST_IMU n)
set(TEST_VEL_WHEEL n)
set(TEST_FORWARD_KINEMATICS n)

if(ENCODER_CALIBRATION)
        project(encoder_calibration)
elseif (ESCS_CALIBRATION)
        project(escs_calibration)
elseif(TEST_WHEELS)
        project(test_wheels)
elseif (TEST_IMU)
        project(test_imu)
elseif (TEST_VEL_WHEEL)
        project(test_vel_wheels)
elseif (TEST_FORWARD_KINEMATICS)
        project(test_forward_kinematics)
else()
        project(robot-mov)
endif()


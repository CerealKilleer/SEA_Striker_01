# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(ENCODER_CALIBRATION n)
set(ESCS_CALIBRATION n)

if(ENCODER_CALIBRATION)
        project(encoder_calibration)
elseif (ESCS_CALIBRATION)
        project(escs_calibration)
else()
        project(robot-mov)
endif()


file(GLOB SRCS_AS5600  "../sensors/AS5600_HAL_Library/*.c")
file(GLOB SRCS_TM151  "../sensors/TM151-driver/*.c")
file(GLOB SRCS_VL53L1X  "../sensors/VL53L1X-driver/*.c")

file(GLOB SRCS_PLATFORM  "../platform/*.c")
file(GLOB SRCS_BLDC_PWM  "../drivers/bldc_pwm/*.c")
file(GLOB SRCS_PID_LIB  "../drivers/pid_lib/*.c")
file(GLOB SRCS_SENSOR_FUSION "../drivers/sensor_fusion/*.c")
file(GLOB SRCS_WIFI_LIB "../drivers/wifi_lib/*.c")


if(ENCODER_CALIBRATION)
        idf_component_register(
                SRCS "control_main.c" "main.c" "test_forward_kinematics.c" "test_IMU.c" "kalman_filter_1d.c" "encoder_calibration.c" ${SRCS_AS5600} ${SRCS_PLATFORM}
                INCLUDE_DIRS "." "../sensors/AS5600_HAL_Library/include" "../platform/include"
        )
elseif (ESCS_CALIBRATION)
        idf_component_register(
                SRCS "escs_calibration.c" ${SRCS_BLDC_PWM} ${SRCS_PLATFORM}
                INCLUDE_DIRS "." "../drivers/bldc_pwm/include" "../platform/include"
        )
elseif (TEST_WHEELS)
        idf_component_register(
                idf_component_register(
                SRCS "test_wheels.c" ${SRCS_AS5600} ${SRCS_TM151} ${SRCS_VL53L1X} ${SRCS_PLATFORM} ${SRCS_BLDC_PWM} ${SRCS_PID_LIB} ${SRCS_SENSOR_FUSION} ${SRCS_WIFI_LIB}
                INCLUDE_DIRS "." "../sensors/AS5600_HAL_Library/include" "../sensors/TM151-driver/include" "../sensors/VL53L1X-driver/include"
                    "../platform/include" "../drivers/bldc_pwm/include" "../drivers/pid_lib/include" "../drivers/sensor_fusion/include"
                    "../drivers/wifi_lib/include" "./include"
                PRIV_REQUIRES esp_wifi nvs_flash driver esp_adc esp_timer
        )
        )
elseif (TEST_IMU)
        idf_component_register(
                SRCS "test_IMU.c" "kalman_filter_1d.c" ${SRCS_TM151} ${SRCS_PLATFORM}
                INCLUDE_DIRS "." "../sensors/TM151-driver/include" "../platform/include" "./include"
        )
elseif (TEST_VEL_WHEEL)
        idf_component_register(
                SRCS "test_velocity_wheel.c" "kalman_filter_1d.c" ${SRCS_BLDC_PWM} ${SRCS_AS5600} ${SRCS_SENSOR_FUSION} ${SRCS_PID_LIB} ${SRCS_PLATFORM}
                INCLUDE_DIRS "." "../sensors/AS5600_HAL_Library/include" "../drivers/bldc_pwm/include" "../platform/include" "./include"
                        "../drivers/sensor_fusion/include" "../drivers/pid_lib/include"
                PRIV_REQUIRES esp_adc esp_timer
        )

elseif (TEST_FORWARD_KINEMATICS)
        idf_component_register(
                SRCS "test_forward_kinematics.c" "kalman_filter_1d.c" ${SRCS_AS5600} ${SRCS_BLDC_PWM} ${SRCS_PID_LIB} ${SRCS_PLATFORM} ${SRCS_SENSOR_FUSION}
                INCLUDE_DIRS "." "../sensors/AS5600_HAL_Library/include" "../drivers/bldc_pwm/include" "../platform/include" "./include"
                        "../drivers/pid_lib/include" "../drivers/sensor_fusion/include"
                PRIV_REQUIRES esp_adc esp_timer
        )
else()
        idf_component_register(
                SRCS "main.c" "control_main.c" "mov_calculation.c"  "kalman_filter_1d.c"  ${SRCS_AS5600} ${SRCS_TM151} ${SRCS_VL53L1X} ${SRCS_PLATFORM} ${SRCS_BLDC_PWM} ${SRCS_PID_LIB} ${SRCS_SENSOR_FUSION} ${SRCS_WIFI_LIB}
                INCLUDE_DIRS "." "../sensors/AS5600_HAL_Library/include" "../sensors/TM151-driver/include" "../sensors/VL53L1X-driver/include"
                    "../platform/include" "../drivers/bldc_pwm/include" "../drivers/pid_lib/include" "../drivers/sensor_fusion/include"
                    "../drivers/wifi_lib/include" "./include"
                PRIV_REQUIRES esp_wifi nvs_flash driver esp_adc esp_timer esp_http_server
        )
endif()


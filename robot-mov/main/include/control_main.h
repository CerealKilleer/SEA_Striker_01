#ifndef CONTROL_H
#define CONTROL_H

// Include standar libraries 
#include <stdio.h>

// Include personalized sensors libraries
#include "as5600_lib.h"
#include "VL53L1X.h"
#include "EasyRetrieve.h"

// Include personalized driver libraries
#include "bldc_pwm.h"
#include "pid_ext.h"
#include "sensor_fusion.h"
#include "mov_calculation.h"
#include "wifi_lib.h"

// Include ESP IDF libraries
#include <assert.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gptimer.h"
#include "esp_http_server.h"

#define TASK_IMU_PERIOD_MS 10 ///< IMU task period in milliseconds
#define SAMPLE_TIME 4 ///< Sample time in ms
#define WHEEL_RADIO 3.0f ///< Radio of the wheel in cm

///<-------------- AS5600 configuration --------------
#define AS5600_OUT_GPIO_RIGHT 5         ///< gpio number for right OUT signal
#define AS5600_OUT_GPIO_LEFT 6          ///< gpio number for left OUT signal
#define AS5600_OUT_GPIO_BACK 7          ///< gpio number for back OUT signal
#define AS5600_ADC_UNIT_ID ADC_UNIT_1   ///< I2C port number for master dev
#define AS5600_MODE 1                   ///< Calibration = 0, Angle through ADC = 1
///<--------------------------------------------------

///<------------- TM151 configuration ----------------
#define TM151_UART_TX 17                          ///< Gpio pin for UART TX
#define TM151_UART_RX 18                          ///< GPIO pin for UART RX
#define TM151_UART_BAUDRATE 115600                ///< Baudrate for UART communication
#define TM151_BUFFER_SIZE 1024                       ///< Buffer size for UART communication
///<--------------------------------------------------

///<------------- VL53L1X configuration --------------
#define VL53L1X_I2C_PORT 1      ///< I2C port number for master dev
#define VL53L1X_SDA_GPIO 41     ///< gpio number for I2C master data 
#define VL53L1X_SCL_GPIO 42     ///< gpio number for I2C mastes clock
///<--------------------------------------------------

///<-------------- BLDC configuration -----------------
#define PWM_GPIO_R 20               ///< GPIO number for right PWM signal
#define PWM_REV_GPIO_R 21           ///< GPIO number for right PWM reverse signal

#define PWM_GPIO_L 47               ///< GPIO number for left PWM signal
#define PWM_REV_GPIO_L 48           ///< GPIO number for left PWM reverse signal

#define PWM_GPIO_B 14               ///< GPIO number for back PWM signal
#define PWM_REV_GPIO_B 12           ///< GPIO number for back PWM reverse signal

#define PWM_FREQ 50                 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000       ///< PWM resolution in bits
#define MAX_PWM_CAL 120             ///< Maximum PWM value
#define MIN_PWM_CAL 35              ///< Minimum PWM value
#define MAX_PWM_RE 119              ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38               ///< Minimum PWM value (does not move)
///<--------------------------------------------------

///<-------------- PID configuration -----------------
#define PID_KP .04//.01f
#define PID_KI .02//.01f
#define PID_KD 0.0//.001f
#define EULER 2.71828
#define PI 3.14159
///<--------------------------------------------------

#define SELECT_LEFT 0
#define SELECT_BACK 1
#define SELECT_RIGHT 2

/***
 * @brief This defines the data necessary to generate PID control for every wheel
 */
typedef struct {
    AS5600_t * gStruct;             ///< Velocity estimation from encoder in cm/s
    encoder_data_t * sensor_data;   ///< Velocity estimation from IMU in cm/s
    pid_block_handle_t * pid_block; ///< Velocity estimation from Lidar in cm/s
    bldc_pwm_motor_t * pwm_motor;   ///< BLDC motor object

    uart_t * myUART;                  ///< UART object for TM151 IMU
    imu_data_t * imu_data;            ///< IMU data

    uint8_t predef_move;            ///< Predefined movements for the robot
    uint8_t vel_selection;          ///< Velocity selection for the robot
    TaskHandle_t *control_task;      ///< Control task for wheel
} control_params_t; 

typedef struct {
    pid_block_handle_t * pid_block_xvel; ///< PID control block for x velocity
    pid_block_handle_t * pid_block_yvel; ///< PID control block for y velocity
    pid_block_handle_t * pid_block_wb;   ///< PID control block for angular velocity
    encoder_data_t * encoder_data_right; ///< Encoder data structure for right wheel
    encoder_data_t * encoder_data_left;  ///< Encoder data structure for left wheel
    encoder_data_t * encoder_data_back;  ///< Encoder data structure for back wheel
} global_control_params_t;

/**
 * @brief Struct to hold current wheel velocities (measured from encoders)
 */
typedef struct {
    float right_wheel_vel;  ///< Right wheel velocity in cm/s
    float left_wheel_vel;   ///< Left wheel velocity in cm/s
    float back_wheel_vel;   ///< Back wheel velocity in cm/s
} wheel_velocities_t;

/**
 * @brief Struct to hold body measurements (fused from encoders and IMU)
 */
typedef struct {
    float v_x_measured;   ///< Measured x velocity in cm/s
    float v_y_measured;   ///< Measured y velocity in cm/s
    float v_wb_measured;  ///< Measured angular velocity in rad/s
    float yaw;            ///< Yaw angle from IMU in degrees
} body_measurements_t;

/**
 * @brief Struct to hold velocity setpoints from server/user
 */
typedef struct {
    float v_x_setpoint;   ///< Desired x velocity in cm/s
    float v_y_setpoint;   ///< Desired y velocity in cm/s
    float v_wb_setpoint;  ///< Desired angular velocity in rad/s
} velocity_setpoints_t;

/**
 * @brief Struct to hold target velocities after applying global control
 */
typedef struct {
    float v_x_target;     ///< Target x velocity after control in cm/s
    float v_y_target;     ///< Target y velocity after control in cm/s
    float v_wb_target;    ///< Target angular velocity after control in rad/s
} velocity_targets_t;

/**
 * @brief Struct to hold wheel target velocities (from inverse kinematics)
 */
typedef struct {
    float right_wheel_target;  ///< Right wheel target velocity in cm/s
    float left_wheel_target;   ///< Left wheel target velocity in cm/s
    float back_wheel_target;   ///< Back wheel target velocity in cm/s
} wheel_targets_t;

typedef struct {
    float target_distance; ///< Distance measurement
    encoder_data_t * encoder_data_right; ///< Encoder data structure for right wheel
    encoder_data_t * encoder_data_left;  ///< Encoder data structure for left wheel
    encoder_data_t * encoder_data_back;  ///< Encoder data structure for back wheel

} distance_params_t;

/// @brief Struct to use as encoder gatekeeper params
struct enc_gk_params {
    AS5600_t *r_enc;
    AS5600_t *l_enc;
    AS5600_t *b_enc;
    TaskHandle_t *r_wheel;
    TaskHandle_t *l_wheel;
    TaskHandle_t *b_wheel;
    TaskHandle_t *global_control;

};

union float_to_int32 {
    float f_value;
    uint32_t int_value;
};

/**
 * @brief Gatekeeper task for encoders readings
 */
void vTaskEncodersGateKeeper(void *pvParameters);

/**
 * @brief Task to read from right encoder
 */
void vTaskEncoderRight(void * pvParameters);

/**
 * @brief Task to read from left encoder
 */
void vTaskEncoderLeft(void * pvParameters);

/**
 * @brief Task to read from back encoder
 */
void vTaskEncoderBack(void * pvParameters);

/**
 * @brief Task to set the right wheel pwm
 */
void vTaskSetPWMRight(void *pvParameters);

/**
 * @brief Task to set the left wheel pwm
 */
void vTaskSetPWMLeft(void *pvParameters);

/**
 * @brief Task to set the back wheel pwm
 */
void vTaskSetPWMBack(void *pvParameters);

/**
 * @brief Task to read from IMU
 */
void vTaskIMU(void * pvParameters);

/**
 * @brief Task to read from Lidar (Currently disabled)
 */
void vTaskLidar(void * pvParameters);

/**
 * @brief Task to control the right wheel
 * 
 * @param pvParameters 
 */
void vTaskControlRight( void * pvParameters );

/**
 * @brief Task to control the left wheel
 * 
 * @param pvParameters 
 */
void vTaskControlLeft( void * pvParameters );

/**
 * @brief Task to control the back wheel
 * 
 * @param pvParameters 
 */
void vTaskControlBack( void * pvParameters );

/**
 * @brief Task to keep track of distace
 * 
 * @param pvParameters
 */
void vTaskDistance(void * pvParameters);

/**
 * @brief Sensor fusion task to combine data from encoders, IMU.
 * Reads the velocities from each wheel and computes the robot's body velocities
 * using forward kinematics.
 * Reads the velocities x,y,w and yaw from IMU and fuses them with encoder data.
 * 
 * @param pvParameters 
 */
void vTaskSensorFusion(void * pvParameters);

/**
 * @brief Global control task to update the desired body velocities
 * and compute wheel velocities using inverse kinematics
 * 
 * @param pvParameters 
 */
void vTaskGlobalControl(void * pvParameters);

/**
 * @brief Task to control all wheels given desired velocities
 * 
 * @param pvParameters 
 */
void vTaskAllWheelsControl(void * pvParameters);

/**
 * @brief udp server task to handle incoming requests
 */
void vTaskUDPServer(void * pvParameters);

httpd_handle_t start_http_server(void);

void init_kalman_parameters(void);
#endif // CONTROL_H
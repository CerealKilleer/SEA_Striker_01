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

// Include ESP IDF libraries
#include <assert.h>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gptimer.h"

#define SAMPLE_TIME 2 ///< Sample time in ms
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

#define PWM_GPIO_L 10               ///< GPIO number for left PWM signal
#define PWM_REV_GPIO_L 11           ///< GPIO number for left PWM reverse signal

#define PWM_GPIO_B 42               ///< GPIO number for back PWM signal
#define PWM_REV_GPIO_B 41           ///< GPIO number for back PWM reverse signal

#define PWM_FREQ 50                 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000       ///< PWM resolution in bits
#define MAX_PWM_CAL 120             ///< Maximum PWM value
#define MIN_PWM_CAL 35              ///< Minimum PWM value
#define MAX_PWM_RE 119              ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38               ///< Minimum PWM value (does not move)
///<--------------------------------------------------

///<-------------- PID configuration -----------------
#define PID_KP .01f
#define PID_KI .01f
#define PID_KD .001f
#define EULER 2.71828
#define PI 3.14159
///<--------------------------------------------------

/***
 * @brief This defines the data necessary to generate PID control for every wheel
 */
typedef struct {
    AS5600_t * gStruct;                 /*< Encoder raw data and handlers*/
    encoder_data_t * sensor_data;       /*< Variables estimated*/
    pid_block_handle_t * pid_block;     /*/< PID_block variables*/
    bldc_pwm_motor_t * pwm_motor;       /*/< BLDC motor struct*/
} control_params_t;

/**
 * @brief Task to read from encoder
 */
void vTaskEncoder(void * pvParameters);

/**
 * @brief Task to read from IMU
 */
void vTaskIMU(void * pvParameters);

/**
 * @brief Task to read from Lidar
 */
void vTaskLidar(void * pvParameters);

/**
 * @brief Task to control the wheel
 * 
 * @param pvParameters 
 */
void vTaskControl( void * pvParameters );

/**
 * @brief UART task to read data from console
 * 
 * @param pvParameters 
 */
void vTaskUART(void * pvParameters);

#endif // CONTROL_H
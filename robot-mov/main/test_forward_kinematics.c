/**
 * @file test_forward_kinematics.c
 * @brief Forward kinematics test for 3-wheeled omnidirectional robot
 * 
 * @description
 * This program tests forward kinematics control for a 3-wheeled omnidirectional robot.
 * It implements:
 * - UART command interface for velocity and direction input
 * - Gatekeeper pattern for synchronized encoder reading (2ms)
 * - Inverse kinematics to calculate individual wheel velocities
 * - Kalman filter for velocity estimation
 * - PID controller for each wheel
 * - PWM output to 3 BLDC motors
 * 
 * Command format: VEL <velocity_cm/s> <direction_degrees>
 * Example: VEL 15.0 90.0
 * 
 * @date December 2025
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include "string.h"

#include "bldc_pwm.h"
#include "as5600_lib.h"

#include "sensor_fusion.h"
#include "kalman_filter_1d.h"
#include "pid_ext.h"

#include <stdio.h>
#include <math.h>


///<-------------- Timing configuration --------------
#define SAMPLE_TIME 2 ///< Sample time in ms (2ms for gatekeeper)
#define WHEEL_RADIO 3.0f ///< Wheel radius in cm
#define WHEEL_CIRCUMFERENCE (2.0f * 3.14159f * WHEEL_RADIO) ///< Circumference in cm
#define PI 3.14159265359f

///<-------------- Robot geometry --------------------
#define WHEEL_ANGLE_RIGHT 30.0f     ///< Right wheel angle in degrees
#define WHEEL_ANGLE_LEFT 150.0f     ///< Left wheel angle in degrees  
#define WHEEL_ANGLE_BACK 270.0f     ///< Back wheel angle in degrees
#define ROBOT_RADIUS 10.0f          ///< Distance from center to wheel in cm

///<-------------- Kinematics constants ---------------
#define DELTA (PI/6.0f)             ///< Angle for transformation (30 degrees)
#define N 16.0f                     ///< Reduction factor (planetary gear ratio)
#define R 3.0f                      ///< Wheel radius in cm (same as WHEEL_RADIO)

///<-------------- AS5600 configuration --------------
#define AS5600_OUT_GPIO_RIGHT 5         ///< GPIO for right encoder OUT signal
#define AS5600_OUT_GPIO_LEFT 6          ///< GPIO for left encoder OUT signal
#define AS5600_OUT_GPIO_BACK 7          ///< GPIO for back encoder OUT signal
#define AS5600_ADC_UNIT_ID ADC_UNIT_1   ///< ADC unit for encoders
///<--------------------------------------------------

///<-------------- BLDC configuration -----------------
#define PWM_GPIO_R 20               ///< GPIO for right PWM signal
#define PWM_REV_GPIO_R 21           ///< GPIO for right PWM reverse signal

#define PWM_GPIO_L 47               ///< GPIO for left PWM signal
#define PWM_REV_GPIO_L 48           ///< GPIO for left PWM reverse signal

#define PWM_GPIO_B 14               ///< GPIO for back PWM signal
#define PWM_REV_GPIO_B 12           ///< GPIO for back PWM reverse signal

#define PWM_FREQ 50                 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000       ///< PWM resolution
#define MAX_PWM_CAL 120             ///< Maximum PWM calibration value
#define MIN_PWM_CAL 35              ///< Minimum PWM calibration value
///<--------------------------------------------------

///<-------------- UART configuration -----------------
#define UART_NUM UART_NUM_0         ///< USB Serial Monitor
#define UART_BAUD_RATE 115200       ///< Baud rate
#define UART_BUF_SIZE 256           ///< UART buffer size
///<--------------------------------------------------

#define MCPWM_GROUP_ID(group_id) (0)


///<============== DATA STRUCTURES ===============================================

/**
 * @brief Union to convert float to uint32_t for task notifications
 */
typedef union {
    float f_value;
    uint32_t int_value;
} float_to_int32;

/**
 * @brief Gatekeeper parameters structure
 */
typedef struct {
    AS5600_t *r_enc;
    AS5600_t *l_enc;
    AS5600_t *b_enc;
    TaskHandle_t *r_wheel;
    TaskHandle_t *l_wheel;
    TaskHandle_t *b_wheel;
} enc_gk_params_t;

/**
 * @brief Control parameters for each wheel
 */
typedef struct {
    AS5600_t *encoder;
    encoder_data_t *sensor_data;
    pid_block_handle_t *pid_block;
    bldc_pwm_motor_t *pwm_motor;
    float *setpoint;
    const char *wheel_name;
} wheel_control_params_t;

///<============== GLOBAL VARIABLES ==============================================

// Encoder data structures
encoder_data_t right_encoder_data = {
    .velocity = 0.0f,
    .last_vel = 0.0f,
    .angle_prev = 0.0f,
    .radio = WHEEL_RADIO,
    .distance = 0.0f,
    .time_interval = SAMPLE_TIME / 1000.0f
};

encoder_data_t left_encoder_data = {
    .velocity = 0.0f,
    .last_vel = 0.0f,
    .angle_prev = 0.0f,
    .radio = WHEEL_RADIO,
    .distance = 0.0f,
    .time_interval = SAMPLE_TIME / 1000.0f
};

encoder_data_t back_encoder_data = {
    .velocity = 0.0f,
    .last_vel = 0.0f,
    .angle_prev = 0.0f,
    .radio = WHEEL_RADIO,
    .distance = 0.0f,
    .time_interval = SAMPLE_TIME / 1000.0f
};

// Kalman filters for each wheel
kalman_filter_t km_right_wheel, km_left_wheel, km_back_wheel;

// Target velocities (set by UART commands)
float target_vx = 0.0f;      ///< Target velocity in X direction (cm/s)
float target_vy = 0.0f;      ///< Target velocity in Y direction (cm/s)
float target_wb = 0.0f;      ///< Target angular velocity (rad/s)

// Individual wheel setpoints
float setpoint_right = 0.0f;
float setpoint_left = 0.0f;
float setpoint_back = 0.0f;

// Measured robot velocities (from forward kinematics)
float measured_vx = 0.0f;
float measured_vy = 0.0f;
float measured_wb = 0.0f;

// Mutexes for thread-safe access
SemaphoreHandle_t right_params_mutex, left_params_mutex, back_params_mutex;

// Task handles
TaskHandle_t xRightEncoderTaskHandle, xLeftEncoderTaskHandle, xBackEncoderTaskHandle;
TaskHandle_t xRightControlTaskHandle, xLeftControlTaskHandle, xBackControlTaskHandle;
TaskHandle_t xEncodersGateKeeper;


///<============== HELPER FUNCTIONS ==============================================

/**
 * @brief Initializes a BLDC motor
 */
static inline void init_blc_motor(bldc_pwm_motor_t *pwm_motor, uint8_t gpio, 
                                  uint8_t gpio_rev, uint16_t freq, uint8_t gpio_group,
                                  uint32_t pwm_resolution, uint8_t min_pwm_cal, uint8_t max_pwm_cal)
{
    bldc_init(pwm_motor, gpio, gpio_rev, freq, gpio_group, pwm_resolution, min_pwm_cal, max_pwm_cal);
    bldc_enable(pwm_motor);
    bldc_set_duty(pwm_motor, 0);
}

/**
 * @brief Initializes an AS5600 encoder sensor
 */
static inline void init_encoder(AS5600_t *gAs5600, AS5600_config_t *conf, adc_oneshot_unit_handle_t *handle,
                                uint8_t output_gpio, uint8_t adc_unit, const char *encoder_name)
{
    gAs5600->conf = *conf;
    gAs5600->out = output_gpio;
    gAs5600->adc_handle.adc_handle = *handle;

    if (!adc_config_channel(&gAs5600->adc_handle, output_gpio, adc_unit)) {
        ESP_LOGE("AS5600_ADC_CH", "AS5600 %s sensor ADC initialization failed", encoder_name);
    }
}

static void cal_inverse_kinematics(float vx, float vy, float wb,
                                   float *wr, float *wl, float *wbk)
{
    const float th_r = 30.0f  * PI / 180.0f;
    const float th_l = 150.0f * PI / 180.0f;
    const float th_b = 270.0f * PI / 180.0f;

    *wr  = -sinf(th_r) * vx + cosf(th_r) * vy + ROBOT_RADIUS * wb;
    *wl  = -sinf(th_l) * vx + cosf(th_l) * vy + ROBOT_RADIUS * wb;
    *wbk = -sinf(th_b) * vx + cosf(th_b) * vy + ROBOT_RADIUS * wb;
}

static void normalize_wheel_setpoints(float *wr, float *wl, float *wb)
{
    float max = fabsf(*wr);
    if (fabsf(*wl) > max) max = fabsf(*wl);
    if (fabsf(*wb) > max) max = fabsf(*wb);

    const float MAX_WHEEL_SPEED = 15.0f; // cm/s seguro

    if (max > MAX_WHEEL_SPEED) {
        float s = MAX_WHEEL_SPEED / max;
        *wr *= s;
        *wl *= s;
        *wb *= s;
    }
}


/**
 * @brief Forward kinematics: converts wheel velocities to robot velocities
 * Based on cal_forward_kinematics from mov_calculation.c
 * 
 * @param vl_cm_s Left wheel velocity (cm/s)
 * @param vb_cm_s Back wheel velocity (cm/s)
 * @param vr_cm_s Right wheel velocity (cm/s)
 * @param x_velocity Output: velocity in X direction (cm/s)
 * @param y_velocity Output: velocity in Y direction (cm/s)
 * @param angular_velocity Output: angular velocity (rad/s)
 */
static void cal_forward_kinematics(float vl, float vb, float vr,
                                   float *vx, float *vy, float *wb)
{
    const float th_r = 30.0f  * PI / 180.0f;
    const float th_l = 150.0f * PI / 180.0f;
    const float th_b = 270.0f * PI / 180.0f;

    *vx = -(2.0f/3.0f) * ( sinf(th_r)*vr + sinf(th_l)*vl + sinf(th_b)*vb );
    *vy =  (2.0f/3.0f) * ( cosf(th_r)*vr + cosf(th_l)*vl + cosf(th_b)*vb );
    *wb =  (1.0f/(3.0f*ROBOT_RADIUS)) * (vr + vl + vb);
}


/**
 * @brief Converts linear velocity and direction to vx, vy components
 * 
 * @param velocity Linear velocity magnitude (cm/s)
 * @param direction Direction angle in degrees (0° = forward, 90° = left)
 * @param vx Output: velocity in X direction
 * @param vy Output: velocity in Y direction
 */
static void velocity_to_components(float velocity, float direction, float *vx, float *vy)
{
    float direction_rad = direction * PI / 180.0f;
    *vx = velocity * sinf(direction_rad);  // sin for X: 0°→0, 90°→1
    *vy = velocity * cosf(direction_rad);  // cos for Y: 0°→1, 90°→0
}

///<============== TIMER ISR =====================================================

/**
 * @brief Timer ISR for encoder gatekeeper
 * Triggers every 2ms to read all encoders
 */
void timer_isr(void *args) {
    xTaskNotifyFromISR(*((TaskHandle_t *)args), 0x01, eSetBits, NULL);
}


///<============== GATEKEEPER TASK ===============================================

/**
 * @brief Gatekeeper task - reads all encoders every 2ms
 * Similar to main.c implementation
 */
void vTaskEncodersGateKeeper(void *pvParameters) {
    enc_gk_params_t *params = (enc_gk_params_t *)pvParameters;
    float_to_int32 angle;
    const char *TAG = "GATEKEEPER";
    
    ESP_LOGI(TAG, "Gatekeeper task started - reading encoders every 2ms");
    
    while (1) {
        // Wait for timer notification
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        
        // Read all encoders and notify respective tasks
        angle.f_value = AS5600_ADC_GetAngle(params->r_enc);
        xTaskNotify(*params->r_wheel, angle.int_value, eSetValueWithOverwrite);
        
        angle.f_value = AS5600_ADC_GetAngle(params->l_enc);
        xTaskNotify(*params->l_wheel, angle.int_value, eSetValueWithOverwrite);
        
        angle.f_value = AS5600_ADC_GetAngle(params->b_enc);
        xTaskNotify(*params->b_wheel, angle.int_value, eSetValueWithOverwrite);
    }
}

///<============== ENCODER TASKS =================================================

/**
 * @brief Right wheel encoder task
 * Processes encoder data and estimates velocity with Kalman filter
 */
void vTaskEncoderRight(void *pvParameters) {
    wheel_control_params_t *params = (wheel_control_params_t *)pvParameters;
    encoder_data_t *encoder_data = params->sensor_data;
    float_to_int32 angle;
    
    while (1) {
        // Wait for gatekeeper notification
        xTaskNotifyWait(0x00000000, 0x00000000, &angle.int_value, portMAX_DELAY);
        
        xSemaphoreTake(right_params_mutex, portMAX_DELAY);
        
        // Get angle and estimate velocity
        encoder_data->angle = angle.f_value;
        estimate_velocity_encoder(encoder_data);
        
        // Apply Kalman filter
        kalman_update(&km_right_wheel, encoder_data->velocity);
        encoder_data->last_vel = encoder_data->velocity;
        
        xSemaphoreGive(right_params_mutex);
        
        // Notify control task
        xTaskNotify(xRightControlTaskHandle, 0x00, eNoAction);
    }
}

/**
 * @brief Left wheel encoder task
 */
void vTaskEncoderLeft(void *pvParameters) {
    wheel_control_params_t *params = (wheel_control_params_t *)pvParameters;
    encoder_data_t *encoder_data = params->sensor_data;
    float_to_int32 angle;
    
    while (1) {
        xTaskNotifyWait(0x00000000, 0x00000000, &angle.int_value, portMAX_DELAY);
        
        xSemaphoreTake(left_params_mutex, portMAX_DELAY);
        
        encoder_data->angle = -angle.f_value;  // Negative for left wheel
        estimate_velocity_encoder(encoder_data);
        
        kalman_update(&km_left_wheel, encoder_data->velocity);
        encoder_data->last_vel = encoder_data->velocity;
        
        xSemaphoreGive(left_params_mutex);
        
        xTaskNotify(xLeftControlTaskHandle, 0x00, eNoAction);
    }
}

/**
 * @brief Back wheel encoder task
 */
void vTaskEncoderBack(void *pvParameters) {
    wheel_control_params_t *params = (wheel_control_params_t *)pvParameters;
    encoder_data_t *encoder_data = params->sensor_data;
    float_to_int32 angle;
    
    while (1) {
        xTaskNotifyWait(0x00000000, 0x00000000, &angle.int_value, portMAX_DELAY);
        
        xSemaphoreTake(back_params_mutex, portMAX_DELAY);
        
        encoder_data->angle = angle.f_value;
        estimate_velocity_encoder(encoder_data);
        
        kalman_update(&km_back_wheel, encoder_data->velocity);
        encoder_data->last_vel = encoder_data->velocity;
        
        xSemaphoreGive(back_params_mutex);
        
        xTaskNotify(xBackControlTaskHandle, 0x00, eNoAction);
    }
}


///<============== CONTROL TASKS =================================================

/**
 * @brief Right wheel PID control task
 */
void vTaskControlRight(void *pvParameters) {
    wheel_control_params_t *params = (wheel_control_params_t *)pvParameters;
    encoder_data_t *encoder_data = params->sensor_data;
    pid_block_handle_t pid_block = *(params->pid_block);
    const char *TAG = "CTRL_RIGHT";
    
    float output = 0.0f;
    uint32_t log_counter = 0;
    
    while (1) {
        // Wait for encoder task notification
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        
        xSemaphoreTake(right_params_mutex, portMAX_DELAY);
        
        // Update PID setpoint and compute output
        pid_update_set_point(pid_block, setpoint_right);
        pid_compute(pid_block, encoder_data->velocity, &output);
        
        xSemaphoreGive(right_params_mutex);
        
        // // Set motor PWM
        // bldc_set_duty(params->pwm_motor, output);
        float safe_output = fmaxf(fminf(output, 60.0f), -60.0f);
        bldc_set_duty(params->pwm_motor, safe_output);
        
        // Log every 1 second (500 cycles at 2ms)
        if (++log_counter >= 500) {
            // Calculate forward kinematics to get robot velocity
            cal_forward_kinematics(left_encoder_data.velocity, back_encoder_data.velocity, 
                                  encoder_data->velocity, &measured_vx, &measured_vy, &measured_wb);
            
            ESP_LOGI(TAG, "SP:%.2f | Vel:%.2f | Out:%.1f | RobotVel(%.1f,%.1f)", 
                     setpoint_right, encoder_data->velocity, output, measured_vx, measured_vy);
            log_counter = 0;
        }
    }
}

/**
 * @brief Left wheel PID control task
 */
void vTaskControlLeft(void *pvParameters) {
    wheel_control_params_t *params = (wheel_control_params_t *)pvParameters;
    encoder_data_t *encoder_data = params->sensor_data;
    pid_block_handle_t pid_block = *(params->pid_block);
    const char *TAG = "CTRL_LEFT";
    
    float output = 0.0f;
    uint32_t log_counter = 0;
    
    while (1) {
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        
        xSemaphoreTake(left_params_mutex, portMAX_DELAY);
        
        pid_update_set_point(pid_block, setpoint_left);
        pid_compute(pid_block, encoder_data->velocity, &output);
        
        xSemaphoreGive(left_params_mutex);
        
        // bldc_set_duty(params->pwm_motor, output);
        float safe_output = fmaxf(fminf(output, 60.0f), -60.0f);
        bldc_set_duty(params->pwm_motor, safe_output);
        
        if (++log_counter >= 500) {
            ESP_LOGI(TAG, "SP:%.2f | Vel:%.2f | Out:%.1f", 
                     setpoint_left, encoder_data->velocity, output);
            log_counter = 0;
        }
    }
}

/**
 * @brief Back wheel PID control task
 */
void vTaskControlBack(void *pvParameters) {
    wheel_control_params_t *params = (wheel_control_params_t *)pvParameters;
    encoder_data_t *encoder_data = params->sensor_data;
    pid_block_handle_t pid_block = *(params->pid_block);
    const char *TAG = "CTRL_BACK";
    
    float output = 0.0f;
    uint32_t log_counter = 0;
    
    while (1) {
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        
        xSemaphoreTake(back_params_mutex, portMAX_DELAY);
        
        pid_update_set_point(pid_block, setpoint_back);
        pid_compute(pid_block, encoder_data->velocity, &output);
        
        xSemaphoreGive(back_params_mutex);
        float safe_output = fmaxf(fminf(output, 60.0f), -60.0f);
        bldc_set_duty(params->pwm_motor, safe_output);
                
        if (++log_counter >= 500) {
            ESP_LOGI(TAG, "SP:%.2f | Vel:%.2f | Out:%.1f", 
                     setpoint_back, encoder_data->velocity, output);
            log_counter = 0;
        }
    }
}

///<============== UART INITIALIZATION FUNCTION ======================================

/**
 * @brief UART command initialization - waits for initial velocity command
 * Blocks until valid velocity command is received
 * 
 * Command format: VEL <velocity> <direction>
 * Example: VEL 15.0 0.0
 */
void uart_wait_for_command(void) {
    const char *TAG = "UART_INIT";
    char buffer[UART_BUF_SIZE];
    int len;
    bool command_received = false;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  WAITING FOR VELOCITY COMMAND");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Command format: VEL <velocity_cm/s> <direction_degrees>");
    ESP_LOGI(TAG, "Example: VEL 15.0 0.0 (forward)");
    ESP_LOGI(TAG, "Example: VEL 15.0 90.0 (left)");
    ESP_LOGI(TAG, "========================================");
    
    while (!command_received) {
        // Read from UART
        len = uart_read_bytes(UART_NUM, (uint8_t *)buffer, UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            buffer[len] = '\0';  // Null terminate
            
            // Parse command
            char cmd[10];
            float velocity, direction;
            
            if (sscanf(buffer, "%s %f %f", cmd, &velocity, &direction) == 3) {
                if (strcmp(cmd, "VEL") == 0) {
                    // Convert velocity and direction to vx, vy
                    velocity_to_components(velocity, direction, &target_vx, &target_vy);
                    target_wb = 0.0f;  // No rotation for now
                    
                    // // Calculate individual wheel setpoints using inverse kinematics
                    // cal_lin_to_ang_velocity(target_vx, target_vy, target_wb, 2, &setpoint_right);  // Right wheel
                    // cal_lin_to_ang_velocity(target_vx, target_vy, target_wb, 0, &setpoint_left);   // Left wheel
                    // cal_lin_to_ang_velocity(target_vx, target_vy, target_wb, 1, &setpoint_back);   // Back wheel
                    cal_inverse_kinematics(target_vx, target_vy, target_wb,
                       &setpoint_right,
                       &setpoint_left,
                       &setpoint_back);

                    normalize_wheel_setpoints(&setpoint_right,
                                            &setpoint_left,
                                            &setpoint_back);

                    
                    ESP_LOGI(TAG, "========================================");
                    ESP_LOGI(TAG, "  COMMAND RECEIVED!");
                    ESP_LOGI(TAG, "========================================");
                    ESP_LOGI(TAG, "Velocity: %.2f cm/s | Direction: %.2f°", velocity, direction);
                    ESP_LOGI(TAG, "vx=%.2f cm/s, vy=%.2f cm/s", target_vx, target_vy);
                    ESP_LOGI(TAG, "Wheel setpoints:");
                    ESP_LOGI(TAG, "  Right: %.2f cm/s", setpoint_right);
                    ESP_LOGI(TAG, "  Left:  %.2f cm/s", setpoint_left);
                    ESP_LOGI(TAG, "  Back:  %.2f cm/s", setpoint_back);
                    ESP_LOGI(TAG, "========================================");
                    
                    command_received = true;
                } else {
                    ESP_LOGW(TAG, "Unknown command: %s", cmd);
                }
            } else {
                ESP_LOGW(TAG, "Invalid format. Use: VEL <velocity> <direction>");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


///<============== MAIN FUNCTION =================================================

void app_main(void) {
    const char *TAG = "MAIN";
    AS5600_t gAs5600R, gAs5600L, gAs5600B;
    bldc_pwm_motor_t pwmR, pwmL, pwmB;
    pid_block_handle_t pidR, pidL, pidB;
    
    ///<-------------- Create mutexes ----------------
    right_params_mutex = xSemaphoreCreateMutex();
    left_params_mutex = xSemaphoreCreateMutex();
    back_params_mutex = xSemaphoreCreateMutex();
    
    if (!right_params_mutex || !left_params_mutex || !back_params_mutex) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return;
    }
    
    ///<-------------- Initialize UART ----------------
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_LOGI(TAG, "UART initialized (115200 baud)");
    
    ///<-------------- Initialize BLDC motors ----------
    init_blc_motor(&pwmR, PWM_GPIO_R, PWM_REV_GPIO_R, PWM_FREQ, MCPWM_GROUP_ID(0), 
                    PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL);
    
    init_blc_motor(&pwmL, PWM_GPIO_L, PWM_REV_GPIO_L, PWM_FREQ, MCPWM_GROUP_ID(0), 
                    PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL);
    
    init_blc_motor(&pwmB, PWM_GPIO_B, PWM_REV_GPIO_B, PWM_FREQ, MCPWM_GROUP_ID(1), 
                    PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL);
    
    ESP_LOGI(TAG, "BLDC motors initialized");
    ESP_LOGI(TAG, "Waiting 3 seconds for ESC initialization...");
    vTaskDelay(pdMS_TO_TICKS(3000));  // Wait 3 seconds for ESCs to initialize
    ESP_LOGI(TAG, "ESC initialization complete");
    
    ///<-------------- Initialize AS5600 encoders ------
    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_2LSB, ///< Hysteresis 2LSB
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_8X, ///< Slow filter 8x
        .FTH = AS5600_FF_THRESHOLD_6LSB, ///< Fast filter threshold 6LSB
        .WD = AS5600_WATCHDOG_OFF, ///< Watchdog off
    };
    adc_oneshot_unit_handle_t handle;
    if (!adc_create_unit(&handle, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE(TAG, "AS5600 ADC initialization failed");
        return;
    }
    
    init_encoder(&gAs5600R, &conf, &handle, AS5600_OUT_GPIO_RIGHT, AS5600_ADC_UNIT_ID, "right");
    init_encoder(&gAs5600L, &conf, &handle, AS5600_OUT_GPIO_LEFT, AS5600_ADC_UNIT_ID, "left");
    init_encoder(&gAs5600B, &conf, &handle, AS5600_OUT_GPIO_BACK, AS5600_ADC_UNIT_ID, "back");
    
    ESP_LOGI(TAG, "AS5600 encoders initialized");
    
    ///<-------------- Initialize Kalman filters -------
    kalman_init(&km_right_wheel, 0.005f, 1.0f);
    kalman_init(&km_left_wheel, 0.005f, 1.0f);
    kalman_init(&km_back_wheel, 0.005f, 1.0f);
    ESP_LOGI(TAG, "Kalman filters initialized (Q=0.005, R=1.0)");
    
    ///<-------------- Initialize PID controllers ------
    pid_parameter_t pid_paramR = {
        .kp = 0.02f,
        .ki = 0.01f,
        .kd = 0.0f,
        .max_output = 70.0f,
        .min_output = -70.0f,
        .set_point = 0.0f,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .beta = 0.0f
    };
    
    pid_parameter_t pid_paramL = {
        .kp = 0.02f,
        .ki = 0.01f,
        .kd = 0.0f,
        .max_output = 70.0f,
        .min_output = -70.0f,
        .set_point = 0.0f,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .beta = 0.0f
    };
    
    pid_parameter_t pid_paramB = {
        .kp = 0.02f,
        .ki = 0.01f,
        .kd = 0.0f,
        .max_output = 70.0f,
        .min_output = -70.0f,
        .set_point = 0.0f,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .beta = 0.0f
    };
    
    pid_config_t pid_config = {.init_param = pid_paramR};
    pid_new_control_block(&pid_config, &pidR);
    
    pid_config.init_param = pid_paramL;
    pid_new_control_block(&pid_config, &pidL);
    
    pid_config.init_param = pid_paramB;
    pid_new_control_block(&pid_config, &pidB);
    
    ESP_LOGI(TAG, "PID controllers initialized");
    
    ///<-------------- Setup control parameters --------
    wheel_control_params_t right_params = {
        .encoder = &gAs5600R,
        .sensor_data = &right_encoder_data,
        .pid_block = &pidR,
        .pwm_motor = &pwmR,
        .setpoint = &setpoint_right,
        .wheel_name = "RIGHT"
    };
    
    wheel_control_params_t left_params = {
        .encoder = &gAs5600L,
        .sensor_data = &left_encoder_data,
        .pid_block = &pidL,
        .pwm_motor = &pwmL,
        .setpoint = &setpoint_left,
        .wheel_name = "LEFT"
    };
    
    wheel_control_params_t back_params = {
        .encoder = &gAs5600B,
        .sensor_data = &back_encoder_data,
        .pid_block = &pidB,
        .pwm_motor = &pwmB,
        .setpoint = &setpoint_back,
        .wheel_name = "BACK"
    };
    
    enc_gk_params_t gk_params = {
        .r_enc = &gAs5600R,
        .l_enc = &gAs5600L,
        .b_enc = &gAs5600B,
        .r_wheel = &xRightEncoderTaskHandle,
        .l_wheel = &xLeftEncoderTaskHandle,
        .b_wheel = &xBackEncoderTaskHandle
    };
    
    ///<-------------- Create tasks ---------------------
    // Gatekeeper task
    if (xTaskCreate(vTaskEncodersGateKeeper, "enc_gk", 2048, 
                    &gk_params, 21, &xEncodersGateKeeper) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create gatekeeper task");
        return;
    }
    
    // Encoder tasks
    if (xTaskCreate(vTaskEncoderRight, "enc_right", 4096, 
                    &right_params, 20, &xRightEncoderTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create right encoder task");
        return;
    }
    
    if (xTaskCreate(vTaskEncoderLeft, "enc_left", 4096, 
                    &left_params, 20, &xLeftEncoderTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create left encoder task");
        return;
    }
    
    if (xTaskCreate(vTaskEncoderBack, "enc_back", 4096, 
                    &back_params, 20, &xBackEncoderTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create back encoder task");
        return;
    }
    
    // Control tasks
    if (xTaskCreate(vTaskControlRight, "ctrl_right", 4096, 
                    &right_params, 19, &xRightControlTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create right control task");
        return;
    }
    
    if (xTaskCreate(vTaskControlLeft, "ctrl_left", 4096, 
                    &left_params, 19, &xLeftControlTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create left control task");
        return;
    }
    
    if (xTaskCreate(vTaskControlBack, "ctrl_back", 4096, 
                    &back_params, 19, &xBackControlTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create back control task");
        return;
    }
    
    ESP_LOGI(TAG, "All tasks created successfully");
    
    ///<-------------- Wait for initial command ---------
    uart_wait_for_command();
    
    ESP_LOGI(TAG, "Starting motion control...");
    
    ///<-------------- Start timer for gatekeeper ------
    esp_timer_handle_t timer_handle;
    
    const esp_timer_create_args_t timer_args = {
        .callback = timer_isr,
        .arg = (void *)&xEncodersGateKeeper,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "encoder_timer",
        .skip_unhandled_events = false
    };
    
    if (esp_timer_create(&timer_args, &timer_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer");
        return;
    }
    
    if (esp_timer_start_periodic(timer_handle, 2000) != ESP_OK) {  // 2ms
        ESP_LOGE(TAG, "Failed to start timer");
        return;
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ROBOT IN MOTION!");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Timer started - 2ms period");
    ESP_LOGI(TAG, "Control loops active");
    ESP_LOGI(TAG, "========================================");
    
    // Main loop - keep alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
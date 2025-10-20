/**
 * @file main.c
 * 
 * @brief main file for obtaining data from sensors and control the 3-wheeled robot
 * 
 * Main file for wheel modeling purposes, this code is used to get the data from the sensors AS5600, VL531Lx and TM151 
 * 
 * @authors Julian Sanchez
 *          Angel Graciano
 *          Nelson Parra
 * 
 * 
 * @date 20-06-2025
 * 
 * @version 1.0
 * 
 * @copyright Copyright (c) RoboCup SISTEMIC 2025 
 * 
 * MIT LICENSE
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include "control_main.h"
#define MCPWM_GROUP_ID(group_id) (0)

uint32_t temp_ctr = 0; ///< Temporary counter to stop move_one_time

/**
 * @brief Initializes the PID controllers and links all control structures.
 *
 * This function creates and initializes a new PID control block using the specified
 * parameters, and associates all required references for feedback control,
 * including sensor data, motor control structures, and PID configuration.
 *
 * @param[in,out] control_params   Pointer to the main control parameters structure.
 * @param[in]     pid_param        Pointer to the structure containing initial PID parameters.
 * @param[in]     gAs5600          Pointer to the AS5600 sensor handle.
 * @param[in]     sensor_data      Pointer to the structure containing encoder or sensor data.
 * @param[in,out] pid_block        Pointer to the PID control block to be initialized.
 * @param[in,out] pwm_motor_data   Pointer to the BLDC motor control structure.
 *
 * @note This function must be called before executing any PID control loop.
 * 
 * @retval None
 */
static inline void init_pid_controllers(control_params_t *control_params, pid_parameter_t *pid_param, AS5600_t *gAs5600, 
                                        encoder_data_t *sensor_data, pid_block_handle_t *pid_block,
                                        bldc_pwm_motor_t *pwm_motor_data)
{
    pid_config_t pid_config = {
        .init_param = *pid_param
    };

    pid_new_control_block(&pid_config, pid_block);

    control_params->gStruct = gAs5600;
    control_params->sensor_data = sensor_data;
    control_params->pid_block = pid_block;
    control_params->pwm_motor = pwm_motor_data;
}

/**
 * @brief Initializes a single AS5600 magnetic encoder sensor.
 *
 * This function configures an AS5600 encoder instance by assigning its
 * configuration, output GPIO pin, and ADC handle. It also attempts to configure
 * the ADC channel used for reading the analog output of the encoder.
 *
 * @param[out] gAs5600 Pointer to the AS5600 encoder instance to initialize.
 * @param[in] conf Pointer to the configuration structure for the AS5600 sensor.
 * @param[in] handle Pointer to the ADC oneshot handle used for analog readings.
 * @param[in] output_gpio GPIO pin connected to the AS5600 OUT pin.
 * @param[in] adc_unit ADC unit number to be used (e.g., ADC_UNIT_1 or ADC_UNIT_2).
 * @param[in] encoder_name Human-readable name of the encoder (for logging purposes).
 *
 * @note The function logs an error message if the ADC channel configuration fails.
 *
 */

static inline void init_encoder(AS5600_t *gAs5600, AS5600_config_t *conf, adc_oneshot_unit_handle_t *handle,
                                uint8_t output_gpio, uint8_t adc_unit, const char *encoder_name)
{
    gAs5600->conf = *conf;                            ///< Set the configuration for one AS5600 sensor
    gAs5600->out = output_gpio;                       ///< Set the OUT GPIO pin for one AS5600 sensor
    gAs5600->adc_handle.adc_handle =  *handle;        ///< Set the ADC handle for one AS5600 sensor

    if (!adc_config_channel(&gAs5600->adc_handle, output_gpio, adc_unit)) {
        ESP_LOGE("AS5600_ADC_CH", "AS5600 %s sensor ADC initialization failed\n", encoder_name);
    }
}
/**
 * @brief Initializes and enables a BLDC motor.
 *
 * This function performs a complete initialization sequence for a BLDC motor.
 * It calls the underlying initialization function, enables the motor, and sets
 * the initial PWM duty cycle to 0%.
 *
 * @param[in,out] pwm_motor Pointer to the BLDC motor structure to initialize.
 * @param[in] gpio GPIO pin used for the main PWM output.
 * @param[in] gpio_rev GPIO pin used for the inverted PWM output.
 * @param[in] freq PWM frequency in hertz.
 * @param[in] gpio_group Identifier for the GPIO group or channel.
 * @param[in] pwm_resolution PWM resolution in bits.
 * @param[in] min_pwm_cal Minimum PWM value used during calibration.
 * @param[in] max_pwm_cal Maximum PWM value used during calibration.
 */
static inline void init_blc_motor(bldc_pwm_motor_t *pwm_motor, uint8_t gpio, 
                                  uint8_t gpio_rev, uint16_t freq, uint8_t gpio_group,
                                  uint32_t pwm_resolution, uint8_t min_pwm_cal, uint8_t max_pwm_cal)
{
    bldc_init(pwm_motor, gpio, gpio_rev, freq, gpio_group, pwm_resolution, min_pwm_cal, max_pwm_cal); ///< Initialize the BLDC motor
    bldc_enable(pwm_motor); ///< Enable the BLDC motor
    bldc_set_duty(pwm_motor, 0); ///< Set the duty cycle to 0%
}


void app_main(void)
{
    static AS5600_t gAs5600R, gAs5600L, gAs5600B;  ///< AS5600 object for angle sensor right, left and back
    static vl53l1x_t gVl53l1x;                     ///< VL53L1X object for distance sensor
    static uart_t myUART;                          ///< UART object for TM151 IMU

    extern encoder_data_t right_encoder_data, left_encoder_data, back_encoder_data; ///< Encoder data for right, left and back wheels
    extern imu_data_t imu_data;
    extern lidar_data_t lidar_data;

    static bldc_pwm_motor_t pwmR, pwmL, pwmB;   ///< BLDC motor object right, left and back
    static pid_block_handle_t pidR, pidL, pidB; ///< PID control block handle

    extern pid_parameter_t pid_paramR, pid_paramL, pid_paramB; ///< PID parameters for right, left and back wheels

    ///<---------------- Initialize the Wifi ----------------
    if (wifi_init_station() != ESP_OK){
        ESP_LOGE("WIFI_INIT", "Could not initialize WiFi station mode...");
    
    } else {    
        ESP_LOGI("WIFI_INIT", "WiFi station mode initialized successfully...");
        get_ip_address(); ///< Get the IP address of the ESP32

    }
    ///<----------------------------------------------------
        
    ///<-------------- Initialize the VL53L1X sensor -----
    // if(!VL53L1X_init(&gVl53l1x, VL53L1X_I2C_PORT, VL53L1X_SCL_GPIO, VL53L1X_SDA_GPIO, 0)){
    //     ESP_LOGE(TAG_VL53L1X, "Could not initialize VL53L1X sensor...");
    //     return;
    // }
    // VL53L1X_setDistanceMode(&gVl53l1x, Short); 
    // VL53L1X_setMeasurementTimingBudget(&gVl53l1x, 20000);
    // VL53L1X_startContinuous(&gVl53l1x, SAMPLE_TIME);
    // vTaskDelay(500 / portTICK_PERIOD_MS); ///< Wait for 500 ms
    ///<--------------------------------------------------

    ///<------- Initialize the BLDC motors PWMs ----------
    init_blc_motor(&pwmR, PWM_GPIO_R, PWM_REV_GPIO_R, PWM_FREQ, MCPWM_GROUP_ID(0), 
                    PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL);
    
    init_blc_motor(&pwmL, PWM_GPIO_L, PWM_REV_GPIO_L, PWM_FREQ, MCPWM_GROUP_ID(0), 
                    PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL);
    
    init_blc_motor(&pwmB, PWM_GPIO_B, PWM_REV_GPIO_B, PWM_FREQ, MCPWM_GROUP_ID(1), 
                    PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL);
    ///<--------------------------------------------------

    ///<---------- Initialize the AS5600 sensors ---------
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
        ESP_LOGE("AS5600_ADC_UNIT", "AS5600 ADC initialization failed.\n");
        return;
    }

    init_encoder(&gAs5600R, &conf, &handle, AS5600_OUT_GPIO_RIGHT, AS5600_ADC_UNIT_ID, "right");
    init_encoder(&gAs5600L, &conf, &handle, AS5600_OUT_GPIO_LEFT, AS5600_ADC_UNIT_ID, "left");
    init_encoder(&gAs5600B, &conf, &handle, AS5600_OUT_GPIO_BACK, AS5600_ADC_UNIT_ID, "back");

    ///<--------------------------------------------------
    
    ///<-------------- Initialize the TM151 sensor ------
    // tm151_init(&myUART, TM151_UART_BAUDRATE, TM151_BUFFER_SIZE, TM151_UART_TX, TM151_UART_RX); ///< Initialize the TM151 sensor
    ///<--------------------------------------------------

    ///<------------- Initialize the PID controllers ------
    control_params_t right_control_params;
    control_params_t left_control_params;
    control_params_t back_control_params;

    init_pid_controllers(&right_control_params, &pid_paramR, &gAs5600R,
                        &right_encoder_data, &pidR, &pwmR);
    init_pid_controllers(&left_control_params, &pid_paramL, &gAs5600L,
                        &left_encoder_data, &pidL, &pwmL);
    init_pid_controllers(&back_control_params, &pid_paramB, &gAs5600B,
                        &back_encoder_data, &pidB, &pwmB);

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 5 seconds to ensure all peripherals are initialized

    ///<-------------- Create the task ---------------

    TaskHandle_t xRightEncoderTaskHandle, xLeftEncoderTaskHandle, xBackEncoderTaskHandle; ///< Task handles for encoders
    /*, xIMUTaskHandle = NULL, xLidarTaskHandle = NULL*/ ///< Task handles

    TaskHandle_t xRightControlTaskHandle, xLeftControlTaskHandle, xBackControlTaskHandle; ///< Task handles for control tasks
    xTaskCreatePinnedToCore(vTaskControl, "rwh_control_task", 4096, &right_control_params, 9, &xRightControlTaskHandle, 1); ///< Create the task to control the right wheel
    xTaskCreatePinnedToCore(vTaskControl, "lwh_control_task", 4096, &left_control_params, 9, &xLeftControlTaskHandle, 1);   ///< Create the task to control the left wheel
    xTaskCreatePinnedToCore(vTaskControl, "bwh_control_task", 4096, &back_control_params, 9, &xBackControlTaskHandle, 1);   ///< Create the task to control the back wheel

    configASSERT(xRightControlTaskHandle); ///< Check if the task was created successfully
    if (xRightControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xLeftControlTaskHandle); ///< Check if the task was created successfully
    if (xLeftControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xBackControlTaskHandle); ///< Check if the task was created successfully
    if (xBackControlTaskHandle == NULL) {
        ESP_LOGE("CTRL_TASK", "Failed to create task...");
        return;
    }

    ESP_LOGI("TASKS", "Right encoder handle: 0x%04X", gAs5600R.out); ///< Log the task handles
    xTaskCreatePinnedToCore(vTaskEncoder, "right_encoder_task", 2048, &right_control_params, 8, &xRightEncoderTaskHandle, 0); ///< Create the task to read from right encoder
    xTaskCreatePinnedToCore(vTaskEncoder, "left_encoder_task", 2048, &left_control_params, 8, &xLeftEncoderTaskHandle, 0);    ///< Create the task to read from left encoder
    xTaskCreatePinnedToCore(vTaskEncoder, "back_encoder_task", 2048, &back_control_params, 8, &xBackEncoderTaskHandle, 0);    ///< Create the task to read from back encoder

    configASSERT(xRightEncoderTaskHandle); ///< Check if the task was created successfully
    if (xRightEncoderTaskHandle == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xLeftEncoderTaskHandle); ///< Check if the task was created successfully
    if (xLeftEncoderTaskHandle == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to create task...");
        return;
    }
    configASSERT(xBackEncoderTaskHandle); ///< Check if the task was created successfully
    if (xBackEncoderTaskHandle == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to create task...");
        return;
    }

    // xTaskCreate(vTaskIMU, "imu_task", 2048, NULL, 9, &xIMUTaskHandle); ///< Create the task to read from IMU
    // configASSERT(xIMUTaskHandle); ///< Check if the task was created successfully
    // if (xIMUTaskHandle == NULL) {
    //     ESP_LOGE("IMU_TASK", "Failed to create task...");
    //     return;
    // }

    // xTaskCreate(vTaskLidar, "lidar_task", 2048, NULL, 9, &xLidarTaskHandle); ///< Create the task to read from Lidar
    // configASSERT(xLidarTaskHandle); ///< Check if the task was created successfully
    // if (xLidarTaskHandle == NULL) {
    //     ESP_LOGE("LIDAR_TASK", "Failed to create task...");
    //     return;
    // }

    
    ///<-------------------------------------------------
    
    
    for (;;) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
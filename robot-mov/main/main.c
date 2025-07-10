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

uint32_t temp_ctr = 0; ///< Temporary counter to stop move_one_time

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
    bldc_init(&pwmR, PWM_GPIO_R, PWM_REV_GPIO_R, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmR); ///< Enable the BLDC motor
    bldc_set_duty(&pwmR, 0); ///< Set the duty cycle to 0%

    bldc_init(&pwmL, PWM_GPIO_L, PWM_REV_GPIO_L, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmL); ///< Enable the BLDC motor
    bldc_set_duty(&pwmL, 0); ///< Set the duty cycle to 0%

    bldc_init(&pwmB, PWM_GPIO_B, PWM_REV_GPIO_B, PWM_FREQ, 1, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_enable(&pwmB); ///< Enable the BLDC motor
    bldc_set_duty(&pwmB, 0); ///< Set the duty cycle to 0%
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

    gAs5600R.conf = conf; ///< Set the configuration for the right AS5600 sensor
    gAs5600R.out = AS5600_OUT_GPIO_RIGHT; ///< Set the OUT GPIO pin for the right AS5600 sensor
    gAs5600R.adc_handle.adc_handle = handle; ///< Set the ADC handle for the right AS5600 sensor
    if (!adc_config_channel(&gAs5600R.adc_handle, AS5600_OUT_GPIO_RIGHT, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE("AS5600_ADC_CH", "AS5600 right sensor ADC initialization failed\n");
    }
    
    gAs5600L.conf = conf; ///< Set the configuration for the left AS5600 sensor
    gAs5600L.out = AS5600_OUT_GPIO_LEFT; ///< Set the OUT
    gAs5600L.adc_handle.adc_handle = handle; ///< Set the ADC handle for the left AS5600 sensor
    if (!adc_config_channel(&gAs5600L.adc_handle, AS5600_OUT_GPIO_LEFT, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE("AS5600_ADC_CH", "AS5600 left sensor ADC initialization failed\n");
    }

    gAs5600B.conf = conf; ///< Set the configuration for the back AS5600 sensor
    gAs5600B.out = AS5600_OUT_GPIO_BACK; ///< Set the OUT GPIO pin for the back AS5600 sensor
    gAs5600B.adc_handle.adc_handle = handle; ///< Set the ADC handle for the back AS5600 sensor
    if (!adc_config_channel(&gAs5600B.adc_handle, AS5600_OUT_GPIO_BACK, AS5600_ADC_UNIT_ID)) {
        ESP_LOGE("AS5600_ADC_CH", "AS5600 back sensor ADC initialization failed\n");
    }
    ///<--------------------------------------------------
    
    ///<-------------- Initialize the TM151 sensor ------
    // tm151_init(&myUART, TM151_UART_BAUDRATE, TM151_BUFFER_SIZE, TM151_UART_TX, TM151_UART_RX); ///< Initialize the TM151 sensor
    ///<--------------------------------------------------

    ///<------------- Initialize the PID controllers ------
    pid_config_t pid_config = {
        .init_param = pid_paramR
    };
    pid_new_control_block(&pid_config, &pidR);

    pid_config.init_param = pid_paramL;
    pid_new_control_block(&pid_config, &pidL);

    pid_config.init_param = pid_paramB;
    pid_new_control_block(&pid_config, &pidB);
    ///<---------------------------------------------------

    static control_params_t right_control_params = {
        .gStruct = &gAs5600R,
        .sensor_data = &right_encoder_data,
        .pid_block = &pidR,
        .pwm_motor = &pwmR,
        .predef_move = 0, ///< Predefined movements for the robot, can be set later
        .vel_selection = 2 ///< Velocity selection for the robot, can be set later
    };

    static control_params_t left_control_params = {
        .gStruct = &gAs5600L,
        .sensor_data = &left_encoder_data,
        .pid_block = &pidL,
        .pwm_motor = &pwmL,
        .predef_move = 1, ///< Predefined movements for the robot, can be set later
        .vel_selection = 0 ///< Velocity selection for the robot, can be set later
    };

    static control_params_t back_control_params = {
        .gStruct = &gAs5600B,
        .sensor_data = &back_encoder_data,
        .pid_block = &pidB,
        .pwm_motor = &pwmB,
        .predef_move = 2, ///< Predefined movements for the robot, can be set later
        .vel_selection = 1 ///< Velocity selection for the robot, can be set later
    };

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 1 second to ensure all peripherals are initialized

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
    xTaskCreatePinnedToCore(vTaskEncoder, "right_encoder_task", 4096, &right_control_params, 8, &xRightEncoderTaskHandle, 0); ///< Create the task to read from right encoder
    xTaskCreatePinnedToCore(vTaskEncoder, "left_encoder_task", 4096, &left_control_params, 8, &xLeftEncoderTaskHandle, 0);    ///< Create the task to read from left encoder
    xTaskCreatePinnedToCore(vTaskEncoder, "back_encoder_task", 4096, &back_control_params, 8, &xBackEncoderTaskHandle, 0);    ///< Create the task to read from back encoder

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

    ///<---------------- Initialize the Wifi ----------------


    // Nelson aqui deberias llamar primero wifi_init_station() y luego start_server()
 
    // en los handlers que puse en wifi_lib.c deberias implementar la logica de movimiento del robot con las estructuras o lo que se te haya ocurrido
    // recuerde cambiar la clabve de wifi y el nombre de la red wifi en wifi_lib.h, cualquier cosa me avisa para ayudarle, no compile el proyecto porque a mi me tira errores
    // muy raros, supongo que a vos que te compila este archivo sin problema no te pondrá problemas, igual me avisa si pasa algo, lo que agregué esta probado en entorno cerrado,
    // no sé si al agregarlo aqui haya roto algo, ojalá no.

    

    // dev_wifi_init(); ///< Initialize the WiFi in soft Access Point mode
    // start_server(); ///< Start the HTTP server

    ///<----------------------------------------------------

    
    ///<-------------------------------------------------
    
    

}
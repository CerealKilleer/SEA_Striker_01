#include "control_main.h"

bool forward_mov[] = {true, true, true, false, false, false, false, true}; ///< Forward movements for the robot
float linear_velocity[] = {15.0f, 0.0f, 15.0f, 0.0f, 15.0f, 0.0f, 15.0f, 0.0f}; ///< Linear velocities for the robot in cm/s
float angle[] = {0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 90.0f, 0.0f}; ///< Angles for the robot in degrees

float predef_move2[3][8] = { // {right, left, back} velocity in cm/s
    {-15.0f, 0.0f, 15.0f, 0.0f, -15.0f, 0.0f, 15.0f, 0.0f}, ///< Predefined movements for the robots right wheel
    {15.0f, 0.0f, 15.0f, 0.0f, -15.0f, 0.0f, -15.0f, 0.0f}, ///< Predefined movements for the robots left wheel
    {0.0f, 0.0f, -15.0f, 0.0f, 15.0f, 0.0f, 0.0f, 0.0f} ///< Predefined movements for the robots back wheel
};

imu_data_t imu_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .prev_acc = 0.0f,         ///< Previous acceleration values
    .window = {}          ///< Window for sampling
};         ///< IMU data structure

encoder_data_t right_encoder_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .last_vel = 0.0f,         ///< Last velocity in cm/s
    .angle_prev = 0.0f,       ///< Previous angle in degrees
    .radio = WHEEL_RADIO,     ///< Radio for the wheel
    .distance = 0.0f,          ///< Distance in cm
    .time_interval = SAMPLE_TIME / 1000.0f ///< Time interval in seconds
}; ///< Encoder data structure

encoder_data_t left_encoder_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .last_vel = 0.0f,         ///< Last velocity in cm/s
    .angle_prev = 0.0f,       ///< Previous angle in degrees
    .radio = WHEEL_RADIO,     ///< Radio for the wheel
    .distance = 0.0f,          ///< Distance in cm
    .time_interval = SAMPLE_TIME / 1000.0f ///< Time interval in seconds
}; ///< Encoder data structure

encoder_data_t back_encoder_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .last_vel = 0.0f,         ///< Last velocity in cm/s
    .angle_prev = 0.0f,       ///< Previous angle in degrees
    .radio = WHEEL_RADIO,     ///< Radio for the wheel
    .distance = 0.0f,          ///< Distance in cm
    .time_interval = SAMPLE_TIME / 1000.0f ///< Time interval in seconds
}; ///< Encoder data structure

lidar_data_t lidar_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .prev_distance = 0,       ///< Previous distance in cm
    .start_distance = 0        ///< Start distance in cm
};     ///< Lidar data structure

pid_parameter_t pid_paramR = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramL = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramB = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

// Task to read from encoder
void vTaskEncoder(void * pvParameters) {

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);

    ///<-------------- Get angle through ADC -------------
    while (1) {

        encoder_data->angle = AS5600_ADC_GetAngle(params->gStruct); ///< Get the angle from the ADC
        estimate_velocity_encoder(encoder_data); ///< Estimate the velocity using encoder data

        // // Log every 100ms because of the ESP_LOGI overhead
        // static int counter = 0;
        // if (++counter >= 50) {  // 2ms × 50 = 100ms
        //     ESP_LOGI(task_name, "Velocity: %.2f", encoder_data->velocity);
        //     counter = 0;
        // }

        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
    ///<--------------------------------------------------
}

// Task to read from IMU
void vTaskIMU(void * pvParameters) {

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    imu_data_t *imu_data = (imu_data_t *)params->imu_data; ///< IMU data structure
    uart_t *myUART = params->myUART; ///< UART object for TM151 IMU

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);

    float acceleration[3], yaw;
    
    while (1) {
        // Read acceleration data from TM151 IMU
        SerialPort_DataReceived_RawAcc(myUART, acceleration); ///< Read acceleration data from TM151 IMU
        SerialPort_DataReceived_RawYaw(myUART, &yaw); ///< Read yaw data from TM151 IMU
        
        // Estimate the velocity using IMU data
        // estimate_velocity_imu(imu_data, acceleration[0], SAMPLE_TIME / 1000.0f); ///< Estimate the velocity using IMU data

        // Log every 100ms because of the ESP_LOGI overhead
        static int counter = 0;
        if (++counter >= 150) {  // 2ms × 150 = 300ms
            ESP_LOGI(task_name, "Acceleration: [\t%.2f,\t%.2f,\t%.2f]\t Yaw: %.2f", acceleration[0], acceleration[1], acceleration[2], yaw);
            counter = 0;
        }
        
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
}

// Task to read from Lidar
void vTaskLidar(void * pvParameters) {
    while (1) {
        // // Read distance data from VL53L1X sensor
        // float distance = VL53L1X_readDistance(&gVl53l1x, 0); ///< Get the distance from the VL53L1X sensor
        // estimate_velocity_lidar(&lidar_data, distance, SAMPLE_TIME / 1000.0f); ///< Estimate the velocity using lidar data
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
}

// Task to control the wheel
void vTaskControl( void * pvParameters ){

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure

    pid_block_handle_t pid_block = *(params->pid_block); ///< PID control block handle

    uint32_t timestamp = 1000000, counter = 0; // 1 second
    bool move = true; ///< Flag to indicate if the robot should move

    float est_velocity = 0.0f, last_est_velocity = 0.0f;
    // float beta = exp(-2 * PI * 1 / 100);  // 10Hz cutoff frequency
    float output = 0.0f;
    float setpoint = 0.0f * 1000;
    float x_vel = 0.0f, y_vel = 0.0f; ///< Initialize setpoint and generalized velocities

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);

    while (1)
    {
        ///<-------------- PID Control ---------------
        // Low-pass filter
        est_velocity = encoder_data->velocity;

        last_est_velocity = est_velocity; ///< Update the last estimated velocity

        // circular_movement(1, 5, 360, 15, &x_vel, &y_vel); ///< Calculate the circular movement
        if (!(encoder_data->distance_reached)){
            linear_movement(1, 7, 0, &x_vel, &y_vel); ///< Calculate the linear movement
            cal_lin_to_ang_velocity(x_vel, y_vel, params->vel_selection, &setpoint); ///< Calculate the setpoint based on the predefined movements
        } else {
            setpoint = 0.0f; ///< Stop the movement
        }

        if (pid_update_set_point(pid_block, setpoint) != PID_OK) {
            ESP_LOGE(task_name, "Failed to update PID parameters for %s", task_name);
        }

        // Update PID Controller
        pid_compute(pid_block, est_velocity, &output);
        bldc_set_duty(params->pwm_motor, output); ///< Set the duty cycle to the output of the PID controller

        // Log every 100ms because of the ESP_LOGI overhead
        static int ctr = 0;
        if (++ctr >= 150) {  // 2ms × 50 = 100ms
            // ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f", est_velocity, output); ///< Log the PID parameters
            ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f\tSetpoint: %.2f", est_velocity, output, setpoint); ///< Log the PID parameters
            ctr = 0;
        }
        
        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }
}

void vTaskDistance(void *pvParameters){
    float goal_time = 30.0/7.0; ///< Goal time for 20 cm at 15 cm/s
    distance_params_t *params = (distance_params_t *)pvParameters; ///< Distance parameters structure
    encoder_data_t *encoder_data_right = params->encoder_data_right; ///< Encoder data structure for right wheel
    encoder_data_t *encoder_data_left = params->encoder_data_left; ///< Encoder data structure for left wheel
    encoder_data_t *encoder_data_back = params->encoder_data_back; ///< Encoder data structure for back wheel
    float dx, dy, distance = 0, beta = 0.9; ///< Variables to store the distance

    while(1){
        static uint16_t time_count = 0; ///< Counter to keep track of the number of iterations

        if(time_count >= goal_time * 1000) {
            encoder_data_right->distance_reached = 1; ///< Set the distance reached flag to true
            encoder_data_left->distance_reached = 1; ///< Set the distance reached flag to true
            encoder_data_back->distance_reached = 1; ///< Set the distance reached flag to true
            
            encoder_data_right->distance = 0.0f; ///< Reset the distance for right wheel
            encoder_data_left->distance = 0.0f; ///< Reset the distance for left
            encoder_data_back->distance = 0.0f; ///< Reset the distance for back wheel
        } else {
            time_count += 5 * SAMPLE_TIME; ///< Increment the time count
        }

        vTaskDelay(5 * SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }

}
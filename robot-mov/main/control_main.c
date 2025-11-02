#include "control_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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

enum movements_num movement; ///< Movement type
float x_vel = 100.0f, y_vel = 0.0f; ///< Generalized velocities for the robot
float goal_time = 10.0f; ///< Goal time for linear movement in seconds

// Task to read from encoder
void vTaskEncoderRight(void * pvParameters) {

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure
    extern SemaphoreHandle_t right_params_mutex;

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);

    ///<-------------- Get angle through ADC -------------
    while (1) {
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        xSemaphoreTake(right_params_mutex, portMAX_DELAY);
        encoder_data->angle = AS5600_ADC_GetAngle(params->gStruct); ///< Get the angle from the ADC
        estimate_velocity_encoder(encoder_data); ///< Estimate the velocity using encoder data
        xSemaphoreGive(right_params_mutex);
        // // Log every 100ms because of the ESP_LOGI overhead
        // static int counter = 0;
        // if (++counter >= 50) {  // 2ms × 50 = 100ms
        //     ESP_LOGI(task_name, "Velocity: %.2f", encoder_data->velocity);
        //     counter = 0;
        // }
        xTaskNotify(*params->control_task, 0x01, eSetBits);
    }
    ///<--------------------------------------------------
}

void vTaskEncoderLeft(void * pvParameters) {

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure
    extern SemaphoreHandle_t left_params_mutex;
    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);

    ///<-------------- Get angle through ADC -------------
    while (1) {
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        xSemaphoreTake(left_params_mutex, portMAX_DELAY);
        encoder_data->angle = AS5600_ADC_GetAngle(params->gStruct); ///< Get the angle from the ADC
        estimate_velocity_encoder(encoder_data); ///< Estimate the velocity using encoder data
        xSemaphoreGive(left_params_mutex);
        // // Log every 100ms because of the ESP_LOGI overhead
        // static int counter = 0;
        // if (++counter >= 50) {  // 2ms × 50 = 100ms
        //     ESP_LOGI(task_name, "Velocity: %.2f", encoder_data->velocity);
        //     counter = 0;
        // }
        xTaskNotify(*params->control_task, 0x01, eSetBits);
    }
    ///<--------------------------------------------------
}

void vTaskEncoderBack(void * pvParameters) {

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure
    extern SemaphoreHandle_t back_params_mutex;
    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);

    ///<-------------- Get angle through ADC -------------
    while (1) {
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        xSemaphoreTake(back_params_mutex, portMAX_DELAY);
        encoder_data->angle = AS5600_ADC_GetAngle(params->gStruct); ///< Get the angle from the ADC
        estimate_velocity_encoder(encoder_data); ///< Estimate the velocity using encoder data
        xSemaphoreGive(back_params_mutex);
        // // Log every 100ms because of the ESP_LOGI overhead
        // static int counter = 0;
        // if (++counter >= 50) {  // 2ms × 50 = 100ms
        //     ESP_LOGI(task_name, "Velocity: %.2f", encoder_data->velocity);
        //     counter = 0;
        // }
        xTaskNotify(*params->control_task, 0x01, eSetBits);
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

void vTaskSetPWMRight(void *pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    extern SemaphoreHandle_t right_params_mutex;
    extern QueueHandle_t rw_pwm_queue;

    float value;

    while (1) {
        xQueueReceive(rw_pwm_queue, (void *)&value, portMAX_DELAY);    
        xSemaphoreTake(right_params_mutex, portMAX_DELAY);
        bldc_set_duty(params->pwm_motor, value);
        xSemaphoreGive(right_params_mutex);
    }
}

void vTaskSetPWMLeft(void *pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    extern SemaphoreHandle_t left_params_mutex;
    extern QueueHandle_t lw_pwm_queue;

    float value;

    while (1) {
        xQueueReceive(lw_pwm_queue, (void *)&value, portMAX_DELAY);    
        xSemaphoreTake(left_params_mutex, portMAX_DELAY);
        bldc_set_duty(params->pwm_motor, value);
        xSemaphoreGive(left_params_mutex);
    }
}

void vTaskSetPWMBack(void *pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    extern SemaphoreHandle_t back_params_mutex;
    extern QueueHandle_t bw_pwm_queue;

    float value;

    while (1) {
        xQueueReceive(bw_pwm_queue, (void *)&value, portMAX_DELAY);    
        xSemaphoreTake(back_params_mutex, portMAX_DELAY);
        bldc_set_duty(params->pwm_motor, value);
        xSemaphoreGive(back_params_mutex);
    }
}

// Task to control the right wheel
void vTaskControlRight( void * pvParameters ){

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure

    pid_block_handle_t pid_block = *(params->pid_block); ///< PID control block handle

    uint32_t timestamp = 1000000, counter = 0; // 1 second
    bool move = true; ///< Flag to indicate if the robot should move

    float est_velocity = 0.0f, last_est_velocity = 0.0f;
    // float beta = exp(-2 * PI * 1 / 100);  // 10Hz cutoff frequency
    float output = 0.0f;
    float setpoint = 0.0f;

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);
    extern SemaphoreHandle_t right_params_mutex;
    extern QueueHandle_t rw_pwm_queue; 

    while (1)
    {
        ///<-------------- PID Control ---------------
        // Low-pass filter
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        xSemaphoreTake(right_params_mutex, portMAX_DELAY);
        est_velocity = encoder_data->velocity;

        last_est_velocity = est_velocity; ///< Update the last estimated velocity
        xSemaphoreGive(right_params_mutex);
        
        switch (movement) ///< Check the movement type
        {
        case LINEAR:
            cal_lin_to_ang_velocity(x_vel, y_vel, SELECT_RIGHT, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case CIRCULAR:
            // circular_movement(1, 5, 360, 15, &x_vel, &y_vel); ///< Calculate the circular movement
            // cal_lin_to_ang_velocity(params->x_vel, params->y_vel, params->vel_selection, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case ROTATION:
            // cal_lin_to_ang_velocity(params->x_vel, params->y_vel, params->vel_selection, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case DO_NOT_MOVE:
            setpoint = 0.0f; ///< Set the setpoint to 0 for no movement
            break;
        
        default:
            break;
        }

        if (pid_update_set_point(pid_block, setpoint) != PID_OK) {
            ESP_LOGE(task_name, "Failed to update PID parameters for %s", task_name);
        }

        // Update PID Controller
        pid_compute(pid_block, est_velocity, &output);
        xQueueSend(rw_pwm_queue, (void *)&output, portMAX_DELAY); ///< Send data to pwm queue

        // Log every 100ms because of the ESP_LOGI overhead
        // static int ctr = 0;
        // if (++ctr >= 150) {  // 2ms × 50 = 100ms
        //     // ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f", est_velocity, output); ///< Log the PID parameters
        //     ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f\tSetpoint: %.2f", est_velocity, output, setpoint); ///< Log the PID parameters
        //     ctr = 0;
        // }
        
    }
}

// Task to control the left wheel
void vTaskControlLeft( void * pvParameters ){

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure

    pid_block_handle_t pid_block = *(params->pid_block); ///< PID control block handle

    uint32_t timestamp = 1000000, counter = 0; // 1 second
    bool move = true; ///< Flag to indicate if the robot should move

    float est_velocity = 0.0f, last_est_velocity = 0.0f;
    // float beta = exp(-2 * PI * 1 / 100);  // 10Hz cutoff frequency
    float output = 0.0f;
    float setpoint = 0.0f;

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);
    extern SemaphoreHandle_t left_params_mutex;
    extern QueueHandle_t lw_pwm_queue; 

    while (1)
    {
        ///<-------------- PID Control ---------------
        // Low-pass filter
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        xSemaphoreTake(left_params_mutex, portMAX_DELAY);
        est_velocity = encoder_data->velocity;

        last_est_velocity = est_velocity; ///< Update the last estimated velocity
        xSemaphoreGive(left_params_mutex);
        
        switch (movement) ///< Check the movement type
        {
        case LINEAR:
            cal_lin_to_ang_velocity(x_vel, y_vel, SELECT_LEFT, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case CIRCULAR:
            // circular_movement(1, 5, 360, 15, &x_vel, &y_vel); ///< Calculate the circular movement
            // cal_lin_to_ang_velocity(params->x_vel, params->y_vel, params->vel_selection, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case ROTATION:
            // cal_lin_to_ang_velocity(params->x_vel, params->y_vel, params->vel_selection, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case DO_NOT_MOVE:
            setpoint = 0.0f; ///< Set the setpoint to 0 for no movement
            break;
        
        default:
            break;
        }

        if (pid_update_set_point(pid_block, setpoint) != PID_OK) {
            ESP_LOGE(task_name, "Failed to update PID parameters for %s", task_name);
        }

        // Update PID Controller
        pid_compute(pid_block, est_velocity, &output);
        xQueueSend(lw_pwm_queue, (void *)&output, portMAX_DELAY); ///< Send data to pwm queue

        // Log every 100ms because of the ESP_LOGI overhead
        // static int ctr = 0;
        // if (++ctr >= 150) {  // 2ms × 50 = 100ms
        //     // ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f", est_velocity, output); ///< Log the PID parameters
        //     ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f\tSetpoint: %.2f", est_velocity, output, setpoint); ///< Log the PID parameters
        //     ctr = 0;
        // }
        
    }
}

void vTaskControlBack( void * pvParameters ){

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure

    pid_block_handle_t pid_block = *(params->pid_block); ///< PID control block handle

    uint32_t timestamp = 1000000, counter = 0; // 1 second
    bool move = true; ///< Flag to indicate if the robot should move

    float est_velocity = 0.0f, last_est_velocity = 0.0f;
    // float beta = exp(-2 * PI * 1 / 100);  // 10Hz cutoff frequency
    float output = 0.0f;
    float setpoint = 0.0f;

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);
    extern SemaphoreHandle_t back_params_mutex;
    extern QueueHandle_t bw_pwm_queue; 

    while (1)
    {
        ///<-------------- PID Control ---------------
        // Low-pass filter
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        xSemaphoreTake(back_params_mutex, portMAX_DELAY);
        est_velocity = encoder_data->velocity;

        last_est_velocity = est_velocity; ///< Update the last estimated velocity
        xSemaphoreGive(back_params_mutex);
        
        switch (movement) ///< Check the movement type
        {
        case LINEAR:
            cal_lin_to_ang_velocity(x_vel, y_vel, SELECT_BACK, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case CIRCULAR:
            // circular_movement(1, 5, 360, 15, &x_vel, &y_vel); ///< Calculate the circular movement
            // cal_lin_to_ang_velocity(params->x_vel, params->y_vel, params->vel_selection, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case ROTATION:
            // cal_lin_to_ang_velocity(params->x_vel, params->y_vel, params->vel_selection, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case DO_NOT_MOVE:
            setpoint = 0.0f; ///< Set the setpoint to 0 for no movement
            break;
        
        default:
            break;
        }

        if (pid_update_set_point(pid_block, setpoint) != PID_OK) {
            ESP_LOGE(task_name, "Failed to update PID parameters for %s", task_name);
        }

        // Update PID Controller
        pid_compute(pid_block, est_velocity, &output);
        xQueueSend(bw_pwm_queue, (void *)&output, portMAX_DELAY); ///< Send data to pwm queue

        // Log every 100ms because of the ESP_LOGI overhead
        // static int ctr = 0;
        // if (++ctr >= 150) {  // 2ms × 50 = 100ms
        //     // ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f", est_velocity, output); ///< Log the PID parameters
        //     ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f\tSetpoint: %.2f", est_velocity, output, setpoint); ///< Log the PID parameters
        //     ctr = 0;
        // }
        
    }
}


void vTaskDistance(void *pvParameters){
    
    distance_params_t *params = (distance_params_t *)pvParameters; ///< Distance parameters structure
    encoder_data_t *encoder_data_right = params->encoder_data_right; ///< Encoder data structure for right wheel
    encoder_data_t *encoder_data_left = params->encoder_data_left; ///< Encoder data structure for left wheel
    encoder_data_t *encoder_data_back = params->encoder_data_back; ///< Encoder data structure for back wheel
    float dx, dy, distance = 0, beta = 0.9; ///< Variables to store the distance

    while(1){
        static uint16_t time_count = 0; ///< Counter to keep track of the number of iterations

        if(time_count >= goal_time * 1000 && movement == LINEAR) { ///< Check if the goal time has been reached
            movement = DO_NOT_MOVE; ///< Set the movement to do not move
            time_count = 0; ///< Reset the time count
        } else {
            time_count += 5 * SAMPLE_TIME; ///< Increment the time count
        }

        vTaskDelay(5 * SAMPLE_TIME / portTICK_PERIOD_MS); ///< Wait for 2 ms
    }

}

void vTaskUDPServer(void *pvParameters)
{

    // ESP_LOGI("WIFI", "UDP server listening on port %d", PORT);

    char rx_buffer[128];
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)};

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE("WIFI", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        ESP_LOGE("WIFI", "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    

    while (1)
    {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len < 0)
        {
            ESP_LOGE("WIFI", "recvfrom failed: errno %d", errno);
            continue;
        }

        rx_buffer[len] = 0;
        ESP_LOGI("WIFI", "Received: %s", rx_buffer);

        if (strncmp(rx_buffer, "L ", 2) == 0)
        {
            char direction[16];
            float degrees, velocity, distance;
            sscanf(rx_buffer + 2, "%s %f %f %f", direction, &degrees, &velocity, &distance);
            uint8_t forward = 0;
            if (strcmp(direction, "Forward") == 0)
            {
                forward = 1; ///< Set forward movement
            }
            else if (strcmp(direction, "Backward") == 0)
            {
                forward = 0; ///< Set backward movement
            }
            else
            {
                ESP_LOGE("WIFI", "Invalid direction: %s", direction);
                continue;
            }
            // lógica de movimiento lineal
            movement = LINEAR; ///< Set the movement type to linear
            goal_time = distance / velocity; ///< Calculate the goal time in seconds
            linear_movement(forward, distance, degrees, &x_vel, &y_vel); ///< Calculate the linear movement
        }
        else if (strncmp(rx_buffer, "C ", 2) == 0)
        {
            char direction[16];
            float degrees, velocity, radius;
            sscanf(rx_buffer + 2, "%s %f %f %f", direction, &degrees, &velocity, &radius);
            // lógica de movimiento circular
        }
        else if (strncmp(rx_buffer, "R ", 2) == 0)
        {
            char direction[16];
            float degrees, velocity;
            sscanf(rx_buffer + 2, "%s %f %f", direction, &degrees, &velocity);
            // lógica de rotación sobre sí mismo
        }
    }

    close(sock);

}
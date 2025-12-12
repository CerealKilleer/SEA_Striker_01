#include "control_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_http_server.h"
#include "kalman_filter_1d.h"
#include "mov_calculation.h"

float degrees;
float velocity;
float radius;
bool cw;
float wb;

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
    .kp = 0.02,
    .ki = 0.01,
    .kd = 0.00,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramL = {
    .kp = 0.02,
    .ki = 0.01,
    .kd = 0.00,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramB = {
    .kp = PID_KP,
    .ki = 0.007,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

kalman_filter_t km_right_wheel, km_left_wheel, km_back_wheel;
enum movements_num movement; ///< Movement type
float x_vel = 0.0f, y_vel = 0.0f; ///< Generalized velocities for the robot
float goal_time = 0.0f; ///< Goal time for linear movement in seconds

/**
 * @brief Inicializa por defecto las estructuras del filtro de kalman para las ruedas
 */
void init_kalman_parameters(void)
{
    kalman_init(&km_right_wheel, 0.005f, 1.0f);
    kalman_init(&km_left_wheel, 0.005f, 1.0f);
    kalman_init(&km_back_wheel, 0.005f, 1.0f);
}

void vTaskEncodersGateKeeper(void *pvParameters) {
    struct enc_gk_params *params = (struct enc_gk_params *)pvParameters;
    union float_to_int32 angle;
    while(1) {
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        angle.f_value = AS5600_ADC_GetAngle(params->r_enc);
        xTaskNotify(*params->r_wheel, angle.int_value, eSetValueWithOverwrite);
        angle.f_value = AS5600_ADC_GetAngle(params->l_enc);
        xTaskNotify(*params->l_wheel, angle.int_value, eSetValueWithOverwrite);
        angle.f_value = AS5600_ADC_GetAngle(params->b_enc);
        xTaskNotify(*params->b_wheel, angle.int_value, eSetValueWithOverwrite);
    }
}

// Task to read from encoder
void vTaskEncoderRight(void * pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure
    extern SemaphoreHandle_t right_params_mutex;

    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);
    
    union float_to_int32 angle;
    
    ///<-------------- Get angle through ADC -------------
    while (1) {
        xTaskNotifyWait(0x00000000, 0x00000000, &angle.int_value, portMAX_DELAY);
        xSemaphoreTake(right_params_mutex, portMAX_DELAY);
        encoder_data->angle = angle.f_value; ///< Get the angle from the ADC
        static int ctr = 0;
        estimate_velocity_encoder(encoder_data); ///< Estimate the velocity using encoder data

        //Se aplica el filtro de Kalman a la velocidad medida en los encoders
        kalman_update(&km_right_wheel, encoder_data->velocity);
        encoder_data->last_vel = encoder_data->velocity;

        xSemaphoreGive(right_params_mutex);
        // // Log every 100ms because of the ESP_LOGI overhead
        // static int counter = 0;
        // if (++counter >= 50) {  // 2ms × 50 = 100ms
        //     ESP_LOGI(task_name, "Velocity: %.2f", encoder_data->velocity);
        //     counter = 0;
        // }
        xTaskNotify(*params->control_task, 0x00, eNoAction);
    }
    ///<--------------------------------------------------
}

void vTaskEncoderLeft(void * pvParameters) {

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure
    extern SemaphoreHandle_t left_params_mutex;
    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
    union float_to_int32 angle;
    // Get task name
    const char *task_name = pcTaskGetName(xTask);

    ///<-------------- Get angle through ADC -------------
    while (1) {
        xTaskNotifyWait(0x00000000, 0x00000000, &angle.int_value, portMAX_DELAY);
        xSemaphoreTake(left_params_mutex, portMAX_DELAY);
        encoder_data->angle = -angle.f_value; ///< Get the angle from the ADC
        estimate_velocity_encoder(encoder_data); ///< Estimate the velocity using encoder data
        
        //Se aplica el filtro de Kalman a la velocidad medida en los encoders
        kalman_update(&km_left_wheel, encoder_data->velocity);
        encoder_data->last_vel = encoder_data->velocity;

        xSemaphoreGive(left_params_mutex);
        // // Log every 100ms because of the ESP_LOGI overhead
        // static int counter = 0;
        // if (++counter >= 50) {  // 2ms × 50 = 100ms
        //     ESP_LOGI(task_name, "Velocity: %.2f", encoder_data->velocity);
        //     counter = 0;
        // }
        xTaskNotify(*params->control_task, 0x00, eNoAction);
    }
    ///<--------------------------------------------------
}

void vTaskEncoderBack(void * pvParameters) {

    control_params_t *params = (control_params_t *)pvParameters; ///< Control parameters structure
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data; ///< Encoder data structure
    
    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);
    extern SemaphoreHandle_t back_params_mutex;
    union float_to_int32 angle;
    ///<-------------- Get angle through ADC -------------
    while (1) {
        xTaskNotifyWait(0x00000000, 0x00000000, &angle.int_value, portMAX_DELAY);
        
        xSemaphoreTake(back_params_mutex, portMAX_DELAY);
        
        encoder_data->angle = angle.f_value; ///< Get the angle from the ADC
        estimate_velocity_encoder(encoder_data); ///< Estimate the velocity using encoder data

        //Se aplica el filtro de Kalman a la velocidad medida en los encoders
        kalman_update(&km_back_wheel, encoder_data->velocity);
        encoder_data->last_vel = encoder_data->velocity;


        xSemaphoreGive(back_params_mutex);
        // // Log every 100ms because of the ESP_LOGI overhead
        // static int counter = 0;
        // if (++counter >= 50) {  // 2ms × 50 = 100ms
        //     ESP_LOGI(task_name, "Velocity: %.2f", encoder_data->velocity);
        //     counter = 0;
        // }
        xTaskNotify(*params->control_task, 0x00, eNoAction);
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
        // SerialPort_DataReceived_RawAcc(myUART, acceleration); ///< Read acceleration data from TM151 IMU
        // SerialPort_DataReceived_RawYaw(myUART, &yaw); ///< Read yaw data from TM151 IMU
        
        // Estimate the velocity using IMU data
        // estimate_velocity_imu(imu_data, acceleration[0], SAMPLE_TIME / 1000.0f); ///< Estimate the velocity using IMU data

        // Log every 100ms because of the ESP_LOGI overhead
        /*ESP_LOGI(task_name, "Acceleration: [\t%.2f,\t%.2f,\t%.2f]\t Yaw: %.2f", acceleration[0], acceleration[1], acceleration[2], yaw);*/
        
        vTaskDelay(300 / portTICK_PERIOD_MS); ///< Wait for 300ms
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
    float wb_rad_s;
    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);
    extern SemaphoreHandle_t right_params_mutex;

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
            cal_lin_to_ang_velocity(x_vel, y_vel, 0, SELECT_RIGHT, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case CIRCULAR:
            circular_movement(cw, velocity, degrees, radius, &x_vel, &y_vel); ///< Calculate the circular movement
            cal_lin_to_ang_velocity(x_vel, y_vel, 0, SELECT_RIGHT, &setpoint);
            break;
        case ROTATION:
            wb_rad_s = rotate_on_axis(cw, wb, degrees, &movement);
            cal_lin_to_ang_velocity(x_vel, y_vel, wb_rad_s, SELECT_RIGHT, &setpoint); ///< Calculate the setpoint based on the predefined movements
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
        bldc_set_duty(params->pwm_motor, output);

        // Log every 100ms because of the ESP_LOGI overhead
         static int ctr = 0;
         if (++ctr >= 150) {  // 2ms × 50 = 100ms
             // ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f", est_velocity, output); ///< Log the PID parameters
             ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f\tSetpoint: %.2f", est_velocity, output, setpoint); ///< Log the PID parameters
             ESP_LOGI(task_name, "X_vel: %.2f\tY_vel: %.2f", x_vel, y_vel); ///< Log the PID parameters
             ctr = 0;
          }
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
    float wb_rad_s;
    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);
    extern SemaphoreHandle_t left_params_mutex;

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
            cal_lin_to_ang_velocity(x_vel, y_vel, 0, SELECT_LEFT, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case CIRCULAR:
            circular_movement(cw, velocity, degrees, radius, &x_vel, &y_vel); ///< Calculate the circular movement
            cal_lin_to_ang_velocity(x_vel, y_vel, 0, SELECT_LEFT, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case ROTATION:
            wb_rad_s = rotate_on_axis(cw, wb, degrees, &movement);
            cal_lin_to_ang_velocity(x_vel, y_vel, wb_rad_s, SELECT_LEFT, &setpoint); ///< Calculate the setpoint based on the predefined movements
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
        bldc_set_duty(params->pwm_motor, output);
        
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
    float wb_rad_s;
    // Get current task handle
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();

    // Get task name
    const char *task_name = pcTaskGetName(xTask);
    extern SemaphoreHandle_t back_params_mutex;

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
            cal_lin_to_ang_velocity(x_vel, y_vel, 0, SELECT_BACK, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case CIRCULAR:
            circular_movement(cw, velocity, degrees, radius, &x_vel, &y_vel); ///< Calculate the circular movement
            cal_lin_to_ang_velocity(x_vel, y_vel, 0, SELECT_BACK, &setpoint); ///< Calculate the setpoint based on the predefined movements
            break;
        case ROTATION:
            wb_rad_s = rotate_on_axis(cw, wb, degrees, &movement);
            cal_lin_to_ang_velocity(x_vel, y_vel, wb_rad_s, SELECT_BACK, &setpoint); ///< Calculate the setpoint based on the predefined movements
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

        static int ctr = 0;
         if (++ctr >= 150) {  // 2ms × 50 = 100ms
             // ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f", est_velocity, output); ///< Log the PID parameters
             ESP_LOGI(task_name, "Input: %.2f\tOutput: %.2f\tSetpoint: %.2f", est_velocity, output, setpoint); ///< Log the PID parameters
             ESP_LOGI(task_name, "X_vel: %.2f\tY_vel: %.2f", x_vel, y_vel); ///< Log the PID parameters
             ctr = 0;
          }
        // Update PID Controller
        pid_compute(pid_block, est_velocity, &output);
        bldc_set_duty(params->pwm_motor, output);

        
        
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
            goal_time = 0;
        } else if (movement != DO_NOT_MOVE) {
            time_count += 5 * SAMPLE_TIME; ///< Increment the time count
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}


static bool get_param(httpd_req_t *req, const char *key, char *value, size_t max_len) {
    char query[200];
    if (httpd_req_get_url_query_len(req) >= sizeof(query)) return false;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;

    if (httpd_query_key_value(query, key, value, max_len) == ESP_OK) {
        return true;
    }
    return false;
}


esp_err_t line_handler(httpd_req_t *req) {
    char direction[16], degrees_s[16], velocity_s[16], time_s[16];
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

    if (!get_param(req, "direction", direction, sizeof(direction)) ||
        !get_param(req, "degrees", degrees_s, sizeof(degrees_s)) ||
        !get_param(req, "velocity", velocity_s, sizeof(velocity_s)) ||
        !get_param(req, "time", time_s, sizeof(time_s))) {

        httpd_resp_sendstr(req, "Missing parameters");
        return ESP_FAIL;
    }

    float degrees = atof(degrees_s);
    float velocity = atof(velocity_s);
    float time = atof(time_s);

    uint8_t forward = strcmp(direction, "Forward") == 0 ? 1 : 0;

    ESP_LOGI("HTTP", "LINE movement: dir=%s deg=%.2f vel=%.2f time=%.2f",
             direction, degrees, velocity, time);

    movement = LINEAR;
    goal_time = time;
    linear_movement(forward, velocity, degrees, &x_vel, &y_vel);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}


esp_err_t circular_handler(httpd_req_t *req) {
    char direction[16], degrees_s[16], velocity_s[16], radius_s[16];
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

    if (!get_param(req, "direction", direction, sizeof(direction)) ||
        !get_param(req, "degrees", degrees_s, sizeof(degrees_s)) ||
        !get_param(req, "velocity", velocity_s, sizeof(velocity_s)) ||
        !get_param(req, "distance", radius_s, sizeof(radius_s))) {

        httpd_resp_sendstr(req, "Missing parameters");
        return ESP_FAIL;
    }

    degrees = atof(degrees_s);
    velocity = atof(velocity_s);
    radius = atof(radius_s);
    
    ESP_LOGI("HTTP", "CIRCULAR: cw = %d dir=%s deg=%.2f vel=%.2f radius=%.2f",
             cw, direction, degrees, velocity, radius);

    movement = CIRCULAR;
    cw = strcmp(direction, "cw") == 0 ? 1 : 0;

    // circular_movement(...)
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}


esp_err_t rotation_handler(httpd_req_t *req) {
    char direction[16], degrees_s[16], velocity_s[16];
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

    if (!get_param(req, "direction", direction, sizeof(direction)) ||
        !get_param(req, "degrees", degrees_s, sizeof(degrees_s)) ||
        !get_param(req, "velocity", velocity_s, sizeof(velocity_s))) {

        httpd_resp_sendstr(req, "Missing parameters");
        return ESP_FAIL;
    }

    degrees = atof(degrees_s);
    wb = atof(velocity_s);
    movement = ROTATION;
    cw = strcmp(direction, "cw") == 0 ? 1 : 0;

    ESP_LOGI("HTTP", "ROTATION: dir=%s deg=%.2f vel=%.2f",
             direction, degrees, velocity);

    // rotate_robot(...)
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

esp_err_t reset_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

    ESP_LOGI("HTTP", "ABORT/RESET: Stopping all movements");

    // Set movement to DO_NOT_MOVE and reset velocities
    movement = DO_NOT_MOVE;
    x_vel = 0.0f;
    y_vel = 0.0f;
    goal_time = 0.0f;


    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "RESET OK");
    return ESP_OK;
}

static esp_err_t cors_options_handler(httpd_req_t *req) {

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

    // Respuesta vacía en OPTIONS
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}


httpd_handle_t start_http_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {

        httpd_uri_t uri_line = {
            .uri = "/lineMovement",
            .method = HTTP_GET,
            .handler = line_handler
        };

        httpd_uri_t uri_circle = {
            .uri = "/circularMovement",
            .method = HTTP_GET,
            .handler = circular_handler
        };

        httpd_uri_t uri_rotate = {
            .uri = "/selfRotation",
            .method = HTTP_GET,
            .handler = rotation_handler
        };

        httpd_uri_t uri_reset = {
            .uri = "/reset",
            .method = HTTP_GET,
            .handler = reset_handler
        };

        httpd_uri_t cors_options = {
            .uri = "/",
            .method = HTTP_OPTIONS,
            .handler = cors_options_handler
        };

        httpd_register_uri_handler(server, &cors_options);
        httpd_register_uri_handler(server, &uri_line);
        httpd_register_uri_handler(server, &uri_circle);
        httpd_register_uri_handler(server, &uri_rotate);
        httpd_register_uri_handler(server, &uri_reset);

        ESP_LOGI("HTTP", "HTTP Server started");
    }

    return server;
}

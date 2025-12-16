#include "control_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_http_server.h"
#include "kalman_filter_1d.h"
#include "mov_calculation.h"

// ================== SHARED VARIABLES WITH MUTEXES ==================

// Current wheel velocities (updated by encoder tasks)
wheel_velocities_t wheel_velocities = {0.0f, 0.0f, 0.0f};
SemaphoreHandle_t wheel_velocities_mutex = NULL;

// Body measurements (updated by sensor fusion task)
body_measurements_t body_measurements = {0.0f, 0.0f, 0.0f, 0.0f};
SemaphoreHandle_t body_measurements_mutex = NULL;

// Velocity setpoints from server
velocity_setpoints_t velocity_setpoints = {0.0f, 0.0f, 0.0f};
SemaphoreHandle_t velocity_setpoints_mutex = NULL;

// Target velocities after global control
velocity_targets_t velocity_targets = {0.0f, 0.0f, 0.0f};
SemaphoreHandle_t velocity_targets_mutex = NULL;

// Wheel target velocities (inverse kinematics output)
wheel_targets_t wheel_targets = {0.0f, 0.0f, 0.0f};
SemaphoreHandle_t wheel_targets_mutex = NULL;


// ================== ENCODER DATA ==================

imu_data_t imu_data = {
    .velocity = 0.0f,         ///< Velocity in cm/s
    .prev_acc = 0.0f,         ///< Previous acceleration values
    .vel_X = 0.0f,            ///< Velocity in X direction cm/s
    .vel_Y = 0.0f,            ///< Velocity in Y direction cm/s
    .last_vel_X = 0.0f,       ///< Last velocity in X direction cm/s
    .last_vel_Y = 0.0f,       ///< Last velocity in Y direction cm/s
    .last_yaw = 0.0f,         ///< Last yaw angle in degrees
    .gyro_z = 0.0f,           ///< Gyro Z axis in degrees/s
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
// PID parameters for each wheel
pid_parameter_t pid_paramR = {
    .kp = 0.03,
    .ki = 0.005,
    .kd = 0.00,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramL = {
    .kp = 0.03,
    .ki = 0.005,
    .kd = 0.00,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

pid_parameter_t pid_paramB = {
    .kp = 0.03,
    .ki = 0.005,
    .kd = PID_KD,
    .max_output = 70.0f,
    .min_output = -70.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .beta = 0.0f
};

// Kalman filter instances for each wheel
kalman_filter_t km_right_wheel, km_left_wheel, km_back_wheel;

kalman_filter_t km_vx_imu, km_vy_imu, km_wb_imu;

// PID parameters for global velocity control (high-level control)
pid_parameter_t pid_xvel_param = {
    .kp = 1.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .max_output = 20.0f,
    .min_output = -20.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_POSITIONAL,
    .beta = 0.0f
};

pid_parameter_t pid_yvel_param = {
    .kp = 1.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .max_output = 20.0f,
    .min_output = -20.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_POSITIONAL,
    .beta = 0.0f
};

pid_parameter_t pid_wb_param = {
    .kp = 0.05f,
    .ki = 0.000f,
    .kd = 0.0f,
    .max_output = 3.0f,
    .min_output = -3.0f,
    .set_point = 0.0f,
    .cal_type = PID_CAL_TYPE_POSITIONAL,
    .beta = 0.0f
};

// body_measurements_t body_meas = {
//     .v_x_measured = 0.0f,
//     .v_y_measured = 0.0f,
//     .v_wb_measured = 0.0f,
//     .yaw = 0.0f
// };
// enum movements_num movement; ///< Movement type
// float x_vel_desired = 0.0f, y_vel_desired = 0.0f; ///< Generalized velocities for the robot
float goal_time = 0.0f; ///< Goal time for linear movement in seconds

float degrees_circular;
float velocity_circular;
float radius_trayectory;
float rotation_velocity; ///< Angular velocity for rotation movement in rad/s
float rotation_duration; ///< Duration for rotation movement in seconds

float delta_time = 0.0f;
float current_time = 0.0f, previous_time = 0.0f;
bool cw;
// float wb;

enum movements_num movement; ///< Movement type
// ================== INITIALIZATION FUNCTIONS ==================

/**
 * @brief Initialize Kalman filter parameters for all wheels
 */
void init_kalman_parameters(void)
{
    kalman_init(&km_right_wheel, 0.005f, 1.0f);
    kalman_init(&km_left_wheel, 0.005f, 1.0f);
    kalman_init(&km_back_wheel, 0.005f, 1.0f);

    kalman_init(&km_vx_imu, 0.01f, 1.0f);
    kalman_init(&km_vy_imu, 0.01f, 1.0f);
    kalman_init(&km_wb_imu, 0.01f, 1.0f);
}

// ================== GATEKEEPER TASK ==================

void vTaskEncodersGateKeeper(void *pvParameters) {
    struct enc_gk_params *params = (struct enc_gk_params *)pvParameters;
    union float_to_int32 angle;

    while(1) {
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

// ================== ENCODER READING TASKS ==================

void vTaskEncoderRight(void * pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data;
    extern SemaphoreHandle_t right_params_mutex;
    extern SemaphoreHandle_t wheel_velocities_mutex;

    union float_to_int32 angle;

    while (1) {
        // Wait for notification from gatekeeper
        xTaskNotifyWait(0x00000000, 0x00000000, &angle.int_value, portMAX_DELAY);

        // Update encoder data with mutex
        xSemaphoreTake(right_params_mutex, portMAX_DELAY);
        encoder_data->angle = angle.f_value;
        estimate_velocity_encoder(encoder_data);
        kalman_update(&km_right_wheel, encoder_data->velocity);
        encoder_data->last_vel = encoder_data->velocity;
        float current_vel = encoder_data->velocity;
        xSemaphoreGive(right_params_mutex);

        // Update shared wheel velocities
        xSemaphoreTake(wheel_velocities_mutex, portMAX_DELAY);
        wheel_velocities.right_wheel_vel = current_vel;
        xSemaphoreGive(wheel_velocities_mutex);

        // Notify control task
        xTaskNotify(*params->control_task, 0x00, eNoAction);
    }
}

void vTaskEncoderLeft(void * pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data;
    extern SemaphoreHandle_t left_params_mutex;
    extern SemaphoreHandle_t wheel_velocities_mutex;

    union float_to_int32 angle;

    while (1) {
        // Wait for notification from gatekeeper
        xTaskNotifyWait(0x00000000, 0x00000000, &angle.int_value, portMAX_DELAY);

        // Update encoder data with mutex
        xSemaphoreTake(left_params_mutex, portMAX_DELAY);
        encoder_data->angle = -angle.f_value;
        estimate_velocity_encoder(encoder_data);
        kalman_update(&km_left_wheel, encoder_data->velocity);
        encoder_data->last_vel = encoder_data->velocity;
        float current_vel = encoder_data->velocity;
        xSemaphoreGive(left_params_mutex);

        // Update shared wheel velocities
        xSemaphoreTake(wheel_velocities_mutex, portMAX_DELAY);
        wheel_velocities.left_wheel_vel = current_vel;
        xSemaphoreGive(wheel_velocities_mutex);

        // Notify control task
        xTaskNotify(*params->control_task, 0x00, eNoAction);
    }
}

void vTaskEncoderBack(void * pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data;
    extern SemaphoreHandle_t back_params_mutex;
    extern SemaphoreHandle_t wheel_velocities_mutex;

    union float_to_int32 angle;

    while (1) {
        // Wait for notification from gatekeeper
        xTaskNotifyWait(0x00000000, 0x00000000, &angle.int_value, portMAX_DELAY);

        // Update encoder data with mutex
        xSemaphoreTake(back_params_mutex, portMAX_DELAY);
        encoder_data->angle = angle.f_value;
        estimate_velocity_encoder(encoder_data);
        kalman_update(&km_back_wheel, encoder_data->velocity);
        encoder_data->last_vel = encoder_data->velocity;
        float current_vel = encoder_data->velocity;
        xSemaphoreGive(back_params_mutex);

        // Update shared wheel velocities
        xSemaphoreTake(wheel_velocities_mutex, portMAX_DELAY);
        wheel_velocities.back_wheel_vel = current_vel;
        xSemaphoreGive(wheel_velocities_mutex);

        // TODO: Revisar si es necesario notificar al control task, o mejor hacerlo desde el gatekeeper o de forma períodica
        xTaskNotify(*params->control_task, 0x00, eNoAction);
    }
}

// ================== IMU READING TASK ==================

void vTaskIMU(void * pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    imu_data_t *imu_data = (imu_data_t *)params->imu_data;
    uart_t *myUART = params->myUART;
    extern SemaphoreHandle_t body_measurements_mutex;

    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
    const char *task_name = pcTaskGetName(xTask);

    float acceleration[3], gyro[3], yaw;
    const float time_interval = 10.0f / 1000.0f; // 10ms period in seconds

    while (1) {
        // Read acceleration and yaw data from TM151 IMU
        SerialPort_DataReceived_RawAcc(myUART, acceleration, gyro);
        SerialPort_DataReceived_RawYaw(myUART, &yaw);

        // Calculate body velocities in X and Y, and store gyro_z
        estimate_body_velocities_imu(imu_data, acceleration[0], acceleration[1], gyro[2], yaw, time_interval);

        // Apply Kalman filter to angular velocity (in rad/s)
        float wb_rad_s_raw = gyro[2] * PI / 180.0f;
        kalman_update(&km_wb_imu, wb_rad_s_raw);
        float wb_rad_s_filtered = km_wb_imu.x;
        
        // Convert filtered wb back to deg/s and store in imu_data
        imu_data->gyro_z = wb_rad_s_filtered * 180.0f / PI;  // Store filtered gyro in deg/s


        //apply kalman filter to imu vx and vy
        kalman_update(&km_vx_imu, imu_data->vel_X);
        kalman_update(&km_vy_imu, imu_data->vel_Y);
        imu_data->vel_X = km_vx_imu.x;
        imu_data->vel_Y = km_vy_imu.x;

        // Update body measurements with IMU data
        xSemaphoreTake(body_measurements_mutex, portMAX_DELAY);
        body_measurements.yaw = yaw;
        xSemaphoreGive(body_measurements_mutex);

        // Log periodically (every 500ms)
        static int ctr = 0;
        if (++ctr >= 250) {  // 2ms × 250 = 500ms
            ESP_LOGI(task_name, "IMU Yaw: %.2f° | Gyro_Z: %.2f deg/s | wb: %.2f rad/s", yaw, imu_data->gyro_z, wb_rad_s_filtered);
            ESP_LOGI(task_name, "IMU Vel_X: %.2f cm/s | Vel_Y: %.2f cm/s", imu_data->vel_X, imu_data->vel_Y);
            ctr = 0;
        }

        // TODO: Revisar período de lectura del IMU
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10ms
    }
}

// ================== SENSOR FUSION TASK ==================

void vTaskSensorFusion(void *pvParameters) {
    extern SemaphoreHandle_t wheel_velocities_mutex;
    extern SemaphoreHandle_t body_measurements_mutex;

    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
    const char *task_name = pcTaskGetName(xTask);

    float right_vel, left_vel, back_vel;
    float v_x_fk, v_y_fk, v_wb_fk;

    float imu_vx, imu_vy, imu_wb;
    
    // ========== SENSOR FUSION FOR ANGULAR VELOCITY ==========
    static float prev_yaw = 0.0f;
    static bool first_run = true;
    const float SENSOR_FUSION_PERIOD = 0.01f;  // 10ms period in seconds
    
    // Complementary filter weight (0.0 = trust yaw derivative only, 1.0 = trust gyro only)
    // Typical values: 0.95-0.98 for gyro bias
    const float ALPHA_GYRO = 0.2f;  // High trust in gyro for short-term accuracy
    const float ALPHA_YAW = 1.0f - ALPHA_GYRO;  // Low trust in yaw derivative (noisy but drift-free)

    while (1) {
        // Wait for notification (triggered by timer interrupt every 10ms)
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);

        // 1. Read current wheel velocities
        xSemaphoreTake(wheel_velocities_mutex, portMAX_DELAY);
        right_vel = wheel_velocities.right_wheel_vel;
        left_vel = wheel_velocities.left_wheel_vel;
        back_vel = wheel_velocities.back_wheel_vel;
        xSemaphoreGive(wheel_velocities_mutex);

        // 2. Apply forward kinematics to get body velocities
        cal_forward_kinematics(left_vel, back_vel, right_vel,
                               &v_x_fk, &v_y_fk, &v_wb_fk);

        // 3. Fuse IMU data with encoders for angular velocity
        imu_wb = imu_data.gyro_z * PI / 180.0f;  // Gyro in rad/s
        imu_vx = imu_data.vel_X;
        imu_vy = imu_data.vel_Y;
        
        // ========== COMPLEMENTARY FILTER FOR ANGULAR VELOCITY ==========
        float wb_fused;
        float current_yaw;
        
        // Read current yaw
        xSemaphoreTake(body_measurements_mutex, portMAX_DELAY);
        current_yaw = body_measurements.yaw;
        xSemaphoreGive(body_measurements_mutex);
        
        if (first_run) {
            // First iteration: initialize with gyro data
            wb_fused = -imu_wb;  // Negative for correct direction
            prev_yaw = current_yaw;
            first_run = false;
        } else {
            // Calculate angular velocity from yaw derivative
            float delta_yaw = current_yaw - prev_yaw;
            
            // Handle yaw wraparound (360° -> 0° or 0° -> 360°)
            if (delta_yaw > 180.0f) {
                delta_yaw -= 360.0f;
            } else if (delta_yaw < -180.0f) {
                delta_yaw += 360.0f;
            }
            
            // Convert yaw derivative to rad/s
            float wb_from_yaw = (delta_yaw * PI / 180.0f) / SENSOR_FUSION_PERIOD;
            
            // Complementary filter: combine gyro (short-term) + yaw derivative (long-term)
            // This corrects gyro drift while maintaining high-frequency response
            wb_fused = ALPHA_GYRO * (-imu_wb) + ALPHA_YAW * wb_from_yaw;
            
            prev_yaw = current_yaw;
        }

        // 4. Update body measurements with fused data
        xSemaphoreTake(body_measurements_mutex, portMAX_DELAY);
        body_measurements.v_x_measured = v_x_fk;
        body_measurements.v_y_measured = v_y_fk;
        body_measurements.v_wb_measured = wb_fused;  // Use fused angular velocity
        // yaw is already updated by IMU task
        xSemaphoreGive(body_measurements_mutex);

        // Log periodically (every 500ms)
        static int ctr = 0;
        if (++ctr >= 50) {  // 10ms × 50 = 500ms
            float wb_from_yaw_dbg = ((current_yaw - prev_yaw) * PI / 180.0f) / SENSOR_FUSION_PERIOD;
            if (ctr == 50) {  // Recalculate for logging
                float delta_yaw_dbg = current_yaw - prev_yaw;
                if (delta_yaw_dbg > 180.0f) delta_yaw_dbg -= 360.0f;
                else if (delta_yaw_dbg < -180.0f) delta_yaw_dbg += 360.0f;
                wb_from_yaw_dbg = (delta_yaw_dbg * PI / 180.0f) / SENSOR_FUSION_PERIOD;
            }
            
            ESP_LOGI(task_name, "Body Vel -> X: %.2f cm/s, Y: %.2f cm/s, W_fk: %.2f rad/s",
                     v_x_fk, v_y_fk, v_wb_fk);
            ESP_LOGI(task_name, "Angular Vel Fusion -> Gyro: %.3f, Yaw_deriv: %.3f, Fused: %.3f rad/s",
                     -imu_wb, wb_from_yaw_dbg, wb_fused);
            ctr = 0;
        }
    }
}

// ================== GLOBAL CONTROL TASK ==================

// void vTaskGlobalControl(void *pvParameters) {
//     global_control_params_t *params = (global_control_params_t *)pvParameters;

//     // PID controllers for global velocities
//     pid_block_handle_t pid_block_xvel = *(params->pid_block_xvel);
//     pid_block_handle_t pid_block_yvel = *(params->pid_block_yvel);
//     pid_block_handle_t pid_block_wb = *(params->pid_block_wb);

//     extern SemaphoreHandle_t body_measurements_mutex;
//     extern SemaphoreHandle_t velocity_setpoints_mutex;
//     extern SemaphoreHandle_t velocity_targets_mutex;
//     extern SemaphoreHandle_t wheel_targets_mutex;

//     TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
//     const char *task_name = pcTaskGetName(xTask);

//     float v_x_measured, v_y_measured, v_wb_measured;
//     float v_x_setpoint=0.0f, v_y_setpoint=0.0f, v_wb_setpoint=0.0f;
//     float v_x_output, v_y_output, v_wb_output;
//     float right_target, left_target, back_target;

    
//     movement = DO_NOT_MOVE;
    
//     // Variables for PID reset and deadband
//     static enum movements_num prev_movement = DO_NOT_MOVE;
//     const float VELOCITY_DEADBAND = 0.5f;   // cm/s - ignore velocities below this
//     const float ANGULAR_DEADBAND = 0.1f;    // rad/s - ignore angular velocities below this


//     while (1) {
//         // Wait for notification from timer (10ms period)
//         xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        
//         // ========== FIX: Reset PIDs on movement state change ==========
//         if (movement != prev_movement) {
//             pid_reset_block(pid_block_xvel);
//             pid_reset_block(pid_block_yvel);
//             pid_reset_block(pid_block_wb);
//             ESP_LOGI(task_name, "Movement changed: %d -> %d, PIDs reset", prev_movement, movement);
//             prev_movement = movement;
//         }

//         // 1. Read velocity setpoints from server
//         // xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
//         // v_x_setpoint = velocity_setpoints.v_x_setpoint;
//         // v_y_setpoint = velocity_setpoints.v_y_setpoint;
//         // v_wb_setpoint = velocity_setpoints.v_wb_setpoint;
//         // xSemaphoreGive(velocity_setpoints_mutex);

//         // 2. Read measured body velocities
//         xSemaphoreTake(body_measurements_mutex, portMAX_DELAY);
//         v_x_measured = body_measurements.v_x_measured;
//         v_y_measured = body_measurements.v_y_measured;
//         v_wb_measured = body_measurements.v_wb_measured;
//         xSemaphoreGive(body_measurements_mutex);

//         switch (movement) {
//             case LINEAR:
//                 // 1. Read velocity setpoints from server
//                 xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
//                 v_x_setpoint = velocity_setpoints.v_x_setpoint;
//                 v_y_setpoint = velocity_setpoints.v_y_setpoint;
//                 v_wb_setpoint = velocity_setpoints.v_wb_setpoint;
//                 xSemaphoreGive(velocity_setpoints_mutex);
//                 break; //it's updated later, no action needed now for linear
//             case CIRCULAR:
//             // 1. Read velocity setpoints from server AND UPDATE CIRCULAR MOVEMENT
//                 //Actualizar tiempo
//                 current_time = esp_timer_get_time() / 1000000.0f; // Convert microseconds to seconds
//                 if (previous_time == 0.0f) {
//                     previous_time = current_time;
//                 }else {
//                     delta_time = current_time - previous_time;
//                     previous_time = current_time;
//                 }
//                 xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
//                 circular_movement(cw, velocity_circular, degrees_circular, radius_trayectory,
//                     &velocity_setpoints.v_x_setpoint, &velocity_setpoints.v_y_setpoint,
//                     &delta_time, &movement);
//                 v_x_setpoint = velocity_setpoints.v_x_setpoint;
//                 v_y_setpoint = velocity_setpoints.v_y_setpoint;
//                 v_wb_setpoint = velocity_setpoints.v_wb_setpoint;
//                 xSemaphoreGive(velocity_setpoints_mutex);

//                 break;
//             case ROTATION:
//                 //rotation_movement(cw, degrees_circular, wb, &v_wb_setpoint);
//                 // 1. Read velocity setpoints from server
//                 xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
//                 v_x_setpoint = velocity_setpoints.v_x_setpoint;
//                 v_y_setpoint = velocity_setpoints.v_y_setpoint;
//                 v_wb_setpoint = velocity_setpoints.v_wb_setpoint;
//                 xSemaphoreGive(velocity_setpoints_mutex);
//                 break;
//             case DO_NOT_MOVE:
//                 v_x_setpoint = 0.0f;
//                 v_y_setpoint = 0.0f;
//                 v_wb_setpoint = 0.0f;

//                                 // ========== FIX: Apply deadband and reset PIDs when stopped ==========
//                 if (fabs(v_x_measured) < VELOCITY_DEADBAND) {
//                     v_x_measured = 0.0f;
//                     pid_reset_block(pid_block_xvel);
//                 }
//                 if (fabs(v_y_measured) < VELOCITY_DEADBAND) {
//                     v_y_measured = 0.0f;
//                     pid_reset_block(pid_block_yvel);
//                 }
//                 if (fabs(v_wb_measured) < ANGULAR_DEADBAND) {
//                     v_wb_measured = 0.0f;
//                     pid_reset_block(pid_block_wb);
//                 }
//                 break;
//             default:
//                 break;
//         }


//         // 3. Update PID setpoints
//         pid_update_set_point(pid_block_xvel, v_x_setpoint);
//         pid_update_set_point(pid_block_yvel, v_y_setpoint);
//         pid_update_set_point(pid_block_wb, v_wb_setpoint);

//         // 4. Compute PID outputs (target velocities after control)
//         pid_compute(pid_block_xvel, v_x_measured, &v_x_output);
//         pid_compute(pid_block_yvel, v_y_measured, &v_y_output);
//         pid_compute(pid_block_wb, v_wb_measured, &v_wb_output);
//         // ========== TEMPORARY: Bypass PID control for testing ==========
//         // v_x_output = v_x_setpoint;  // TEMPORARY: Bypass PID
//         // v_y_output = v_y_setpoint;
//         // v_wb_output = v_wb_setpoint;
//         // Recordar borrar lineas anteriores

//         // 5. Store target velocities
//         xSemaphoreTake(velocity_targets_mutex, portMAX_DELAY);
//         velocity_targets.v_x_target = v_x_output;
//         velocity_targets.v_y_target = v_y_output;
//         velocity_targets.v_wb_target = v_wb_output;
//         xSemaphoreGive(velocity_targets_mutex);

//         // 6. Apply inverse kinematics to get wheel target velocities
//         cal_lin_to_ang_velocity(v_x_output, v_y_output, v_wb_output, SELECT_RIGHT, &right_target);
//         cal_lin_to_ang_velocity(v_x_output, v_y_output, v_wb_output, SELECT_LEFT, &left_target);
//         cal_lin_to_ang_velocity(v_x_output, v_y_output, v_wb_output, SELECT_BACK, &back_target);

//         // 7. Update wheel target velocities for each wheel
//         xSemaphoreTake(wheel_targets_mutex, portMAX_DELAY);
//         wheel_targets.right_wheel_target = right_target;
//         wheel_targets.left_wheel_target = left_target;
//         wheel_targets.back_wheel_target = back_target;
//         xSemaphoreGive(wheel_targets_mutex);

//         // Log periodically (every 500ms)
//         static int ctr = 0;
//         if (++ctr >= 50) {  // 10ms × 50 = 500ms
//             ESP_LOGI(task_name, "Setpoints -> X: %.2f, Y: %.2f, W: %.2f",
//                      v_x_setpoint, v_y_setpoint, v_wb_setpoint);
//             ESP_LOGI(task_name, "Measured -> X: %.2f, Y: %.2f, W: %.2f",
//                      v_x_measured, v_y_measured, v_wb_measured);
//             ESP_LOGI(task_name, "Targets -> X: %.2f, Y: %.2f, W: %.2f",
//                      v_x_output, v_y_output, v_wb_output);
//             ESP_LOGI(task_name, "Wheel Targets -> R: %.2f, L: %.2f, B: %.2f",
//                      right_target, left_target, back_target);
//             ctr = 0;
//         }
//     }
// }
void vTaskGlobalControl(void *pvParameters) {
    global_control_params_t *params = (global_control_params_t *)pvParameters;
    pid_block_handle_t pid_block_xvel = *(params->pid_block_xvel);
    pid_block_handle_t pid_block_yvel = *(params->pid_block_yvel);
    pid_block_handle_t pid_block_wb = *(params->pid_block_wb);

    extern SemaphoreHandle_t body_measurements_mutex;
    extern SemaphoreHandle_t velocity_setpoints_mutex;
    extern SemaphoreHandle_t velocity_targets_mutex;
    extern SemaphoreHandle_t wheel_targets_mutex;

    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
    const char *task_name = pcTaskGetName(xTask);

    float v_x_measured, v_y_measured, v_wb_measured;
    float v_x_setpoint = 0.0f, v_y_setpoint = 0.0f, v_wb_setpoint = 0.0f;
    float v_x_output, v_y_output, v_wb_output;
    float right_target, left_target, back_target;

    static enum movements_num prev_movement = DO_NOT_MOVE;
    
    const float VELOCITY_DEADBAND = 0.5f;
    const float ANGULAR_DEADBAND = 0.1f;
    const float CONTROL_PERIOD = 0.005f;  // 5ms
    
    // ========== Time tracking for movements ==========
    static float circular_time_accum = 0.0f;
    static float accumulated_angle = 0.0f;
    static float accumulated_time = 0.0f;
    static uint32_t control_iterations = 0;

    while (1) {
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        control_iterations++;
        
        // Reset state on movement change
        if (movement != prev_movement) {
            pid_reset_block(pid_block_xvel);
            pid_reset_block(pid_block_yvel);
            pid_reset_block(pid_block_wb);
            
            // Reset all time accumulators
            circular_time_accum = 0.0f;
            accumulated_angle = 0.0f;
            accumulated_time = 0.0f;
            
            ESP_LOGI(task_name, "Movement changed: %d -> %d, state reset", prev_movement, movement);
            prev_movement = movement;
        }

        // Read measured body velocities
        xSemaphoreTake(body_measurements_mutex, portMAX_DELAY);
        v_x_measured = body_measurements.v_x_measured;
        v_y_measured = body_measurements.v_y_measured;
        v_wb_measured = body_measurements.v_wb_measured;
        xSemaphoreGive(body_measurements_mutex);

        switch (movement) {
            case LINEAR:
                xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
                v_x_setpoint = velocity_setpoints.v_x_setpoint;
                v_y_setpoint = velocity_setpoints.v_y_setpoint;
                v_wb_setpoint = velocity_setpoints.v_wb_setpoint;
                xSemaphoreGive(velocity_setpoints_mutex);
                
                accumulated_time += CONTROL_PERIOD;
                
                if (goal_time > 0.0f && accumulated_time >= goal_time) {
                    movement = DO_NOT_MOVE;
                    ESP_LOGI(task_name, "Linear movement complete: %.2f seconds", accumulated_time);
                }
                break;

            case CIRCULAR:
                // ========== FIX: Increment time HERE, then call circular_movement ==========
                circular_time_accum += CONTROL_PERIOD;  // Increment by 5ms
                
                // Calculate total trajectory time
                float total_time = (degrees_circular / 360.0f) * 2.0f * PI * radius_trayectory / velocity_circular;
                
                // Check if movement is complete
                if (circular_time_accum >= total_time) {
                    movement = DO_NOT_MOVE;
                    circular_time_accum = 0.0f;  // Reset for next time
                    
                    xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
                    velocity_setpoints.v_x_setpoint = 0.0f;
                    velocity_setpoints.v_y_setpoint = 0.0f;
                    velocity_setpoints.v_wb_setpoint = 0.0f;
                    xSemaphoreGive(velocity_setpoints_mutex);
                    
                    ESP_LOGI(task_name, "Circular movement complete: %.2f seconds", circular_time_accum);
                } else {
                    // Still moving - calculate current velocities based on trajectory
                    float x_vel_traj, y_vel_traj;
                    
                    if (cw) {
                        x_vel_traj = -velocity_circular * sinf((velocity_circular / radius_trayectory) * circular_time_accum);
                        y_vel_traj = -velocity_circular * cosf((velocity_circular / radius_trayectory) * circular_time_accum);
                    } else {
                        x_vel_traj = -velocity_circular * sinf((velocity_circular / radius_trayectory) * circular_time_accum);
                        y_vel_traj = velocity_circular * cosf((velocity_circular / radius_trayectory) * circular_time_accum);
                    }
                    
                    xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
                    velocity_setpoints.v_x_setpoint = x_vel_traj;
                    velocity_setpoints.v_y_setpoint = y_vel_traj;
                    velocity_setpoints.v_wb_setpoint = 0.0f;
                    xSemaphoreGive(velocity_setpoints_mutex);
                }
                
                xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
                v_x_setpoint = velocity_setpoints.v_x_setpoint;
                v_y_setpoint = velocity_setpoints.v_y_setpoint;
                v_wb_setpoint = velocity_setpoints.v_wb_setpoint;
                xSemaphoreGive(velocity_setpoints_mutex);
                
                // Log progress
                if (control_iterations % 20 == 0) {  // Every ~100ms
                    ESP_LOGD(task_name, "Circular progress: %.2fs / %.2fs (%.1f%%)", 
                             circular_time_accum, total_time, (circular_time_accum / total_time) * 100.0f);
                }
                break;

            case ROTATION:
                // ========== Use rotate_on_axis() with time-based control ==========
                xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
                
                // Call rotate_on_axis() which manages timing internally and updates movement state
                float wb_output = rotate_on_axis(cw, rotation_velocity, rotation_duration, &movement);
                
                // Set the velocity setpoints
                velocity_setpoints.v_x_setpoint = 0.0f;
                velocity_setpoints.v_y_setpoint = 0.0f;
                velocity_setpoints.v_wb_setpoint = wb_output;
                
                v_x_setpoint = velocity_setpoints.v_x_setpoint;
                v_y_setpoint = velocity_setpoints.v_y_setpoint;
                v_wb_setpoint = velocity_setpoints.v_wb_setpoint;
                xSemaphoreGive(velocity_setpoints_mutex);
                
                // Log completion when movement changes to DO_NOT_MOVE
                if (movement == DO_NOT_MOVE) {
                    ESP_LOGI(task_name, "Rotation complete: %.2fs at %.2f rad/s", rotation_duration, rotation_velocity);
                }
                break;

            case DO_NOT_MOVE:
                v_x_setpoint = 0.0f;
                v_y_setpoint = 0.0f;
                v_wb_setpoint = 0.0f;

                if (fabs(v_x_measured) < VELOCITY_DEADBAND) {
                    v_x_measured = 0.0f;
                    pid_reset_block(pid_block_xvel);
                }
                if (fabs(v_y_measured) < VELOCITY_DEADBAND) {
                    v_y_measured = 0.0f;
                    pid_reset_block(pid_block_yvel);
                }
                if (fabs(v_wb_measured) < ANGULAR_DEADBAND) {
                    v_wb_measured = 0.0f;
                    pid_reset_block(pid_block_wb);
                }
                break;

            default:
                break;
        }

        // Update PID setpoints
        pid_update_set_point(pid_block_xvel, v_x_setpoint);
        pid_update_set_point(pid_block_yvel, v_y_setpoint);
        pid_update_set_point(pid_block_wb, v_wb_setpoint);

        // Compute PID outputs
        pid_compute(pid_block_xvel, v_x_measured, &v_x_output);
        pid_compute(pid_block_yvel, v_y_measured, &v_y_output);
        pid_compute(pid_block_wb, v_wb_measured, &v_wb_output);

        // Store target velocities
        xSemaphoreTake(velocity_targets_mutex, portMAX_DELAY);
        velocity_targets.v_x_target = v_x_output;
        velocity_targets.v_y_target = v_y_output;
        velocity_targets.v_wb_target = v_wb_output;
        xSemaphoreGive(velocity_targets_mutex);

        // Apply inverse kinematics
        cal_lin_to_ang_velocity(v_x_output, v_y_output, v_wb_output, SELECT_RIGHT, &right_target);
        cal_lin_to_ang_velocity(v_x_output, v_y_output, v_wb_output, SELECT_LEFT, &left_target);
        cal_lin_to_ang_velocity(v_x_output, v_y_output, v_wb_output, SELECT_BACK, &back_target);

        // Update wheel targets
        xSemaphoreTake(wheel_targets_mutex, portMAX_DELAY);
        wheel_targets.right_wheel_target = right_target;
        wheel_targets.left_wheel_target = left_target;
        wheel_targets.back_wheel_target = back_target;
        xSemaphoreGive(wheel_targets_mutex);

        // Log every 500ms (100 iterations at 5ms)
        if (control_iterations >= 100) {
            ESP_LOGI(task_name, "Movement: %d | Setpoints -> X: %.2f, Y: %.2f, W: %.2f",
                     movement, v_x_setpoint, v_y_setpoint, v_wb_setpoint);
            ESP_LOGI(task_name, "Measured -> X: %.2f, Y: %.2f, W: %.2f",
                     v_x_measured, v_y_measured, v_wb_measured);
            ESP_LOGI(task_name, "Targets -> X: %.2f, Y: %.2f, W: %.2f",
                     v_x_output, v_y_output, v_wb_output);
            
            if (movement == LINEAR && goal_time > 0.0f) {
                ESP_LOGI(task_name, "Linear progress: %.2fs / %.2fs",
                         accumulated_time, goal_time);
            }
            
            control_iterations = 0;
        }
    }
}
// ================== UNIFIED WHEEL CONTROL TASK ==================

/**
 * @brief Unified control task for all three wheels
 * 
 * This task controls all three wheels simultaneously:
 * - Reads all wheel target velocities at once (single mutex acquisition)
 * - Computes PID for each wheel
 * - Actuates all motors together
 * 
 * Advantages: Synchronized control, reduced mutex contention, lower overhead
 * Disadvantages: Single point of failure, less modularity
 */
void vTaskAllWheelsControl(void *pvParameters) {
    // Cast parameters - expects an array of 3 control_params_t
    control_params_t *wheel_params = (control_params_t *)pvParameters;
    
    control_params_t *right_params = &wheel_params[0];
    control_params_t *left_params = &wheel_params[1];
    control_params_t *back_params = &wheel_params[2];
    
    encoder_data_t *right_encoder = (encoder_data_t *)right_params->sensor_data;
    encoder_data_t *left_encoder = (encoder_data_t *)left_params->sensor_data;
    encoder_data_t *back_encoder = (encoder_data_t *)back_params->sensor_data;
    
    pid_block_handle_t pid_right = *(right_params->pid_block);
    pid_block_handle_t pid_left = *(left_params->pid_block);
    pid_block_handle_t pid_back = *(back_params->pid_block);
    
    extern SemaphoreHandle_t right_params_mutex;
    extern SemaphoreHandle_t left_params_mutex;
    extern SemaphoreHandle_t back_params_mutex;
    extern SemaphoreHandle_t wheel_targets_mutex;
    
    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
    const char *task_name = pcTaskGetName(xTask);
    
    const TickType_t MUTEX_TIMEOUT = pdMS_TO_TICKS(3);
    
    float right_vel, left_vel, back_vel;
    float right_target, left_target, back_target;
    float right_output, left_output, back_output;
    
    while (1) {
        // Wait for notification from any encoder task (triggered every 2ms)
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);
        
        // 1. Read all measured velocities with timeout
        if (xSemaphoreTake(right_params_mutex, MUTEX_TIMEOUT) == pdTRUE) {
            right_vel = right_encoder->velocity;
            xSemaphoreGive(right_params_mutex);
        } else {
            ESP_LOGW(task_name, "Right encoder mutex timeout");
            right_vel = 0.0f;
        }
        
        if (xSemaphoreTake(left_params_mutex, MUTEX_TIMEOUT) == pdTRUE) {
            left_vel = left_encoder->velocity;
            xSemaphoreGive(left_params_mutex);
        } else {
            ESP_LOGW(task_name, "Left encoder mutex timeout");
            left_vel = 0.0f;
        }
        
        if (xSemaphoreTake(back_params_mutex, MUTEX_TIMEOUT) == pdTRUE) {
            back_vel = back_encoder->velocity;
            xSemaphoreGive(back_params_mutex);
        } else {
            ESP_LOGW(task_name, "Back encoder mutex timeout");
            back_vel = 0.0f;
        }
        
        // 2. Read all target velocities (single mutex acquisition)
        if (xSemaphoreTake(wheel_targets_mutex, MUTEX_TIMEOUT) == pdTRUE) {
            right_target = wheel_targets.right_wheel_target;
            left_target = wheel_targets.left_wheel_target;
            back_target = wheel_targets.back_wheel_target;
            xSemaphoreGive(wheel_targets_mutex);
        } else {
            ESP_LOGW(task_name, "Wheel targets mutex timeout");
            right_target = left_target = back_target = 0.0f;
        }
        
        // 3. Compute PID for all wheels
        pid_update_set_point(pid_right, right_target);
        pid_compute(pid_right, right_vel, &right_output);
        
        pid_update_set_point(pid_left, left_target);
        pid_compute(pid_left, left_vel, &left_output);
        
        pid_update_set_point(pid_back, back_target);
        pid_compute(pid_back, back_vel, &back_output);
        
        // 4. Actuate all motors simultaneously
        bldc_set_duty(right_params->pwm_motor, right_output);
        bldc_set_duty(left_params->pwm_motor, left_output);
        bldc_set_duty(back_params->pwm_motor, back_output);
        
        // Log periodically (every 500ms)
        static int ctr = 0;
        if (++ctr >= 250) {  // 2ms × 250 = 500ms
            ESP_LOGI(task_name, "R: %.2f->%.2f [%.2f] | L: %.2f->%.2f [%.2f] | B: %.2f->%.2f [%.2f]",
                     right_vel, right_target, right_output,
                     left_vel, left_target, left_output,
                     back_vel, back_target, back_output);
            ctr = 0;
        }
    }
}

// ================== INDIVIDUAL WHEEL CONTROL TASKS ==================

// ================== WHEEL CONTROL TASKS ==================

void vTaskControlRight(void * pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data;
    pid_block_handle_t pid_block = *(params->pid_block);

    extern SemaphoreHandle_t right_params_mutex;
    extern SemaphoreHandle_t wheel_targets_mutex;

    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
    const char *task_name = pcTaskGetName(xTask);

    float measured_velocity, target_velocity, output;

    while (1) {
        // Wait for notification from encoder task (2ms period)
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);

        // 1. Read measured velocity
        xSemaphoreTake(right_params_mutex, portMAX_DELAY);
        measured_velocity = encoder_data->velocity;
        xSemaphoreGive(right_params_mutex);

        // 2. Read target velocity
        xSemaphoreTake(wheel_targets_mutex, portMAX_DELAY);
        target_velocity = wheel_targets.right_wheel_target;
        xSemaphoreGive(wheel_targets_mutex);

        // 3. Update PID setpoint and compute output
        pid_update_set_point(pid_block, target_velocity);
        pid_compute(pid_block, measured_velocity, &output);

        // 4. Actuate motor
        bldc_set_duty(params->pwm_motor, output);

        // Log periodically
        static int ctr = 0;
        if (++ctr >= 250) {  // 2ms × 250 = 500ms
            ESP_LOGI(task_name, "Vel: %.2f | Target: %.2f | PWM: %.2f",
                     measured_velocity, target_velocity, output);
            ctr = 0;
        }
    }
}

void vTaskControlLeft(void * pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data;
    pid_block_handle_t pid_block = *(params->pid_block);

    extern SemaphoreHandle_t left_params_mutex;
    extern SemaphoreHandle_t wheel_targets_mutex;

    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
    const char *task_name = pcTaskGetName(xTask);

    float measured_velocity, target_velocity, output;

    while (1) {
        // Wait for notification from encoder task (2ms period)
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);

        // 1. Read measured velocity
        xSemaphoreTake(left_params_mutex, portMAX_DELAY);
        measured_velocity = encoder_data->velocity;
        xSemaphoreGive(left_params_mutex);

        // 2. Read target velocity
        xSemaphoreTake(wheel_targets_mutex, portMAX_DELAY);
        target_velocity = wheel_targets.left_wheel_target;
        xSemaphoreGive(wheel_targets_mutex);

        // 3. Update PID setpoint and compute output
        pid_update_set_point(pid_block, target_velocity);
        pid_compute(pid_block, measured_velocity, &output);

        // 4. Actuate motor
        bldc_set_duty(params->pwm_motor, output);

        // Log periodically
        static int ctr = 0;
        if (++ctr >= 250) {  // 2ms × 250 = 500ms
            ESP_LOGI(task_name, "Vel: %.2f | Target: %.2f | PWM: %.2f",
                     measured_velocity, target_velocity, output);
            ctr = 0;
        }
    }
}

void vTaskControlBack(void * pvParameters) {
    control_params_t *params = (control_params_t *)pvParameters;
    encoder_data_t *encoder_data = (encoder_data_t *)params->sensor_data;
    pid_block_handle_t pid_block = *(params->pid_block);

    extern SemaphoreHandle_t back_params_mutex;
    extern SemaphoreHandle_t wheel_targets_mutex;

    TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
    const char *task_name = pcTaskGetName(xTask);

    float measured_velocity, target_velocity, output;

    while (1) {
        // Wait for notification from encoder task (2ms period)
        xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, NULL, portMAX_DELAY);

        // 1. Read measured velocity
        xSemaphoreTake(back_params_mutex, portMAX_DELAY);
        measured_velocity = encoder_data->velocity;
        xSemaphoreGive(back_params_mutex);

        // 2. Read target velocity
        xSemaphoreTake(wheel_targets_mutex, portMAX_DELAY);
        target_velocity = wheel_targets.back_wheel_target;
        xSemaphoreGive(wheel_targets_mutex);

        // 3. Update PID setpoint and compute output
        pid_update_set_point(pid_block, target_velocity);
        pid_compute(pid_block, measured_velocity, &output);

        // 4. Actuate motor
        bldc_set_duty(params->pwm_motor, output);

        // Log periodically
        static int ctr = 0;
        if (++ctr >= 250) {  // 2ms × 250 = 500ms
            ESP_LOGI(task_name, "Vel: %.2f | Target: %.2f | PWM: %.2f",
                     measured_velocity, target_velocity, output);
            ctr = 0;
        }
    }
}

// ================== DISTANCE/TIMING TASK ==================

void vTaskDistance(void *pvParameters) {
    extern SemaphoreHandle_t velocity_setpoints_mutex;

    while(1) {
        // This task can monitor movement completion or timeouts
        // For now, it just delays
        static uint16_t time_count = 0; ///< Counter to keep track of the number of iterations

        if(time_count >= goal_time * 1000 && movement == LINEAR) { ///< Check if the goal time has been reached
            movement = DO_NOT_MOVE; ///< Set the movement to do not move
            time_count = 0; ///< Reset the time count
            goal_time = 0;
        } else if (movement != DO_NOT_MOVE) {
            time_count += 100; ///< Increment the time count by the task delay (100 ms)
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//************************* COMANDOS DE SERVIDOR **************************************************************************** */
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
    goal_time = atof(time_s);
    uint8_t forward = strcmp(direction, "Forward") == 0 ? 1 : 0;

    ESP_LOGI("HTTP", "LINE movement: dir=%s deg=%.2f vel=%.2f time=%.2f",
             direction, degrees, velocity, goal_time);
    // Calculate linear velocities
    float x_vel, y_vel;
    linear_movement(forward, velocity, degrees, &x_vel, &y_vel);
    movement = LINEAR;
    // Update velocity setpoints
    extern SemaphoreHandle_t velocity_setpoints_mutex;
    xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
    velocity_setpoints.v_x_setpoint = x_vel;
    velocity_setpoints.v_y_setpoint = y_vel;
    velocity_setpoints.v_wb_setpoint = 0.0f;
    xSemaphoreGive(velocity_setpoints_mutex);

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

    degrees_circular = atof(degrees_s);
    velocity_circular = atof(velocity_s);
    radius_trayectory = atof(radius_s);
    cw = strcmp(direction, "cw") == 0 ? 1 : 0;

    ESP_LOGI("HTTP", "CIRCULAR: cw = %d dir=%s deg=%.2f vel=%.2f radius=%.2f",
             cw, direction, degrees_circular, velocity_circular, radius_trayectory);
    // Calculate circular movement velocities
    //TODO: Revisar movimiento circular. Debe actualizarse en contorl global.
    float x_vel, y_vel;
    
    previous_time = 0.0f; // Reset previous time for circular movement
    delta_time = 0.0f;
    movement = CIRCULAR;
    circular_movement(cw, velocity_circular, degrees_circular, radius_trayectory, &x_vel, &y_vel,
        &delta_time, &movement);
    // Update velocity setpoints
    extern SemaphoreHandle_t velocity_setpoints_mutex;
    xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
    velocity_setpoints.v_x_setpoint = x_vel;
    velocity_setpoints.v_y_setpoint = y_vel;
    velocity_setpoints.v_wb_setpoint = 0.0f;  // Circular movement without rotation
    xSemaphoreGive(velocity_setpoints_mutex);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

esp_err_t rotation_handler(httpd_req_t *req) {
    char direction[16], duration_s[16], velocity_s[16];
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

    if (!get_param(req, "direction", direction, sizeof(direction)) ||
        !get_param(req, "duration", duration_s, sizeof(duration_s)) ||
        !get_param(req, "velocity", velocity_s, sizeof(velocity_s))) {

        httpd_resp_sendstr(req, "Missing parameters");
        return ESP_FAIL;
    }

    movement = ROTATION;
    rotation_duration = atof(duration_s);  // Store duration in seconds
    rotation_velocity = atof(velocity_s);  // Store angular velocity in rad/s
    cw = strcmp(direction, "cw") == 0 ? 1 : 0;
    
    ESP_LOGI("HTTP", "ROTATION: dir=%s duration=%.2fs wb=%.2f rad/s",
             direction, rotation_duration, rotation_velocity);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

esp_err_t reset_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*");

    ESP_LOGI("HTTP", "RESET: Stopping all movements");

    // Reset all velocity setpoints to zero
    extern SemaphoreHandle_t velocity_setpoints_mutex;
    xSemaphoreTake(velocity_setpoints_mutex, portMAX_DELAY);
    velocity_setpoints.v_x_setpoint = 0.0f;
    velocity_setpoints.v_y_setpoint = 0.0f;
    velocity_setpoints.v_wb_setpoint = 0.0f;
    xSemaphoreGive(velocity_setpoints_mutex);

    movement = DO_NOT_MOVE;

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
            .uri = "/*",
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

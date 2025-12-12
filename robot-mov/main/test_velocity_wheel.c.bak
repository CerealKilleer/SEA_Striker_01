// Include standar libraries 
#include <stdio.h>
#include <math.h>

/**
 * @file test_velocity_wheel.c
 * @brief Single wheel velocity control test with encoder feedback, Kalman filter, and PID control
 * 
 * @description
 * This program tests velocity control for a single wheel (RIGHT wheel by default).
 * It implements:
 * - ADC reading from AS5600 magnetic encoder every 5ms
 * - Velocity estimation from encoder angle changes
 * - Kalman filter for noise reduction
 * - PID controller for velocity tracking
 * - PWM output to BLDC motor
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

#define SAMPLE_TIME 5 ///< Sample time in ms (5ms as requested)
#define WHEEL_RADIO 3.0f ///< Radio of the wheel in cm
#define WHEEL_CIRCUMFERENCE (2.0f * 3.14159f * WHEEL_RADIO) ///< Circumference in cm

///<-------------- AS5600 configuration --------------
#define AS5600_OUT_GPIO_RIGHT 5         ///< gpio number for right OUT signal
#define AS5600_OUT_GPIO_LEFT 6          ///< gpio number for left OUT signal
#define AS5600_OUT_GPIO_BACK 7          ///< gpio number for back OUT signal
#define AS5600_ADC_UNIT_ID ADC_UNIT_1   ///< I2C port number for master dev
#define AS5600_MODE 1                   ///< Calibration = 0, Angle through ADC = 1
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


#define MCPWM_GROUP_ID(group_id) (0)

// Global structures for test
encoder_data_t right_encoder_data = {
    .velocity = 0.0f,
    .last_vel = 0.0f,
    .angle_prev = 0.0f,
    .radio = WHEEL_RADIO,
    .distance = 0.0f,
    .time_interval = SAMPLE_TIME / 1000.0f
};

kalman_filter_t km_right_wheel;
pid_block_handle_t pid_right;

// Statistics
float total_distance = 0.0f;
float total_rotations = 0.0f;

// User configurable parameters
typedef struct {
    float velocity;
    float duration;
    float kp;
    float ki;
    float kd;
    bool start_test;
} test_config_t;

test_config_t test_config = {
    .velocity = 18.85f,  // Default: 1 rotation per second (WHEEL_RADIO * 2 * PI)
    .duration = 5.0f,
    .kp = 0.02f,
    .ki = 0.01f,
    .kd = 0.0f,
    .start_test = false
};
//============== Command Processing Functions ==========================================
void print_menu(void) {
    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf("           WHEEL VELOCITY TEST - CONFIGURATION MENU        \n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf("Commands:\n");
    printf("  vel <value>    - Set target velocity in cm/s (current: %.2f)\n", test_config.velocity);
    printf("  time <value>   - Set test duration in seconds (current: %.2f)\n", test_config.duration);
    printf("  kp <value>     - Set PID Kp gain (current: %.4f)\n", test_config.kp);
    printf("  ki <value>     - Set PID Ki gain (current: %.4f)\n", test_config.ki);
    printf("  kd <value>     - Set PID Kd gain (current: %.4f)\n", test_config.kd);
    printf("  show           - Show current configuration\n");
    printf("  start          - Start the test\n");
    printf("  help           - Show this menu\n");
    printf("═══════════════════════════════════════════════════════════\n");
    printf("Wheel radius: %.2f cm | Circumference: %.2f cm\n", WHEEL_RADIO, WHEEL_CIRCUMFERENCE);
    printf("═══════════════════════════════════════════════════════════\n\n");
    printf("> ");
    fflush(stdout);
}

void process_command(char *cmd) {
    char command[32];
    float value;
    
    // Remove newline characters
    cmd[strcspn(cmd, "\r\n")] = 0;
    
    if (strlen(cmd) == 0) {
        printf("> ");
        fflush(stdout);
        return;
    }
    
    if (sscanf(cmd, "%s %f", command, &value) >= 1) {
        if (strcmp(command, "vel") == 0 && sscanf(cmd, "%s %f", command, &value) == 2) {
            if (value >= 0 && value <= 200.0f) {
                test_config.velocity = value;
                printf("✓ Velocity set to %.2f cm/s\n", test_config.velocity);
            } else {
                printf("✗ Error: Velocity must be between 0 and 200 cm/s\n");
            }
        }
        else if (strcmp(command, "time") == 0 && sscanf(cmd, "%s %f", command, &value) == 2) {
            if (value > 0 && value <= 60.0f) {
                test_config.duration = value;
                printf("✓ Duration set to %.2f seconds\n", test_config.duration);
            } else {
                printf("✗ Error: Duration must be between 0 and 60 seconds\n");
            }
        }
        else if (strcmp(command, "kp") == 0 && sscanf(cmd, "%s %f", command, &value) == 2) {
            if (value >= 0 && value <= 10.0f) {
                test_config.kp = value;
                printf("✓ Kp set to %.4f\n", test_config.kp);
            } else {
                printf("✗ Error: Kp must be between 0 and 10\n");
            }
        }
        else if (strcmp(command, "ki") == 0 && sscanf(cmd, "%s %f", command, &value) == 2) {
            if (value >= 0 && value <= 10.0f) {
                test_config.ki = value;
                printf("✓ Ki set to %.4f\n", test_config.ki);
            } else {
                printf("✗ Error: Ki must be between 0 and 10\n");
            }
        }
        else if (strcmp(command, "kd") == 0 && sscanf(cmd, "%s %f", command, &value) == 2) {
            if (value >= 0 && value <= 10.0f) {
                test_config.kd = value;
                printf("✓ Kd set to %.4f\n", test_config.kd);
            } else {
                printf("✗ Error: Kd must be between 0 and 10\n");
            }
        }
        else if (strcmp(command, "show") == 0) {
            printf("\n📊 Current Configuration:\n");
            printf("   Velocity:  %.2f cm/s\n", test_config.velocity);
            printf("   Duration:  %.2f seconds\n", test_config.duration);
            printf("   PID Kp:    %.4f\n", test_config.kp);
            printf("   PID Ki:    %.4f\n", test_config.ki);
            printf("   PID Kd:    %.4f\n", test_config.kd);
            printf("   Expected distance: %.2f cm\n", test_config.velocity * test_config.duration);
            printf("   Expected rotations: %.2f turns\n", 
                   (test_config.velocity * test_config.duration) / WHEEL_CIRCUMFERENCE);
        }
        else if (strcmp(command, "start") == 0) {
            test_config.start_test = true;
            printf("✓ Starting test...\n");
            return;
        }
        else if (strcmp(command, "help") == 0) {
            print_menu();
            return;
        }
        else {
            printf("✗ Unknown command: '%s'. Type 'help' for available commands.\n", command);
        }
    }
    
    printf("> ");
    fflush(stdout);
}

//============== Funcion Motores Init====================================================
static inline void init_blc_motor(bldc_pwm_motor_t *pwm_motor, uint8_t gpio, 
                                  uint8_t gpio_rev, uint16_t freq, uint8_t gpio_group,
                                  uint32_t pwm_resolution, uint8_t min_pwm_cal, uint8_t max_pwm_cal)
{
    bldc_init(pwm_motor, gpio, gpio_rev, freq, gpio_group, pwm_resolution, min_pwm_cal, max_pwm_cal); ///< Initialize the BLDC motor
    bldc_enable(pwm_motor); ///< Enable the BLDC motor
    bldc_set_duty(pwm_motor, 0); ///< Set the duty cycle to 0%
}
//============== Funcion Encoder Init====================================================

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

void app_main(void){
    const char *TAG = "MAIN";
    AS5600_t gAs5600R, gAs5600L, gAs5600B;
    //============== MOTORES ====================================================
    bldc_pwm_motor_t pwmR, pwmL, pwmB;   

    ESP_LOGI(TAG, "  Single Wheel Velocity Test");
    ESP_LOGI(TAG, "  Testing RIGHT wheel only");

    init_blc_motor(&pwmR, PWM_GPIO_R, PWM_REV_GPIO_R, PWM_FREQ, MCPWM_GROUP_ID(0), 
                    PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL);
    
    init_blc_motor(&pwmL, PWM_GPIO_L, PWM_REV_GPIO_L, PWM_FREQ, MCPWM_GROUP_ID(0), 
                    PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL);
    
    init_blc_motor(&pwmB, PWM_GPIO_B, PWM_REV_GPIO_B, PWM_FREQ, MCPWM_GROUP_ID(1), 
                    PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL);

    ESP_LOGI(TAG, "Motors initialized");

    //================ ENCODERS ==================================================
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
        ESP_LOGE(TAG, "AS5600 ADC initialization failed");
        return;
    }

    init_encoder(&gAs5600R, &conf, &handle, AS5600_OUT_GPIO_RIGHT, AS5600_ADC_UNIT_ID, "right");
    init_encoder(&gAs5600L, &conf, &handle, AS5600_OUT_GPIO_LEFT, AS5600_ADC_UNIT_ID, "left");
    init_encoder(&gAs5600B, &conf, &handle, AS5600_OUT_GPIO_BACK, AS5600_ADC_UNIT_ID, "back");

    ESP_LOGI(TAG, "Encoders initialized");

    //================ KALMAN FILTER =============================================
    kalman_init(&km_right_wheel, 0.005f, 1.0f);
    ESP_LOGI(TAG, "Kalman filter initialized (Q=0.005, R=1.0)");

    //================ COMMAND INTERFACE =========================================
    ESP_LOGI(TAG, "System ready. Waiting for user configuration...");
    print_menu();
    
    // Wait for user commands
    char line[128];
    int line_idx = 0;
    
    while (!test_config.start_test) {
        int c = getchar();
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                line[line_idx] = '\0';
                if (line_idx > 0) {
                    printf("\n");
                    process_command(line);
                    line_idx = 0;
                }
            } else if (c == 127 || c == 8) { // Backspace
                if (line_idx > 0) {
                    line_idx--;
                    printf("\b \b");
                    fflush(stdout);
                }
            } else if (line_idx < sizeof(line) - 1) {
                line[line_idx++] = c;
                putchar(c);
                fflush(stdout);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    //================ PID CONTROLLER ============================================
    pid_parameter_t pid_param = {
        .kp = test_config.kp,
        .ki = test_config.ki,
        .kd = test_config.kd,
        .max_output = 70.0f,
        .min_output = -70.0f,
        .set_point = 0.0f,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .beta = 0.0f
    };

    pid_config_t pid_config = {
        .init_param = pid_param
    };

    if (pid_new_control_block(&pid_config, &pid_right) != PID_OK) {
        ESP_LOGE(TAG, "Failed to create PID controller");
        return;
    }
    ESP_LOGI(TAG, "PID controller initialized (Kp=%.4f, Ki=%.4f, Kd=%.4f)", 
             pid_param.kp, pid_param.ki, pid_param.kd);

    //================ TEST CONFIGURATION ========================================
    float desired_velocity_cm_s = test_config.velocity;
    float test_duration_s = test_config.duration;
    
    float expected_distance = desired_velocity_cm_s * test_duration_s;
    float expected_rotations = expected_distance / WHEEL_CIRCUMFERENCE;
    
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, "  TEST PARAMETERS");
    ESP_LOGI(TAG, "  Target velocity: %.2f cm/s", desired_velocity_cm_s);
    ESP_LOGI(TAG, "  Test duration: %.2f seconds", test_duration_s);
    ESP_LOGI(TAG, "  Wheel radius: %.2f cm", WHEEL_RADIO);
    ESP_LOGI(TAG, "  Wheel circumference: %.2f cm", WHEEL_CIRCUMFERENCE);
    ESP_LOGI(TAG, "  Expected distance: %.2f cm", expected_distance);
    ESP_LOGI(TAG, "  Expected rotations: %.2f turns", expected_rotations);
    ESP_LOGI(TAG, "  PID: Kp=%.4f Ki=%.4f Kd=%.4f", test_config.kp, test_config.ki, test_config.kd);
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Starting test in 3...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Starting test in 2...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Starting test in 1...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "GO!");

    //================ MAIN CONTROL LOOP =========================================
    uint32_t start_time = xTaskGetTickCount();
    uint32_t log_counter = 0;
    float output = 0.0f;
    
    while (1) {
        uint32_t loop_start = xTaskGetTickCount();
        
        // 1. READ ENCODER
        float angle = AS5600_ADC_GetAngle(&gAs5600R);
        right_encoder_data.angle = angle;
        
        // 2. CALCULATE VELOCITY
        estimate_velocity_encoder(&right_encoder_data);
        
        // 3. APPLY KALMAN FILTER
        float filtered_velocity = kalman_update(&km_right_wheel, right_encoder_data.velocity);
        right_encoder_data.velocity = filtered_velocity;
        
        // 4. CALCULATE DISTANCE AND ROTATIONS
        float angle_diff = angle - right_encoder_data.angle_prev;
        if (angle_diff > 180.0f) angle_diff -= 360.0f;
        if (angle_diff < -180.0f) angle_diff += 360.0f;
        
        float distance_increment = (right_encoder_data.radio * 3.14159f / 180.0f) * angle_diff;
        total_distance += fabsf(distance_increment);
        total_rotations = total_distance / WHEEL_CIRCUMFERENCE;
        
        // 5. UPDATE PID SETPOINT
        pid_update_set_point(pid_right, desired_velocity_cm_s);
        
        // 6. COMPUTE PID OUTPUT
        pid_compute(pid_right, right_encoder_data.velocity, &output);
        
        // 7. SET MOTOR PWM
        bldc_set_duty(&pwmR, output);
        
        // 8. CHECK IF TEST IS COMPLETE
        uint32_t elapsed_ms = (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;
        if (elapsed_ms >= (test_duration_s * 1000)) {
            ESP_LOGI(TAG, "═══════════════════════════════════════");
            ESP_LOGI(TAG, "  TEST COMPLETED!");
            ESP_LOGI(TAG, "  Target velocity: %.2f cm/s", desired_velocity_cm_s);
            ESP_LOGI(TAG, "  Test duration: %.2f seconds", test_duration_s);
            ESP_LOGI(TAG, "  Total distance: %.2f cm", total_distance);
            ESP_LOGI(TAG, "  Total rotations: %.2f turns", total_rotations);
            ESP_LOGI(TAG, "  Average velocity: %.2f cm/s", total_distance / test_duration_s);
            ESP_LOGI(TAG, "═══════════════════════════════════════");
            
            // Stop motor
            desired_velocity_cm_s = 0.0f;
            bldc_set_duty(&pwmR, 0.0f);
            break;
        }
        
        // 9. LOG EVERY 200ms (40 cycles at 5ms)
        if (++log_counter >= 40) {
            ESP_LOGI(TAG, "T:%.1fs | Ang:%.1f° | Vel:%.2f cm/s | Out:%.1f | Dist:%.1fcm | Rot:%.2f", 
                     elapsed_ms/1000.0f, angle, right_encoder_data.velocity, 
                     output, total_distance, total_rotations);
            log_counter = 0;
        }
        
        // 10. WAIT 5ms FOR NEXT CYCLE (ensure minimum 1 tick delay)
        TickType_t delay_ticks = pdMS_TO_TICKS(SAMPLE_TIME);
        if (delay_ticks == 0) delay_ticks = 1; // Minimum 1 tick to avoid watchdog
        vTaskDelay(delay_ticks);
    }
    
    ESP_LOGI(TAG, "Test finished. Motor stopped.");
    
    // Keep program running
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
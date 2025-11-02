// Include standar libraries 
#include <stdio.h>

// Include personalized driver libraries
#include "bldc_pwm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

///<-------------- BLDC configuration -----------------
#define PWM_GPIO_R 20               ///< GPIO number for right PWM signal
#define PWM_REV_GPIO_R 21           ///< GPIO number for right PWM reverse signal

#define PWM_GPIO_L 47               ///< GPIO number for left PWM signal
#define PWM_REV_GPIO_L 48           ///< GPIO number for left PWM reverse signal

#define PWM_GPIO_B 14               ///< GPIO number for back PWM signal
#define PWM_REV_GPIO_B 13           ///< GPIO number for back PWM reverse signal

#define PWM_FREQ 50                 ///< PWM frequency in Hz
#define PWM_RESOLUTION 100000       ///< PWM resolution in bits
#define MAX_PWM_CAL 120             ///< Maximum PWM value
#define MIN_PWM_CAL 35              ///< Minimum PWM value
#define MAX_PWM_RE 119              ///< Maximum PWM value (moves fully)
#define MIN_PWM_RE 38               ///< Minimum PWM value (does not move)
///<--------------------------------------------------

bldc_pwm_motor_t pwmR, pwmR2, pwmL, pwmL2, pwmB, pwmB2; ///< BLDC motor structures

void app_main(void)
{

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds
    
    bldc_init(&pwmR, PWM_GPIO_R, 10, PWM_FREQ, 1, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_init(&pwmR2, PWM_REV_GPIO_R, 11, PWM_FREQ, 1, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor

    //bldc_init(&pwmL, PWM_GPIO_L, 10, PWM_FREQ, 1, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    //bldc_init(&pwmL2, PWM_REV_GPIO_L, 11, PWM_FREQ, 1, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor

    bldc_init(&pwmB, PWM_GPIO_B, 15, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor
    bldc_init(&pwmB2, PWM_REV_GPIO_B, 16, PWM_FREQ, 0, PWM_RESOLUTION, MIN_PWM_CAL, MAX_PWM_CAL); ///< Initialize the BLDC motor

    bldc_enable(&pwmR); ///< Enable the BLDC motor
    bldc_enable(&pwmR2); ///< Enable the BLDC motor

    //bldc_enable(&pwmL); ///< Enable the BLDC motor
    //bldc_enable(&pwmL2); ///< Enable the BLDC motor

    bldc_enable(&pwmB); ///< Enable the BLDC motor
    bldc_enable(&pwmB2); ///< Enable the BLDC motor

    ESP_LOGI("PWM", "Starting calibration!"); ///< Log message
    bldc_set_duty_old(&pwmR, MAX_PWM_CAL); ///< Set the duty cycle to 0%
    bldc_set_duty_old(&pwmR2, MAX_PWM_CAL); ///< Set the duty cycle to 0%

    //bldc_set_duty_old(&pwmL, MAX_PWM_CAL); ///< Set the duty cycle to 0%
    //bldc_set_duty_old(&pwmL2, MAX_PWM_CAL); ///< Set the duty cycle to 0%

    bldc_set_duty_old(&pwmB, MAX_PWM_CAL); ///< Set the duty cycle to 0%
    bldc_set_duty_old(&pwmB2, MAX_PWM_CAL); ///< Set the duty cycle to 0%

    vTaskDelay(7000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds
    
    ESP_LOGI("PWM", "PWM to 0!"); ///< Log message
    bldc_set_duty_old(&pwmR, MIN_PWM_CAL); ///< Set the duty cycle to 0%
    bldc_set_duty_old(&pwmR2, MIN_PWM_CAL); ///< Set the duty cycle to 0%

    //bldc_set_duty_old(&pwmL, MIN_PWM_CAL); ///< Set the duty cycle to 0%
    //bldc_set_duty_old(&pwmL2, MIN_PWM_CAL); ///< Set the duty cycle to 0%

    bldc_set_duty_old(&pwmB, MIN_PWM_CAL); ///< Set the duty cycle to 0%
    bldc_set_duty_old(&pwmB2, MIN_PWM_CAL); ///< Set the duty cycle to 0%

    vTaskDelay(2000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds

    ESP_LOGI("PWM", "ESC Calibrated!"); ///< Log message

    vTaskDelay(5000 / portTICK_PERIOD_MS); ///< Wait for 10 seconds

    ESP_LOGI("PWM", "Starting test..."); ///< Log message

    int32_t duty = MIN_PWM_CAL; ///< Duty cycle variable
    bool reverse = false; ///< Reverse variable
    while (1)
    {
        ESP_LOGI("PWM", "ESC running!"); ///< Log message

        duty += 5; ///< Increase the duty cycle
        if (duty > MAX_PWM_CAL) ///< If the duty cycle is greater than 100%
        {
            duty = MIN_PWM_CAL; ///< Set the duty cycle to 0%
            reverse = !reverse; ///< Reverse the direction
        }
        if (reverse) { ///< If the direction is reversed
            ESP_LOGI("PWM", "Motor running in reverse! With duty: %li", duty); ///< Log message
            bldc_set_duty_old(&pwmR2, 90); ///< Set the duty cycle to the current PWM value
            bldc_set_duty_old(&pwmR, duty); ///< Set the duty cycle to 0%

            //bldc_set_duty_old(&pwmL2, 90); ///< Set the duty cycle to the current PWM value
            //bldc_set_duty_old(&pwmL, duty); ///< Set the duty cycle to 0%

            bldc_set_duty_old(&pwmB2, 90); ///< Set the duty cycle to the current PWM value
            bldc_set_duty_old(&pwmB, duty); ///< Set the duty cycle to 0%
        } else {
            ESP_LOGI("PWM", "Motor running in forward! With duty: %li", duty); ///< Log message
            bldc_set_duty_old(&pwmR2, 10); ///< Set the duty cycle to 0%
            bldc_set_duty_old(&pwmR, duty); ///< Set the duty cycle to the current PWM value

            //bldc_set_duty_old(&pwmL2, 10); ///< Set the duty cycle to 0%
            //bldc_set_duty_old(&pwmL, duty); ///< Set the duty cycle to the current PWM value

            bldc_set_duty_old(&pwmB2, 10); ///< Set the duty cycle to 0%
            bldc_set_duty_old(&pwmB, duty); ///< Set the duty cycle to the current PWM value
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS); ///< Wait for 2 seconds
    }
    
}
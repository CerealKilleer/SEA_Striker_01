/**
 * \file        bldc_pwm.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

#ifndef __PWM_MOTOR_H__
#define __PWM_MOTOR_H__


#include <stdint.h>
#include <stdbool.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

typedef struct {
    uint8_t rev_gpio_num;   ///< GPIO number
    uint8_t pwm_gpio_num;   ///< GPIO number
    uint32_t pwm_freq_hz;   ///< PWM frequency in Hz
    uint32_t pwm_duty_us;   ///< PWM duty cycle in microseconds
    uint32_t max_cmp;       ///< Maximum speed in Hz

    uint16_t duty_cycle;    ///< Duty cycle in percentage
    int group_id;           ///< MCPWM group number 
    uint32_t resolution_hz; ///< MCPWM timer frequency

    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t cmp;
    mcpwm_cmpr_handle_t cmp_rev;
    mcpwm_gen_handle_t gen;
    mcpwm_gen_handle_t gen_rev;

    // PWM signal characteristics
    uint8_t h_duty;  ///< Higher duty cycle
    uint8_t l_duty;  ///< Lower duty cycle

} bldc_pwm_motor_t;

/**
 * @brief Initialize the BLDC motor
 * 
 * @param motor instance of the motor
 * @param pwm_gpio_num pwm gpio num
 * @param rev_gpio_num pwm reverse signal
 * @param pwm_freq_hz pwm frequency
 * @param group_id MCPWM group ID 0/1
 * @param resolution_hz frequency timer resolution
 * @param high_duty Maximum duty
 * @param low_duty Minimum duty
 */
esp_err_t bldc_init(bldc_pwm_motor_t *motor, uint8_t pwm_gpio_num, uint8_t rev_gpio_num,uint32_t pwm_freq_hz, uint32_t group_id, uint32_t resolution_hz, uint8_t low_duty, uint8_t high_duty);

/**
 * @brief Enable the motor
 * 
 * @param motor instance of the motor
 */
esp_err_t bldc_enable(bldc_pwm_motor_t *motor);

/**
 * @brief Disable the motor
 * 
 * @param motor instance of the motor
 */
esp_err_t bldc_disable(bldc_pwm_motor_t *motor);

/**
 * @brief Set the duty which will receive the ESC. The duty is a value between 0 and 1000
 * 
 * @param motor instance of the motor
 * @param duty range from 0 to 1000, where 0 is 0% and 1000 is 100%
 */
esp_err_t bldc_set_duty_old(bldc_pwm_motor_t *motor, uint32_t duty);

/**
 * @brief Set the duty which will receive the ESC. The duty is a value between 0 and 1000
 * 
 * @param motor instance of the motor
 * @param duty range from -100 to 100, negative means reverse
 */
esp_err_t bldc_set_duty(bldc_pwm_motor_t *motor, int16_t duty);

#endif // __PWM_MOTOR_H__
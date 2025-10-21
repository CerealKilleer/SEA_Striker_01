
#include <stdbool.h>
#include <sys/param.h>
#include <stdlib.h>
#include <stdio.h>
#include "pid_ext.h"

typedef struct pid_block_t pid_block_t;
typedef void (*pid_cal_func_t)(pid_block_t *pid);

struct pid_block_t {
    float Kp; // PID Kp value
    float Ki; // PID Ki value
    float Kd; // PID Kd value
    float error; // PID error e(n)
    float previous_err1; // e(n-1)
    float previous_err2; // e(n-2)
    float integral_err;  // Sum of error
    float derivative_err; // Derivative of error
    float previous_derivative_err; // Derivative of error in last control period
    float previous_output;  // PID output in last control period u(n-1)
    float max_output;   // PID maximum output limitation
    float min_output;   // PID minimum output limitation
    float set_point;    // PID set point
    float output;       // PID output
    float beta;         // PID beta filter coefficient of derivative term
    pid_cal_func_t calculate_func; // calculation function, depends on actual PID type set by user
};

static void pid_calc_positional(pid_block_t *pid)
{
    float output = 0;

    // Anti-windup: Only integrate if the output is not saturated or if the error is driving the output back within limits
    if ((pid->output < pid->max_output && pid->output > pid->min_output) || 
        (pid->error > 0 && pid->output < pid->max_output) || 
        (pid->error < 0 && pid->output > pid->min_output)) {
        pid->integral_err += pid->error;
    }
    
    /* Limit the integral term */
    // pid->integral_err = MIN(pid->integral_err, pid->max_output);
    // pid->integral_err = MAX(pid->integral_err, pid->min_output);

    /* Derivative term with beta filter */
    pid->derivative_err = pid->error - pid->previous_err1;
    pid->derivative_err = pid->beta * pid->previous_derivative_err + (1 - pid->beta) * pid->derivative_err;
    /* Calculate the pid control value by location formula */
    /* u(n) = e(n)*Kp + (e(n)-e(n-1))*Kd + integral*Ki */
    output = pid->error * pid->Kp +
            pid->derivative_err * pid->Kd +
            pid->integral_err * pid->Ki;

    /* If the output is out of the range, it will be limited */
    output = MIN(output, pid->max_output);
    output = MAX(output, pid->min_output);

    /* Update previous error */
    pid->previous_err1 = pid->error;
    pid->previous_derivative_err = pid->derivative_err;

    pid->output = output;
}

static void pid_calc_incremental(pid_block_t *pid)
{
    float output = 0;

    /* Calculate the pid control value by increment formula */
    /* du(n) = (e(n)-e(n-1))*Kp + (e(n)-2*e(n-1)+e(n-2))*Kd + e(n)*Ki */
    /* u(n) = du(n) + u(n-1) */
    output = (pid->error - pid->previous_err1) * pid->Kp +
            (pid->error - 2 * pid->previous_err1 + pid->previous_err2) * pid->Kd +
            pid->error * pid->Ki +
            pid->previous_output;

    /* If the output is beyond the range, it will be limited */
    output = MIN(output, pid->max_output);
    output = MAX(output, pid->min_output);

    /* Update previous error */
    pid->previous_err2 = pid->previous_err1;
    pid->previous_err1 = pid->error;

    /* Update last output */
    pid->previous_output = output;

    pid->output = output;
}

int pid_new_control_block(const pid_config_t *config, pid_block_handle_t *ret_pid)
{
    int ret = PID_OK;
    pid_block_t *pid = NULL;
    /* Check the input pointer */
    if (!config || !ret_pid) {
        ret = PID_ERR_INVALID_ARG; // Invalid argument
        goto err;
    }

    /* Allocate memory for PID control block */
    pid = calloc(1, sizeof(pid_block_t));
    if (!pid) {
        ret = PID_ERR_NO_MEM; // Out of memory
        goto err;
    }

    /* Initialize the PID control block */
    if (pid_update_parameters(pid, &config->init_param) != 0) {
        goto err;
    }
    *ret_pid = pid;
    return ret;

err:
    if (pid) {
        free(pid);
    }
    return ret;
}

int pid_del_control_block(pid_block_handle_t pid)
{
    if (!pid) {
        return PID_ERR_INVALID_ARG; // Invalid argument
    }

    free(pid);
    return PID_OK;
}

int pid_compute(pid_block_handle_t pid, float input, float *ouput)
{
    if (!pid || !ouput) {
        return PID_ERR_INVALID_ARG; // Invalid argument
    }
    
    pid->error = pid->set_point - input;
    pid->calculate_func(pid);
    *ouput = pid->output;
    return PID_OK;
}

int pid_update_parameters(pid_block_handle_t pid, const pid_parameter_t *params)
{
    if (!pid || !params) {
        return PID_ERR_INVALID_ARG; // Invalid argument
    }

    pid->Kp = params->kp;
    pid->Ki = params->ki;
    pid->Kd = params->kd;
    pid->max_output = params->max_output;
    pid->min_output = params->min_output;
    pid->set_point = params->set_point;
    pid->beta = params->beta;


    /* Set the calculate function according to the PID type */
    switch (params->cal_type) {
    case PID_CAL_TYPE_INCREMENTAL:
        pid->calculate_func = pid_calc_incremental;
        break;
    case PID_CAL_TYPE_POSITIONAL:
        pid->calculate_func = pid_calc_positional;
        break;
    default:
        return PID_ERR_INVALID_ARG; // Invalid PID calculation type
    }
    return PID_OK;
}

int pid_update_set_point(pid_block_handle_t pid, float set_point)
{
    if (!pid) {
        return PID_ERR_INVALID_ARG; // Invalid argument
    }
    
    pid->set_point = set_point;
    
    return PID_OK;
}

int pid_reset_block(pid_block_handle_t pid)
{
    if (!pid) {
        return PID_ERR_INVALID_ARG; // Invalid argument
    }

    pid->integral_err = 0;
    pid->previous_err1 = 0;
    pid->previous_err2 = 0;
    pid->previous_output = 0;
    pid->previous_derivative_err = 0;

    return PID_OK;
}
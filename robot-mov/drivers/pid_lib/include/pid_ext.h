#ifndef PID_EXT_H
#define PID_EXT_H

#ifdef __cplusplus
extern "C" {
#endif

#define PID_OK 0               // PID operation successfully
#define PID_ERR_INVALID_ARG -1 // Invalid argument
#define PID_ERR_NO_MEM -2      // Out of memory


/**
 * @brief PID calculation type
 *
 */
typedef enum {
    PID_CAL_TYPE_INCREMENTAL, /*!< Incremental PID control */
    PID_CAL_TYPE_POSITIONAL,  /*!< Positional PID control */
} pid_calculate_type_t;

/**
 * @brief Type of PID control block handle
 *
 */
typedef struct pid_block_t *pid_block_handle_t;

/**
 * @brief PID control parameters
 *
 */
typedef struct {
    float kp;                      // PID Kp parameter
    float ki;                      // PID Ki parameter
    float kd;                      // PID Kd parameter
    float max_output;              // PID maximum output limitation
    float min_output;              // PID minimum output limitation
    float set_point;               // PID set point
    pid_calculate_type_t cal_type; // PID calculation type
    float beta;                    // PID beta filter coefficient of derivative term
} pid_parameter_t;

/**
 * @brief PID control configuration
 *
 */
typedef struct {
    pid_parameter_t init_param; // Initial parameters
} pid_config_t;


/**
 * @brief Create a new PID control session, returns the handle of control block
 *
 * @param[in] config PID control configuration
 * @param[out] ret_pid Returned PID control block handle
 * @return
 *      - PID_OK: Created PID control block successfully
 *      - PID_ERR_INVALID_ARG: Created PID control block failed because of invalid argument
 *      - PID_ERR_NO_MEM: Created PID control block failed because out of memory
 */
int pid_new_control_block(const pid_config_t *config, pid_block_handle_t *ret_pid);

/**
 * @brief Delete the PID control block
 *
 * @param[in] pid PID control block handle, created by `pid_new_control_block()`
 * @return
 *      - PID_OK: Delete PID control block successfully
 *      - PID_ERR_INVALID_ARG: Delete PID control block failed because of invalid argument
 */
int pid_del_control_block(pid_block_handle_t pid);

/**
 * @brief Update PID parameters
 *
 * @param[in] pid PID control block handle, created by `pid_new_control_block()`
 * @param[in] params PID parameters
 * @return
 *      - PID_OK: Update PID parameters successfully
 *      - PID_ERR_INVALID_ARG: Update PID parameters failed because of invalid argument
 */
int pid_update_parameters(pid_block_handle_t pid, const pid_parameter_t *params);

/**
 * @brief Update PID parameters
 *
 * @param[in] pid PID control block handle, created by `pid_new_control_block()`
 * @param[in] set_point New set point value for PID control
 * @return
 *      - PID_OK: Update PID parameters successfully
 *      - PID_ERR_INVALID_ARG: Update PID parameters failed because of invalid argument
 */
int pid_update_set_point(pid_block_handle_t pid, float set_point);

/**
 * @brief Input error and get PID control result
 *
 * @param[in] pid PID control block handle, created by `pid_new_control_block()`
 * @param[in] input Feedback value
 * @param[out] ouput result after PID calculation
 * @return
 *      - PID_OK: Run a PID compute successfully
 *      - PID_ERR_INVALID_ARG: Run a PID compute failed because of invalid argument
 */
int pid_compute(pid_block_handle_t pid, float input, float *ouput);

/**
 * @brief Reset the accumulation in pid_block
 *
 * @param[in] pid PID control block handle, created by `pid_new_control_block()`
 * @return
 *      - PID_OK: Reset successfully
 *      - PID_ERR_INVALID_ARG: Reset failed because of invalid argument
 */
int pid_reset_block(pid_block_handle_t pid);

#ifdef __cplusplus
}
#endif

#endif // PID_EXT_H
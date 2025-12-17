#ifndef MOV_CALCULATION_H
#define MOV_CALCULATION_H

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define PI 3.14159265358979323846
#define DELTA PI/6.0  ///< Angle in degrees for the transformation (orientation angle of the body)
#define N     16      ///< Reduction factor for the transformation (planetary gear ratio)
#define R     3.25f   ///< Radius of the wheel in cm
#define ROBOT_RADIUS 8.5f  ///< Distance from robot center to each wheel in cm

enum movements_num {
    LINEAR = 0,   ///< Linear movement
    CIRCULAR = 1, ///< Circular movement
    ROTATION = 2, ///< Rotation movement
    DO_NOT_MOVE = 3 ///< Do not move
};

/**
 * @brief Calculate the linear movement of the robot.
 * @param forward True if the movement is forward, false if backward.
 * @param linear_velocity Linear velocity in cm/s.
 * @param angle Angle in degrees for the movement (0 for forward).
 * @param x_velocity Pointer to store the x component of the velocity.
 * @param y_velocity Pointer to store the y component of the velocity.
 */
void linear_movement(bool forward, float linear_velocity, float angle, float *x_velocity, float *y_velocity);

/**
 * @brief Calculate the circular movement of the robot.
 * @param cw True if the movement is clockwise, false if counter-clockwise.
 * @param linear_velocity Linear velocity in cm/s.
 * @param angle Angle in degrees for the movement.
 * @param radius Radius of the circular path in cm.
 * @param x_velocity Pointer to store the x component of the velocity.
 * @param y_velocity Pointer to store the y component of the velocity.
 * @param t Pointer to the current time variable for the movement.
 * @param movements Pointer to the current movement state.
 */
void circular_movement(bool cw, float linear_velocity, float angle, 
    float radius, float *x_velocity, float *y_velocity,float *t, enum movements_num *movements);

/**
 * @brief Inverse Kinematics. Calculate the angular velocity of the wheels based on linear velocities.
 * @param x_velocity X component of the linear velocity in cm/s.
 * @param y_velocity Y component of the linear velocity in cm/s.
 * @param vel_selection Selection for which wheel's velocity to calculate (0: left, 1: back, 2: right).
 * @param wheel_velocity Pointer to store the calculated wheel velocity in cm/s.
 */
void cal_lin_to_ang_velocity(float x_velocity, float y_velocity, float wb,
                             uint8_t vel_selection, float *wheel_velocity);

/**
 * @brief Forward Kinematics. Calculate the linear velocities based on the angular velocities of the wheels.
 * @param wl_rad_s Angular velocity of the left wheel in rad/s.
 * @param wb_rad_s Angular velocity of the back wheel in rad/s.
 * @param wr_rad_s Angular velocity of the right wheel in rad/s.
 * @param x_velocity Pointer to store the x component of the linear velocity in cm/s.
 * @param y_velocity Pointer to store the y component of the linear velocity in cm/s.
 * @param angular_velocity Pointer to store the angular velocity of the robot in rad/s.
 */
void cal_forward_kinematics(float wl_rad_s, float wb_rad_s, float wr_rad_s,
                             float *x_velocity, float *y_velocity, float *angular_velocity);

/**
 * @brief Rotate the robot on its axis for a specified duration.
 * @param cw True if clockwise rotation, false for counter-clockwise.
 * @param wb_rad_s Angular velocity in rad/s.
 * @param duration_seconds Duration of rotation in seconds.
 * @param movements Pointer to the current movement state.
 * @return Angular velocity to apply (with direction).
 */
float rotate_on_axis(bool cw, float wb_rad_s, float duration_seconds, enum movements_num *movements);
#endif // MOV_CALCULATION_H
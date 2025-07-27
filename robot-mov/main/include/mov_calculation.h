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
 */
void circular_movement(bool cw, float linear_velocity, float angle, float radius, float *x_velocity, float *y_velocity);

/**
 * @brief Calculate the angular velocity of the wheels based on linear velocities.
 * @param x_velocity X component of the linear velocity in cm/s.
 * @param y_velocity Y component of the linear velocity in cm/s.
 * @param vel_selection Selection for which wheel's velocity to calculate (0: left, 1: back, 2: right).
 * @param wheel_velocity Pointer to store the calculated wheel velocity in cm/s.
 */
void cal_lin_to_ang_velocity(float x_velocity, float y_velocity, uint8_t vel_selection, float *wheel_velocity);

#endif // MOV_CALCULATION_H
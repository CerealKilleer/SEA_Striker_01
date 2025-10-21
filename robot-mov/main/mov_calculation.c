#include "mov_calculation.h"

void linear_movement(bool forward, float linear_velocity, float angle, float *x_velocity, float *y_velocity) {
    if (forward) {
        *x_velocity = -linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = -linear_velocity * cosf(angle * PI / 180.0f);
    } else {
        *x_velocity = linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = linear_velocity * cosf(angle * PI / 180.0f);
    }
}

void circular_movement(bool cw, float linear_velocity, float angle, float radius, float *x_velocity, float *y_velocity) {
    static float t = 0;

    if (t < (angle / 360.0) * 2 * PI * radius / linear_velocity) { ///< Time to reach the goal angle in seconds 360° is a full circle

        if (cw) {
            *x_velocity = -radius * sinf((linear_velocity / radius) * t);
            *y_velocity =  radius * cosf((linear_velocity / radius) * t);
        } else {
            *x_velocity = -radius * sinf((linear_velocity / radius) * t);
            *y_velocity = -radius * cosf((linear_velocity / radius) * t);
        }

        t += 0.002; ///< Increment time by the sample time in seconds

    } else {

        *x_velocity = 0.0f; ///< Stop the movement
        *y_velocity = 0.0f; ///< Stop the movement
        
    }
}

void cal_lin_to_ang_velocity(float x_velocity, float y_velocity, uint8_t vel_selection, float *wheel_velocity) {

    float scale = N / R;

    float cos_d = cosf(DELTA);
    float sin_d = sinf(DELTA);

    // wb = 0, so ignore third column
    switch (vel_selection)  // vel_selection is a constant expression
    {
    case 0: // left wheel
        *wheel_velocity = scale * ( - sin_d * x_velocity - cos_d * y_velocity ) / 5;
        break;
    case 1: // back wheel
        *wheel_velocity = scale * ( x_velocity ) / 5; ///< Adjusted for consistency;
        break;
    case 2: // right wheel
        *wheel_velocity = scale * ( -sin_d * x_velocity + cos_d * y_velocity ) / 5; ///< Adjusted for consistency;
        break;
    
    default:
        break;
    }
}
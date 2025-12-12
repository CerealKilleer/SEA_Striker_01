#include "mov_calculation.h"

void linear_movement(bool forward, float linear_velocity, float angle, float *x_velocity, float *y_velocity) {
    if (forward) {
        *x_velocity = linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = linear_velocity * cosf(angle * PI / 180.0f);
    } else {
        *x_velocity = -linear_velocity * sinf(angle * PI / 180.0f);
        *y_velocity = -linear_velocity * cosf(angle * PI / 180.0f);
    }
}

void circular_movement(bool cw, float linear_velocity, float angle, float radius, float *x_velocity, float *y_velocity) {
    
    static float t;
    if (t < (angle / 360.0) * 2 * PI * radius / linear_velocity) { ///< Time to reach the goal angle in seconds 360° is a full circle

        if (cw) {
            *x_velocity = -radius * sinf((linear_velocity / radius) * t);
            *y_velocity = -radius * cosf((linear_velocity / radius) * t);
        } else {
            *x_velocity = -radius * sinf((linear_velocity / radius) * t);
            *y_velocity = radius * cosf((linear_velocity / radius) * t);
        }

        t += 0.002; ///< Increment time by the sample time in seconds

    } else {

        *x_velocity = 0.0f; ///< Stop the movement
        *y_velocity = 0.0f; ///< Stop the movement
    }
}

void cal_lin_to_ang_velocity(float x_velocity, float y_velocity, float wb,
                             uint8_t vel_selection, float *wheel_velocity)
{
    float scale = N / R;

    const float sin_d = sinf(DELTA);   // = 0.5
    const float cos_d = cosf(DELTA);   // = 0.8660

    switch (vel_selection)
    {
    case 0: // Left wheel at 150°
        // vector unitario = (sin150, cos150) = (0.5, -0.866)
        *wheel_velocity = -scale * ( sin_d * x_velocity  - cos_d * y_velocity + R * wb ) /2.5;
        break;

    case 1: // Back wheel at 270°
        // vector = (sin270, cos270) = (-1, 0)
        *wheel_velocity = scale * ( -1.0f * x_velocity + R * wb ) / 2;
        break;

    case 2: // Right wheel at 30°
        // vector = (sin30, cos30) = (0.5, 0.866)
        *wheel_velocity = scale * ( sin_d * x_velocity + cos_d * y_velocity + R * wb ) / 2.8;
        break;

    default:
        *wheel_velocity = 0.0f;
        break;
    }
}

float rotate_on_axis(bool cw, float wb_rad_s, float angle_deg, enum movements_num *movements)
{
    static float t;
    float dir = cw ? -1.0f : 1.0f;
    float wb = wb_rad_s * dir;

    float angle_rad  = angle_deg * PI / 180.0f;
    float total_time = fabsf(angle_rad / wb_rad_s);

    if (t < total_time) {
        t += 0.0007;
        return wb;   // aún girando
    } else {
        t = 0;
        *movements = DO_NOT_MOVE;
        return 0.0f; // terminó el giro
    }
}

void cal_forward_kinematics(float wl_rad_s, float wb_rad_s, float wr_rad_s,
                             float *x_velocity, float *y_velocity, float *angular_velocity)
{
    const float sin_d = sinf(DELTA);   // = 0.5
    const float cos_d = cosf(DELTA);   // = 0.8660
    const float scale = R / N;

    *x_velocity = scale * ( sin_d* wl_rad_s- sin_d * wr_rad_s );
    *y_velocity = scale * (  cos_d * wl_rad_s               + cos_d * wr_rad_s );
    *angular_velocity         = scale * ( (1.0f / 3.0f) * ( wl_rad_s + wb_rad_s + wr_rad_s ) );
}

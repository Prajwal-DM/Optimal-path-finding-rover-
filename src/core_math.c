/**
 * @file core_math.c
 * @brief Core scalar math implementations.
 */

#include "core_math.h"

float math_calc_slope_deg(float delta_h, float horizontal_dist) {
    float abs_dist = fabsf(horizontal_dist);
    if (abs_dist <= 1e-6f) {
        return (fabsf(delta_h) > 1e-6f) ? 90.0f : 0.0f;
    }
    float gradient = fabsf(delta_h) / abs_dist;
    return atanf(gradient) * RAD_TO_DEG;
}

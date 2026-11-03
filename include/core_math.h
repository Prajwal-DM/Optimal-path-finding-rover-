/**
 * @file core_math.h
 * @brief Fast scalar math routines, distance metrics, and vector helpers.
 */

#ifndef CORE_MATH_H
#define CORE_MATH_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MATH_PI          3.14159265358979323846f
#define MATH_SQRT2       1.41421356237309504880f
#define MATH_SQRT2_MINUS_1 (MATH_SQRT2 - 1.0f) /* ~0.41421356f */
#define DEG_TO_RAD       (MATH_PI / 180.0f)
#define RAD_TO_DEG       (180.0f / MATH_PI)

/**
 * @brief Fast floating-point clamp.
 */
static inline float math_clampf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief Fast floating-point min/max.
 */
static inline float math_minf(float a, float b) {
    return (a < b) ? a : b;
}

static inline float math_maxf(float a, float b) {
    return (a > b) ? a : b;
}

/**
 * @brief Integer absolute value.
 */
static inline int32_t math_absi(int32_t v) {
    return (v < 0) ? -v : v;
}

/**
 * @brief Linear interpolation.
 */
static inline float math_lerpf(float a, float b, float t) {
    return a + t * (b - a);
}

/**
 * @brief Octile distance between two 2D grid coordinates.
 * Formula: max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)
 * In unscaled grid units (cardinal = 1.0, diagonal = sqrt(2)).
 */
static inline float math_octile_distance(int32_t x1, int32_t y1, int32_t x2, int32_t y2) {
    int32_t dx = math_absi(x1 - x2);
    int32_t dy = math_absi(y1 - y2);
    int32_t max_d = (dx > dy) ? dx : dy;
    int32_t min_d = (dx < dy) ? dx : dy;
    return (float)max_d + MATH_SQRT2_MINUS_1 * (float)min_d;
}

/**
 * @brief Physical octile distance scaled by cell size in meters.
 */
static inline float math_octile_distance_scaled(int32_t x1, int32_t y1, int32_t x2, int32_t y2, float cell_size_meters) {
    return math_octile_distance(x1, y1, x2, y2) * cell_size_meters;
}

/**
 * @brief Provably admissible and consistent heuristic for Rover A* search.
 * Formula: h(n) = alpha * f_min * (octile_distance * cell_size)
 * Since no legal move can cost less than (alpha * f_min * cell_dist), this never overestimates.
 */
static inline float math_admissible_heuristic(int32_t x1, int32_t y1, int32_t x2, int32_t y2,
                                              float cell_size_meters, float alpha, float f_min) {
    return alpha * f_min * math_octile_distance_scaled(x1, y1, x2, y2, cell_size_meters);
}

/**
 * @brief Euclidean distance in 2D.
 */
static inline float math_euclidean_distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * @brief Calculate slope angle in degrees from elevation change and horizontal distance.
 * @param delta_h Height difference (meters)
 * @param horizontal_dist Horizontal distance (meters)
 * @return Slope angle in degrees [0, 90]
 */
float math_calc_slope_deg(float delta_h, float horizontal_dist);

#ifdef __cplusplus
}
#endif

#endif /* CORE_MATH_H */

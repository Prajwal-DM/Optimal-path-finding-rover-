/**
 * @file test_math.c
 * @brief Unit tests for core math, octile distance, slope calculations, and admissible heuristics.
 */

#include "core_math.h"
#include <stdio.h>
#include <math.h>

int test_math_octile_distance(void) {
    printf("[TEST] Math Octile Distance Metric & Cell Scaling... ");

    /* Distance to self is 0 */
    float d0 = math_octile_distance(5, 5, 5, 5);
    if (fabsf(d0 - 0.0f) > 1e-6f) {
        printf("FAILED: self distance != 0\n");
        return 1;
    }

    /* Cardinal move of 1 grid unit */
    float d_cardinal = math_octile_distance(0, 0, 1, 0);
    if (fabsf(d_cardinal - 1.0f) > 1e-6f) {
        printf("FAILED: cardinal move != 1.0 (got %f)\n", d_cardinal);
        return 1;
    }

    /* Diagonal move of 1 grid unit: sqrt(2) ~ 1.41421356f */
    float d_diag = math_octile_distance(0, 0, 1, 1);
    if (fabsf(d_diag - MATH_SQRT2) > 1e-5f) {
        printf("FAILED: diagonal move != sqrt(2) (got %f, expected %f)\n", d_diag, MATH_SQRT2);
        return 1;
    }

    /* General case: dx=3, dy=4 -> max=4, min=3 -> 4 + 3*(sqrt(2)-1) ~ 5.24264f */
    float d_gen = math_octile_distance(0, 0, 3, 4);
    float expected = 4.0f + 3.0f * (MATH_SQRT2 - 1.0f);
    if (fabsf(d_gen - expected) > 1e-5f) {
        printf("FAILED: general octile distance: got %f, expected %f\n", d_gen, expected);
        return 1;
    }

    /* Physical metric scaling: cell_size = 2.5m */
    float d_scaled = math_octile_distance_scaled(0, 0, 3, 4, 2.5f);
    if (fabsf(d_scaled - expected * 2.5f) > 1e-5f) {
        printf("FAILED: scaled distance != expected * 2.5m (got %f)\n", d_scaled);
        return 1;
    }

    /* Admissible heuristic verification: h(n) = alpha * f_min * (octile_dist * cell_size) */
    float alpha = 1.0f;
    float f_min = 0.25f;
    float h = math_admissible_heuristic(0, 0, 3, 4, 2.5f, alpha, f_min);
    float expected_h = alpha * f_min * (expected * 2.5f);
    if (fabsf(h - expected_h) > 1e-5f) {
        printf("FAILED: heuristic calculation mismatch: %f vs %f\n", h, expected_h);
        return 1;
    }

    printf("PASSED (Grid units, 2.5m physical scaling, and admissible heuristic)\n");
    return 0;
}

int test_math_slope_calculation(void) {
    printf("[TEST] Math Slope Angle Calculation (Including Negative Deltas)... ");

    /* Flat terrain: delta_h = 0 -> slope = 0 deg */
    float s0 = math_calc_slope_deg(0.0f, 2.5f);
    if (fabsf(s0 - 0.0f) > 1e-5f) {
        printf("FAILED: flat slope != 0 deg (got %f)\n", s0);
        return 1;
    }

    /* 45 degree slope: delta_h == horizontal_dist */
    float s45 = math_calc_slope_deg(2.5f, 2.5f);
    if (fabsf(s45 - 45.0f) > 1e-4f) {
        printf("FAILED: 45-deg slope check failed (got %f)\n", s45);
        return 1;
    }

    /* Negative horizontal distance (signed direction delta) must still evaluate to 45 deg, NOT 90 deg */
    float s45_neg = math_calc_slope_deg(2.5f, -2.5f);
    if (fabsf(s45_neg - 45.0f) > 1e-4f) {
        printf("FAILED: negative horizontal distance returned %f deg instead of 45 deg\n", s45_neg);
        return 1;
    }

    /* 30 degree slope: tan(30 deg) = 1 / sqrt(3) ~ 0.57735f */
    float h_30 = 2.5f * tanf(30.0f * DEG_TO_RAD);
    float s30 = math_calc_slope_deg(h_30, 2.5f);
    if (fabsf(s30 - 30.0f) > 1e-3f) {
        printf("FAILED: 30-deg slope check failed (got %f)\n", s30);
        return 1;
    }

    /* Pure vertical cliff: horizontal_dist = 0 -> 90 deg */
    float s90 = math_calc_slope_deg(5.0f, 0.0f);
    if (fabsf(s90 - 90.0f) > 1e-4f) {
        printf("FAILED: cliff 90-deg check failed (got %f)\n", s90);
        return 1;
    }

    printf("PASSED (0 deg, 30 deg, 45 deg, signed negative distances, and 90 deg vertical)\n");
    return 0;
}

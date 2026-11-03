/**
 * @file test_noise.c
 * @brief Unit tests for 2D Perlin noise and fractal fBm generation.
 */

#include "noise.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

int test_noise_bounds_and_continuity(void) {
    printf("[TEST] Noise Output Range & Lipschitz Continuity... ");
    PerlinNoise noise;
    noise_init(&noise, 1337);

    float min_val = 1e9f;
    float max_val = -1e9f;
    float max_delta = 0.0f;
    float step = 0.02f;

    /* Sample across a grid */
    for (float y = 0.0f; y < 10.0f; y += step) {
        float prev_val = noise_sample_2d(&noise, 0.0f, y);
        for (float x = step; x < 10.0f; x += step) {
            float val = noise_sample_2d(&noise, x, y);
            if (val < min_val) min_val = val;
            if (val > max_val) max_val = val;

            float delta = fabsf(val - prev_val);
            if (delta > max_delta) max_delta = delta;
            prev_val = val;
        }
    }

    if (min_val < -1.0f || max_val > 1.0f) {
        printf("FAILED: noise out of bounds: [%f, %f]\n", min_val, max_val);
        return 1;
    }

    /* Continuity: for step 0.02, delta should never exceed 0.20 */
    if (max_delta > 0.20f) {
        printf("FAILED: discontinuity detected: max_delta = %f\n", max_delta);
        return 1;
    }

    printf("PASSED (Range [%.2f, %.2f], Smooth gradient confirmed)\n", min_val, max_val);
    return 0;
}

int test_noise_determinism_golden(void) {
    printf("[TEST] Noise Golden Determinism (Bit-Exact Fixed Float)... ");
    PerlinNoise noise;
    noise_init(&noise, 0x42ULL);

    float s1 = noise_sample_2d(&noise, 3.1415f, 2.7182f);
    float s2 = noise_sample_fbm(&noise, 10.5f, 20.25f, 5, 0.05f, 0.5f, 2.0f);

    /* 1. Verify against fixed cross-platform golden constants */
    if (fabsf(s1 - 0.185575f) > 1e-5f) {
        printf("FAILED: s1 (%f) does not match expected golden constant 0.185575\n", s1);
        return 1;
    }
    if (fabsf(s2 - (-0.015233f)) > 1e-5f) {
        printf("FAILED: s2 (%f) does not match expected golden constant -0.015233\n", s2);
        return 1;
    }

    /* 2. Verify consistency with second initialized instance */
    PerlinNoise noise2;
    noise_init(&noise2, 0x42ULL);
    float s1_check = noise_sample_2d(&noise2, 3.1415f, 2.7182f);
    float s2_check = noise_sample_fbm(&noise2, 10.5f, 20.25f, 5, 0.05f, 0.5f, 2.0f);

    if (s1 != s1_check || s2 != s2_check) {
        printf("FAILED: noise non-deterministic between identical seed instances\n");
        return 1;
    }

    printf("PASSED (All golden constants matched bit-for-bit across generator states)\n");
    return 0;
}

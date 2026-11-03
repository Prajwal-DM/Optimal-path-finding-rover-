/**
 * @file noise.h
 * @brief 2D Perlin Noise and Fractal Brownian Motion (fBm) generator.
 * Deterministically seeded using PrngState. Zero runtime allocations.
 */

#ifndef NOISE_H
#define NOISE_H

#include "prng.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NOISE_PERM_SIZE 256

typedef struct {
    uint8_t perm[NOISE_PERM_SIZE * 2]; /* Duplicated permutation table for fast wrap-around */
} PerlinNoise;

/**
 * @brief Initialize Perlin noise lookup tables from an integer seed.
 */
void noise_init(PerlinNoise *noise, uint64_t seed);

/**
 * @brief Compute single-octave 2D Perlin noise at continuous coordinates (x, y).
 * @return Normalized value in range [-1.0f, 1.0f].
 */
float noise_sample_2d(const PerlinNoise *noise, float x, float y);

/**
 * @brief Compute multi-octave Fractal Brownian Motion (fBm) height.
 * @param noise Noise state
 * @param x World X coordinate
 * @param y World Y coordinate
 * @param octaves Number of noise layers (e.g. 4-6)
 * @param frequency Base spatial frequency
 * @param persistence Amplitude multiplier per octave (e.g. 0.5f)
 * @param lacunarity Frequency multiplier per octave (e.g. 2.0f)
 * @return Value roughly in [-1.0f, 1.0f].
 */
float noise_sample_fbm(const PerlinNoise *noise,
                       float x, float y,
                       int octaves,
                       float frequency,
                       float persistence,
                       float lacunarity);

#ifdef __cplusplus
}
#endif

#endif /* NOISE_H */

/**
 * @file noise.c
 * @brief Deterministic 2D Perlin noise and fractal noise implementation.
 */

#include "noise.h"
#include "core_math.h"
#include <math.h>

/* 8 2D gradient vectors normalized to unit length or standard orthogonal directions */
static const float GRAD2[8][2] = {
    {  1.0f,  0.0f },
    { -1.0f,  0.0f },
    {  0.0f,  1.0f },
    {  0.0f, -1.0f },
    {  0.70710678f,  0.70710678f },
    { -0.70710678f,  0.70710678f },
    {  0.70710678f, -0.70710678f },
    { -0.70710678f, -0.70710678f }
};

/* Ken Perlin's quintic fade curve: 6t^5 - 15t^4 + 10t^3 */
static inline float fade(float t) {
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

/* Gradient dot product */
static inline float grad(uint8_t hash, float x, float y) {
    int g = hash & 7;
    return GRAD2[g][0] * x + GRAD2[g][1] * y;
}

void noise_init(PerlinNoise *noise, uint64_t seed) {
    if (!noise) return;

    PrngState rng;
    prng_init(&rng, seed);

    /* Initialize identity array [0..255] */
    uint8_t p[NOISE_PERM_SIZE];
    for (int i = 0; i < NOISE_PERM_SIZE; ++i) {
        p[i] = (uint8_t)i;
    }

    /* Fisher-Yates shuffle using deterministic PRNG */
    for (int i = NOISE_PERM_SIZE - 1; i > 0; --i) {
        int j = prng_next_int_range(&rng, 0, i);
        uint8_t temp = p[i];
        p[i] = p[j];
        p[j] = temp;
    }

    /* Duplicate permutation to eliminate modulo operations in hot inner loop */
    for (int i = 0; i < NOISE_PERM_SIZE; ++i) {
        noise->perm[i] = p[i];
        noise->perm[NOISE_PERM_SIZE + i] = p[i];
    }
}

float noise_sample_2d(const PerlinNoise *noise, float x, float y) {
    if (!noise) return 0.0f;

    /* Unit grid cell coordinates */
    int xi = (int)floorf(x);
    int yi = (int)floorf(y);

    /* Relative coordinates inside cell */
    float xf = x - (float)xi;
    float yf = y - (float)yi;

    /* Wrap cell coordinates to [0..255] */
    int X = xi & 255;
    int Y = yi & 255;

    /* Smooth fade weights */
    float u = fade(xf);
    float v = fade(yf);

    /* Hash coordinates of 4 corners */
    uint8_t aa = noise->perm[noise->perm[X] + Y];
    uint8_t ab = noise->perm[noise->perm[X] + Y + 1];
    uint8_t ba = noise->perm[noise->perm[X + 1] + Y];
    uint8_t bb = noise->perm[noise->perm[X + 1] + Y + 1];

    /* Dot products from 4 corners */
    float g_aa = grad(aa, xf, yf);
    float g_ba = grad(ba, xf - 1.0f, yf);
    float g_ab = grad(ab, xf, yf - 1.0f);
    float g_bb = grad(bb, xf - 1.0f, yf - 1.0f);

    /* Bilinear interpolation with quintic weights */
    float x1 = math_lerpf(g_aa, g_ba, u);
    float x2 = math_lerpf(g_ab, g_bb, u);
    return math_lerpf(x1, x2, v);
}

float noise_sample_fbm(const PerlinNoise *noise,
                       float x, float y,
                       int octaves,
                       float frequency,
                       float persistence,
                       float lacunarity) {
    if (!noise || octaves <= 0) return 0.0f;

    float total = 0.0f;
    float amplitude = 1.0f;
    float max_amplitude = 0.0f;
    float current_freq = frequency;

    for (int o = 0; o < octaves; ++o) {
        total += noise_sample_2d(noise, x * current_freq, y * current_freq) * amplitude;
        max_amplitude += amplitude;
        amplitude *= persistence;
        current_freq *= lacunarity;
    }

    return (max_amplitude > 1e-6f) ? (total / max_amplitude) : 0.0f;
}

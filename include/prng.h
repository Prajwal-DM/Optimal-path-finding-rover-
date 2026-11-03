/**
 * @file prng.h
 * @brief Deterministic, platform-independent Pseudo-Random Number Generator (Xorshift64* / SplitMix64).
 * Guarantees bit-identical output across native x86/x64 and WebAssembly (wasm32).
 */

#ifndef PRNG_H
#define PRNG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t state;
} PrngState;

/**
 * @brief Initialize PRNG with an arbitrary integer seed.
 * Uses SplitMix64 to scramble the seed so even low or 0 seeds produce rich entropy.
 */
void prng_init(PrngState *rng, uint64_t seed);

/**
 * @brief Generate next 64-bit unsigned integer using Xorshift64*.
 */
uint64_t prng_next_u64(PrngState *rng);

/**
 * @brief Generate next 32-bit unsigned integer.
 */
uint32_t prng_next_u32(PrngState *rng);

/**
 * @brief Generate uniform floating-point number in [0.0f, 1.0f).
 */
float prng_next_float(PrngState *rng);

/**
 * @brief Generate uniform float in range [min_val, max_val).
 */
float prng_next_float_range(PrngState *rng, float min_val, float max_val);

/**
 * @brief Generate uniform integer in range [min_val, max_val] inclusive.
 */
int32_t prng_next_int_range(PrngState *rng, int32_t min_val, int32_t max_val);

/* Global default PRNG instance helpers */
void prng_global_init(uint64_t seed);
uint32_t prng_global_u32(void);
float prng_global_float(void);
float prng_global_float_range(float min_val, float max_val);
int32_t prng_global_int_range(int32_t min_val, int32_t max_val);

#ifdef __cplusplus
}
#endif

#endif /* PRNG_H */

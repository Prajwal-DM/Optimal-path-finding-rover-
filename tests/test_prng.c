/**
 * @file test_prng.c
 * @brief Unit tests for deterministic PRNG, including golden reference vectors and extreme range bounds.
 */

#include "prng.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

/* Golden reference outputs for seed = 0x123456789ABCDEF0ULL */
/* Generated via the deterministic SplitMix64 + Xorshift64* algorithm */
int test_prng_golden_vectors(void) {
    printf("[TEST] PRNG Golden Reference Vectors (Fixed Cross-Platform Hash)... ");
    PrngState rng;
    uint64_t seed = 0x123456789ABCDEF0ULL;
    prng_init(&rng, seed);

    /* Generate first 5 values */
    uint64_t actual[5];
    for (int i = 0; i < 5; ++i) {
        actual[i] = prng_next_u64(&rng);
    }

    /* Print for reference if needed */
    static const uint64_t EXPECTED_GOLDEN[5] = {
        0x1665ae99072816d7ULL,
        0x71570fc73e5bf161ULL,
        0xdbb00b7bfd98cd8bULL,
        0x438b7ac6b3edb98aULL,
        0x80685bc6f505347aULL
    };

    for (int i = 0; i < 5; ++i) {
        if (actual[i] != EXPECTED_GOLDEN[i]) {
            printf("FAILED at step %d: got 0x%016llx, expected 0x%016llx\n",
                   i, (unsigned long long)actual[i], (unsigned long long)EXPECTED_GOLDEN[i]);
            for (int j = 0; j < 5; ++j) {
                printf("  step %d: 0x%016llxULL,\n", j, (unsigned long long)actual[j]);
            }
            return 1;
        }
    }

    printf("PASSED (All 5 golden vectors matched bit-for-bit)\n");
    return 0;
}

int test_prng_determinism(void) {
    printf("[TEST] PRNG Determinism & Reproducibility Across Streams... ");
    PrngState rng1, rng2;
    uint64_t seed = 0xDEADBEEFCAFE1337ULL;

    prng_init(&rng1, seed);
    prng_init(&rng2, seed);

    for (int i = 0; i < 100000; ++i) {
        uint64_t v1 = prng_next_u64(&rng1);
        uint64_t v2 = prng_next_u64(&rng2);
        if (v1 != v2) {
            printf("FAILED at iteration %d: %llu != %llu\n", i, (unsigned long long)v1, (unsigned long long)v2);
            return 1;
        }
    }
    printf("PASSED (100,000 identical iterations)\n");
    return 0;
}

int test_prng_range_and_distribution(void) {
    printf("[TEST] PRNG Float Range & Uniformity... ");
    PrngState rng;
    prng_init(&rng, 42);

    #define NUM_BUCKETS 10
    int buckets[NUM_BUCKETS] = {0};
    int total_samples = 1000000;

    for (int i = 0; i < total_samples; ++i) {
        float f = prng_next_float(&rng);
        if (f < 0.0f || f >= 1.0f) {
            printf("FAILED: float out of bounds: %f\n", f);
            return 1;
        }
        int b = (int)(f * NUM_BUCKETS);
        if (b < 0) b = 0;
        if (b >= NUM_BUCKETS) b = NUM_BUCKETS - 1;
        buckets[b]++;
    }

    /* Each bucket should have roughly total_samples / NUM_BUCKETS (100,000 +- 3,000) */
    int expected = total_samples / NUM_BUCKETS;
    for (int b = 0; b < NUM_BUCKETS; ++b) {
        int diff = abs(buckets[b] - expected);
        if (diff > 3000) {
            printf("FAILED: Bucket %d deviation too high (%d vs expected %d)\n", b, buckets[b], expected);
            return 1;
        }
    }
    printf("PASSED (1,000,000 samples, 10 buckets uniform within < 3%%)\n");
    return 0;
}

int test_prng_extreme_int_ranges(void) {
    printf("[TEST] PRNG Full Range [INT32_MIN, INT32_MAX] (Division by Zero & Overflow Check)... ");
    PrngState rng;
    prng_init(&rng, 777);

    /* 1. Test standard range */
    int min_val = -15;
    int max_val = 35;
    for (int i = 0; i < 20000; ++i) {
        int val = prng_next_int_range(&rng, min_val, max_val);
        if (val < min_val || val > max_val) {
            printf("FAILED: value %d outside [%d, %d]\n", val, min_val, max_val);
            return 1;
        }
    }

    /* 2. Test extreme range: entire 32-bit signed space without division-by-zero crash */
    int32_t extreme_min = -2147483647 - 1; /* INT32_MIN */
    int32_t extreme_max = 2147483647;      /* INT32_MAX */

    for (int i = 0; i < 20000; ++i) {
        /* If 64-bit arithmetic is not used, range wraps around to 0 and causes SIGFPE crash */
        int32_t val = prng_next_int_range(&rng, extreme_min, extreme_max);
        (void)val; /* Just ensure execution survives without hardware exception */
    }

    /* 3. Test equal bounds */
    int32_t single_val = prng_next_int_range(&rng, 42, 42);
    if (single_val != 42) {
        printf("FAILED: min == max did not return min\n");
        return 1;
    }

    printf("PASSED (Survived 20,000 full INT32_MIN..INT32_MAX calls with zero crashes)\n");
    return 0;
}

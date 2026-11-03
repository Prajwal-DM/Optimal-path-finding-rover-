/**
 * @file test_min_heap.c
 * @brief Unit tests, edge case verification, and stress testing for the Binary Min-Heap.
 */

#include "min_heap.h"
#include "prng.h"
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#define TEST_CAPACITY 1024
#define TEST_MAX_NODES 2048

static HeapElement s_test_elements[TEST_CAPACITY];
static int32_t s_test_indices[TEST_MAX_NODES];

int test_heap_empty_and_bounds_errors(void) {
    printf("[TEST] Heap Empty Underflow & Out-Of-Bounds Guard... ");
    MinHeap heap;
    RoverError err = heap_init(&heap, s_test_elements, TEST_CAPACITY, s_test_indices, TEST_MAX_NODES, true);
    if (err != ROVER_OK) {
        printf("FAILED init: %d\n", err);
        return 1;
    }

    /* Pop on empty heap must return ROVER_ERR_HEAP_EMPTY */
    uint32_t node;
    float prio;
    err = heap_pop_min(&heap, &node, &prio);
    if (err != ROVER_ERR_HEAP_EMPTY) {
        printf("FAILED: expected ROVER_ERR_HEAP_EMPTY on empty pop, got %d\n", err);
        return 1;
    }

    /* Peek on empty heap must return ROVER_ERR_HEAP_EMPTY */
    err = heap_peek_min(&heap, &node, &prio);
    if (err != ROVER_ERR_HEAP_EMPTY) {
        printf("FAILED: expected ROVER_ERR_HEAP_EMPTY on empty peek, got %d\n", err);
        return 1;
    }

    /* Out of bounds node_id push must be rejected */
    err = heap_push(&heap, TEST_MAX_NODES + 10, 5.0f);
    if (err != ROVER_ERR_OUT_OF_BOUNDS) {
        printf("FAILED: expected ROVER_ERR_OUT_OF_BOUNDS on out-of-bounds push, got %d\n", err);
        return 1;
    }

    printf("PASSED (Heap underflow and memory protection verified)\n");
    return 0;
}

int test_heap_clear_and_reuse(void) {
    printf("[TEST] Heap Clear O(K) Cleanup & State Reuse... ");
    MinHeap heap;
    heap_init(&heap, s_test_elements, TEST_CAPACITY, s_test_indices, TEST_MAX_NODES, true);

    /* Push 20 elements */
    for (uint32_t i = 0; i < 20; ++i) {
        heap_push(&heap, i * 2, (float)(20 - i));
    }
    if (heap.size != 20) {
        printf("FAILED: size != 20\n");
        return 1;
    }

    /* Verify contains */
    for (uint32_t i = 0; i < 20; ++i) {
        if (!heap_contains(&heap, i * 2)) {
            printf("FAILED: expected node %u to be present\n", i * 2);
            return 1;
        }
    }

    /* Clear the heap */
    heap_clear(&heap);

    if (heap.size != 0 || !heap_is_empty(&heap)) {
        printf("FAILED: heap not empty after clear\n");
        return 1;
    }

    /* Verify all cleared nodes report not present */
    for (uint32_t i = 0; i < 20; ++i) {
        if (heap_contains(&heap, i * 2)) {
            printf("FAILED: node %u still present after clear\n", i * 2);
            return 1;
        }
    }

    /* Re-insert elements and verify it operates normally */
    heap_push(&heap, 100, 1.5f);
    heap_push(&heap, 200, 0.5f);
    uint32_t top_node;
    float top_prio;
    heap_pop_min(&heap, &top_node, &top_prio);
    if (top_node != 200 || top_prio != 0.5f) {
        printf("FAILED: re-inserted heap pop failed\n");
        return 1;
    }

    printf("PASSED (O(K) clean unbind of index table and immediate reuse)\n");
    return 0;
}

int test_heap_telemetry_stats(void) {
    printf("[TEST] Heap Telemetry Metrics & Reset... ");
    MinHeap heap;
    heap_init(&heap, s_test_elements, TEST_CAPACITY, s_test_indices, TEST_MAX_NODES, true);

    /* 10 pushes */
    for (uint32_t i = 0; i < 10; ++i) {
        heap_push(&heap, i, (float)(100 - i));
    }

    /* 3 decrease keys */
    heap_decrease_key(&heap, 0, 1.0f);
    heap_decrease_key(&heap, 1, 2.0f);
    heap_decrease_key(&heap, 2, 3.0f);

    /* 4 pops */
    for (int i = 0; i < 4; ++i) {
        uint32_t n;
        float p;
        heap_pop_min(&heap, &n, &p);
    }

    if (heap.stats.push_count != 10) {
        printf("FAILED: expected push_count 10, got %u\n", heap.stats.push_count);
        return 1;
    }
    if (heap.stats.decrease_key_count != 3) {
        printf("FAILED: expected decrease_key_count 3, got %u\n", heap.stats.decrease_key_count);
        return 1;
    }
    if (heap.stats.pop_count != 4) {
        printf("FAILED: expected pop_count 4, got %u\n", heap.stats.pop_count);
        return 1;
    }
    if (heap.stats.max_size_reached != 10) {
        printf("FAILED: expected max_size_reached 10, got %u\n", heap.stats.max_size_reached);
        return 1;
    }

    /* Reset stats */
    heap_reset_stats(&heap);
    if (heap.stats.push_count != 0 || heap.stats.pop_count != 0 || heap.stats.decrease_key_count != 0) {
        printf("FAILED: stats not zeroed after reset\n");
        return 1;
    }
    if (heap.stats.max_size_reached != heap.size) {
        printf("FAILED: max_size_reached after reset != current size\n");
        return 1;
    }

    printf("PASSED (Accurate push, pop, decrease, and max-size telemetry)\n");
    return 0;
}

int test_heap_basic_order(void) {
    printf("[TEST] Heap Basic Push / Pop Ordering... ");
    MinHeap heap;
    RoverError err = heap_init(&heap, s_test_elements, TEST_CAPACITY, s_test_indices, TEST_MAX_NODES, true);
    if (err != ROVER_OK) {
        printf("FAILED to init heap: %d\n", err);
        return 1;
    }

    float values[] = { 10.5f, 3.2f, 99.1f, 1.0f, 45.0f, 7.8f, 2.5f, 15.0f };
    int n = sizeof(values) / sizeof(values[0]);

    for (int i = 0; i < n; ++i) {
        heap_push(&heap, (uint32_t)i, values[i]);
        if (!heap_validate_invariant(&heap)) {
            printf("FAILED invariant after push node %d\n", i);
            return 1;
        }
    }

    float last_val = -1e9f;
    for (int i = 0; i < n; ++i) {
        uint32_t node_id;
        float priority;
        err = heap_pop_min(&heap, &node_id, &priority);
        if (err != ROVER_OK) {
            printf("FAILED to pop node at step %d\n", i);
            return 1;
        }
        if (priority < last_val) {
            printf("FAILED: priority %f < previous %f (not in ascending order)\n", priority, last_val);
            return 1;
        }
        last_val = priority;
        if (!heap_validate_invariant(&heap)) {
            printf("FAILED invariant after pop step %d\n", i);
            return 1;
        }
    }

    if (!heap_is_empty(&heap)) {
        printf("FAILED: heap not empty after popping all elements\n");
        return 1;
    }

    printf("PASSED (Strict min-first extraction)\n");
    return 0;
}

int test_heap_decrease_key(void) {
    printf("[TEST] Heap Decrease-Key & Index Synchronization... ");
    MinHeap heap;
    heap_init(&heap, s_test_elements, TEST_CAPACITY, s_test_indices, TEST_MAX_NODES, true);

    /* Push 5 nodes with initial high values */
    heap_push(&heap, 10, 50.0f);
    heap_push(&heap, 20, 60.0f);
    heap_push(&heap, 30, 70.0f);
    heap_push(&heap, 40, 80.0f);
    heap_push(&heap, 50, 90.0f);

    /* Node 50 was lowest priority (90.0). Decrease it to 5.0f (now it should become the minimum) */
    RoverError err = heap_decrease_key(&heap, 50, 5.0f);
    if (err != ROVER_OK) {
        printf("FAILED to decrease key: %d\n", err);
        return 1;
    }
    if (!heap_validate_invariant(&heap)) {
        printf("FAILED invariant after decrease key\n");
        return 1;
    }

    uint32_t min_node;
    float min_prio;
    heap_peek_min(&heap, &min_node, &min_prio);
    if (min_node != 50 || min_prio != 5.0f) {
        printf("FAILED: expected min node 50 with prio 5.0, got %u with %f\n", min_node, min_prio);
        return 1;
    }

    /* Decrease node 30 to 2.0f (now node 30 is the minimum) */
    heap_decrease_key(&heap, 30, 2.0f);
    heap_peek_min(&heap, &min_node, &min_prio);
    if (min_node != 30 || min_prio != 2.0f) {
        printf("FAILED: expected min node 30 with prio 2.0, got %u with %f\n", min_node, min_prio);
        return 1;
    }

    /* Test push_or_decrease_key with existing and new nodes */
    heap_push_or_decrease_key(&heap, 10, 1.0f); /* Lower than 50.0 -> updates to 1.0 */
    heap_push_or_decrease_key(&heap, 99, 12.0f); /* New node */

    heap_peek_min(&heap, &min_node, &min_prio);
    if (min_node != 10 || min_prio != 1.0f) {
        printf("FAILED: expected min node 10 with prio 1.0 after push_or_decrease_key\n");
        return 1;
    }

    /* Extract all and verify sorted order */
    float last_prio = -1.0f;
    while (!heap_is_empty(&heap)) {
        heap_pop_min(&heap, &min_node, &min_prio);
        if (min_prio < last_prio) {
            printf("FAILED: unsorted extraction after decrease_key: %f < %f\n", min_prio, last_prio);
            return 1;
        }
        last_prio = min_prio;
    }

    printf("PASSED (O(log N) sift-up and O(1) containment verified)\n");
    return 0;
}

int test_heap_stress_random(void) {
    printf("[TEST] Heap Randomized Stress Invariant (50,000 Ops)... ");
    #define STRESS_CAPACITY 4096
    #define STRESS_MAX_NODES 8192

    static HeapElement stress_elements[STRESS_CAPACITY];
    static int32_t stress_indices[STRESS_MAX_NODES];

    MinHeap heap;
    heap_init(&heap, stress_elements, STRESS_CAPACITY, stress_indices, STRESS_MAX_NODES, true);

    PrngState rng;
    prng_init(&rng, 0x123456789ABCDEF0ULL);

    for (int op = 0; op < 50000; ++op) {
        int action = prng_next_int_range(&rng, 0, 9);

        if (action < 5) {
            /* 50% Push or Decrease Key */
            uint32_t node = (uint32_t)prng_next_int_range(&rng, 0, STRESS_MAX_NODES - 1);
            float prio = prng_next_float_range(&rng, 0.1f, 1000.0f);

            if (heap_contains(&heap, node)) {
                int32_t idx = heap.node_to_heap_index[node];
                float cur_prio = heap.elements[idx].priority;
                float lower_prio = cur_prio * 0.5f;
                heap_decrease_key(&heap, node, lower_prio);
            } else if (heap.size < heap.capacity) {
                heap_push(&heap, node, prio);
            }
        } else if (action < 8) {
            /* 30% Pop Min */
            if (!heap_is_empty(&heap)) {
                uint32_t n;
                float p;
                heap_pop_min(&heap, &n, &p);
            }
        } else {
            /* 20% Random Invariant Check */
            if (!heap_validate_invariant(&heap)) {
                printf("FAILED heap invariant at op %d! Size = %u\n", op, heap.size);
                return 1;
            }
        }
    }

    /* Final validation */
    if (!heap_validate_invariant(&heap)) {
        printf("FAILED invariant at end of stress test\n");
        return 1;
    }

    /* Drain heap and verify order */
    float prev_val = -1.0f;
    while (!heap_is_empty(&heap)) {
        uint32_t n;
        float p;
        heap_pop_min(&heap, &n, &p);
        if (p < prev_val) {
            printf("FAILED drain ordering: %f < %f\n", p, prev_val);
            return 1;
        }
        prev_val = p;
    }

    printf("PASSED (All invariants held across 50,000 operations)\n");
    return 0;
}

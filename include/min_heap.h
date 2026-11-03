/**
 * @file min_heap.h
 * @brief High-performance Binary Min-Heap Priority Queue with $O(1)$ lookup and $O(\log N)$ decrease-key.
 *
 * Implements an indexed binary heap using contiguous flat arrays.
 * Zero dynamic memory allocations during search execution.
 */

#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include "common.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEAP_NOT_PRESENT -1

/**
 * @brief Key-value pair stored in the binary heap.
 */
typedef struct {
    uint32_t node_id;   /* Grid cell index: y * width + x */
    float priority;     /* Search score (e.g. f_score) */
} HeapElement;

/**
 * @brief Statistics collected for performance benchmarking and DSA analysis.
 */
typedef struct {
    uint32_t push_count;
    uint32_t pop_count;
    uint32_t decrease_key_count;
    uint32_t max_size_reached;
} HeapStats;

/**
 * @brief Binary Min-Heap structure.
 */
typedef struct {
    HeapElement *elements;        /* Array of heap nodes (size capacity) */
    int32_t *node_to_heap_index;  /* Mapping from node_id -> index in elements[] (size max_nodes) */
    uint32_t size;                /* Current number of elements in heap */
    uint32_t capacity;            /* Maximum capacity of elements array */
    uint32_t max_nodes;           /* Upper bound on node_id (e.g. MAX_GRID_CELLS) */
    bool use_decrease_key;        /* If true, use indexed decrease-key; if false, lazy duplicates */
    HeapStats stats;              /* DSA metrics */
} MinHeap;

/**
 * @brief Initialize heap with pre-allocated memory buffers.
 * @param heap Pointer to MinHeap structure
 * @param elements_buffer Buffer of HeapElement with capacity elements
 * @param capacity Maximum number of elements heap can store
 * @param index_buffer Buffer of int32_t of size max_nodes (or NULL if lazy mode only)
 * @param max_nodes Maximum valid node_id + 1
 * @param use_decrease_key Enable indexed O(log N) decrease_key
 * @return ROVER_OK on success
 */
RoverError heap_init(MinHeap *heap,
                     HeapElement *elements_buffer,
                     uint32_t capacity,
                     int32_t *index_buffer,
                     uint32_t max_nodes,
                     bool use_decrease_key);

/**
 * @brief Reset the heap to empty state in O(K) where K = heap->size (without deallocating).
 * Only clears index table entries for active nodes, avoiding full O(V) array sweeps.
 */
void heap_clear(MinHeap *heap);

/**
 * @brief Push a new node into the heap.
 * @return ROVER_OK, ROVER_ERR_HEAP_FULL, or ROVER_ERR_OUT_OF_BOUNDS
 */
RoverError heap_push(MinHeap *heap, uint32_t node_id, float priority);

/**
 * @brief Extract the node with minimum priority.
 * @param out_node_id Pointer to receive the extracted node ID
 * @param out_priority Pointer to receive the extracted priority
 * @return ROVER_OK on success, ROVER_ERR_HEAP_EMPTY if heap is empty
 */
RoverError heap_pop_min(MinHeap *heap, uint32_t *out_node_id, float *out_priority);

/**
 * @brief Peek at the root without removing it.
 */
RoverError heap_peek_min(const MinHeap *heap, uint32_t *out_node_id, float *out_priority);

/**
 * @brief Update the priority of an existing node to a lower value.
 * Requires use_decrease_key = true.
 */
RoverError heap_decrease_key(MinHeap *heap, uint32_t node_id, float new_priority);

/**
 * @brief Push node if not present, or decrease key if new priority is lower.
 * Canonical operation for A* and Dijkstra.
 */
RoverError heap_push_or_decrease_key(MinHeap *heap, uint32_t node_id, float priority);

/**
 * @brief Check if a node currently exists in the heap in O(1) time.
 */
bool heap_contains(const MinHeap *heap, uint32_t node_id);

/**
 * @brief Get current heap size.
 */
static inline uint32_t heap_size(const MinHeap *heap) {
    return heap ? heap->size : 0;
}

/**
 * @brief Check if heap is empty.
 */
static inline bool heap_is_empty(const MinHeap *heap) {
    return (!heap || heap->size == 0);
}

/**
 * @brief Reset heap telemetry stats.
 */
void heap_reset_stats(MinHeap *heap);

/**
 * @brief Validate the heap invariant and index table integrity (for unit tests / debug).
 * @return true if valid, false if corrupt
 */
bool heap_validate_invariant(const MinHeap *heap);

#ifdef __cplusplus
}
#endif

#endif /* MIN_HEAP_H */

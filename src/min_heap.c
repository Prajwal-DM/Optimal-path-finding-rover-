/**
 * @file min_heap.c
 * @brief High-performance Binary Min-Heap Priority Queue implementation.
 */

#include "min_heap.h"
#include <string.h>

static inline void swap_elements(MinHeap *heap, uint32_t i, uint32_t j) {
    HeapElement temp = heap->elements[i];
    heap->elements[i] = heap->elements[j];
    heap->elements[j] = temp;

    if (heap->node_to_heap_index) {
        if (heap->elements[i].node_id < heap->max_nodes) {
            heap->node_to_heap_index[heap->elements[i].node_id] = (int32_t)i;
        }
        if (heap->elements[j].node_id < heap->max_nodes) {
            heap->node_to_heap_index[heap->elements[j].node_id] = (int32_t)j;
        }
    }
}

static void sift_up(MinHeap *heap, uint32_t index) {
    while (index > 0) {
        uint32_t parent = (index - 1) / 2;
        if (heap->elements[index].priority < heap->elements[parent].priority) {
            swap_elements(heap, index, parent);
            index = parent;
        } else {
            break;
        }
    }
}

static void sift_down(MinHeap *heap, uint32_t index) {
    uint32_t size = heap->size;
    while ((2 * index + 1) < size) {
        uint32_t left = 2 * index + 1;
        uint32_t right = left + 1;
        uint32_t smallest = index;

        if (heap->elements[left].priority < heap->elements[smallest].priority) {
            smallest = left;
        }
        if (right < size && heap->elements[right].priority < heap->elements[smallest].priority) {
            smallest = right;
        }

        if (smallest != index) {
            swap_elements(heap, index, smallest);
            index = smallest;
        } else {
            break;
        }
    }
}

RoverError heap_init(MinHeap *heap,
                     HeapElement *elements_buffer,
                     uint32_t capacity,
                     int32_t *index_buffer,
                     uint32_t max_nodes,
                     bool use_decrease_key) {
    if (!heap || !elements_buffer || capacity == 0) {
        return ROVER_ERR_INVALID_PARAM;
    }

    if (use_decrease_key && (!index_buffer || max_nodes == 0)) {
        return ROVER_ERR_INVALID_PARAM;
    }

    heap->elements = elements_buffer;
    heap->capacity = capacity;
    heap->node_to_heap_index = index_buffer;
    heap->max_nodes = max_nodes;
    heap->use_decrease_key = use_decrease_key;
    heap->size = 0;

    if (heap->node_to_heap_index) {
        for (uint32_t i = 0; i < max_nodes; ++i) {
            heap->node_to_heap_index[i] = HEAP_NOT_PRESENT;
        }
    }

    heap_reset_stats(heap);
    return ROVER_OK;
}

void heap_clear(MinHeap *heap) {
    if (!heap) return;

    /* Fast O(size) cleanup of index table instead of wiping entire grid buffer */
    if (heap->node_to_heap_index) {
        for (uint32_t i = 0; i < heap->size; ++i) {
            uint32_t node_id = heap->elements[i].node_id;
            if (node_id < heap->max_nodes) {
                heap->node_to_heap_index[node_id] = HEAP_NOT_PRESENT;
            }
        }
    }

    heap->size = 0;
}

RoverError heap_push(MinHeap *heap, uint32_t node_id, float priority) {
    if (!heap) return ROVER_ERR_INVALID_PARAM;

    if (heap->size >= heap->capacity) {
        return ROVER_ERR_HEAP_FULL;
    }

    /* Defensive bounds check: if an index table is present, prevent any out-of-bounds node write */
    if (heap->node_to_heap_index && node_id >= heap->max_nodes) {
        return ROVER_ERR_OUT_OF_BOUNDS;
    }

    uint32_t index = heap->size;
    heap->elements[index].node_id = node_id;
    heap->elements[index].priority = priority;

    if (heap->node_to_heap_index) {
        heap->node_to_heap_index[node_id] = (int32_t)index;
    }

    heap->size++;
    heap->stats.push_count++;
    if (heap->size > heap->stats.max_size_reached) {
        heap->stats.max_size_reached = heap->size;
    }

    sift_up(heap, index);
    return ROVER_OK;
}

RoverError heap_pop_min(MinHeap *heap, uint32_t *out_node_id, float *out_priority) {
    if (!heap || heap->size == 0) {
        return ROVER_ERR_HEAP_EMPTY;
    }

    if (out_node_id) *out_node_id = heap->elements[0].node_id;
    if (out_priority) *out_priority = heap->elements[0].priority;

    uint32_t root_node = heap->elements[0].node_id;
    if (heap->node_to_heap_index && root_node < heap->max_nodes) {
        heap->node_to_heap_index[root_node] = HEAP_NOT_PRESENT;
    }

    heap->stats.pop_count++;
    heap->size--;

    if (heap->size > 0) {
        heap->elements[0] = heap->elements[heap->size];
        if (heap->node_to_heap_index && heap->elements[0].node_id < heap->max_nodes) {
            heap->node_to_heap_index[heap->elements[0].node_id] = 0;
        }
        sift_down(heap, 0);
    }

    return ROVER_OK;
}

RoverError heap_peek_min(const MinHeap *heap, uint32_t *out_node_id, float *out_priority) {
    if (!heap || heap->size == 0) {
        return ROVER_ERR_HEAP_EMPTY;
    }
    if (out_node_id) *out_node_id = heap->elements[0].node_id;
    if (out_priority) *out_priority = heap->elements[0].priority;
    return ROVER_OK;
}

RoverError heap_decrease_key(MinHeap *heap, uint32_t node_id, float new_priority) {
    if (!heap || !heap->use_decrease_key || !heap->node_to_heap_index) {
        return ROVER_ERR_INVALID_PARAM;
    }

    if (node_id >= heap->max_nodes) {
        return ROVER_ERR_OUT_OF_BOUNDS;
    }

    int32_t index = heap->node_to_heap_index[node_id];
    if (index == HEAP_NOT_PRESENT || index >= (int32_t)heap->size) {
        return ROVER_ERR_INVALID_PARAM;
    }

    /* Heap property: new priority must be <= current priority */
    if (new_priority > heap->elements[index].priority) {
        return ROVER_ERR_INVALID_PARAM;
    }

    heap->elements[index].priority = new_priority;
    heap->stats.decrease_key_count++;
    sift_up(heap, (uint32_t)index);

    return ROVER_OK;
}

RoverError heap_push_or_decrease_key(MinHeap *heap, uint32_t node_id, float priority) {
    if (!heap) return ROVER_ERR_INVALID_PARAM;

    if (heap->use_decrease_key && heap->node_to_heap_index) {
        if (node_id >= heap->max_nodes) {
            return ROVER_ERR_OUT_OF_BOUNDS;
        }

        int32_t index = heap->node_to_heap_index[node_id];
        if (index != HEAP_NOT_PRESENT) {
            if (priority < heap->elements[index].priority) {
                return heap_decrease_key(heap, node_id, priority);
            }
            return ROVER_OK; /* Existing priority is already as good or better */
        }
    }

    return heap_push(heap, node_id, priority);
}

bool heap_contains(const MinHeap *heap, uint32_t node_id) {
    if (!heap || !heap->node_to_heap_index || node_id >= heap->max_nodes) {
        return false;
    }
    return heap->node_to_heap_index[node_id] != HEAP_NOT_PRESENT;
}

void heap_reset_stats(MinHeap *heap) {
    if (!heap) return;
    heap->stats.push_count = 0;
    heap->stats.pop_count = 0;
    heap->stats.decrease_key_count = 0;
    heap->stats.max_size_reached = heap->size;
}

bool heap_validate_invariant(const MinHeap *heap) {
    if (!heap) return false;

    for (uint32_t i = 1; i < heap->size; ++i) {
        uint32_t parent = (i - 1) / 2;
        if (heap->elements[i].priority < heap->elements[parent].priority) {
            return false; /* Min-heap invariant violated */
        }
    }

    if (heap->node_to_heap_index) {
        for (uint32_t i = 0; i < heap->size; ++i) {
            uint32_t node_id = heap->elements[i].node_id;
            if (node_id >= heap->max_nodes) return false;
            if (heap->node_to_heap_index[node_id] != (int32_t)i) {
                return false; /* Index map discrepancy */
            }
        }
    }

    return true;
}

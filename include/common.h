/**
 * @file common.h
 * @brief Common definitions, fixed types, and status codes for the Rover Pathfinding Engine.
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum supported map dimensions (256x256 = 65,536 cells) */
#define MAX_GRID_WIDTH   256
#define MAX_GRID_HEIGHT  256
#define MAX_GRID_CELLS   (MAX_GRID_WIDTH * MAX_GRID_HEIGHT)

/* Default safety margin over rover length */
#define ROVER_LENGTH_METERS   2.0f
#define ROVER_SAFETY_MARGIN   1.25f
#define DEFAULT_CELL_SIZE     (ROVER_LENGTH_METERS * ROVER_SAFETY_MARGIN) /* 2.5m per cell */

/* Physical constraints and costs */
#define DEFAULT_MAX_SLOPE_DEG 30.0f
#define DOWN_HILL_COST_FLOOR  0.25f /* f_min: minimum possible edge cost multiplier */
#define ASCENT_PENALTY_K      2.5f  /* k: steep uphill penalty multiplier */

/* Terrain classification flags */
typedef enum {
    TERRAIN_FLAT  = 0,
    TERRAIN_SAND  = 1,
    TERRAIN_ROUGH = 2,
    TERRAIN_ROCK  = 3,
    TERRAIN_CLIFF = 4  /* Impassable / Blocked */
} TerrainType;

/* Cell passability & search status bits (packed for cache locality & WASM export) */
#define CELL_FLAG_BLOCKED    (1 << 0) /* 1 = impassable (cliff or obstacle) */
#define CELL_FLAG_OPEN       (1 << 1) /* In priority queue open set */
#define CELL_FLAG_CLOSED     (1 << 2) /* Evaluated in closed set */
#define CELL_FLAG_PATH       (1 << 3) /* Part of final reconstructed path */
#define CELL_FLAG_START      (1 << 4) /* Start node */
#define CELL_FLAG_GOAL       (1 << 5) /* Goal node */

/* Search status for the step-based engine API */
typedef enum {
    SEARCH_IDLE         = 0,
    SEARCH_RUNNING      = 1,
    SEARCH_FOUND        = 2,
    SEARCH_UNREACHABLE  = 3,
    SEARCH_ERROR        = 4
} SearchStatus;

/* Search algorithm selection */
typedef enum {
    ALGORITHM_ASTAR   = 0,
    ALGORITHM_DIJKSTRA = 1,
    ALGORITHM_GREEDY  = 2
} SearchAlgorithm;

/* Return error codes */
typedef enum {
    ROVER_OK                = 0,
    ROVER_ERR_INVALID_PARAM = -1,
    ROVER_ERR_OUT_OF_BOUNDS = -2,
    ROVER_ERR_BLOCKED_CELL  = -3,
    ROVER_ERR_NO_PATH       = -4,
    ROVER_ERR_HEAP_FULL     = -5,
    ROVER_ERR_HEAP_EMPTY    = -6,
    ROVER_ERR_ALLOCATION    = -7
} RoverError;

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H */

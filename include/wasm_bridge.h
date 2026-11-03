/**
 * @file wasm_bridge.h
 * @brief Public WebAssembly exported C API.
 * Provides zero-copy memory pointers and stepping controls directly to JavaScript.
 */

#ifndef WASM_BRIDGE_H
#define WASM_BRIDGE_H

#include "common.h"
#include "terrain.h"
#include "cost_model.h"
#include "search.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the global engine instance with map dimensions.
 */
EMSCRIPTEN_KEEPALIVE
int wasm_engine_init(uint32_t width, uint32_t height);

/**
 * @brief Generate procedural terrain using multi-octave fBm and 36-pair slope analysis.
 */
EMSCRIPTEN_KEEPALIVE
int wasm_terrain_generate(uint32_t seed,
                         float cell_size,
                         float height_scale,
                         float frequency,
                         int octaves,
                         float persistence,
                         float lacunarity,
                         float max_slope_deg);

/**
 * @brief Start a new pathfinding search.
 */
EMSCRIPTEN_KEEPALIVE
int wasm_search_start(uint32_t start_x, uint32_t start_y,
                      uint32_t goal_x, uint32_t goal_y,
                      int algorithm,
                      float alpha,
                      float beta,
                      float ascent_k,
                      float downhill_fmin);

/**
 * @brief Step the search by up to max_expansions nodes.
 * @return SearchStatus: 1=RUNNING, 2=FOUND, 3=UNREACHABLE, 4=ERROR
 */
EMSCRIPTEN_KEEPALIVE
int wasm_search_step(uint32_t max_expansions);

/**
 * @brief Solve search to completion.
 */
EMSCRIPTEN_KEEPALIVE
int wasm_search_solve(void);

/* --- ZERO-COPY SHARED MEMORY POINTER EXPORTS --- */

EMSCRIPTEN_KEEPALIVE
const float *wasm_get_heightmap_ptr(void);

EMSCRIPTEN_KEEPALIVE
const uint8_t *wasm_get_passable_mask_ptr(void);

EMSCRIPTEN_KEEPALIVE
const CellData *wasm_get_cells_ptr(void);

EMSCRIPTEN_KEEPALIVE
const uint32_t *wasm_get_path_ptr(void);

EMSCRIPTEN_KEEPALIVE
uint32_t wasm_get_path_length(void);

EMSCRIPTEN_KEEPALIVE
const SearchStats *wasm_get_stats_ptr(void);

EMSCRIPTEN_KEEPALIVE
uint32_t wasm_get_grid_width(void);

EMSCRIPTEN_KEEPALIVE
uint32_t wasm_get_grid_height(void);

EMSCRIPTEN_KEEPALIVE
float wasm_get_grid_cell_size(void);

EMSCRIPTEN_KEEPALIVE
uint32_t wasm_get_cell_flags(uint32_t x, uint32_t y);

EMSCRIPTEN_KEEPALIVE
float wasm_get_cell_height(uint32_t x, uint32_t y);

EMSCRIPTEN_KEEPALIVE
float wasm_get_cell_slope(uint32_t x, uint32_t y);

EMSCRIPTEN_KEEPALIVE
float wasm_get_cell_roughness(uint32_t x, uint32_t y);

EMSCRIPTEN_KEEPALIVE
uint32_t wasm_get_cell_terrain_type(uint32_t x, uint32_t y);

/* --- 3D SURFACE & ROVER ORIENTATION HELPERS (COMPUTED IN C) --- */

/**
 * @brief Bilinearly interpolated elevation at continuous world coordinates (meters).
 */
EMSCRIPTEN_KEEPALIVE
float wasm_get_interpolated_height(float world_x, float world_z);

/**
 * @brief Compute the 3D surface normal vector [nx, ny, nz] at continuous coordinates (meters).
 * Used for conforming rover pitch and roll to terrain slopes.
 */
EMSCRIPTEN_KEEPALIVE
void wasm_get_terrain_normal(float world_x, float world_z, float *out_nx, float *out_ny, float *out_nz);

/**
 * @brief Pointer to static 3-float vector [nx, ny, nz] representing the terrain surface normal.
 */
EMSCRIPTEN_KEEPALIVE
const float *wasm_get_terrain_normal_ptr(float world_x, float world_z);

/**
 * @brief Calculate local ground slope angle in degrees at continuous coordinates.
 */
EMSCRIPTEN_KEEPALIVE
float wasm_get_terrain_slope_at(float world_x, float world_z);

/**
 * @brief Get exact 3D coordinates (X, Y elevation, Z) of a waypoint on the computed path.
 */
EMSCRIPTEN_KEEPALIVE
int wasm_get_path_waypoint(uint32_t index, float *out_x, float *out_y, float *out_z);

/**
 * @brief Pointer to static 3-float vector [x, y, z] representing a path waypoint.
 */
EMSCRIPTEN_KEEPALIVE
const float *wasm_get_path_waypoint_ptr(uint32_t index);

/**
 * @brief Run C greedy line-of-sight path smoothing over current solved path.
 * @return Number of waypoints in smoothed path
 */
EMSCRIPTEN_KEEPALIVE
uint32_t wasm_smooth_path(void);

/**
 * @brief Pointer to array of smoothed path 1D node indices.
 */
EMSCRIPTEN_KEEPALIVE
const uint32_t *wasm_get_smoothed_path_ptr(void);

/**
 * @brief Number of waypoints in smoothed path.
 */
EMSCRIPTEN_KEEPALIVE
uint32_t wasm_get_smoothed_path_length(void);

/**
 * @brief Pointer to static 3-float vector [x, y, z] for smoothed waypoint.
 */
EMSCRIPTEN_KEEPALIVE
const float *wasm_get_smoothed_waypoint_ptr(uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* WASM_BRIDGE_H */

/**
 * @file wasm_bridge.c
 * @brief Implementation of WebAssembly export API and zero-copy shared memory interface.
 */

#include "wasm_bridge.h"
#include "core_math.h"
#include <math.h>

/* Fixed static engine memory buffers (up to 256x256 = 65,536 cells) */
static TerrainGrid g_grid;
static SearchContext g_search_ctx;
static bool g_initialized = false;

static float s_heightmap[MAX_GRID_CELLS];
static CellData s_cells[MAX_GRID_CELLS];
static uint8_t s_passable[MAX_GRID_CELLS];

static float s_g_score[MAX_GRID_CELLS];
static float s_f_score[MAX_GRID_CELLS];
static int32_t s_came_from[MAX_GRID_CELLS];
static uint8_t s_node_state[MAX_GRID_CELLS];

static HeapElement s_heap_elements[MAX_GRID_CELLS];
static int32_t s_heap_indices[MAX_GRID_CELLS];
static uint32_t s_path_nodes[MAX_GRID_CELLS];
static uint32_t s_smoothed_path_nodes[MAX_GRID_CELLS];
static uint32_t s_smoothed_path_len = 0;

static uint32_t s_bfs_queue[MAX_GRID_CELLS];
static uint8_t s_bfs_visited[MAX_GRID_CELLS];

int wasm_engine_init(uint32_t width, uint32_t height) {
    if (width == 0 || width > MAX_GRID_WIDTH || height == 0 || height > MAX_GRID_HEIGHT) {
        return ROVER_ERR_INVALID_PARAM;
    }

    RoverError err = terrain_init(&g_grid, width, height, s_heightmap, s_cells, s_passable);
    if (err != ROVER_OK) return (int)err;

    err = search_init(&g_search_ctx, &g_grid,
                      s_g_score, s_f_score, s_came_from, s_node_state,
                      s_heap_elements, s_heap_indices,
                      s_path_nodes, MAX_GRID_CELLS,
                      s_bfs_queue, s_bfs_visited);
    if (err != ROVER_OK) return (int)err;

    g_initialized = true;
    return ROVER_OK;
}

int wasm_terrain_generate(uint32_t seed,
                         float cell_size,
                         float height_scale,
                         float frequency,
                         int octaves,
                         float persistence,
                         float lacunarity,
                         float max_slope_deg) {
    if (!g_initialized) {
        wasm_engine_init(64, 64);
    }

    TerrainParams p;
    p.seed = seed;
    p.width = g_grid.width;
    p.height = g_grid.height;
    p.cell_size = (cell_size > 0.1f) ? cell_size : DEFAULT_CELL_SIZE;
    p.height_scale = (height_scale > 0.0f) ? height_scale : 18.0f;
    p.frequency = (frequency > 0.0f) ? frequency : 0.035f;
    p.octaves = (octaves > 0) ? octaves : 5;
    p.persistence = (persistence > 0.0f) ? persistence : 0.5f;
    p.lacunarity = (lacunarity > 0.0f) ? lacunarity : 2.0f;
    p.max_slope_deg = (max_slope_deg > 0.0f) ? max_slope_deg : DEFAULT_MAX_SLOPE_DEG;

    return (int)terrain_generate(&g_grid, &p);
}

int wasm_search_start(uint32_t start_x, uint32_t start_y,
                      uint32_t goal_x, uint32_t goal_y,
                      int algorithm,
                      float alpha,
                      float beta,
                      float ascent_k,
                      float downhill_fmin) {
    if (!g_initialized) return ROVER_ERR_INVALID_PARAM;
    if (start_x >= g_grid.width || start_y >= g_grid.height ||
        goal_x >= g_grid.width || goal_y >= g_grid.height) {
        return ROVER_ERR_OUT_OF_BOUNDS;
    }

    uint32_t start_node = start_y * g_grid.width + start_x;
    uint32_t goal_node = goal_y * g_grid.width + goal_x;

    CostParams cp = cost_default_params();
    if (alpha >= 0.0f) cp.alpha = alpha;
    if (beta >= 0.0f) cp.beta = beta;
    if (ascent_k >= 0.0f) cp.ascent_penalty_k = ascent_k;
    if (downhill_fmin > 0.0f) cp.downhill_fmin = downhill_fmin;

    SearchAlgorithm algo = (SearchAlgorithm)algorithm;
    if (algo < ALGORITHM_ASTAR || algo > ALGORITHM_GREEDY) {
        algo = ALGORITHM_ASTAR;
    }

    s_smoothed_path_len = 0;
    return (int)search_start(&g_search_ctx, start_node, goal_node, algo, &cp);
}

int wasm_search_step(uint32_t max_expansions) {
    if (!g_initialized) return SEARCH_ERROR;
    int st = (int)search_step(&g_search_ctx, max_expansions);
    if (st == SEARCH_FOUND) {
        wasm_smooth_path();
    }
    return st;
}

int wasm_search_solve(void) {
    if (!g_initialized) return SEARCH_ERROR;
    int st = (int)search_solve(&g_search_ctx);
    if (st == SEARCH_FOUND) {
        wasm_smooth_path();
    }
    return st;
}

/* Zero-copy shared memory pointer accessors */

const float *wasm_get_heightmap_ptr(void) {
    return s_heightmap;
}

const uint8_t *wasm_get_passable_mask_ptr(void) {
    return s_passable;
}

const CellData *wasm_get_cells_ptr(void) {
    return s_cells;
}

const uint32_t *wasm_get_path_ptr(void) {
    return s_path_nodes;
}

uint32_t wasm_get_path_length(void) {
    return g_search_ctx.stats.path_length_nodes;
}

const SearchStats *wasm_get_stats_ptr(void) {
    return &g_search_ctx.stats;
}

uint32_t wasm_get_grid_width(void) {
    return g_grid.width;
}

uint32_t wasm_get_grid_height(void) {
    return g_grid.height;
}

float wasm_get_grid_cell_size(void) {
    return g_grid.cell_size;
}

uint32_t wasm_get_cell_flags(uint32_t x, uint32_t y) {
    if (x >= g_grid.width || y >= g_grid.height) return 0;
    return (uint32_t)s_cells[y * g_grid.width + x].flags;
}

float wasm_get_cell_height(uint32_t x, uint32_t y) {
    if (x >= g_grid.width || y >= g_grid.height) return 0.0f;
    return s_cells[y * g_grid.width + x].height;
}

float wasm_get_cell_slope(uint32_t x, uint32_t y) {
    if (x >= g_grid.width || y >= g_grid.height) return 0.0f;
    return s_cells[y * g_grid.width + x].slope_deg;
}

float wasm_get_cell_roughness(uint32_t x, uint32_t y) {
    if (x >= g_grid.width || y >= g_grid.height) return 0.0f;
    return s_cells[y * g_grid.width + x].roughness;
}

uint32_t wasm_get_cell_terrain_type(uint32_t x, uint32_t y) {
    if (x >= g_grid.width || y >= g_grid.height) return 0;
    return (uint32_t)s_cells[y * g_grid.width + x].terrain_type;
}

float wasm_get_interpolated_height(float world_x, float world_z) {
    if (!g_initialized || g_grid.cell_size <= 1e-4f) return 0.0f;

    float cs = g_grid.cell_size;
    float gx = world_x / cs;
    float gz = world_z / cs;

    int32_t x0 = (int32_t)floorf(gx);
    int32_t z0 = (int32_t)floorf(gz);

    if (x0 < 0) x0 = 0;
    if (z0 < 0) z0 = 0;
    if (x0 >= (int32_t)g_grid.width - 1) x0 = (int32_t)g_grid.width - 1;
    if (z0 >= (int32_t)g_grid.height - 1) z0 = (int32_t)g_grid.height - 1;

    int32_t x1 = (x0 + 1 < (int32_t)g_grid.width) ? x0 + 1 : x0;
    int32_t z1 = (z0 + 1 < (int32_t)g_grid.height) ? z0 + 1 : z0;

    float tx = gx - (float)x0;
    float tz = gz - (float)z0;
    tx = math_clampf(tx, 0.0f, 1.0f);
    tz = math_clampf(tz, 0.0f, 1.0f);

    uint32_t w = g_grid.width;
    float h00 = s_cells[z0 * w + x0].height;
    float h10 = s_cells[z0 * w + x1].height;
    float h01 = s_cells[z1 * w + x0].height;
    float h11 = s_cells[z1 * w + x1].height;

    float h0 = math_lerpf(h00, h10, tx);
    float h1 = math_lerpf(h01, h11, tx);
    return math_lerpf(h0, h1, tz);
}

void wasm_get_terrain_normal(float world_x, float world_z, float *out_nx, float *out_ny, float *out_nz) {
    if (!out_nx || !out_ny || !out_nz) return;
    if (!g_initialized || g_grid.cell_size <= 1e-4f) {
        *out_nx = 0.0f; *out_ny = 1.0f; *out_nz = 0.0f;
        return;
    }

    float delta = 0.5f * g_grid.cell_size;
    float h_xp = wasm_get_interpolated_height(world_x + delta, world_z);
    float h_xm = wasm_get_interpolated_height(world_x - delta, world_z);
    float h_zp = wasm_get_interpolated_height(world_x, world_z + delta);
    float h_zm = wasm_get_interpolated_height(world_x, world_z - delta);

    float dh_dx = (h_xp - h_xm) / (2.0f * delta);
    float dh_dz = (h_zp - h_zm) / (2.0f * delta);

    /* Normal vector N = (-dh_dx, 1.0, -dh_dz) */
    float nx = -dh_dx;
    float ny = 1.0f;
    float nz = -dh_dz;

    float len = sqrtf(nx * nx + ny * ny + nz * nz);
    if (len > 1e-6f) {
        *out_nx = nx / len;
        *out_ny = ny / len;
        *out_nz = nz / len;
    } else {
        *out_nx = 0.0f;
        *out_ny = 1.0f;
        *out_nz = 0.0f;
    }
}

int wasm_get_path_waypoint(uint32_t index, float *out_x, float *out_y, float *out_z) {
    if (!out_x || !out_y || !out_z) return ROVER_ERR_INVALID_PARAM;
    if (index >= g_search_ctx.stats.path_length_nodes) {
        return ROVER_ERR_OUT_OF_BOUNDS;
    }

    uint32_t node = s_path_nodes[index];
    uint32_t w = g_grid.width;
    uint32_t x = node % w;
    uint32_t y = node / w;

    *out_x = ((float)x + 0.5f) * g_grid.cell_size;
    *out_z = ((float)y + 0.5f) * g_grid.cell_size;
    *out_y = s_cells[node].height;

    return ROVER_OK;
}

static float s_vec3_scratch[3] = {0.0f, 0.0f, 0.0f};

const float *wasm_get_terrain_normal_ptr(float world_x, float world_z) {
    wasm_get_terrain_normal(world_x, world_z, &s_vec3_scratch[0], &s_vec3_scratch[1], &s_vec3_scratch[2]);
    return s_vec3_scratch;
}

float wasm_get_terrain_slope_at(float world_x, float world_z) {
    float nx, ny, nz;
    wasm_get_terrain_normal(world_x, world_z, &nx, &ny, &nz);
    /* ny = cos(slope), since world up vector is (0, 1, 0) */
    ny = math_clampf(ny, -1.0f, 1.0f);
    return acosf(ny) * RAD_TO_DEG;
}

const float *wasm_get_path_waypoint_ptr(uint32_t index) {
    int ret = wasm_get_path_waypoint(index, &s_vec3_scratch[0], &s_vec3_scratch[1], &s_vec3_scratch[2]);
    if (ret != ROVER_OK) {
        s_vec3_scratch[0] = 0.0f;
        s_vec3_scratch[1] = 0.0f;
        s_vec3_scratch[2] = 0.0f;
    }
    return s_vec3_scratch;
}

uint32_t wasm_smooth_path(void) {
    if (!g_initialized || g_search_ctx.stats.path_length_nodes == 0) {
        s_smoothed_path_len = 0;
        return 0;
    }
    RoverError err = search_smooth_path(&g_grid, s_path_nodes,
                                       g_search_ctx.stats.path_length_nodes,
                                       s_smoothed_path_nodes,
                                       &s_smoothed_path_len);
    if (err != ROVER_OK) {
        s_smoothed_path_len = 0;
    }
    return s_smoothed_path_len;
}

const uint32_t *wasm_get_smoothed_path_ptr(void) {
    return s_smoothed_path_nodes;
}

uint32_t wasm_get_smoothed_path_length(void) {
    return s_smoothed_path_len;
}

const float *wasm_get_smoothed_waypoint_ptr(uint32_t index) {
    if (index >= s_smoothed_path_len) {
        s_vec3_scratch[0] = 0.0f;
        s_vec3_scratch[1] = 0.0f;
        s_vec3_scratch[2] = 0.0f;
        return s_vec3_scratch;
    }
    uint32_t node = s_smoothed_path_nodes[index];
    uint32_t w = g_grid.width;
    uint32_t x = node % w;
    uint32_t y = node / w;

    s_vec3_scratch[0] = ((float)x + 0.5f) * g_grid.cell_size;
    s_vec3_scratch[2] = ((float)y + 0.5f) * g_grid.cell_size;
    s_vec3_scratch[1] = s_cells[node].height;

    return s_vec3_scratch;
}

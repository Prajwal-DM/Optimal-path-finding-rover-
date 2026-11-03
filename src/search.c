/**
 * @file search.c
 * @brief Search Engine Core implementing stepped A*, Dijkstra, Greedy Best-First,
 * and early-exit BFS connectivity pre-checking.
 */

#include "search.h"
#include "core_math.h"
#include <string.h>
#include <time.h>

/* 8-connected directional neighbor offsets */
static const int NEIGHBOR_DX[8] = {  1, -1,  0,  0,  1, -1,  1, -1 };
static const int NEIGHBOR_DY[8] = {  0,  0,  1, -1,  1,  1, -1, -1 };

RoverError search_init(SearchContext *ctx,
                       const TerrainGrid *grid,
                       float *g_score_buf,
                       float *f_score_buf,
                       int32_t *came_from_buf,
                       uint8_t *node_state_buf,
                       HeapElement *heap_elements_buf,
                       int32_t *heap_indices_buf,
                       uint32_t *path_buf,
                       uint32_t path_capacity,
                       uint32_t *bfs_queue_buf,
                       uint8_t *bfs_visited_buf) {
    if (!ctx || !grid || !g_score_buf || !f_score_buf || !came_from_buf ||
        !node_state_buf || !heap_elements_buf || !heap_indices_buf ||
        !path_buf || path_capacity == 0 || !bfs_queue_buf || !bfs_visited_buf) {
        return ROVER_ERR_INVALID_PARAM;
    }

    ctx->grid = grid;
    ctx->cost_params = cost_default_params();
    ctx->algorithm = ALGORITHM_ASTAR;
    ctx->status = SEARCH_IDLE;
    ctx->start_node = 0;
    ctx->goal_node = 0;

    ctx->g_score = g_score_buf;
    ctx->f_score = f_score_buf;
    ctx->came_from = came_from_buf;
    ctx->node_state = node_state_buf;

    ctx->path_nodes = path_buf;
    ctx->path_capacity = path_capacity;

    ctx->bfs_queue = bfs_queue_buf;
    ctx->bfs_visited = bfs_visited_buf;

    RoverError err = heap_init(&ctx->open_heap,
                               heap_elements_buf,
                               path_capacity,
                               heap_indices_buf,
                               grid->total_cells,
                               true);
    if (err != ROVER_OK) return err;

    memset(&ctx->stats, 0, sizeof(SearchStats));
    return ROVER_OK;
}

bool search_check_connectivity(SearchContext *ctx, uint32_t start, uint32_t goal) {
    if (!ctx || !ctx->grid) return false;
    const TerrainGrid *grid = ctx->grid;

    if (start >= grid->total_cells || goal >= grid->total_cells) return false;
    if (grid->passable_mask[start] == 0 || grid->passable_mask[goal] == 0) return false;
    if (start == goal) return true;

    /* Wipe visited buffer for active grid cells */
    memset(ctx->bfs_visited, 0, grid->total_cells * sizeof(uint8_t));

    uint32_t head = 0;
    uint32_t tail = 0;

    ctx->bfs_queue[tail++] = start;
    ctx->bfs_visited[start] = 1;

    while (head < tail) {
        uint32_t u = ctx->bfs_queue[head++];
        if (u == goal) {
            return true;
        }

        uint32_t ux, uy;
        terrain_index_to_coord(grid, u, &ux, &uy);

        for (int k = 0; k < 8; ++k) {
            int32_t vx = (int32_t)ux + NEIGHBOR_DX[k];
            int32_t vy = (int32_t)uy + NEIGHBOR_DY[k];

            if (vx < 0 || vx >= (int32_t)grid->width || vy < 0 || vy >= (int32_t)grid->height) {
                continue;
            }

            if (cost_is_move_legal(grid, ux, uy, (uint32_t)vx, (uint32_t)vy)) {
                uint32_t v = (uint32_t)vy * grid->width + (uint32_t)vx;
                if (!ctx->bfs_visited[v]) {
                    ctx->bfs_visited[v] = 1;
                    ctx->bfs_queue[tail++] = v;
                }
            }
        }
    }

    return false; /* Disjoint components, completely unreachable */
}

static void reconstruct_path(SearchContext *ctx) {
    const TerrainGrid *grid = ctx->grid;
    uint32_t current = ctx->goal_node;
    uint32_t count = 0;

    /* Count path nodes backwards */
    while (current != ctx->start_node && (int32_t)current != -1 && count < ctx->path_capacity) {
        count++;
        current = (uint32_t)ctx->came_from[current];
    }
    count++; /* Include start node */

    if (count > ctx->path_capacity) {
        count = ctx->path_capacity;
    }

    ctx->stats.path_length_nodes = count;

    /* Fill backwards so path_nodes[0] = start_node */
    current = ctx->goal_node;
    for (int32_t i = (int32_t)count - 1; i >= 0; --i) {
        ctx->path_nodes[i] = current;
        grid->cells[current].flags |= CELL_FLAG_PATH;
        if (current == ctx->start_node) break;
        if (ctx->came_from[current] < 0 || (uint32_t)ctx->came_from[current] >= grid->total_cells) break;
        current = (uint32_t)ctx->came_from[current];
    }

    /* Accumulate path cost, fuel, risk, and ground distance */
    float total_cost = 0.0f;
    float total_fuel = 0.0f;
    float total_risk = 0.0f;
    float total_distance = 0.0f;

    for (uint32_t i = 0; i + 1 < count; ++i) {
        uint32_t u = ctx->path_nodes[i];
        uint32_t v = ctx->path_nodes[i + 1];

        uint32_t ux, uy, vx, vy;
        terrain_index_to_coord(grid, u, &ux, &uy);
        terrain_index_to_coord(grid, v, &vx, &vy);

        float fuel = 0.0f;
        float risk = 0.0f;
        float cost = cost_compute_edge(grid, ux, uy, vx, vy, &ctx->cost_params, &fuel, &risk);

        int32_t dx = math_absi((int32_t)vx - (int32_t)ux);
        int32_t dy = math_absi((int32_t)vy - (int32_t)uy);
        float step_dist = ((dx + dy) == 1) ? grid->cell_size : (MATH_SQRT2 * grid->cell_size);

        total_cost += cost;
        total_fuel += fuel;
        total_risk += risk;
        total_distance += step_dist;
    }

    ctx->stats.path_cost_total = total_cost;
    ctx->stats.path_cost_fuel = total_fuel;
    ctx->stats.path_cost_risk = total_risk;
    ctx->stats.path_distance_meters = total_distance;
}

RoverError search_start(SearchContext *ctx,
                        uint32_t start_node,
                        uint32_t goal_node,
                        SearchAlgorithm algorithm,
                        const CostParams *cost_params) {
    if (!ctx || !ctx->grid) return ROVER_ERR_INVALID_PARAM;
    const TerrainGrid *grid = ctx->grid;

    if (start_node >= grid->total_cells || goal_node >= grid->total_cells) {
        return ROVER_ERR_OUT_OF_BOUNDS;
    }

    if (grid->passable_mask[start_node] == 0 || grid->passable_mask[goal_node] == 0) {
        ctx->status = SEARCH_ERROR;
        return ROVER_ERR_BLOCKED_CELL;
    }

    ctx->algorithm = algorithm;
    ctx->cost_params = cost_params ? *cost_params : cost_default_params();
    ctx->start_node = start_node;
    ctx->goal_node = goal_node;

    /* Always reset telemetry stats at the start so unreachable/error returns do not leak old metrics */
    memset(&ctx->stats, 0, sizeof(SearchStats));

    /* Handle trivial start == goal */
    if (start_node == goal_node) {
        ctx->path_nodes[0] = start_node;
        ctx->stats.path_length_nodes = 1;
        ctx->status = SEARCH_FOUND;
        grid->cells[start_node].flags |= (CELL_FLAG_START | CELL_FLAG_GOAL | CELL_FLAG_PATH);
        return ROVER_OK;
    }

    /* Fast BFS connectivity pre-check */
    if (!search_check_connectivity(ctx, start_node, goal_node)) {
        ctx->status = SEARCH_UNREACHABLE;
        return ROVER_ERR_NO_PATH;
    }

    /* Reset search arrays */
    heap_clear(&ctx->open_heap);
    for (uint32_t i = 0; i < grid->total_cells; ++i) {
        ctx->g_score[i] = COST_INFINITE;
        ctx->f_score[i] = COST_INFINITE;
        ctx->came_from[i] = -1;
        ctx->node_state[i] = 0;

        /* Clear transient search flags from grid */
        grid->cells[i].flags &= ~(CELL_FLAG_OPEN | CELL_FLAG_CLOSED | CELL_FLAG_PATH | CELL_FLAG_START | CELL_FLAG_GOAL);
    }

    grid->cells[start_node].flags |= CELL_FLAG_START;
    grid->cells[goal_node].flags |= CELL_FLAG_GOAL;

    /* Initialize start node */
    ctx->g_score[start_node] = 0.0f;
    float start_h = cost_heuristic(grid, start_node, goal_node, &ctx->cost_params);

    float start_f;
    if (algorithm == ALGORITHM_ASTAR) {
        start_f = start_h;
    } else if (algorithm == ALGORITHM_DIJKSTRA) {
        start_f = 0.0f;
    } else { /* ALGORITHM_GREEDY */
        start_f = start_h;
    }

    ctx->f_score[start_node] = start_f;
    ctx->node_state[start_node] = CELL_FLAG_OPEN;
    grid->cells[start_node].flags |= CELL_FLAG_OPEN;

    heap_push(&ctx->open_heap, start_node, start_f);

    /* Reset telemetry */
    ctx->stats.nodes_expanded = 0;
    ctx->stats.max_heap_size = 1;
    ctx->stats.path_length_nodes = 0;
    ctx->stats.path_cost_total = 0.0f;
    ctx->stats.path_cost_fuel = 0.0f;
    ctx->stats.path_cost_risk = 0.0f;
    ctx->stats.path_distance_meters = 0.0f;
    ctx->stats.search_time_ms = 0.0;

    ctx->status = SEARCH_RUNNING;
    return ROVER_OK;
}

SearchStatus search_step(SearchContext *ctx, uint32_t max_expansions) {
    if (!ctx || ctx->status != SEARCH_RUNNING) {
        return ctx ? ctx->status : SEARCH_ERROR;
    }

    const TerrainGrid *grid = ctx->grid;
    clock_t start_t = clock();

    for (uint32_t exp = 0; exp < max_expansions; ++exp) {
        if (heap_is_empty(&ctx->open_heap)) {
            ctx->status = SEARCH_UNREACHABLE;
            break;
        }

        uint32_t current;
        float cur_prio;
        heap_pop_min(&ctx->open_heap, &current, &cur_prio);

        ctx->node_state[current] = CELL_FLAG_CLOSED;
        grid->cells[current].flags &= ~CELL_FLAG_OPEN;
        grid->cells[current].flags |= CELL_FLAG_CLOSED;
        ctx->stats.nodes_expanded++;

        /* Goal reached! */
        if (current == ctx->goal_node) {
            ctx->status = SEARCH_FOUND;
            reconstruct_path(ctx);
            break;
        }

        uint32_t ux, uy;
        terrain_index_to_coord(grid, current, &ux, &uy);
        float current_g = ctx->g_score[current];

        for (int k = 0; k < 8; ++k) {
            int32_t vx = (int32_t)ux + NEIGHBOR_DX[k];
            int32_t vy = (int32_t)uy + NEIGHBOR_DY[k];

            if (vx < 0 || vx >= (int32_t)grid->width || vy < 0 || vy >= (int32_t)grid->height) {
                continue;
            }

            uint32_t v = (uint32_t)vy * grid->width + (uint32_t)vx;

            /* Admissible consistent heuristic guarantees closed nodes never need reopening */
            if (ctx->node_state[v] == CELL_FLAG_CLOSED) {
                continue;
            }

            if (!cost_is_move_legal(grid, ux, uy, (uint32_t)vx, (uint32_t)vy)) {
                continue;
            }

            float edge_cost = cost_compute_edge(grid, ux, uy, (uint32_t)vx, (uint32_t)vy,
                                                &ctx->cost_params, NULL, NULL);
            float tentative_g = current_g + edge_cost;

            if (tentative_g < ctx->g_score[v]) {
                ctx->came_from[v] = (int32_t)current;
                ctx->g_score[v] = tentative_g;

                float f;
                if (ctx->algorithm == ALGORITHM_ASTAR) {
                    float h = cost_heuristic(grid, v, ctx->goal_node, &ctx->cost_params);
                    f = tentative_g + h;
                } else if (ctx->algorithm == ALGORITHM_DIJKSTRA) {
                    f = tentative_g;
                } else { /* ALGORITHM_GREEDY */
                    float h = cost_heuristic(grid, v, ctx->goal_node, &ctx->cost_params);
                    f = h;
                }

                ctx->f_score[v] = f;

                if (ctx->node_state[v] == CELL_FLAG_OPEN) {
                    heap_decrease_key(&ctx->open_heap, v, f);
                } else {
                    RoverError push_err = heap_push(&ctx->open_heap, v, f);
                    if (push_err == ROVER_OK) {
                        ctx->node_state[v] = CELL_FLAG_OPEN;
                        grid->cells[v].flags |= CELL_FLAG_OPEN;
                    }
                }
            }
        }
    }

    ctx->stats.max_heap_size = ctx->open_heap.stats.max_size_reached;

    clock_t end_t = clock();
    ctx->stats.search_time_ms += (double)(end_t - start_t) * 1000.0 / CLOCKS_PER_SEC;

    return ctx->status;
}

SearchStatus search_solve(SearchContext *ctx) {
    if (!ctx) return SEARCH_ERROR;
    return search_step(ctx, UINT32_MAX);
}

const uint32_t *search_get_path(const SearchContext *ctx, uint32_t *out_count) {
    if (!ctx || ctx->status != SEARCH_FOUND) {
        if (out_count) *out_count = 0;
        return NULL;
    }
    if (out_count) *out_count = ctx->stats.path_length_nodes;
    return ctx->path_nodes;
}

const SearchStats *search_get_stats(const SearchContext *ctx) {
    return ctx ? &ctx->stats : NULL;
}

bool search_has_line_of_sight(const TerrainGrid *grid, uint32_t u, uint32_t v) {
    if (!grid || u >= grid->total_cells || v >= grid->total_cells) return false;
    if (u == v) return true;

    /* Check passability of endpoints */
    if (grid->passable_mask[u] == 0 || grid->passable_mask[v] == 0) return false;

    uint32_t ux, uy, vx, vy;
    terrain_index_to_coord(grid, u, &ux, &uy);
    terrain_index_to_coord(grid, v, &vx, &vy);

    /* If immediate neighbor, use exact directional legality check */
    int32_t dx_cells = (int32_t)vx - (int32_t)ux;
    int32_t dy_cells = (int32_t)vy - (int32_t)uy;
    if (math_absi(dx_cells) <= 1 && math_absi(dy_cells) <= 1) {
        return cost_is_move_legal(grid, ux, uy, vx, vy);
    }

    /* Overall slope angle check between endpoints */
    float delta_h = fabsf(grid->cells[v].height - grid->cells[u].height);
    float horiz_dist = sqrtf((float)(dx_cells * dx_cells + dy_cells * dy_cells)) * grid->cell_size;
    if (horiz_dist > 1e-4f) {
        float direct_slope_deg = atanf(delta_h / horiz_dist) * RAD_TO_DEG;
        if (direct_slope_deg > grid->max_slope_deg) {
            return false;
        }
    }

    /* Raymarch sub-cell sample steps (every 0.25 cells) to catch obstacles and micro-cliffs */
    float dist = sqrtf((float)(dx_cells * dx_cells + dy_cells * dy_cells));
    int32_t steps = (int32_t)ceilf(dist * 4.0f);
    if (steps < 2) steps = 2;

    float x0 = (float)ux + 0.5f;
    float y0 = (float)uy + 0.5f;
    float x1 = (float)vx + 0.5f;
    float y1 = (float)vy + 0.5f;

    int32_t prev_cx = (int32_t)ux;
    int32_t prev_cy = (int32_t)uy;

    for (int32_t s = 1; s <= steps; ++s) {
        float t = (float)s / (float)steps;
        float cur_x = math_lerpf(x0, x1, t);
        float cur_y = math_lerpf(y0, y1, t);

        int32_t cx = (int32_t)floorf(cur_x);
        int32_t cy = (int32_t)floorf(cur_y);

        if (cx < 0 || cx >= (int32_t)grid->width || cy < 0 || cy >= (int32_t)grid->height) {
            return false;
        }

        uint32_t cidx = (uint32_t)cy * grid->width + (uint32_t)cx;

        /* Impassable cliff or obstacle */
        if (grid->passable_mask[cidx] == 0) {
            return false;
        }
        if (grid->cells[cidx].slope_deg > grid->max_slope_deg) {
            return false;
        }

        /* Diagonal transition: prevent cutting blocked corners */
        if (cx != prev_cx && cy != prev_cy) {
            uint32_t adj1 = (uint32_t)prev_cy * grid->width + (uint32_t)cx;
            uint32_t adj2 = (uint32_t)cy * grid->width + (uint32_t)prev_cx;
            if (grid->passable_mask[adj1] == 0 || grid->passable_mask[adj2] == 0) {
                return false;
            }
        }

        prev_cx = cx;
        prev_cy = cy;
    }

    return true;
}

RoverError search_smooth_path(const TerrainGrid *grid,
                              const uint32_t *raw_path,
                              uint32_t raw_len,
                              uint32_t *out_smoothed,
                              uint32_t *out_smoothed_len) {
    if (!grid || !raw_path || !out_smoothed || !out_smoothed_len) {
        return ROVER_ERR_INVALID_PARAM;
    }

    if (raw_len == 0) {
        *out_smoothed_len = 0;
        return ROVER_OK;
    }

    if (raw_len <= 2) {
        for (uint32_t i = 0; i < raw_len; ++i) {
            out_smoothed[i] = raw_path[i];
        }
        *out_smoothed_len = raw_len;
        return ROVER_OK;
    }

    out_smoothed[0] = raw_path[0];
    uint32_t smoothed_count = 1;
    uint32_t curr = 0;

    while (curr < raw_len - 1) {
        uint32_t next = curr + 1;
        /* Look ahead as far as possible down the path for line of sight */
        for (uint32_t candidate = raw_len - 1; candidate > curr + 1; --candidate) {
            if (search_has_line_of_sight(grid, raw_path[curr], raw_path[candidate])) {
                next = candidate;
                break;
            }
        }
        out_smoothed[smoothed_count++] = raw_path[next];
        curr = next;
    }

    *out_smoothed_len = smoothed_count;
    return ROVER_OK;
}

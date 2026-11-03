# Terrain-Aware Rover Pathfinding using A\*

**Project type:** Data Structures & Algorithms project **Stack:** C (compiled to WebAssembly) + JavaScript / Three.js **Document:** Problem definition

## 1. Overview

Planetary rovers cannot take the shortest straight line to a target. Slopes, rough ground and cliffs make some routes dangerous, and every metre travelled burns limited fuel. This project builds a simulator in which a rover navigates a procedurally generated 3D terrain from a start position to a goal, using A\* search over a weighted grid. The system also compares A\* against other search algorithms and visualizes the search as it runs.

## 2. Problem Statement

Given a randomly generated 3D terrain, a start location and a goal location, find a path for the rover that **minimizes a combined cost of fuel consumption and traversal risk**, while never entering terrain that is physically unsafe (impassable). If no safe path exists, the system must report that clearly and quickly.

## 3. Objectives

1. Generate reproducible random terrain from a seed.
2. Discretize the terrain into a grid whose cells are slightly larger than the rover.
3. Derive a physically grounded traversal cost for every move between cells.
4. Implement A\* with a self-built priority queue (binary heap) and prove the heuristic is admissible.
5. Compare A\* with Dijkstra and Greedy Best-First Search on identical maps using measurable metrics.
6. Visualize the search process and the final path in a 3D environment.

## 4. Formal Model

### 4.1 Terrain

The terrain is a heightmap `H(x, y)` produced by fractal noise (Perlin or Simplex, multiple octaves) from an integer seed `S`. The same seed must always produce the same terrain on every platform, so a custom PRNG (e.g. xorshift) is used instead of the C library `rand()`.

### 4.2 Grid discretization

The map is divided into an `N × M` grid of square cells. Cell side length:

`cell_size = rover_length × safety_margin` (safety margin ≈ 1.2 to 1.5)

Each cell is sampled at several points (corners, edges, center), not just one, so that ridges or pits inside a cell are not missed.

### 4.3 Cell attributes

For each cell `v`, computed from its height samples:

- `h(v)`: mean height
- `slope(v)`: maximum slope angle inside the cell
- `rough(v)`: height variance inside the cell
- `terrain_type(v)`: e.g. flat, sand, rock, derived from slope and roughness
- `blocked(v)`: true if `slope(v) > θ_max` (default 30°) or the cell is a hard obstacle

### 4.4 Graph

Nodes are the non-blocked cells. Edges connect each cell to its 8 neighbours. A diagonal move is allowed only if both adjacent cardinal cells are not blocked (no corner cutting).

### 4.5 Edge cost

Cost depends on the move from `u` to `v`, not only on the destination:

- `d(u,v)`: 1 for cardinal moves, √2 for diagonal moves (scaled by `cell_size`)
- `Δh = h(v) − h(u)`
- `fuel(u,v) = d · τ(v) · (1 + k · max(0, Δh / d))`, where `τ(v)` is a terrain multiplier (sand costs more than flat rock) and `k` penalizes climbing. Downhill moves are discounted but clamped to a positive floor `f_min`, so no edge cost is ever zero or negative.
- `risk(v) = w₁ · slope(v) / θ_max + w₂ · rough(v)`, normalized to roughly 0 to 1

The single scalar cost used by the search is:

`cost(u,v) = α · fuel(u,v) + β · risk(v) · d(u,v)`

`α` and `β` are user-adjustable. Setting `α` high gives a **fuel-saver** route; setting `β` high gives a **safe** route. All costs are strictly positive.

### 4.6 Objective

Find a path `P = (s = v₀, v₁, …, vₖ = g)` that minimizes

`C(P) = Σ cost(vᵢ, vᵢ₊₁)`

subject to every `vᵢ` being non-blocked and every move being legal under 4.4.

### 4.7 Heuristic

`h(n) = α · f_min · octile_distance(n, goal)`

where `octile_distance(dx, dy) = max(dx, dy) + (√2 − 1) · min(dx, dy)`.

This never overestimates the true remaining cost, because no move can cost less than `α · f_min` per unit distance and the risk term is non-negative. It is therefore admissible and consistent, which guarantees A\* returns an optimal path and never needs to re-open closed nodes.

## 5. Assumptions and Constraints

- The rover is a point-like body occupying one cell; its footprint fits within a cell by construction.
- Terrain is static during a plan (dynamic replanning is an extension, see section 9).
- Fuel is treated as a **cost to minimize** in the core system. A hard fuel budget is an optional extension.
- Start and goal must lie in non-blocked cells; invalid input is rejected with an error.
- Grid size target: up to 256 × 256 (65,536 cells) interactive in the browser.
- If no path exists, the planner returns `UNREACHABLE` without exhausting unnecessary work, for example by first checking that start and goal are in the same connected region.

## 6. Algorithms and Data Structures

| Component | Choice | Reason |
| --- | --- | --- |
| Priority queue | Self-implemented binary min-heap | Core DSA content; compare lazy deletion vs decrease-key |
| Grid storage | Flat arrays indexed by `y * width + x` | Cache-friendly, shareable with JS through WASM memory |
| Per-node data | Flat arrays: `g_score`, `came_from`, `closed` | No per-node allocation |
| Main algorithm | A\* (octile heuristic) | Optimal with admissible heuristic |
| Baselines | Dijkstra (`h = 0`), Greedy Best-First (`f = h`) | Show the value of the heuristic |
| Terrain | Fractal Perlin/Simplex noise + custom PRNG | Reproducible random maps |
| Connectivity check | BFS / flood fill | Fast unreachable detection |

## 7. System Architecture

**C core (compiled to WebAssembly with Emscripten):** terrain generation, cost model, heap, A\*, Dijkstra, Greedy, metrics. All logic lives here.

**JavaScript + Three.js layer:** renders the terrain mesh from the C heightmap, draws the rover and path, visualizes open and closed sets, and provides the UI (seed, `α`, `β`, algorithm selector, start/goal picking). It contains no pathfinding logic.

**Planned C API:**

- `terrain_generate(seed, width, height, params)`
- `astar_init(start, goal, algorithm, params)`
- `astar_step(n)`: expand up to `n` nodes and return a status (`RUNNING`, `FOUND`, `UNREACHABLE`)
- `astar_get_path()`
- `astar_stats()`

The stepping API lets the front end animate the search frame by frame.

## 8. Inputs and Outputs

**Inputs:** seed, grid dimensions, rover size, terrain parameters, `θ_max`, `α`, `β`, start cell, goal cell, algorithm choice.

**Outputs:**

- Path as an ordered list of cells (or `UNREACHABLE`)
- Total cost, split into fuel and risk components
- Metrics: nodes expanded, maximum heap size, run time, path length
- Visualization data: open set, closed set, final path

## 9. Scope

### Core (must deliver)

- Seeded terrain generation and grid discretization
- Cost model with impassable cells
- A\*, Dijkstra and Greedy with a custom binary heap
- Metrics comparison on identical maps
- Search visualization and rover animation along the path in 3D
- Unit tests on small grids with known optimal costs

### Extensions (if time permits, in priority order)

1. Path smoothing via line-of-sight (Theta\*-style) for natural rover motion
2. Hard fuel budget with refuel stations (state becomes `(cell, fuel_left)`)
3. Dynamic obstacles (rockfall) with replanning from the current cell, or D\* Lite

### Out of scope

Rover physics and suspension simulation, multi-rover coordination, real planetary data.

## 10. Evaluation Plan

- **Correctness:** A\* and Dijkstra must return the same path cost on every test map. Greedy may be worse and should be shown to be.
- **Efficiency:** compare nodes expanded and run time across maps of increasing size and obstacle density.
- **Parameter effect:** show that changing `α` and `β` visibly changes the route (fuel-saver vs safe).
- **Robustness:** unreachable goals, start equals goal, blocked start/goal and very rough terrain are handled without crashes.
- **Reproducibility:** the same seed and parameters always give identical terrain and paths.

## 11. Milestones

1. **Toolchain check:** Emscripten "hello world" called from JS (do this first).
2. **C core with CLI tests:** grid, heap, A\*, unit tests.
3. **Terrain and cost model:** verified through ASCII maps.
4. **WASM + 2D canvas:** stepping API and live search visualization.
5. **Algorithm comparison and metrics.**
6. **3D Three.js scene and rover animation.**
7. **Extensions, report and demo preparation.**

## 12. Risks and Mitigations

| Risk | Mitigation |
| --- | --- |
| Emscripten setup problems | Verify on day one; fallback is C as a CLI that outputs JSON for the JS viewer |
| 3D work consuming the schedule | Build it last; the 2D view is a complete fallback |
| Non-admissible heuristic silently breaking optimality | Cross-check A\* against Dijkstra in tests |
| Terrain and path data drifting apart | Terrain generated only in C; JS reads the same heightmap |
| Unreachable maps wasting search time | Connectivity pre-check and guaranteed-solvable seed option |

/*
 * tt_phys.h - the rigid-body engine's public surface.
 *
 * The engine knows nothing about tetrominoes, rows, scoring or the screen. Its outputs are the
 * body pool and the render snapshot. tt_slice.c is the one game module allowed to reach in
 * further, which is why the geometry helpers are exported here deliberately.
 */
#ifndef TT_PHYS_H
#define TT_PHYS_H

#include "tt_types.h"

/* Called once per substep, after velocities are integrated and before the solve. This is where
 * the game applies its velocity servos to the active piece; there is no other way in, and there
 * is deliberately no way to write a position. */
typedef void (*tt_substep_fn)(void* ctx, float h);

/* ---- lifetime ----------------------------------------------------------------------------- */

/*
 * The pools are carved from SEVERAL blocks, not one.
 *
 * A single ~70 KB allocation is the wrong shape for this device: OS_RAM is one 512 KB heap region
 * with well under 100 KB free by the time a tapp loads, and the ELF loader has just fragmented it.
 * Five blocks of at most ~31 KB fit where one of 70 KB does not. No pool ever straddles a block —
 * tt_take() moves to the next block rather than splitting an array.
 *
 * tt_phys_block_plan() fills `sizes` with the blocks to allocate and returns how many (<=
 * TT_MAX_BLOCKS). Allocate each, then hand them to tt_phys_init IN THE SAME ORDER.
 */
#define TT_MAX_BLOCKS 5

uint8_t tt_phys_block_plan(uint16_t body_cap, uint16_t mf_cap, uint32_t* sizes);

/* Total across the plan — for reporting, not for allocating. */
uint32_t tt_phys_bytes(uint16_t body_cap, uint16_t mf_cap);

tt_world_t* tt_phys_init(
    void* const* blocks,
    const uint32_t* sizes,
    uint8_t nblocks,
    uint16_t body_cap,
    uint16_t mf_cap);
void tt_phys_clear(tt_world_t* w);

/* ---- bodies ------------------------------------------------------------------------------- */

int32_t tt_body_alloc(tt_world_t* w);
void tt_body_destroy(tt_world_t* w, uint16_t bi);

/*
 * Finish a body whose poly[].v are filled in WORLD space with q == identity, p == the frame the
 * vertices were built around and vel/w describing that frame. Recomputes area, centroid, inertia
 * and the outward normals, re-anchors the vertices to the centroid and carries the velocity
 * through the change of reference point. Returns false (and leaves the body dead) if what is
 * left is smaller than TT_MIN_BODY_AREA.
 */
bool tt_body_reanchor(tt_world_t* w, uint16_t bi, float density, bool make_static);

void tt_body_update_aabb(tt_world_t* w, uint16_t bi);
void tt_phys_wake_all(tt_world_t* w);
void tt_phys_wake_island(tt_world_t* w, uint16_t bi);

/* ---- stepping ----------------------------------------------------------------------------- */

void tt_phys_step(tt_world_t* w, float dt, tt_substep_fn cb, void* ctx);
void tt_phys_snapshot(tt_world_t* w);
const tt_snap_t* tt_phys_published(const tt_world_t* w);

/* ---- geometry helpers (shared with the slice path) ---------------------------------------- */

float tt_poly_area(const vec2* v, int n); /* signed */
void tt_poly_moments(const vec2* v, int n, float* area, vec2* centroid, float* i_origin);

/* Sutherland-Hodgman against one horizontal half-plane.
 *   keep_above == true  -> keep y <= c   (the part above the cut, +y being down)
 *   keep_above == false -> keep y >= c
 * `out` must hold TT_CLIP_SCRATCH entries. Returns the deduped vertex count, or 0. */
int tt_clip_half(const vec2* in, int n, float c, bool keep_above, vec2* out);
int tt_dedup(vec2* v, int n);

/* World-space SAT overlap test with a positive tolerance, used by the connectivity pass after a
 * cut: abutting fragments gap exactly 0 and register as touching. */
bool tt_sat_overlap(const vec2* va, int na, const vec2* vb, int nb, float eps);

/* ---- internal, shared between the engine translation units -------------------------------- */

void tt_collide_all(tt_world_t* w);
const tt_pc_t* tt_pc_lookup(const tt_world_t* w, uint32_t key);
void tt_pc_rebuild(tt_world_t* w);
void tt_solver_prepare(tt_world_t* w, float h);
void tt_solver_warm_start(tt_world_t* w);
void tt_solver_solve(tt_world_t* w, float inv_h, bool use_bias);
void tt_solver_restitution(tt_world_t* w);
void tt_solver_store(tt_world_t* w);

#endif /* TT_PHYS_H */

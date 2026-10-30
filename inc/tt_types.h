/*
 * tt_types.h - shared types for tapetris.
 *
 * Conventions, fixed once so SAT/clipping/rendering cannot disagree:
 *
 *   - World units are screen pixels, +y DOWN.
 *   - Polygon winding is "positive shoelace": sum(x_i*y_{i+1} - x_{i+1}*y_i) > 0, which in a
 *     y-down frame looks clockwise on screen.
 *   - With that winding the outward normal of edge (v[i] -> v[i+1]) is normalize(e.y, -e.x).
 *   - Rotation is a ROTOR {s, c} = {sin, cos}, never an angle. The engine therefore calls no
 *     transcendental function at all - which matters, because the desktop emulator does not
 *     bridge sinf/cosf/atan2f (they would silently return junk).
 *   - cross(a,b) = a.x*b.y - a.y*b.x ; the velocity of a point at offset r is v + w*(-r.y, r.x).
 */
#ifndef TT_TYPES_H
#define TT_TYPES_H

#include "tapp_api.h"
#include "tt_config.h"

typedef struct {
    float x, y;
} vec2;

typedef struct {
    float s, c;
} rot2;

/* ---- geometry ----------------------------------------------------------------------------- */

typedef struct {
    vec2 v[TT_POLY_MAXV]; /* body-local vertices */
    vec2 n[TT_POLY_MAXV]; /* body-local outward unit normal of edge i */
    uint8_t nv;
    uint8_t style; /* index into the render style table */
    uint8_t pad_[2];
} tt_poly_t;

/* A polygon in world space, used by the slice path before it is re-anchored into a body. */
typedef struct {
    vec2 v[TT_CLIP_SCRATCH];
    uint8_t nv;
    uint8_t style;
    uint8_t pad_[2];
} tt_wpoly_t;

/* ---- bodies ------------------------------------------------------------------------------- */

enum {
    TT_F_ALIVE = 1u << 0,
    TT_F_STATIC = 1u << 1,
    TT_F_AWAKE = 1u << 2,
    TT_F_ACTIVE = 1u << 3, /* the piece the player is steering */
    TT_F_NOSLEEP = 1u << 4,
};

typedef struct {
    vec2 p; /* world position of the centre of mass */
    rot2 q;
    vec2 vel;
    float w;

    float im, ii; /* inverse mass / inverse inertia; 0 = static */
    vec2 dp; /* COM delta accumulated since prepare_contacts */
    rot2 dq; /* rotation delta accumulated since prepare_contacts */

    vec2 lo, hi; /* fat world AABB */
    float radius; /* max |vertex| from the COM */
    float sleep_t;
    float mu;

    uint16_t island;
    uint8_t npoly;
    uint8_t flags;
    uint8_t contacts; /* manifolds touching this body last step (settle detection) */
    uint8_t pad_[3];

    tt_poly_t poly[TT_BODY_MAXPOLY];
} tt_body_t;

/* ---- contacts ----------------------------------------------------------------------------- */

typedef struct {
    vec2 ra, rb; /* prepare-time anchors from each COM */
    float sep0; /* separation base; per substep sep = sep0 + dot(dp + rb' - ra', n) */
    float nmass, tmass;
    float nimp, timp; /* accumulated impulses */
    float maxni; /* peak normal impulse, gates restitution */
    float relv0; /* approach velocity at prepare */
    uint32_t id; /* feature id, for warm starting */
} tt_cpoint_t;

typedef struct {
    vec2 n; /* world normal, A -> B */
    uint16_t ba, bb;
    uint16_t sa, sb; /* shape ids (body*TT_BODY_MAXPOLY + poly), sa < sb */
    float mu;
    uint8_t count;
    uint8_t pad_[3];
    tt_cpoint_t pt[2];
} tt_manifold_t;

/* Warm-start carrier: last step's impulses, looked up by shape pair + feature id. */
typedef struct {
    uint32_t key; /* (sa << 16) | sb */
    uint32_t id[2];
    float nimp[2];
    float timp[2];
    int16_t next; /* hash chain, -1 = end */
    uint8_t count;
    uint8_t pad_;
} tt_pc_t;

typedef struct {
    float lo_y, hi_y, lo_x, hi_x;
    uint16_t body;
    uint16_t pad_;
} tt_proxy_t;

typedef struct {
    uint16_t a, b;
} tt_pair_t;

/* ---- render snapshot (crosses the tick -> redraw task boundary) --------------------------- */

typedef struct {
    int16_t x[TT_POLY_MAXV];
    int16_t y[TT_POLY_MAXV];
    uint8_t nv;
    uint8_t style;
} tt_snap_poly_t;

/*
 * The polygon array is allocated, not inline: sized at the compile-time maximum it stayed 6.7 KB
 * per buffer however small the runtime caps were, so the low-memory fallback barely fell back.
 */
typedef struct {
    uint16_t count;
    uint16_t cap;
    tt_snap_poly_t* p;
} tt_snap_t;

/* ---- world -------------------------------------------------------------------------------- */

typedef struct {
    tt_body_t* body;
    uint16_t body_cap;
    uint16_t nbody; /* high-water mark; slots below may be dead */

    tt_manifold_t* mf;
    uint16_t mf_cap, mf_count;

    tt_pc_t* pc; /* previous step's manifolds, for warm starting */
    uint16_t pc_cap, pc_count;
    int16_t* bucket; /* TT_HASH_BUCKETS */

    tt_proxy_t* proxy;
    uint16_t proxy_count;
    tt_pair_t* pair;
    uint16_t pair_count;

    uint16_t* uf; /* union-find parent, body_cap entries */
    float* island_t; /* min sleep_t per island root */

    tt_snap_t snap[2];
    volatile uint32_t published;

    /* soft-constraint coefficients, recomputed per step */
    float bias_rate, mass_scale, impulse_scale, max_bias;

    bool topology_changed; /* a body was created/destroyed: drop warm-start state */
    uint8_t degrade; /* 1 = fewer substeps while the transient after a clear settles */
} tt_world_t;

#endif /* TT_TYPES_H */

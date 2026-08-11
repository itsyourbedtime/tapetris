/*
 * tt_util.h - header-only helpers.
 *
 * Everything here is `static inline` on purpose. The tapp image budget counts 24 bytes of veneer
 * reserve per relocation, and with -ffunction-sections every out-of-line call site is one; these
 * are called from the innermost loops, so inlining them costs nothing and saves kilobytes.
 *
 * Nothing here may become an import. In particular: no rand(), no __builtin_popcount (clang can
 * lower it to __popcountsi2), no trig.
 */
#ifndef TT_UTIL_H
#define TT_UTIL_H

#include "tt_types.h"

/* ---- scalar ------------------------------------------------------------------------------- */

static inline float tt_clampf(float v, float lo, float hi) {
    return fminf(fmaxf(v, lo), hi);
}

static inline int tt_clampi(int v, int lo, int hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/*
 * Integer finiteness test. -ffast-math implies -ffinite-math-only, so isnan()/isinf() fold to
 * constant false and the documented `x > 0 && x < 1e30f` range check is not guaranteed either.
 * Looking at the exponent bits cannot be optimised away because no float comparison happens.
 */
static inline int tt_finite(float x) {
    union {
        float f;
        uint32_t u;
    } b = {.f = x};
    return (b.u & 0x7F800000u) != 0x7F800000u;
}

/* ---- vectors ------------------------------------------------------------------------------ */

static inline vec2 tt_v(float x, float y) {
    vec2 r = {x, y};
    return r;
}
static inline vec2 tt_add(vec2 a, vec2 b) {
    return tt_v(a.x + b.x, a.y + b.y);
}
static inline vec2 tt_sub(vec2 a, vec2 b) {
    return tt_v(a.x - b.x, a.y - b.y);
}
static inline vec2 tt_scale(vec2 a, float s) {
    return tt_v(a.x * s, a.y * s);
}
static inline float tt_dot(vec2 a, vec2 b) {
    return a.x * b.x + a.y * b.y;
}
static inline float tt_cross(vec2 a, vec2 b) {
    return a.x * b.y - a.y * b.x;
}
/* velocity contribution of angular rate w at offset r */
static inline vec2 tt_wxr(float w, vec2 r) {
    return tt_v(-w * r.y, w * r.x);
}
/* outward normal of an edge under the positive-shoelace winding, unnormalised */
static inline vec2 tt_edge_normal(vec2 e) {
    return tt_v(e.y, -e.x);
}

static inline vec2 tt_normalise(vec2 a) {
    const float l2 = a.x * a.x + a.y * a.y;
    if(l2 < 1.0e-12f) return tt_v(1.0f, 0.0f);
    const float inv = 1.0f / sqrtf(l2);
    return tt_v(a.x * inv, a.y * inv);
}

/* ---- rotors ------------------------------------------------------------------------------- */

static inline rot2 tt_rot_id(void) {
    rot2 r = {0.0f, 1.0f};
    return r;
}
static inline vec2 tt_rot(rot2 q, vec2 v) {
    return tt_v(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}
static inline vec2 tt_unrot(rot2 q, vec2 v) { /* q^T * v */
    return tt_v(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
}
static inline rot2 tt_rot_mul(rot2 a, rot2 b) {
    rot2 r = {a.s * b.c + a.c * b.s, a.c * b.c - a.s * b.s};
    return r;
}
static inline rot2 tt_rot_unmul(rot2 a, rot2 b) { /* a^-1 * b */
    rot2 r = {a.c * b.s - a.s * b.c, a.c * b.c + a.s * b.s};
    return r;
}

/*
 * Integrate a rotor by dw = h*w radians, then renormalise. One vsqrt.f32 and no trig; with
 * |dw| <= 0.13 rad the angle error is dw^3/6 ~ 4e-4 rad per substep and the renormalisation
 * removes the magnitude error entirely. Nothing ever needs the angle back.
 */
static inline rot2 tt_rot_integrate(rot2 q, float dw) {
    rot2 r = {q.s + dw * q.c, q.c - dw * q.s};
    const float m2 = r.s * r.s + r.c * r.c;
    if(m2 > 1.0e-12f) {
        const float inv = 1.0f / sqrtf(m2);
        r.s *= inv;
        r.c *= inv;
    } else {
        r.s = 0.0f;
        r.c = 1.0f;
    }
    return r;
}

/* ---- prng --------------------------------------------------------------------------------- */

/* rand()/random() are not bridged by the desktop emulator and there is no srand() anywhere. */
static inline uint32_t tt_rand(uint32_t* s) {
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

/* ---- bit twiddling ------------------------------------------------------------------------ */

static inline uint32_t tt_popc32(uint32_t x) {
    x = x - ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    x = (x + (x >> 4)) & 0x0F0F0F0Fu;
    return (x * 0x01010101u) >> 24;
}

/* ---- clipping ----------------------------------------------------------------------------- */

/*
 * Liang-Barsky. MUST run before every gfx_draw_line: the primitive takes uint16_t and does not
 * clip its endpoints (lib/gfx/gfx_line.c), so a vertex at y = -3 wraps to 65533 and Bresenham
 * grinds through ~65k iterations that the per-pixel clip throws away - about 3.5 ms for one edge.
 * A vertical I-piece at spawn genuinely reaches y < 0, so this is reachable, not theoretical.
 */
static inline bool
    tt_clip_seg(float* ax, float* ay, float* bx, float* by, float xmin, float ymin, float xmax, float ymax) {
    float t0 = 0.0f, t1 = 1.0f;
    const float dx = *bx - *ax, dy = *by - *ay;
    const float p[4] = {-dx, dx, -dy, dy};
    const float q[4] = {*ax - xmin, xmax - *ax, *ay - ymin, ymax - *ay};

    for(int i = 0; i < 4; i++) {
        if(p[i] > -1.0e-6f && p[i] < 1.0e-6f) {
            if(q[i] < 0.0f) return false;
            continue;
        }
        const float r = q[i] / p[i];
        if(p[i] < 0.0f) {
            if(r > t1) return false;
            if(r > t0) t0 = r;
        } else {
            if(r < t0) return false;
            if(r < t1) t1 = r;
        }
    }
    const float nax = *ax + t0 * dx, nay = *ay + t0 * dy;
    *bx = *ax + t1 * dx;
    *by = *ay + t1 * dy;
    *ax = nax;
    *ay = nay;
    return true;
}

#endif /* TT_UTIL_H */

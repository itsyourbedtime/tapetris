/*
 * tt_phys.c - world storage, geometry, integration, broadphase, islands, sleep, snapshot.
 *
 * The step pipeline is Box2D 3.x style soft substepping: narrowphase runs ONCE per step and the
 * substeps update separation analytically from the accumulated position/rotation deltas. That is
 * what makes 4 position updates affordable for the price of one collision pass.
 */

#include "tt_phys.h"
#include "tt_util.h"

/* ================================================================================ arena ==== */

#define TT_ALIGN8(n) (((uint32_t)(n) + 7u) & ~7u)

typedef struct {
    uint8_t* base;
    uint32_t used, cap;
} tt_arena_t;

static void* tt_take(tt_arena_t* a, uint32_t n) {
    n = TT_ALIGN8(n);
    if(a->used + n > a->cap) return NULL;
    void* p = a->base + a->used;
    a->used += n;
    return p;
}

uint32_t tt_phys_bytes(uint16_t body_cap, uint16_t mf_cap) {
    uint32_t n = 0;
    n += TT_ALIGN8(sizeof(tt_world_t));
    n += TT_ALIGN8((uint32_t)body_cap * sizeof(tt_body_t));
    n += TT_ALIGN8((uint32_t)mf_cap * sizeof(tt_manifold_t));
    n += TT_ALIGN8((uint32_t)mf_cap * sizeof(tt_pc_t));
    n += TT_ALIGN8(TT_HASH_BUCKETS * sizeof(int16_t));
    n += TT_ALIGN8((uint32_t)body_cap * sizeof(tt_proxy_t));
    n += TT_ALIGN8(TT_MAX_PAIRS * sizeof(tt_pair_t));
    n += TT_ALIGN8((uint32_t)body_cap * sizeof(uint16_t));
    n += TT_ALIGN8((uint32_t)body_cap * sizeof(float));
    n += 2u * TT_ALIGN8(sizeof(tt_snap_t));
    return n + 64u;
}

tt_world_t* tt_phys_init(void* mem, uint32_t bytes, uint16_t body_cap, uint16_t mf_cap) {
    if(!mem || bytes < tt_phys_bytes(body_cap, mf_cap)) return NULL;

    tt_arena_t a = {(uint8_t*)mem, 0, bytes};
    tt_world_t* w = (tt_world_t*)tt_take(&a, sizeof(tt_world_t));
    if(!w) return NULL;
    memset(w, 0, sizeof(*w));

    w->body_cap = body_cap;
    w->mf_cap = mf_cap;
    w->pc_cap = mf_cap;

    w->body = (tt_body_t*)tt_take(&a, (uint32_t)body_cap * sizeof(tt_body_t));
    w->mf = (tt_manifold_t*)tt_take(&a, (uint32_t)mf_cap * sizeof(tt_manifold_t));
    w->pc = (tt_pc_t*)tt_take(&a, (uint32_t)mf_cap * sizeof(tt_pc_t));
    w->bucket = (int16_t*)tt_take(&a, TT_HASH_BUCKETS * sizeof(int16_t));
    w->proxy = (tt_proxy_t*)tt_take(&a, (uint32_t)body_cap * sizeof(tt_proxy_t));
    w->pair = (tt_pair_t*)tt_take(&a, TT_MAX_PAIRS * sizeof(tt_pair_t));
    w->uf = (uint16_t*)tt_take(&a, (uint32_t)body_cap * sizeof(uint16_t));
    w->island_t = (float*)tt_take(&a, (uint32_t)body_cap * sizeof(float));
    w->snap[0] = (tt_snap_t*)tt_take(&a, sizeof(tt_snap_t));
    w->snap[1] = (tt_snap_t*)tt_take(&a, sizeof(tt_snap_t));

    if(!w->body || !w->mf || !w->pc || !w->bucket || !w->proxy || !w->pair || !w->uf ||
       !w->island_t || !w->snap[0] || !w->snap[1])
        return NULL;

    tt_phys_clear(w);
    return w;
}

void tt_phys_clear(tt_world_t* w) {
    memset(w->body, 0, (uint32_t)w->body_cap * sizeof(tt_body_t));
    w->nbody = 0;
    w->mf_count = 0;
    w->pc_count = 0;
    w->pair_count = 0;
    w->proxy_count = 0;
    w->topology_changed = true;
    w->degrade = 0;
    w->snap[0]->count = 0;
    w->snap[1]->count = 0;
    w->published = 0;
}

const tt_snap_t* tt_phys_published(const tt_world_t* w) {
    /* One aligned 32-bit load; the writer's flip is one aligned 32-bit store. That is an atomic
     * pair on Cortex-M7, which is the whole handoff - redraw() runs on the GUI task, concurrently
     * with and at a higher priority than the tick that produces this. */
    const uint32_t idx = w->published;
    return w->snap[idx & 1u];
}

/* ============================================================================= geometry ==== */

float tt_poly_area(const vec2* v, int n) {
    float a2 = 0.0f;
    for(int i = 0, j = n - 1; i < n; j = i++) a2 += tt_cross(v[j], v[i]);
    return 0.5f * a2;
}

void tt_poly_moments(const vec2* v, int n, float* area, vec2* centroid, float* i_origin) {
    float a2 = 0.0f, cx6 = 0.0f, cy6 = 0.0f, i12 = 0.0f;
    for(int i = 0, j = n - 1; i < n; j = i++) {
        const float x0 = v[j].x, y0 = v[j].y, x1 = v[i].x, y1 = v[i].y;
        const float cr = x0 * y1 - x1 * y0;
        a2 += cr;
        cx6 += (x0 + x1) * cr;
        cy6 += (y0 + y1) * cr;
        i12 += cr * (x0 * x0 + x0 * x1 + x1 * x1 + y0 * y0 + y0 * y1 + y1 * y1);
    }
    if(a2 > -1.0e-5f && a2 < 1.0e-5f) {
        *area = 0.0f;
        *centroid = tt_v(0.0f, 0.0f);
        *i_origin = 0.0f;
        return;
    }
    const float inv = 1.0f / (3.0f * a2);
    *area = 0.5f * a2; /* signed */
    *centroid = tt_v(cx6 * inv, cy6 * inv);
    *i_origin = i12 * (1.0f / 12.0f); /* signed, same sign as area */
}

int tt_dedup(vec2* v, int n) {
    if(n < 3) return 0;
    int m = 0;
    for(int i = 0; i < n; i++) {
        const vec2 p = v[i], q = v[(i + 1) % n];
        const float dx = q.x - p.x, dy = q.y - p.y;
        if(dx * dx + dy * dy > TT_MIN_EDGE2) v[m++] = p;
    }
    return (m >= 3) ? m : 0;
}

int tt_clip_half(const vec2* in, int n, float c, bool keep_above, vec2* out) {
    int m = 0;
    for(int i = 0, j = n - 1; i < n; j = i++) {
        const vec2 a = in[j], b = in[i];
        const float da = keep_above ? (c - a.y) : (a.y - c);
        const float db = keep_above ? (c - b.y) : (b.y - c);
        const bool ain = (da >= 0.0f), bin = (db >= 0.0f);

        if(ain != bin && m < TT_CLIP_SCRATCH) {
            const float t = da / (da - db);
            out[m].x = a.x + t * (b.x - a.x);
            /* Snap exactly onto the plane. Letting fp drift leave a 1e-7 slope here is how a
             * later cut turns the flat edge into a sliver. */
            out[m].y = c;
            m++;
        }
        if(bin && m < TT_CLIP_SCRATCH) out[m++] = b;
    }
    return tt_dedup(out, m);
}

/* Max separation of `b` along `a`'s outward face normals, world space, normals derived on the fly.
 * Only used by the connectivity pass, so it does not need the cached-normal fast path. */
static float tt_sat_axis(const vec2* va, int na, const vec2* vb, int nb) {
    float best = -1.0e30f;
    for(int i = 0, j = na - 1; i < na; j = i++) {
        const vec2 n = tt_normalise(tt_edge_normal(tt_sub(va[i], va[j])));
        float mind = 1.0e30f;
        for(int k = 0; k < nb; k++) {
            const float d = tt_dot(vb[k], n);
            if(d < mind) mind = d;
        }
        const float s = mind - tt_dot(va[j], n);
        if(s > best) best = s;
    }
    return best;
}

bool tt_sat_overlap(const vec2* va, int na, const vec2* vb, int nb, float eps) {
    if(na < 3 || nb < 3) return false;
    if(tt_sat_axis(va, na, vb, nb) > eps) return false;
    if(tt_sat_axis(vb, nb, va, na) > eps) return false;
    return true;
}

/* ================================================================================ bodies ==== */

int32_t tt_body_alloc(tt_world_t* w) {
    for(uint16_t i = 0; i < w->body_cap; i++) {
        if(w->body[i].flags & TT_F_ALIVE) continue;
        memset(&w->body[i], 0, sizeof(tt_body_t));
        w->body[i].flags = TT_F_ALIVE | TT_F_AWAKE;
        w->body[i].q = tt_rot_id();
        w->body[i].mu = TT_MU_BLOCK;
        if(i >= w->nbody) w->nbody = (uint16_t)(i + 1);
        w->topology_changed = true;
        return (int32_t)i;
    }
    return -1;
}

void tt_body_destroy(tt_world_t* w, uint16_t bi) {
    if(bi >= w->body_cap) return;
    w->body[bi].flags = 0;
    w->body[bi].npoly = 0;
    w->topology_changed = true;
}

void tt_body_update_aabb(tt_world_t* w, uint16_t bi) {
    tt_body_t* b = &w->body[bi];
    float lox = 1.0e30f, loy = 1.0e30f, hix = -1.0e30f, hiy = -1.0e30f;
    for(uint8_t p = 0; p < b->npoly; p++) {
        const tt_poly_t* pl = &b->poly[p];
        for(uint8_t k = 0; k < pl->nv; k++) {
            const vec2 g = tt_add(b->p, tt_rot(b->q, pl->v[k]));
            if(g.x < lox) lox = g.x;
            if(g.x > hix) hix = g.x;
            if(g.y < loy) loy = g.y;
            if(g.y > hiy) hiy = g.y;
        }
    }
    b->lo = tt_v(lox - TT_AABB_FAT, loy - TT_AABB_FAT);
    b->hi = tt_v(hix + TT_AABB_FAT, hiy + TT_AABB_FAT);
}

bool tt_body_reanchor(tt_world_t* w, uint16_t bi, float density, bool make_static) {
    tt_body_t* b = &w->body[bi];
    if(b->npoly == 0) {
        tt_body_destroy(w, bi);
        return false;
    }

    /* Pass 1: area + centroid, fixing the winding as we go. */
    float A = 0.0f;
    vec2 sum = tt_v(0.0f, 0.0f);
    for(uint8_t p = 0; p < b->npoly; p++) {
        tt_poly_t* pl = &b->poly[p];
        float area, io;
        vec2 c;
        tt_poly_moments(pl->v, pl->nv, &area, &c, &io);
        if(area < 0.0f) { /* reverse to the positive-shoelace winding */
            for(uint8_t i = 0, j = (uint8_t)(pl->nv - 1); i < j; i++, j--) {
                const vec2 t = pl->v[i];
                pl->v[i] = pl->v[j];
                pl->v[j] = t;
            }
            area = -area;
        }
        A += area;
        sum = tt_add(sum, tt_scale(c, area));
    }
    if(A < TT_MIN_BODY_AREA || !tt_finite(A)) {
        tt_body_destroy(w, bi);
        return false;
    }

    const vec2 c = tt_scale(sum, 1.0f / A);

    /* Shift to centroid-local BEFORE the second moment: computing I about a world origin ~200 px
     * away and subtracting A*|c|^2 loses three significant digits to cancellation in float32. */
    for(uint8_t p = 0; p < b->npoly; p++)
        for(uint8_t k = 0; k < b->poly[p].nv; k++) b->poly[p].v[k] = tt_sub(b->poly[p].v[k], c);

    float i_c = 0.0f;
    for(uint8_t p = 0; p < b->npoly; p++) {
        float area, io;
        vec2 cc;
        tt_poly_moments(b->poly[p].v, b->poly[p].nv, &area, &cc, &io);
        i_c += io;
    }

    /* Outward normals, now that the winding and the frame are final. */
    float r2 = 0.0f;
    for(uint8_t p = 0; p < b->npoly; p++) {
        tt_poly_t* pl = &b->poly[p];
        for(uint8_t i = 0, j = (uint8_t)(pl->nv - 1); i < pl->nv; j = i++) {
            pl->n[j] = tt_normalise(tt_edge_normal(tt_sub(pl->v[i], pl->v[j])));
            const float d = tt_dot(pl->v[i], pl->v[i]);
            if(d > r2) r2 = d;
        }
    }
    b->radius = sqrtf(r2);

    /* v at the new reference point = v_old + w x (c_new - c_old) */
    const vec2 r = tt_sub(c, b->p);
    b->vel = tt_add(b->vel, tt_wxr(b->w, r));
    b->p = c;
    b->q = tt_rot_id();

    if(make_static) {
        b->flags |= TT_F_STATIC;
        b->flags &= (uint8_t)~TT_F_AWAKE;
        b->im = 0.0f;
        b->ii = 0.0f;
        b->vel = tt_v(0.0f, 0.0f);
        b->w = 0.0f;
    } else {
        const float mass = density * A;
        b->im = 1.0f / mass;
        const float inertia = density * i_c;
        b->ii = (inertia > 1.0e-3f) ? (1.0f / inertia) : 0.0f;
        b->flags |= TT_F_AWAKE;
        b->sleep_t = 0.0f;
    }

    tt_body_update_aabb(w, bi);
    return true;
}

void tt_phys_wake_all(tt_world_t* w) {
    for(uint16_t i = 0; i < w->nbody; i++) {
        tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        b->flags |= TT_F_AWAKE;
        b->sleep_t = 0.0f;
    }
}

void tt_phys_wake_island(tt_world_t* w, uint16_t bi) {
    if(bi >= w->body_cap) return;
    const uint16_t isl = w->body[bi].island;
    for(uint16_t i = 0; i < w->nbody; i++) {
        tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        if(b->island != isl && i != bi) continue;
        b->flags |= TT_F_AWAKE;
        b->sleep_t = 0.0f;
    }
}

/* =========================================================================== broadphase ==== */

/*
 * 1-D sweep and prune on Y. The well is 19 rows tall and 10 wide, so Y prunes ~1.7x better than X,
 * and the proxy array is near-sorted every frame (most bodies are asleep and do not move at all),
 * which makes the insertion sort effectively O(n).
 *
 * Statics are kept OUT of the swept array - a wall proxy spans the whole Y range and would drag
 * the sweep back to O(n^2) by itself - and tested directly instead.
 */
static void tt_broadphase(tt_world_t* w) {
    w->proxy_count = 0;
    w->pair_count = 0;

    for(uint16_t i = 0; i < w->nbody; i++) {
        const tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        tt_proxy_t* px = &w->proxy[w->proxy_count++];
        px->lo_y = b->lo.y;
        px->hi_y = b->hi.y;
        px->lo_x = b->lo.x;
        px->hi_x = b->hi.x;
        px->body = i;
    }

    for(uint16_t i = 1; i < w->proxy_count; i++) {
        const tt_proxy_t key = w->proxy[i];
        int32_t j = (int32_t)i - 1;
        while(j >= 0 && w->proxy[j].lo_y > key.lo_y) {
            w->proxy[j + 1] = w->proxy[j];
            j--;
        }
        w->proxy[j + 1] = key;
    }

    for(uint16_t i = 0; i < w->proxy_count; i++) {
        const tt_proxy_t* a = &w->proxy[i];
        const bool a_awake = (w->body[a->body].flags & TT_F_AWAKE) != 0;
        for(uint16_t j = (uint16_t)(i + 1); j < w->proxy_count; j++) {
            const tt_proxy_t* b = &w->proxy[j];
            if(b->lo_y > a->hi_y) break;
            /* The single most valuable line in the broadphase: in a settled well only ~5 bodies
             * are awake, so the emitted pair count collapses by an order of magnitude. */
            if(!a_awake && !(w->body[b->body].flags & TT_F_AWAKE)) continue;
            if(a->hi_x < b->lo_x || b->hi_x < a->lo_x) continue;
            if(w->pair_count >= TT_MAX_PAIRS) return;
            w->pair[w->pair_count].a = a->body;
            w->pair[w->pair_count].b = b->body;
            w->pair_count++;
        }
    }

    for(uint16_t s = 0; s < w->nbody; s++) {
        const tt_body_t* sb = &w->body[s];
        if(!(sb->flags & TT_F_ALIVE) || !(sb->flags & TT_F_STATIC)) continue;
        for(uint16_t i = 0; i < w->proxy_count; i++) {
            const tt_proxy_t* a = &w->proxy[i];
            if(!(w->body[a->body].flags & TT_F_AWAKE)) continue;
            if(a->hi_x < sb->lo.x || sb->hi.x < a->lo_x) continue;
            if(a->hi_y < sb->lo.y || sb->hi.y < a->lo_y) continue;
            if(w->pair_count >= TT_MAX_PAIRS) return;
            /* Keep a < b so the warm-start key (built from shape ids) is order-independent. */
            w->pair[w->pair_count].a = (a->body < s) ? a->body : s;
            w->pair[w->pair_count].b = (a->body < s) ? s : a->body;
            w->pair_count++;
        }
    }
}

/* ========================================================================== integration ==== */

static void tt_integrate_velocities(tt_world_t* w, float h) {
    for(uint16_t i = 0; i < w->nbody; i++) {
        tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC) || !(b->flags & TT_F_AWAKE))
            continue;

        b->vel.y += TT_GRAVITY * h;

        const float ld = 1.0f / (1.0f + h * TT_LIN_DAMP);
        const float ad = 1.0f / (1.0f + h * ((b->flags & TT_F_ACTIVE) ? TT_ACTIVE_ANG_DAMP
                                                                     : TT_ANG_DAMP));
        b->vel = tt_scale(b->vel, ld);
        b->w *= ad;

        /* Firewall layer 1: cheap, and it catches almost every instability before it compounds.
         * A velocity that is already NaN fails this test silently, which is what tt_validate is
         * for. */
        const float v2 = tt_dot(b->vel, b->vel);
        if(v2 > TT_MAX_SPEED * TT_MAX_SPEED) b->vel = tt_scale(b->vel, TT_MAX_SPEED / sqrtf(v2));
        b->w = tt_clampf(b->w, -TT_MAX_OMEGA, TT_MAX_OMEGA);
    }
}

static void tt_integrate_positions(tt_world_t* w, float h) {
    for(uint16_t i = 0; i < w->nbody; i++) {
        tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC) || !(b->flags & TT_F_AWAKE))
            continue;
        const vec2 d = tt_scale(b->vel, h);
        b->p = tt_add(b->p, d);
        b->dp = tt_add(b->dp, d);
        b->q = tt_rot_integrate(b->q, h * b->w);
        b->dq = tt_rot_integrate(b->dq, h * b->w);
    }
}

/*
 * Firewall layer 2. The recovery action is DELETION, not repair: deletion is O(1), unconditionally
 * terminating and cannot cascade, whereas clamping a body back into range risks re-entering the
 * same bad configuration every step forever. Losing one fragment out of ~50 is invisible.
 */
static void tt_validate(tt_world_t* w) {
    for(uint16_t i = 0; i < w->nbody; i++) {
        tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        const bool bad = !tt_finite(b->p.x) || !tt_finite(b->p.y) || !tt_finite(b->q.s) ||
                         !tt_finite(b->q.c) || !tt_finite(b->vel.x) || !tt_finite(b->vel.y) ||
                         !tt_finite(b->w) || b->p.x < (float)TT_WELL_X0 - 80.0f ||
                         b->p.x > (float)TT_WELL_X1 + 80.0f ||
                         b->p.y > (float)TT_WELL_Y1 + 120.0f || b->p.y < -400.0f;
        if(bad) tt_body_destroy(w, i);
    }
}

/* =============================================================== islands and sleeping ====== */

static uint16_t tt_uf_find(uint16_t* p, uint16_t x) {
    while(p[x] != x) {
        p[x] = p[p[x]]; /* path halving */
        x = p[x];
    }
    return x;
}

static void tt_sleep_update(tt_world_t* w, float dt) {
    for(uint16_t i = 0; i < w->nbody; i++) {
        tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE)) continue;
        w->uf[i] = i;
        if(b->flags & (TT_F_STATIC | TT_F_NOSLEEP)) {
            b->sleep_t = 0.0f;
            continue;
        }
        const float v2 = tt_dot(b->vel, b->vel);
        if(v2 > TT_SLEEP_LIN * TT_SLEEP_LIN || b->w * b->w > TT_SLEEP_ANG * TT_SLEEP_ANG)
            b->sleep_t = 0.0f;
        else
            b->sleep_t += dt;
    }

    /* Union only DYNAMIC-DYNAMIC touching pairs. Unioning through the floor would merge every
     * tower in the well into one island and nothing would ever sleep. */
    for(uint16_t m = 0; m < w->mf_count; m++) {
        const tt_manifold_t* mf = &w->mf[m];
        if(mf->count == 0) continue;
        const tt_body_t* a = &w->body[mf->ba];
        const tt_body_t* b = &w->body[mf->bb];
        if((a->flags & TT_F_STATIC) || (b->flags & TT_F_STATIC)) continue;
        const uint16_t ra = tt_uf_find(w->uf, mf->ba), rb = tt_uf_find(w->uf, mf->bb);
        if(ra != rb) w->uf[rb] = ra;
    }

    for(uint16_t i = 0; i < w->nbody; i++) w->island_t[i] = 1.0e30f;

    for(uint16_t i = 0; i < w->nbody; i++) {
        tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        const uint16_t r = tt_uf_find(w->uf, i);
        b->island = r;
        if(b->sleep_t < w->island_t[r]) w->island_t[r] = b->sleep_t;
    }

    for(uint16_t i = 0; i < w->nbody; i++) {
        tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        if(w->island_t[b->island] >= TT_SLEEP_TIME) {
            b->flags &= (uint8_t)~TT_F_AWAKE;
            b->vel = tt_v(0.0f, 0.0f);
            b->w = 0.0f;
        }
    }
}

/* ============================================================================== snapshot ==== */

void tt_phys_snapshot(tt_world_t* w) {
    const uint32_t back = (w->published + 1u) & 1u;
    tt_snap_t* s = w->snap[back];
    uint16_t n = 0;
    const uint16_t cap = (uint16_t)(TT_MAX_BODIES * TT_BODY_MAXPOLY);

    for(uint16_t i = 0; i < w->nbody && n < cap; i++) {
        const tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        if(b->lo.y > (float)TT_CLIP_Y1) continue; /* fully below the floor */
        for(uint8_t p = 0; p < b->npoly && n < cap; p++) {
            const tt_poly_t* pl = &b->poly[p];
            tt_snap_poly_t* d = &s->p[n++];
            d->nv = pl->nv;
            d->style = pl->style;
            for(uint8_t k = 0; k < pl->nv; k++) {
                const vec2 g = tt_add(b->p, tt_rot(b->q, pl->v[k]));
                /* Clamp well outside the well but inside int16: the rasteriser clips properly,
                 * this only stops a runaway coordinate from overflowing the cast. */
                d->x[k] = (int16_t)tt_clampi((int)(g.x + 0.5f), -4000, 4000);
                d->y[k] = (int16_t)tt_clampi((int)(g.y + 0.5f), -4000, 4000);
            }
            for(uint8_t k = pl->nv; k < TT_POLY_MAXV; k++) {
                d->x[k] = 0;
                d->y[k] = 0;
            }
        }
    }
    s->count = n;
    w->published = back; /* single aligned 32-bit store: the publish */
}

/* ================================================================================== step ==== */

void tt_phys_step(tt_world_t* w, float dt, tt_substep_fn cb, void* ctx) {
    const int nsub = w->degrade ? 3 : TT_SUBSTEPS;
    const float h = dt / (float)nsub;
    const float inv_h = 1.0f / h;

    /* Soft (spring-damper) contact coefficients. Never stiffer than a quarter of the substep
     * rate, or the constraint itself becomes the instability. */
    const float hz = fminf(TT_CONTACT_HZ, 0.25f * inv_h);
    const float omega = 2.0f * M_PI * hz;
    const float den = 2.0f * TT_CONTACT_ZETA + h * omega;
    const float cterm = h * omega * den;
    w->bias_rate = omega / den;
    w->mass_scale = cterm / (1.0f + cterm);
    w->impulse_scale = 1.0f / (1.0f + cterm);
    w->max_bias = TT_MAX_BIAS_VEL;

    for(uint16_t i = 0; i < w->nbody; i++) {
        tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE)) continue;
        if((b->flags & TT_F_AWAKE) && !(b->flags & TT_F_STATIC)) tt_body_update_aabb(w, i);
        b->dp = tt_v(0.0f, 0.0f);
        b->dq = tt_rot_id();
    }

    tt_broadphase(w);
    tt_collide_all(w);
    tt_solver_prepare(w, h);

    for(int s = 0; s < nsub; s++) {
        tt_integrate_velocities(w, h);
        if(cb) cb(ctx, h);
        tt_solver_warm_start(w);
        tt_solver_solve(w, inv_h, true);
        tt_integrate_positions(w, h);
        if(!w->degrade) tt_solver_solve(w, inv_h, false); /* relax: drains the bias energy */
    }

    tt_solver_restitution(w);
    tt_solver_store(w);
    tt_validate(w);
    tt_sleep_update(w, dt);

    /* Explicit, bounded degradation for the ~1 s transient after a line clear. Stated out loud
     * rather than silently dropping work. */
    w->degrade = (uint8_t)(w->mf_count > (uint16_t)(w->mf_cap * 2u / 3u) ? 1u : 0u);
}

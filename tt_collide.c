/*
 * tt_collide.c - convex polygon narrowphase.
 *
 * b2CollidePolygons in this codebase's types: max face separation over each polygon's normals ->
 * reference face -> incident face -> Sutherland-Hodgman clip against the reference face's side
 * planes -> up to two contact points with a stable feature id.
 *
 * Also owns the warm-start carrier: last step's impulses, hashed by shape pair, looked up per
 * contact point by feature id.
 */

#include "tt_phys.h"
#include "tt_util.h"

typedef struct {
    rot2 q;
    vec2 p;
} tt_xf_t;

static inline vec2 tt_xf_apply(tt_xf_t t, vec2 v) {
    return tt_add(tt_rot(t.q, v), t.p);
}

/* a^-1 * b : maps b's local frame into a's local frame */
static inline tt_xf_t tt_xf_rel(rot2 qa, vec2 pa, rot2 qb, vec2 pb) {
    tt_xf_t c;
    c.q = tt_rot_unmul(qa, qb);
    c.p = tt_unrot(qa, tt_sub(pb, pa));
    return c;
}

/*
 * Largest separation of B along A's outward face normals, everything expressed in A's frame.
 * Bails the instant it can prove the pair is separated, which is the common case.
 */
static float tt_max_sep(int* best_i, const tt_poly_t* A, const tt_poly_t* B, tt_xf_t xab) {
    float best = -1.0e30f;
    int bi = 0;
    for(int i = 0; i < A->nv; i++) {
        const vec2 n = A->n[i];
        const vec2 v = A->v[i];
        const vec2 ndir = tt_unrot(xab.q, n); /* n in B's frame: support without transforming B */

        float mind = 1.0e30f;
        int mj = 0;
        for(int j = 0; j < B->nv; j++) {
            const float d = tt_dot(B->v[j], ndir);
            if(d < mind) {
                mind = d;
                mj = j;
            }
        }
        const vec2 sb = tt_xf_apply(xab, B->v[mj]);
        const float s = tt_dot(tt_sub(sb, v), n);

        if(s > TT_SPECULATIVE) {
            *best_i = i;
            return s;
        }
        if(s > best) {
            best = s;
            bi = i;
        }
    }
    *best_i = bi;
    return best;
}

/* Emits the incident edge in the REFERENCE frame. x_ri maps incident-local -> reference-local. */
static void tt_incident_edge(
    vec2 out[2],
    uint8_t oid[2],
    const tt_poly_t* ref,
    int ri,
    const tt_poly_t* inc,
    tt_xf_t x_ri) {
    const vec2 rn = tt_unrot(x_ri.q, ref->n[ri]);
    int best = 0;
    float mind = 1.0e30f;
    for(int i = 0; i < inc->nv; i++) {
        const float d = tt_dot(inc->n[i], rn);
        if(d < mind) {
            mind = d;
            best = i;
        }
    }
    const int i1 = best;
    const int i2 = (best + 1 == inc->nv) ? 0 : best + 1;
    out[0] = tt_xf_apply(x_ri, inc->v[i1]);
    oid[0] = (uint8_t)i1;
    out[1] = tt_xf_apply(x_ri, inc->v[i2]);
    oid[1] = (uint8_t)i2;
}

static int tt_clip_seg_plane(
    vec2 out[2],
    uint8_t oid[2],
    const vec2 in[2],
    const uint8_t iid[2],
    vec2 n,
    float offset,
    uint8_t new_id) {
    int num = 0;
    const float d0 = tt_dot(n, in[0]) - offset;
    const float d1 = tt_dot(n, in[1]) - offset;

    if(d0 <= 0.0f) {
        out[num] = in[0];
        oid[num] = iid[0];
        num++;
    }
    if(d1 <= 0.0f) {
        out[num] = in[1];
        oid[num] = iid[1];
        num++;
    }
    /* Strictly-opposite-signs test. Written as a product on purpose: -fno-signed-zeros makes
     * -0.0f == 0.0f, so a sign comparison admits d0 == d1 and a zero denominator below. d0*d1 < 0
     * implies d0 - d1 != 0. */
    if(d0 * d1 < 0.0f && num < 2) {
        const float t = d0 / (d0 - d1);
        out[num].x = in[0].x + t * (in[1].x - in[0].x);
        out[num].y = in[0].y + t * (in[1].y - in[0].y);
        oid[num] = new_id;
        num++;
    }
    return num;
}

/* Fills mf->n and mf->pt[k] with {world point in .ra, separation in .sep0, feature id}. */
static int tt_collide_polys(
    tt_manifold_t* mf,
    const tt_poly_t* A,
    rot2 qa,
    vec2 pa,
    const tt_poly_t* B,
    rot2 qb,
    vec2 pb) {
    mf->count = 0;
    if(A->nv < 3 || B->nv < 3) return 0;

    const tt_xf_t xab = tt_xf_rel(qa, pa, qb, pb);
    int ea;
    const float sa = tt_max_sep(&ea, A, B, xab);
    if(sa > TT_SPECULATIVE) return 0;

    const tt_xf_t xba = tt_xf_rel(qb, pb, qa, pa);
    int eb;
    const float sb = tt_max_sep(&eb, B, A, xba);
    if(sb > TT_SPECULATIVE) return 0;

    const tt_poly_t *ref, *inc;
    tt_xf_t x_ri;
    rot2 q_ref;
    vec2 p_ref;
    int ri;
    uint32_t flip;

    /* The 0.1*slop hysteresis is load-bearing: without it the reference face flip-flops between
     * steps on a near-symmetric contact, the feature ids change every step, warm starting is lost
     * and the stack buzzes. */
    if(sb > sa + 0.1f * TT_SLOP) {
        ref = B;
        inc = A;
        ri = eb;
        x_ri = xba;
        q_ref = qb;
        p_ref = pb;
        flip = 1u;
    } else {
        ref = A;
        inc = B;
        ri = ea;
        x_ri = xab;
        q_ref = qa;
        p_ref = pa;
        flip = 0u;
    }

    vec2 iv[2];
    uint8_t iid[2];
    tt_incident_edge(iv, iid, ref, ri, inc, x_ri);

    const int i1 = ri;
    const int i2 = (ri + 1 == ref->nv) ? 0 : ri + 1;
    const vec2 v1 = ref->v[i1], v2 = ref->v[i2];

    vec2 tan = tt_sub(v2, v1);
    const float len2 = tt_dot(tan, tan);
    if(len2 < 1.0e-12f) return 0;
    const float inv = 1.0f / sqrtf(len2);
    tan = tt_scale(tan, inv);

    vec2 c1[2], c2[2];
    uint8_t d1[2], d2[2];
    if(tt_clip_seg_plane(
           c1, d1, iv, iid, tt_v(-tan.x, -tan.y), -tt_dot(tan, v1), (uint8_t)(0x80u | (unsigned)i1)) <
       2)
        return 0;
    if(tt_clip_seg_plane(c2, d2, c1, d1, tan, tt_dot(tan, v2), (uint8_t)(0x80u | (unsigned)i2)) < 2)
        return 0;

    const vec2 rn = ref->n[i1];
    const float front = tt_dot(rn, v1);

    vec2 wn = tt_rot(q_ref, rn);
    if(flip) wn = tt_v(-wn.x, -wn.y); /* always A -> B */
    mf->n = wn;

    int k = 0;
    for(int i = 0; i < 2; i++) {
        const float sep = tt_dot(rn, c2[i]) - front;
        if(sep <= TT_SPECULATIVE) {
            const tt_xf_t xr = {q_ref, p_ref};
            mf->pt[k].ra = tt_xf_apply(xr, c2[i]); /* world contact point, converted in prepare */
            mf->pt[k].sep0 = sep;
            mf->pt[k].id = (flip << 16) | ((uint32_t)i1 << 8) | (uint32_t)d2[i];
            k++;
        }
    }
    mf->count = (uint8_t)k;
    return k;
}

/* ========================================================================== warm start ===== */

static inline uint32_t tt_hash(uint32_t key) {
    return ((key * 2654435761u) >> 16) & (uint32_t)(TT_HASH_BUCKETS - 1);
}

const tt_pc_t* tt_pc_lookup(const tt_world_t* w, uint32_t key) {
    int16_t i = w->bucket[tt_hash(key)];
    while(i >= 0) {
        if(w->pc[i].key == key) return &w->pc[i];
        i = w->pc[i].next;
    }
    return NULL;
}

void tt_pc_rebuild(tt_world_t* w);
void tt_pc_rebuild(tt_world_t* w) {
    for(int i = 0; i < TT_HASH_BUCKETS; i++) w->bucket[i] = -1;
    w->pc_count = 0;
    for(uint16_t m = 0; m < w->mf_count && w->pc_count < w->pc_cap; m++) {
        const tt_manifold_t* mf = &w->mf[m];
        if(mf->count == 0) continue;
        tt_pc_t* pc = &w->pc[w->pc_count];
        pc->key = ((uint32_t)mf->sa << 16) | (uint32_t)mf->sb;
        pc->count = mf->count;
        for(int k = 0; k < 2; k++) {
            pc->id[k] = mf->pt[k].id;
            pc->nimp[k] = mf->pt[k].nimp;
            pc->timp[k] = mf->pt[k].timp;
        }
        const uint32_t h = tt_hash(pc->key);
        pc->next = w->bucket[h];
        w->bucket[h] = (int16_t)w->pc_count;
        w->pc_count++;
    }
}

/* ============================================================================ dispatch ===== */

void tt_collide_all(tt_world_t* w) {
    w->mf_count = 0;

    for(uint16_t i = 0; i < w->nbody; i++)
        if(w->body[i].flags & TT_F_ALIVE) w->body[i].contacts = 0;

    /* A body slot can be recycled between steps, which would let a stale key match a completely
     * different contact. Dropping the whole table on any create/destroy is one step of softer
     * contacts and is trivially correct. */
    if(w->topology_changed) {
        for(int i = 0; i < TT_HASH_BUCKETS; i++) w->bucket[i] = -1;
        w->pc_count = 0;
        w->topology_changed = false;
    }

    for(uint16_t pi = 0; pi < w->pair_count && w->mf_count < w->mf_cap; pi++) {
        const uint16_t ia = w->pair[pi].a, ib = w->pair[pi].b;
        tt_body_t* A = &w->body[ia];
        tt_body_t* B = &w->body[ib];
        if(!(A->flags & TT_F_ALIVE) || !(B->flags & TT_F_ALIVE)) continue;

        const float mu = sqrtf(A->mu * B->mu);
        bool touched = false;

        for(uint8_t pa = 0; pa < A->npoly && w->mf_count < w->mf_cap; pa++) {
            for(uint8_t pb = 0; pb < B->npoly && w->mf_count < w->mf_cap; pb++) {
                tt_manifold_t* mf = &w->mf[w->mf_count];
                if(tt_collide_polys(mf, &A->poly[pa], A->q, A->p, &B->poly[pb], B->q, B->p) == 0)
                    continue;
                mf->ba = ia;
                mf->bb = ib;
                mf->sa = (uint16_t)(ia * TT_BODY_MAXPOLY + pa);
                mf->sb = (uint16_t)(ib * TT_BODY_MAXPOLY + pb);
                mf->mu = mu;
                w->mf_count++;
                touched = true;
            }
        }

        if(touched) {
            if(A->contacts < 255) A->contacts++;
            if(B->contacts < 255) B->contacts++;
            /* Wake-on-touch: a moving body reaching a sleeping tower must wake the whole tower,
             * not one body per step. */
            const bool aw = (A->flags & TT_F_AWAKE) != 0;
            const bool bw = (B->flags & TT_F_AWAKE) != 0;
            if(aw != bw) {
                if(aw && !(B->flags & TT_F_STATIC)) tt_phys_wake_island(w, ib);
                if(bw && !(A->flags & TT_F_STATIC)) tt_phys_wake_island(w, ia);
            }
        }
    }
}

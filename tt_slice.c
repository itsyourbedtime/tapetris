/*
 * tt_slice.c - the Not Tetris line clear: measure row coverage, then cut the band out of the
 * geometry rather than deleting a row of cells.
 */

#include "tt_slice.h"
#include "tt_phys.h"
#include "tt_util.h"

#define TT_DENSITY (1.0f / 576.0f) /* an intact tetromino weighs exactly 1.0 */
#define TT_BAND_SAMPLES (TT_SCOLS * 3)

/* ============================================================================== coverage ==== */

static void tt_cov_accum(uint32_t* cov, const vec2* v, int n) {
    float ymin = v[0].y, ymax = v[0].y;
    for(int i = 1; i < n; i++) {
        if(v[i].y < ymin) ymin = v[i].y;
        if(v[i].y > ymax) ymax = v[i].y;
    }

    int s0 = (int)ceilf((ymin - (float)TT_SAMP_Y0) * (1.0f / (float)TT_SAMP_STEP));
    int s1 = (int)floorf((ymax - (float)TT_SAMP_Y0) * (1.0f / (float)TT_SAMP_STEP));
    s0 = tt_clampi(s0, 0, TT_SROWS - 1);
    s1 = tt_clampi(s1, 0, TT_SROWS - 1);

    for(int s = s0; s <= s1; s++) {
        const float y = (float)TT_SAMP_Y0 + (float)(TT_SAMP_STEP * s);
        float xl = 1.0e9f, xr = -1.0e9f;
        for(int i = 0, j = n - 1; i < n; j = i++) {
            const float ya = v[j].y, yb = v[i].y;
            if((y >= ya) == (y >= yb)) continue; /* half-open: exactly two crossings, convex */
            const float x = v[j].x + (y - ya) * (v[i].x - v[j].x) / (yb - ya);
            if(x < xl) xl = x;
            if(x > xr) xr = x;
        }
        if(xr < xl) continue;

        int k0 = (int)ceilf((xl - (float)TT_SAMP_X0) * (1.0f / (float)TT_SAMP_STEP));
        int k1 = (int)floorf((xr - (float)TT_SAMP_X0) * (1.0f / (float)TT_SAMP_STEP));
        if(k0 < 0) k0 = 0;
        if(k1 > TT_SCOLS - 1) k1 = TT_SCOLS - 1;
        if(k0 > k1) continue;
        /* k1-k0 is 0..29, so the shift is 2..31 and never the UB case of 32. */
        cov[s] |= (0xFFFFFFFFu >> (31 - (k1 - k0))) << k0;
    }
}

void tt_cov_scan(const tt_world_t* w, tt_slice_ctx_t* sc) {
    memset(sc->cov, 0, sizeof(sc->cov));

    for(uint16_t i = 0; i < w->nbody; i++) {
        const tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        if(b->hi.y < (float)TT_SAMP_Y0 || b->lo.y > (float)TT_WELL_Y1) continue;
        for(uint8_t p = 0; p < b->npoly; p++) {
            const tt_poly_t* pl = &b->poly[p];
            vec2 wv[TT_POLY_MAXV];
            for(uint8_t k = 0; k < pl->nv; k++) wv[k] = tt_add(b->p, tt_rot(b->q, pl->v[k]));
            tt_cov_accum(sc->cov, wv, pl->nv);
        }
    }

    for(int r = 0; r < TT_ROWS; r++) {
        const int s = TT_SROWS - 3 - 3 * r; /* band r owns sample rows s, s+1, s+2 */
        const uint32_t bits =
            tt_popc32(sc->cov[s]) + tt_popc32(sc->cov[s + 1]) + tt_popc32(sc->cov[s + 2]);
        sc->row_pct[r] = (uint8_t)((bits * 100u) / (uint32_t)TT_BAND_SAMPLES);
    }
}

int tt_scan_bands(const tt_slice_ctx_t* sc, tt_band_t* out) {
    int n = 0;
    for(int r = 0; r < TT_ROWS && n < TT_MAX_BANDS; r++) {
        if(sc->row_pct[r] < (uint8_t)TT_FILL_PCT) continue;
        out[n].y_bot = (float)(TT_WELL_Y1 - TT_CELL * r);
        out[n].y_top = out[n].y_bot - (float)TT_CELL;
        out[n].pct = sc->row_pct[r];
        n++;
    }
    return n;
}

/* ========================================================================== connectivity ==== */

static uint8_t tt_uf_find8(uint8_t* p, uint8_t i) {
    while(p[i] != i) {
        p[i] = p[p[i]];
        i = p[i];
    }
    return i;
}

static bool tt_frag_aabb_touch(const tt_wpoly_t* a, const tt_wpoly_t* b, float eps) {
    float alx = a->v[0].x, ahx = a->v[0].x, aly = a->v[0].y, ahy = a->v[0].y;
    for(uint8_t i = 1; i < a->nv; i++) {
        alx = fminf(alx, a->v[i].x);
        ahx = fmaxf(ahx, a->v[i].x);
        aly = fminf(aly, a->v[i].y);
        ahy = fmaxf(ahy, a->v[i].y);
    }
    float blx = b->v[0].x, bhx = b->v[0].x, bly = b->v[0].y, bhy = b->v[0].y;
    for(uint8_t i = 1; i < b->nv; i++) {
        blx = fminf(blx, b->v[i].x);
        bhx = fmaxf(bhx, b->v[i].x);
        bly = fminf(bly, b->v[i].y);
        bhy = fmaxf(bhy, b->v[i].y);
    }
    return !(ahx + eps < blx || bhx + eps < alx || ahy + eps < bly || bhy + eps < aly);
}

/* Groups fragments into connected sets; returns the group count, group[i] is dense 0-based. */
static int tt_connect(tt_slice_ctx_t* sc, int nf) {
    for(int i = 0; i < nf; i++) sc->parent[i] = (uint8_t)i;

    for(int i = 0; i < nf; i++) {
        for(int j = i + 1; j < nf; j++) {
            if(!tt_frag_aabb_touch(&sc->frag[i], &sc->frag[j], TT_TOUCH_EPS)) continue;
            if(!tt_sat_overlap(
                   sc->frag[i].v, sc->frag[i].nv, sc->frag[j].v, sc->frag[j].nv, TT_TOUCH_EPS))
                continue;
            const uint8_t ra = tt_uf_find8(sc->parent, (uint8_t)i);
            const uint8_t rb = tt_uf_find8(sc->parent, (uint8_t)j);
            if(ra != rb) sc->parent[rb] = ra;
        }
    }

    uint8_t map[TT_SPLIT_MAXPOLY];
    for(int i = 0; i < nf; i++) map[i] = 0xFFu;
    int k = 0;
    for(int i = 0; i < nf; i++) {
        const uint8_t r = tt_uf_find8(sc->parent, (uint8_t)i);
        if(map[r] == 0xFFu) map[r] = (uint8_t)k++;
        sc->group[i] = map[r];
    }
    return k;
}

/* ================================================================================= slice ==== */

static bool tt_push_frag(tt_slice_ctx_t* sc, int* nn, int base, const vec2* v, int nv, uint8_t style) {
    if(nv < 3 || nv > TT_POLY_MAXV) return false;
    if(fabsf(tt_poly_area(v, nv)) < TT_MIN_POLY_AREA) return false;
    const int idx = base + *nn;
    if(idx >= TT_SPLIT_MAXPOLY) return false;
    sc->work[idx].nv = (uint8_t)nv;
    sc->work[idx].style = style;
    memcpy(sc->work[idx].v, v, sizeof(vec2) * (size_t)nv);
    (*nn)++;
    return true;
}

void tt_apply_clears(tt_world_t* w, tt_slice_ctx_t* sc, const tt_band_t* band, int nbands) {
    if(nbands <= 0) return;

    float band_min_y = band[0].y_top, band_max_y = band[0].y_bot;
    for(int k = 1; k < nbands; k++) {
        band_min_y = fminf(band_min_y, band[k].y_top);
        band_max_y = fmaxf(band_max_y, band[k].y_bot);
    }

    /* Snapshot the high-water mark: tt_body_alloc appends, and without this the loop would walk
     * into the fragments it just created. */
    const uint16_t n0 = w->nbody;

    for(uint16_t bi = 0; bi < n0; bi++) {
        tt_body_t* b = &w->body[bi];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        if(b->hi.y < band_min_y || b->lo.y > band_max_y) continue;

        int nf = 0;

        for(uint8_t pi = 0; pi < b->npoly; pi++) {
            const tt_poly_t* pl = &b->poly[pi];
            int nw = 1;
            sc->work[0].nv = pl->nv;
            sc->work[0].style = pl->style;
            for(uint8_t k = 0; k < pl->nv; k++)
                sc->work[0].v[k] = tt_add(b->p, tt_rot(b->q, pl->v[k]));

            for(int k = 0; k < nbands && nw > 0; k++) {
                int nn = 0;
                for(int q = 0; q < nw; q++) {
                    vec2 tmp[TT_CLIP_SCRATCH];
                    int c;
                    c = tt_clip_half(sc->work[q].v, sc->work[q].nv, band[k].y_top, true, tmp);
                    tt_push_frag(sc, &nn, nw, tmp, c, sc->work[q].style);
                    c = tt_clip_half(sc->work[q].v, sc->work[q].nv, band[k].y_bot, false, tmp);
                    tt_push_frag(sc, &nn, nw, tmp, c, sc->work[q].style);
                    /* whatever lay strictly inside the band is simply gone */
                }
                memmove(sc->work, sc->work + nw, sizeof(tt_wpoly_t) * (size_t)nn);
                nw = nn;
            }

            for(int q = 0; q < nw && nf < TT_SPLIT_MAXPOLY; q++) sc->frag[nf++] = sc->work[q];
        }

        if(nf == 0) {
            tt_body_destroy(w, bi);
            continue;
        }

        const int ngrp = tt_connect(sc, nf);

        /* Snapshot the OLD frame before the first rebuild overwrites b->p: a later group copying
         * *b would otherwise inherit the NEW centroid as its old reference and get the velocity
         * transfer wrong. */
        const vec2 src_p = b->p;
        const vec2 src_v = b->vel;
        const float src_w = b->w;

        for(int g = 0; g < ngrp; g++) {
            tt_body_t* dst;
            if(g == 0) {
                dst = b;
            } else {
                const int32_t ni = tt_body_alloc(w);
                if(ni < 0) break; /* pool full: drop the remaining groups */
                dst = &w->body[ni];
            }

            dst->flags = (uint8_t)(TT_F_ALIVE | TT_F_AWAKE);
            dst->p = src_p;
            dst->q = tt_rot_id();
            dst->vel = src_v;
            dst->w = src_w;
            dst->mu = TT_MU_BLOCK;
            dst->sleep_t = 0.0f;
            dst->npoly = 0;

            /* Keep the biggest fragments if a group has more than a body can hold. Insertion sort
             * by area, not qsort - qsort is not bridged by the desktop emulator. */
            uint8_t pick[TT_SPLIT_MAXPOLY];
            float parea[TT_SPLIT_MAXPOLY];
            int np = 0;
            for(int q = 0; q < nf; q++) {
                if(sc->group[q] != (uint8_t)g) continue;
                const float ar = fabsf(tt_poly_area(sc->frag[q].v, sc->frag[q].nv));
                int ins = np;
                while(ins > 0 && parea[ins - 1] < ar) {
                    parea[ins] = parea[ins - 1];
                    pick[ins] = pick[ins - 1];
                    ins--;
                }
                parea[ins] = ar;
                pick[ins] = (uint8_t)q;
                np++;
            }
            if(np > TT_BODY_MAXPOLY) np = TT_BODY_MAXPOLY;

            for(int q = 0; q < np; q++) {
                const tt_wpoly_t* f = &sc->frag[pick[q]];
                tt_poly_t* out = &dst->poly[dst->npoly++];
                out->nv = f->nv;
                out->style = f->style;
                memcpy(out->v, f->v, sizeof(vec2) * (size_t)f->nv);
            }

            const uint16_t di = (uint16_t)(dst - w->body);
            tt_body_reanchor(w, di, TT_DENSITY, false);
        }
    }

    tt_phys_wake_all(w);
    w->topology_changed = true;
}

/*
 * tt_render.c - everything drawn, read from the published physics snapshot.
 *
 * redraw() runs on the GUI task, concurrently with and at a HIGHER priority than the tick that
 * produces the geometry, so nothing here touches the live world - only the snapshot the tick
 * published. There is no polygon primitive in the API, so convex fills are scanline-rasterised
 * here out of gfx_draw_hline[_dithered].
 */

#include "tt_render.h"
#include "tt_phys.h"
#include "tt_util.h"

#define TT_SOLID 0xFFu

typedef struct {
    uint8_t shade; /* 0..7 dithered, or TT_SOLID */
    uint8_t row_mask; /* paint row only if bit (y & 7) is set; 0xFF = every row */
    uint8_t outline; /* 1 or 2 px */
} tt_style_t;

/*
 * Fill density has to stay at or under 50%: against 62-87% ink a 1 px outline is invisible and two
 * touching pieces merge into an indistinct blob. That caps the usable palette at shades 1-4, so
 * outline weight is the second axis. S vs Z - the pair that matters most - differ on both.
 * J vs L differ only in outline weight; if that proves too subtle on the real panel, giving L
 * {TT_SOLID, 0x33, 1} turns it into 2-on/2-off ink bands, which is unlike any Bayer shade.
 */
static const tt_style_t tt_style[TT_NPIECE] = {
    /* I */ {4u, 0xFFu, 2u},
    /* O */ {4u, 0xFFu, 1u},
    /* T */ {3u, 0xFFu, 2u},
    /* J */ {2u, 0xFFu, 2u},
    /* L */ {TT_SOLID, 0x33u, 1u}, /* 2-on/2-off ink bands: unlike any Bayer shade, so L cannot
                                     be mistaken for J, which is 25% dots with a heavy edge */
    /* S */ {1u, 0xFFu, 2u},
    /* Z */ {3u, 0xFFu, 1u},
};

/* ============================================================================ primitives ==== */

static void tt_fill_convex(gfx_t* g, const vec2* v, int n, const tt_style_t* st) {
    if(n < 3) return;

    float ymin = v[0].y, ymax = v[0].y;
    for(int i = 1; i < n; i++) {
        ymin = fminf(ymin, v[i].y);
        ymax = fmaxf(ymax, v[i].y);
    }

    int y0 = (int)ceilf(ymin - 0.5f);
    int y1 = (int)floorf(ymax - 0.5f);
    if(y0 < TT_CLIP_Y0) y0 = TT_CLIP_Y0;
    if(y1 > TT_CLIP_Y1 - 1) y1 = TT_CLIP_Y1 - 1;
    if(y0 > y1) return;

    /* Hoist the per-edge constants out of the row loop. */
    float ex[TT_POLY_MAXV], eya[TT_POLY_MAXV], eyb[TT_POLY_MAXV], einv[TT_POLY_MAXV];
    int ne = 0;
    for(int i = 0, j = n - 1; i < n; j = i++) {
        const float ya = v[j].y, yb = v[i].y;
        const float dy = yb - ya;
        if(dy > -1.0e-5f && dy < 1.0e-5f) continue; /* horizontal edge: no crossings */
        ex[ne] = v[j].x;
        eya[ne] = ya;
        eyb[ne] = yb;
        einv[ne] = (v[i].x - v[j].x) / dy;
        ne++;
    }
    if(ne < 2) return;

    const uint8_t rm = st->row_mask;
    const uint8_t shade = st->shade;

    for(int y = y0; y <= y1; y++) {
        if(!((rm >> (y & 7)) & 1u)) continue;

        const float fy = (float)y + 0.5f;
        float xl = 1.0e9f, xr = -1.0e9f;
        for(int e = 0; e < ne; e++) {
            /* half-open span test: convex, so exactly two edges straddle any scanline */
            if((fy >= eya[e]) == (fy >= eyb[e])) continue;
            const float x = ex[e] + (fy - eya[e]) * einv[e];
            if(x < xl) xl = x;
            if(x > xr) xr = x;
        }
        if(xr < xl) continue;

        int ix0 = (int)ceilf(xl - 0.5f);
        int ix1 = (int)floorf(xr - 0.5f);
        /* Clamp by hand. gfx_draw_hline honours gfx->clip_*, but gfx_draw_hline_dithered checks
         * only against SCREEN_WIDTH/HEIGHT and then writes framebuffer bytes directly, so
         * gfx_set_clip alone would let the dithered path leak out of the well. */
        if(ix0 < TT_WELL_X0) ix0 = TT_WELL_X0;
        if(ix1 > TT_WELL_X1 - 1) ix1 = TT_WELL_X1 - 1;
        if(ix0 > ix1) continue;

        const gfx_uint_t w = (gfx_uint_t)(ix1 - ix0 + 1);
        if(shade == TT_SOLID)
            gfx_draw_hline(g, (gfx_uint_t)ix0, (gfx_uint_t)y, w);
        else
            gfx_draw_hline_dithered(g, (gfx_uint_t)ix0, (gfx_uint_t)y, w, shade);
    }
}

static void tt_outline(gfx_t* g, const vec2* v, int n, uint8_t th) {
    for(int i = 0, j = n - 1; i < n; j = i++) {
        float ax = v[j].x, ay = v[j].y, bx = v[i].x, by = v[i].y;
        if(!tt_clip_seg(
               &ax,
               &ay,
               &bx,
               &by,
               (float)TT_WELL_X0,
               (float)TT_CLIP_Y0,
               (float)(TT_WELL_X1 - 1),
               (float)(TT_CLIP_Y1 - 1)))
            continue;
        const gfx_uint_t x0 = (gfx_uint_t)(ax + 0.5f), y0 = (gfx_uint_t)(ay + 0.5f);
        const gfx_uint_t x1 = (gfx_uint_t)(bx + 0.5f), y1 = (gfx_uint_t)(by + 0.5f);
        if(th > 1)
            gfx_draw_line_thick(g, x0, y0, x1, y1, th);
        else
            gfx_draw_line(g, x0, y0, x1, y1);
    }
}

static void tt_draw_poly(gfx_t* g, const vec2* v, int n, uint8_t style) {
    const tt_style_t* st = &tt_style[style % TT_NPIECE];
    tt_fill_convex(g, v, n, st);
    tt_outline(g, v, n, st->outline);
}

/* ================================================================================== text ==== */

static void tt_str_right(gfx_t* g, int right, int y, const char* s) {
    const gfx_uint_t w = gfx_get_str_width(g, s);
    gfx_draw_str(g, (gfx_uint_t)tt_clampi(right - (int)w, 0, SCREEN_WIDTH - 1), (gfx_uint_t)y, s);
}

static void tt_label_value(gfx_t* g, int y, const char* label, uint32_t value) {
    char buf[16];
    gfx_set_font(g, DepartureMono_Regular_10);
    gfx_draw_str(g, 4, (gfx_uint_t)y, label);
    snprintf(buf, sizeof(buf), "%u", (unsigned)value);
    gfx_set_font(g, gfx_nunito_semibold_14);
    tt_str_right(g, 126, y + 20, buf);
}

/* ================================================================================ panels ==== */

static void tt_draw_well(gfx_t* g) {
    gfx_draw_vline(g, TT_WELL_X0 - 2, 4, TT_WELL_Y1 - 4 + 2);
    gfx_draw_vline(g, TT_WELL_X0 - 1, 4, TT_WELL_Y1 - 4 + 2);
    gfx_draw_vline(g, TT_WELL_X1, 4, TT_WELL_Y1 - 4 + 2);
    gfx_draw_vline(g, TT_WELL_X1 + 1, 4, TT_WELL_Y1 - 4 + 2);
    gfx_draw_hline(g, TT_WELL_X0 - 2, TT_WELL_Y1, TT_WELL_X1 - TT_WELL_X0 + 4);
    gfx_draw_hline(g, TT_WELL_X0 - 2, TT_WELL_Y1 + 1, TT_WELL_X1 - TT_WELL_X0 + 4);
    /* top-out line, stippled so it reads as a limit rather than a surface */
    gfx_draw_hline_dithered(g, TT_WELL_X0, TT_TOPOUT_Y, TT_WELL_X1 - TT_WELL_X0, 4);
}

static void tt_draw_meter(gfx_t* g, const tt_model_t* m) {
    for(int r = 0; r < TT_ROWS; r++) {
        const int y = TT_WELL_Y1 - TT_CELL * r - TT_CELL / 2;
        const int pct = m->slice.row_pct[r];
        const int w = 1 + (pct * 7) / 100;
        gfx_draw_hline(g, (gfx_uint_t)(136 - w), (gfx_uint_t)(y - 1), (gfx_uint_t)w);
        if(pct >= (int)TT_FILL_PCT) gfx_draw_rect_fill(g, 128, (gfx_uint_t)(y - 3), 9, 6);
    }
}

static void tt_draw_piece_preview(gfx_t* g, uint8_t type, int cx, int cy) {
    const tt_piece_t* pc = &tt_pieces[type % TT_NPIECE];
    const float hc = (float)TT_CELL * 0.5f;
    static const float sx[4] = {-1.0f, 1.0f, 1.0f, -1.0f};
    static const float sy[4] = {-1.0f, -1.0f, 1.0f, 1.0f};

    for(int i = 0; i < pc->nb; i++) {
        vec2 v[4];
        const float bx = (float)cx + (float)pc->b[i][0] * hc;
        const float by = (float)cy + (float)pc->b[i][1] * hc;
        const float hw = (float)pc->b[i][2] * hc, hh = (float)pc->b[i][3] * hc;
        for(int k = 0; k < 4; k++) v[k] = tt_v(bx + sx[k] * hw, by + sy[k] * hh);
        /* The preview goes through the same fill + outline as the real piece, so what you see in
         * the box is pixel-identical to what spawns. The rasteriser clamps to the well, so lift
         * the clamp for this one call by drawing it as solid geometry inside the panel. */
        const tt_style_t* st = &tt_style[type % TT_NPIECE];
        for(int y = (int)(by - hh + 0.5f); y < (int)(by + hh + 0.5f); y++) {
            if(y < 0 || y >= SCREEN_HEIGHT) continue;
            if(!((st->row_mask >> (y & 7)) & 1u)) continue;
            const int x0 = (int)(bx - hw + 0.5f), x1 = (int)(bx + hw + 0.5f) - 1;
            if(x1 < x0) continue;
            if(st->shade == TT_SOLID)
                gfx_draw_hline(g, (gfx_uint_t)x0, (gfx_uint_t)y, (gfx_uint_t)(x1 - x0 + 1));
            else
                gfx_draw_hline_dithered(
                    g, (gfx_uint_t)x0, (gfx_uint_t)y, (gfx_uint_t)(x1 - x0 + 1), st->shade);
        }
        for(int k = 0, j = 3; k < 4; j = k++) {
            if(st->outline > 1)
                gfx_draw_line_thick(
                    g,
                    (gfx_uint_t)(v[j].x + 0.5f),
                    (gfx_uint_t)(v[j].y + 0.5f),
                    (gfx_uint_t)(v[k].x + 0.5f),
                    (gfx_uint_t)(v[k].y + 0.5f),
                    st->outline);
            else
                gfx_draw_line(
                    g,
                    (gfx_uint_t)(v[j].x + 0.5f),
                    (gfx_uint_t)(v[j].y + 0.5f),
                    (gfx_uint_t)(v[k].x + 0.5f),
                    (gfx_uint_t)(v[k].y + 0.5f));
        }
    }
}

static void tt_draw_hint(gfx_t* g, int y, const char* key, const char* txt) {
    gfx_draw_rect_r(g, 268, (gfx_uint_t)(y - 14), 18, 18, 3);
    gfx_set_font(g, DepartureMono_Regular_10);
    gfx_draw_str(g, 273, (gfx_uint_t)y, key);
    gfx_draw_str(g, 290, (gfx_uint_t)y, txt);
}

static void tt_draw_panels(gfx_t* g, const tt_model_t* m) {
    tt_label_value(g, 16, "SCORE", m->score);
    tt_label_value(g, 62, "LINES", m->lines);
    tt_label_value(g, 108, "LEVEL", m->level);
    tt_label_value(g, 154, "BEST", m->hiscore);

    if(m->banner_ms && m->banner[0]) {
        gfx_set_font(g, DepartureMono_Regular_10);
        gfx_draw_str(g, 4, 212, m->banner);
    }

    gfx_set_font(g, DepartureMono_Regular_10);
    gfx_draw_str(g, 268, 16, "NEXT");
    gfx_draw_rect_r(g, 268, 22, 78, 62, 3);
    tt_draw_piece_preview(g, m->next_piece, 307, 53);
    tt_draw_hint(g, 112, "1", "move <");
    tt_draw_hint(g, 132, "3", "move >");
    tt_draw_hint(g, 152, "2", "drop");

    gfx_draw_str(g, 268, 172, "ENC: spin");
    tt_draw_hint(g, 190, "4", "hold: exit");
}

static void tt_draw_centre_box(gfx_t* g, const char* line1, const char* line2) {
    gfx_set_color(g, 0);
    gfx_draw_rect_fill(g, 100, 92, 200, 56);
    gfx_set_color(g, 1);
    gfx_draw_rect_r(g, 100, 92, 200, 56, 4);
    gfx_set_font(g, gfx_nunito_bold_18);
    gfx_uint_t w = gfx_get_str_width(g, line1);
    gfx_draw_str(g, (gfx_uint_t)(200 - (int)w / 2), 120, line1);
    if(line2) {
        gfx_set_font(g, DepartureMono_Regular_10);
        w = gfx_get_str_width(g, line2);
        gfx_draw_str(g, (gfx_uint_t)(200 - (int)w / 2), 138, line2);
    }
}

/* ================================================================================== frame ==== */

void tt_render(gfx_t* gfx, const tt_model_t* m) {
    /* The clip window is sticky across frames and across apps; reset it before anything else or a
     * window left set elsewhere silently swallows the whole UI. */
    gfx_clip_reset(gfx);
    gfx_set_color(gfx, 0);
    gfx_draw_rect_fill(gfx, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
    gfx_set_color(gfx, 1);

    if(m->pool_failed || !m->world) {
        gfx_set_font(gfx, gfx_nunito_semibold_14);
        gfx_draw_str(gfx, 90, 116, "out of memory - cannot start");
        return;
    }

    tt_draw_well(gfx);
    tt_draw_meter(gfx, m);
    tt_draw_panels(gfx, m);

    gfx_set_clip(gfx, TT_WELL_X0, TT_CLIP_Y0, TT_WELL_X1 - 1, TT_CLIP_Y1 - 1);

    const tt_snap_t* s = tt_phys_published(m->world);
    for(uint16_t i = 0; i < s->count; i++) {
        const tt_snap_poly_t* p = &s->p[i];
        vec2 v[TT_POLY_MAXV];
        if(p->nv < 3 || p->nv > TT_POLY_MAXV) continue;
        for(uint8_t k = 0; k < p->nv; k++) v[k] = tt_v((float)p->x[k], (float)p->y[k]);
        tt_draw_poly(gfx, v, p->nv, p->style);
    }

    /* Doomed bands, inverted. Physics is paused for this, so the flash lands on exactly the
     * geometry that is about to be cut out. */
    if(m->state == TT_ST_CLEAR_FLASH && ((m->flash_ms / 55u) & 1u) == 0u) {
        gfx_set_color(gfx, 2); /* XOR */
        for(int k = 0; k < m->nbands; k++)
            gfx_draw_rect_fill(
                gfx,
                TT_WELL_X0,
                (gfx_uint_t)(int)m->band[k].y_top,
                TT_WELL_X1 - TT_WELL_X0,
                TT_CELL);
        gfx_set_color(gfx, 1);
    }

    gfx_clip_reset(gfx);

    if(m->state == TT_ST_PAUSED) tt_draw_centre_box(gfx, "PAUSED", "press 5 to resume");
    if(m->state == TT_ST_GAMEOVER) tt_draw_centre_box(gfx, "GAME OVER", "press 2 to restart");
}

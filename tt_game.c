/*
 * tt_game.c - spawn, the state machine, settle detection, scoring and top-out.
 *
 * Nothing here writes a position or an angle. Every control is a velocity servo applied from the
 * substep callback, so the solver always gets the last word and a piece cannot be pushed through
 * the stack.
 */

#include "tt_game.h"
#include "tt_phys.h"
#include "tt_util.h"

#define TT_DENSITY (1.0f / 576.0f)

/* Boxes in half-cells: {centre x, centre y, half width, half height}. Each piece is decomposed
 * into at most two rectangles rather than four unit cells - a seam between two co-planar sub-boxes
 * of the same body is exactly where a neighbour sliding past catches a bogus normal. */
const tt_piece_t tt_pieces[TT_NPIECE] = {
    /* I */ {1, {{0, 0, 4, 1}, {0, 0, 0, 0}}},
    /* O */ {1, {{0, 0, 2, 2}, {0, 0, 0, 0}}},
    /* T */ {2, {{0, -1, 3, 1}, {0, 1, 1, 1}}},
    /* J */ {2, {{-2, -1, 1, 1}, {0, 1, 3, 1}}},
    /* L */ {2, {{2, -1, 1, 1}, {0, 1, 3, 1}}},
    /* S */ {2, {{1, -1, 2, 1}, {-1, 1, 2, 1}}},
    /* Z */ {2, {{-1, -1, 2, 1}, {1, 1, 2, 1}}},
};

/* Spawn tilts as precomputed rotors: the engine calls no trig, and sinf/cosf are not bridged by
 * the desktop emulator anyway (they would silently return junk). +/- 3, 9, 15, 21 degrees. */
static const rot2 tt_tilt[8] = {
    {0.05234f, 0.99863f},
    {-0.05234f, 0.99863f},
    {0.15643f, 0.98769f},
    {-0.15643f, 0.98769f},
    {0.25882f, 0.96593f},
    {-0.25882f, 0.96593f},
    {0.35837f, 0.93358f},
    {-0.35837f, 0.93358f},
};

/* ================================================================================ helpers ==== */

static void tt_make_box(tt_body_t* b, uint8_t slot, vec2 c, float hw, float hh, rot2 q, vec2 org, uint8_t style) {
    static const float sx[4] = {-1.0f, 1.0f, 1.0f, -1.0f};
    static const float sy[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
    tt_poly_t* p = &b->poly[slot];
    p->nv = 4;
    p->style = style;
    for(int i = 0; i < 4; i++) {
        const vec2 l = tt_v(c.x + sx[i] * hw, c.y + sy[i] * hh);
        p->v[i] = tt_add(org, tt_rot(q, l));
    }
}

static int32_t tt_make_static(tt_world_t* w, float x0, float y0, float x1, float y1, float mu) {
    const int32_t bi = tt_body_alloc(w);
    if(bi < 0) return -1;
    tt_body_t* b = &w->body[bi];
    b->npoly = 1;
    b->p = tt_v(0.0f, 0.0f);
    b->q = tt_rot_id();
    tt_make_box(
        b,
        0,
        tt_v(0.5f * (x0 + x1), 0.5f * (y0 + y1)),
        0.5f * (x1 - x0),
        0.5f * (y1 - y0),
        tt_rot_id(),
        tt_v(0.0f, 0.0f),
        0);
    if(!tt_body_reanchor(w, (uint16_t)bi, TT_DENSITY, true)) return -1;
    w->body[bi].mu = mu;
    return bi;
}

static uint8_t tt_bag_next(tt_model_t* m) {
    if(m->bag_n == 0) {
        for(uint8_t i = 0; i < TT_NPIECE; i++) m->bag[i] = i;
        for(uint8_t i = TT_NPIECE - 1; i > 0; i--) {
            const uint8_t j = (uint8_t)(tt_rand(&m->rng) % (uint32_t)(i + 1));
            const uint8_t t = m->bag[i];
            m->bag[i] = m->bag[j];
            m->bag[j] = t;
        }
        m->bag_n = TT_NPIECE;
    }
    return m->bag[--m->bag_n];
}

/* True if the freshly built body overlaps anything already settled. */
static bool tt_spawn_blocked(tt_world_t* w, uint16_t bi) {
    const tt_body_t* n = &w->body[bi];
    for(uint16_t i = 0; i < w->nbody; i++) {
        if(i == bi) continue;
        const tt_body_t* o = &w->body[i];
        if(!(o->flags & TT_F_ALIVE) || (o->flags & TT_F_STATIC)) continue;
        if(n->hi.x < o->lo.x || o->hi.x < n->lo.x) continue;
        if(n->hi.y < o->lo.y || o->hi.y < n->lo.y) continue;

        for(uint8_t pa = 0; pa < n->npoly; pa++) {
            vec2 va[TT_POLY_MAXV];
            for(uint8_t k = 0; k < n->poly[pa].nv; k++)
                va[k] = tt_add(n->p, tt_rot(n->q, n->poly[pa].v[k]));
            for(uint8_t pb = 0; pb < o->npoly; pb++) {
                vec2 vb[TT_POLY_MAXV];
                for(uint8_t k = 0; k < o->poly[pb].nv; k++)
                    vb[k] = tt_add(o->p, tt_rot(o->q, o->poly[pb].v[k]));
                if(tt_sat_overlap(va, n->poly[pa].nv, vb, o->poly[pb].nv, -0.5f)) return true;
            }
        }
    }
    return false;
}

static bool tt_spawn(tt_model_t* m) {
    tt_world_t* w = m->world;
    const uint8_t type = m->next_piece;
    m->next_piece = tt_bag_next(m);

    const int32_t bi = tt_body_alloc(w);
    if(bi < 0) return false;

    rot2 q = tt_rot_id();
    if(m->level > 5u) {
        const uint32_t span = (m->level - 5u) < 4u ? (m->level - 5u) : 4u;
        q = tt_tilt[tt_rand(&m->rng) % (span * 2u)];
    }

    tt_body_t* b = &w->body[bi];
    const tt_piece_t* pc = &tt_pieces[type];
    const vec2 org = tt_v(TT_SPAWN_X, TT_SPAWN_Y);
    const float hc = (float)TT_CELL * 0.5f;

    b->npoly = (uint8_t)pc->nb;
    b->p = org; /* reanchor treats this as the old reference frame */
    b->q = tt_rot_id();
    b->vel = tt_v(0.0f, m->fall_v * 0.5f);
    b->w = 0.0f;
    for(int i = 0; i < pc->nb; i++)
        tt_make_box(
            b,
            (uint8_t)i,
            tt_v((float)pc->b[i][0] * hc, (float)pc->b[i][1] * hc),
            (float)pc->b[i][2] * hc,
            (float)pc->b[i][3] * hc,
            q,
            org,
            type);

    if(!tt_body_reanchor(w, (uint16_t)bi, TT_DENSITY, false)) return false;

    if(tt_spawn_blocked(w, (uint16_t)bi)) {
        tt_body_destroy(w, (uint16_t)bi);
        return false;
    }

    w->body[bi].flags |= TT_F_ACTIVE | TT_F_NOSLEEP;
    m->active = bi;
    m->lock_ms = 0;
    m->touch_ms = 0;
    m->hard_dropped = false;
    m->spin_budget = 0.0f;
    return true;
}

static bool tt_world_quiet(const tt_world_t* w) {
    for(uint16_t i = 0; i < w->nbody; i++) {
        const tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        if(!(b->flags & TT_F_AWAKE)) continue;
        if(tt_dot(b->vel, b->vel) > TT_QUIET_V * TT_QUIET_V) return false;
        if(b->w * b->w > TT_QUIET_W * TT_QUIET_W) return false;
    }
    return true;
}

static bool tt_topped_out(const tt_world_t* w) {
    for(uint16_t i = 0; i < w->nbody; i++) {
        const tt_body_t* b = &w->body[i];
        if(!(b->flags & TT_F_ALIVE) || (b->flags & TT_F_STATIC)) continue;
        /* lo is the FAT aabb, so add the margin back before comparing. There are no rows to test
         * at arbitrary angles - the criterion is purely geometric, and it is only ever sampled
         * once the world has stopped moving, so a piece that pokes over and topples back is
         * forgiven. */
        if(b->lo.y + TT_AABB_FAT < (float)TT_TOPOUT_Y) return true;
    }
    return false;
}

/* ============================================================================== controls ==== */

static void tt_controls_substep(void* ctx, float h) {
    tt_model_t* m = (tt_model_t*)ctx;
    if(m->active < 0) return;
    tt_body_t* a = &m->world->body[m->active];
    if(!(a->flags & TT_F_ALIVE)) return;

    /* rotation: drive toward a target angular rate, then consume the budget by the rotation that
     * ACTUALLY happened. Jammed against the stack the solver zeroes omega, the budget survives,
     * and the piece snaps round the instant it is free. */
    float w_want = tt_clampf(m->spin_budget / TT_SPIN_TAU, -TT_SPIN_W_MAX, TT_SPIN_W_MAX);
    const float dwmax = TT_SPIN_ALPHA * h;
    a->w += tt_clampf(w_want - a->w, -dwmax, dwmax);
    m->spin_budget -= a->w * h;

    float vx = 0.0f;
    if(m->hold_left) vx -= TT_MOVE_V;
    if(m->hold_right) vx += TT_MOVE_V;
    const float dvmax = TT_MOVE_ACCEL * h;
    a->vel.x += tt_clampf(vx - a->vel.x, -dvmax, dvmax);

    /* Drop speed is a terminal-velocity CAP, never a position write. 400 px/s at h = 5 ms is
     * 2 px per substep - half the 12 px minimum feature, and inside the speculative margin. */
    float cap = m->fall_v;
    if(m->soft) cap = TT_SOFT_V;
    if(m->hard_dropped) cap = TT_HARD_V;
    if(a->vel.y > cap) a->vel.y = cap;
}

static bool tt_active_settled(tt_model_t* m) {
    tt_body_t* a = &m->world->body[m->active];
    const bool slow = tt_dot(a->vel, a->vel) < TT_SETTLE_V * TT_SETTLE_V &&
                      a->w * a->w < TT_SETTLE_W * TT_SETTLE_W;

    /* The contact requirement is essential: at the apex of a bounce the piece is momentarily at
     * rest in mid-air and would otherwise lock there. */
    if(slow && a->contacts > 0)
        m->lock_ms += TT_STEP_MS;
    else
        m->lock_ms = 0;
    if(a->contacts > 0) m->touch_ms += TT_STEP_MS;

    const uint32_t need = m->hard_dropped ? TT_LOCK_HARD_MS : TT_LOCK_MS;
    return (m->lock_ms >= need) || (m->touch_ms >= TT_LOCK_MAX_MS);
}

/* ================================================================================ scoring ==== */

static const uint16_t tt_base_score[5] = {0u, 100u, 300u, 500u, 800u};

static void tt_score_clears(tt_model_t* m) {
    const int n = m->nbands;
    uint32_t add = (n <= 4) ? tt_base_score[n] : (800u + 300u * (uint32_t)(n - 4));
    add *= m->level;
    /* Neatness bonus: a row cleared at 97% is worth meaningfully more than one scraped at 80%.
     * This is the Not-Tetris-specific incentive to pack tightly rather than get lucky. */
    for(int k = 0; k < n; k++) {
        const int over = (int)m->band[k].pct - (int)TT_FILL_PCT;
        if(over > 0) add += (uint32_t)(over * 25);
    }
    m->score += add;
    m->lines += (uint32_t)n;
    m->level = 1u + m->lines / TT_LEVEL_LINES;
    if(m->level > TT_LEVEL_MAX) m->level = TT_LEVEL_MAX;
    m->fall_v = TT_FALL_V0 + TT_FALL_VSTEP * (float)(m->level - 1u);

    m->clear_count = (uint8_t)n;
    m->clear_flash_ms = 600u;
    m->banner_ms = 1200u;
    if(n >= 4)
        snprintf(m->banner, sizeof(m->banner), "TETRIS +%u", (unsigned)add);
    else
        snprintf(m->banner, sizeof(m->banner), "+%u", (unsigned)add);
}

/* ============================================================================== lifecycle ==== */

bool tt_game_init(tt_model_t* m,
                  void* const* blocks,
                  const uint32_t* sizes,
                  uint8_t nblocks,
                  uint16_t body_cap,
                  uint16_t mf_cap) {
    m->body_cap = body_cap;
    m->mf_cap = mf_cap;
    m->world = tt_phys_init(blocks, sizes, nblocks, body_cap, mf_cap);
    /* Report the failure, do NOT latch it. One rung failing is the ladder working as designed —
     * the caller frees these blocks and tries a smaller cap. Setting m->pool_failed here made that
     * recovery invisible: tt_render() tests `pool_failed || !world` and nothing ever cleared the
     * flag, so a run that failed at rung 0 and then succeeded at rung 1 built a perfectly good
     * world and still drew "out of memory - cannot start" over it. Only tt_arena_claim() running
     * out of rungs is a real failure, and app_init() already sets the flag there. */
    if(!m->world) return false;
    /* Only now do we own them — a failed init leaves the caller free to try a smaller rung
     * with the blocks it still holds. */
    for(uint8_t i = 0; i < nblocks; i++) m->pool[i] = blocks[i];
    m->nblocks = nblocks;
    return true;
}

void tt_game_free(tt_model_t* m) {
    for(uint8_t i = 0; i < m->nblocks; i++) {
        if(m->pool[i]) os_free(m->pool[i]);
        m->pool[i] = NULL;
    }
    m->nblocks = 0;
    m->world = NULL;
}

void tt_game_reset(tt_model_t* m) {
    tt_world_t* w = m->world;
    tt_phys_clear(w);

    const float x0 = (float)TT_WELL_X0, x1 = (float)TT_WELL_X1, y1 = (float)TT_WELL_Y1;
    /* The physical walls run far above the drawn ones so a piece is contained from the moment it
     * spawns, while the well still reads as open-topped. */
    tt_make_static(w, x0 - 24.0f, -600.0f, x0, y1, TT_MU_WALL);
    tt_make_static(w, x1, -600.0f, x1 + 24.0f, y1, TT_MU_WALL);
    tt_make_static(w, x0 - 24.0f, y1, x1 + 24.0f, y1 + 24.0f, TT_MU_FLOOR);

    m->active = TT_NO_BODY;
    m->bag_n = 0;
    m->next_piece = tt_bag_next(m);
    m->score = 0;
    m->lines = 0;
    m->level = 1;
    m->fall_v = TT_FALL_V0;
    m->nbands = 0;
    m->acc_ms = 0;
    m->lock_ms = m->touch_ms = m->settle_ms = m->quiet_ms = m->flash_ms = m->over_ms = 0;
    m->hold_left = m->hold_right = m->soft = m->hard_dropped = false;
    m->spin_budget = 0.0f;
    m->banner[0] = 0;
    m->banner_ms = 0;
    m->clear_flash_ms = 0;
    m->clear_count = 0;
    m->state = TT_ST_SPAWN;

    memset(&m->slice, 0, sizeof(m->slice));
    tt_phys_snapshot(w);
}

/* ================================================================================== step ===== */

static void tt_game_step(tt_model_t* m) {
    tt_world_t* w = m->world;

    switch(m->state) {
    case TT_ST_TITLE:
    case TT_ST_PAUSED:
    case TT_ST_GAMEOVER:
        break;

    case TT_ST_SPAWN:
        if(!tt_spawn(m)) {
            m->over_ms = 0;
            m->state = TT_ST_TOPOUT;
        } else {
            m->state = TT_ST_FALLING;
        }
        break;

    case TT_ST_FALLING:
        tt_phys_step(w, TT_STEP_DT, tt_controls_substep, m);
        if(m->active >= 0 && !(w->body[m->active].flags & TT_F_ALIVE)) {
            m->active = TT_NO_BODY; /* validated away: just move on */
            m->settle_ms = 0;
            m->quiet_ms = 0;
            m->state = TT_ST_SETTLING;
        } else if(tt_active_settled(m)) {
            w->body[m->active].flags &= (uint8_t)~(TT_F_ACTIVE | TT_F_NOSLEEP);
            m->active = TT_NO_BODY;
            m->soft = false;
            m->settle_ms = 0;
            m->quiet_ms = 0;
            m->state = TT_ST_SETTLING;
        }
        break;

    case TT_ST_SETTLING:
        tt_phys_step(w, TT_STEP_DT, NULL, NULL);
        m->settle_ms += TT_STEP_MS;
        m->quiet_ms = tt_world_quiet(w) ? (m->quiet_ms + TT_STEP_MS) : 0u;
        if(m->quiet_ms >= TT_QUIET_MS || m->settle_ms > TT_WORLD_SETTLE_MAX_MS) {
            tt_cov_scan(w, &m->slice);
            m->nbands = tt_scan_bands(&m->slice, m->band);
            if(m->nbands > 0) {
                m->flash_ms = 0;
                m->state = TT_ST_CLEAR_FLASH;
            } else if(tt_topped_out(w)) {
                m->over_ms = 0;
                m->state = TT_ST_TOPOUT;
            } else {
                m->state = TT_ST_SPAWN;
            }
        }
        break;

    case TT_ST_CLEAR_FLASH:
        /* Physics is paused here on purpose: it is what makes the flash land on exactly the
         * geometry that is about to be removed. */
        m->flash_ms += TT_STEP_MS;
        if(m->flash_ms >= TT_CLEAR_FLASH_MS) {
            tt_score_clears(m); /* score before the geometry goes */
            tt_apply_clears(w, &m->slice, m->band, m->nbands);
            m->nbands = 0;
            m->settle_ms = 0;
            m->quiet_ms = 0;
            m->state = TT_ST_SETTLING; /* cascade: re-scan once it settles again */
        }
        break;

    case TT_ST_TOPOUT:
        tt_phys_step(w, TT_STEP_DT, NULL, NULL);
        m->over_ms += TT_STEP_MS;
        if(m->over_ms > TT_TOPOUT_ANIM_MS) m->state = TT_ST_GAMEOVER;
        break;

    default:
        break;
    }
}

void tt_game_poll_encoder(tt_model_t* m) {
    /* Read ONCE per tick and only here. The accumulator is consumed on read, so reading it from
     * on_input as well would halve sensitivity nondeterministically. */
    const int32_t d = os_controls_encoder_get_delta();
    if(d == 0) return;
    if(m->state != TT_ST_FALLING) return;
    m->spin_budget += (float)d * TT_ENC_RAD_PER_DETENT;
    m->spin_budget = tt_clampf(m->spin_budget, -TT_SPIN_BUDGET_MAX, TT_SPIN_BUDGET_MAX);
}

void tt_game_tick(tt_model_t* m, uint32_t now) {
    if(!m->world) return;

    uint32_t dt = now - m->last_ms; /* unsigned: wrap-safe at 49.7 days */
    m->last_ms = now;
    if(dt > TT_MAX_TICK_MS) dt = TT_MAX_TICK_MS;
    m->acc_ms += dt;

    tt_game_poll_encoder(m);

    int steps = 0;
    while(m->acc_ms >= TT_STEP_MS && steps < TT_MAX_STEPS_PER_TICK) {
        m->acc_ms -= TT_STEP_MS;
        tt_game_step(m);
        steps++;
    }
    if(steps == TT_MAX_STEPS_PER_TICK) m->acc_ms = 0; /* shed the backlog, never spiral */

    if(m->banner_ms) m->banner_ms = (m->banner_ms > dt) ? (m->banner_ms - dt) : 0u;
    if(m->clear_flash_ms) m->clear_flash_ms = (m->clear_flash_ms > dt) ? (m->clear_flash_ms - dt) : 0u;

    /* Coverage is cheap enough to run every frame so the HUD meter is honest, but it is only
     * ACTED on when a piece locks (see TT_ST_SETTLING). */
    if(m->state == TT_ST_FALLING) tt_cov_scan(m->world, &m->slice);

    if(steps > 0) tt_phys_snapshot(m->world);
}

/* ================================================================================= input ===== */

void tt_game_input(tt_model_t* m, uint8_t btn, KeyStateEnum st) {
    if(btn == 3 && st == KEY_STATE_HOLD) {
        os_app_exit();
        return;
    }

    if(m->state == TT_ST_GAMEOVER || m->state == TT_ST_TITLE) {
        if(btn == 1 && st == KEY_STATE_PRESSED) tt_game_reset(m);
        return;
    }

    if(btn == 4 && st == KEY_STATE_PRESSED) {
        if(m->state == TT_ST_PAUSED) {
            m->state = m->prev_state;
            m->last_ms = os_tick_get();
            m->acc_ms = 0;
        } else {
            m->prev_state = m->state;
            m->state = TT_ST_PAUSED;
        }
        return;
    }
    if(m->state == TT_ST_PAUSED) return;

    switch(btn) {
    case 0:
        if(st == KEY_STATE_PRESSED || st == KEY_STATE_HOLD) {
            m->hold_left = true;
            /* An immediate nudge, so a tap shorter than one 40 ms tick still does something. */
            if(st == KEY_STATE_PRESSED && m->active >= 0)
                m->world->body[m->active].vel.x =
                    fmaxf(m->world->body[m->active].vel.x - TT_MOVE_TAP, -TT_MOVE_V);
        } else {
            m->hold_left = false;
        }
        break;

    case 2:
        if(st == KEY_STATE_PRESSED || st == KEY_STATE_HOLD) {
            m->hold_right = true;
            if(st == KEY_STATE_PRESSED && m->active >= 0)
                m->world->body[m->active].vel.x =
                    fminf(m->world->body[m->active].vel.x + TT_MOVE_TAP, TT_MOVE_V);
        } else {
            m->hold_right = false;
        }
        break;

    case 1:
        /* The firmware sends PRESSED at once and HOLD only after ~500 ms, so "press = hard, hold
         * = soft" cannot be read literally - a hard drop on PRESSED makes soft drop unreachable.
         * Soft drop applies while held; a release inside TT_TAP_MS also commits a hard drop. */
        if(st == KEY_STATE_PRESSED) {
            m->soft = true;
            m->btn2_t = os_tick_get();
        } else if(st == KEY_STATE_HOLD) {
            m->soft = true;
        } else {
            m->soft = false;
            if(os_tick_get() - m->btn2_t < TT_TAP_MS) {
                m->hard_dropped = true;
                if(m->active >= 0) m->world->body[m->active].vel.y = TT_HARD_V;
            }
        }
        break;

    default:
        break;
    }
}

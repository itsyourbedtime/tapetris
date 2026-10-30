/*
 * tapetris - Tetris where the pieces are rigid bodies.
 *
 * Free 360-degree rotation, physical stacking, and rows cleared by cutting the band out of the
 * geometry rather than deleting cells. A Not Tetris 2 for tape!.
 *
 * Build:   ./sdk/tapp-build "$PWD" tapetris.tapp     (NOT `.` - basename "." is ".")
 * Deploy:  copy tapetris.tapp into the "apps" folder on the SD card via disk mode.
 */

#include "tt_game.h"
#include "tt_phys.h"
#include "tt_render.h"
#include "tt_util.h"

typedef struct {
    tt_model_t game;
} app_model_t;

/* ============================================================================== storage ===== */

#if TT_ENABLE_HISCORE
static void tt_load_hiscore(tt_model_t* m) {
    uint32_t rec[2] = {0u, 0u};
    if(!storage_file_exists(TT_SCORE_PATH)) return;
    if(storage_read_file(TT_SCORE_PATH, rec, sizeof(rec)) != sizeof(rec)) return;
    if(rec[0] != TT_SCORE_MAGIC) return;
    m->hiscore = rec[1];
}

static void tt_save_hiscore(const tt_model_t* m) {
    const uint32_t rec[2] = {TT_SCORE_MAGIC, m->hiscore};
    storage_mkdir(TT_SCORE_DIR);
    storage_write_file(TT_SCORE_PATH, rec, sizeof(rec));
}
#endif

/* =============================================================================== memory ===== */

/* PSRAM is fine, and refusing it is what broke startup.
 *
 * This used to reject any block at or above 0x90000000 to keep the physics arrays out of
 * uncached memory. That premise was wrong: the heap's PSRAM region is mapped write-back
 * write-allocate (TEX=001 C=1 B=1 S=0) and is ACTUALLY CACHED, so an array there costs a
 * cache miss, not the ~200 ns/instruction the guard was written against. That penalty
 * belongs to the tapp IMAGE window at 0x90600000 — a different MPU region, and never
 * something os_malloc hands back.
 *
 * The guard was also self-defeating. The firmware heap has exactly two regions, OS_RAM
 * (512 KB) and PSRAM (6 MB); since DTCM left that list, every small allocation in the
 * system comes out of OS_RAM, so by the time this tapp loads the first-fit walk reaches
 * PSRAM almost immediately. Refusing it failed all four rungs and showed "out of memory"
 * on a device with megabytes free. */
static void* tt_alloc_zeroed(uint32_t bytes) {
    void* p = os_malloc(bytes);
    if(!p) return NULL;
    memset(p, 0, bytes);
    return p;
}

/* Try each rung of the fallback ladder, largest caps first, and stop at the first one the heap
 * can actually serve. Each rung asks for several SMALL blocks rather than one big one: the tapp
 * heap fragments, so a 40 KB request can fail while five 8 KB ones succeed. A rung that cannot be
 * filled completely releases what it did get before the next one is tried. */
static bool tt_arena_claim(tt_model_t* m) {
    static const uint16_t rung_bodies[TT_RUNGS] = TT_RUNG_BODIES;
    static const uint16_t rung_contacts[TT_RUNGS] = TT_RUNG_CONTACTS;

    for(int r = 0; r < TT_RUNGS; r++) {
        const uint16_t bc = rung_bodies[r], mc = rung_contacts[r];
        uint32_t sizes[TT_MAX_BLOCKS];
        void* blocks[TT_MAX_BLOCKS] = {0};
        const uint8_t n = tt_phys_block_plan(bc, mc, sizes);

        uint8_t got = 0;
        while(got < n && (blocks[got] = tt_alloc_zeroed(sizes[got])) != NULL) got++;

        if(got == n && tt_game_init(m, blocks, sizes, n, bc, mc)) return true;

        for(uint8_t i = 0; i < got; i++) os_free(blocks[i]);
    }
    return false;
}


/* ============================================================================ lifecycle ===== */

static bool app_init(os_app_t* app) {
    app_model_t* a = os_app_get_model(app);
    tt_model_t* m = &a->game;

    m->rng = os_tick_get() * 2654435761u + 0x9E3779B9u;
    if(m->rng == 0u) m->rng = 0x1234567u;
    m->last_ms = os_tick_get();

    ui_hints_show(false);
    ui_statusbar_show(false);

    if(!tt_arena_claim(m)) {
        m->pool_failed = true;
        return true; /* still load: redraw shows why rather than vanishing */
    }

#if TT_ENABLE_HISCORE
    tt_load_hiscore(m);
#endif
    tt_game_reset(m);
    return true;
}

static void app_deinit(os_app_t* app) {
    app_model_t* a = os_app_get_model(app);
    tt_model_t* m = &a->game;

    if(m->score > m->hiscore) m->hiscore = m->score;
#if TT_ENABLE_HISCORE
    if(!m->pool_failed) tt_save_hiscore(m);
#endif

    tt_game_free(m);

    /* Nothing restores what we took: put the device back the way we found it. */
    ui_statusbar_show(true);
    ui_hints_show(true);
}

static bool app_tick(os_app_t* app) {
    app_model_t* a = os_app_get_model(app);
    tt_model_t* m = &a->game;
    if(m->pool_failed || !m->world) return true;

    tt_game_tick(m, os_tick_get());
    if(m->score > m->hiscore) m->hiscore = m->score;
    return true;
}

static void app_redraw(gfx_t* gfx, const os_app_t* app) {
    const app_model_t* a = os_app_get_model(app);
    tt_render(gfx, &a->game);
}

static void app_input(os_app_t* app, uint8_t btn, KeyStateEnum state) {
    app_model_t* a = os_app_get_model(app);
    /* btn 5 is the encoder pseudo-event: its state is meaningless, and the delta is polled once
     * per tick in tt_game_poll_encoder. Never branch on it here. */
    if(btn == 5) return;
    if(a->game.pool_failed || !a->game.world) {
        if(btn == 3 && state == KEY_STATE_HOLD) os_app_exit();
        return;
    }
    tt_game_input(&a->game, btn, state);
}

static void app_resume(os_app_t* app) {
    app_model_t* a = os_app_get_model(app);
    /* A system modal drains the encoder accumulator; throw away whatever is pending so coming back
     * from one cannot fling the piece. */
    (void)os_controls_encoder_get_delta();
    a->game.spin_budget = 0.0f;
    a->game.last_ms = os_tick_get();
    a->game.acc_ms = 0;
}

TAPP_APP("tapetris", app_model_t, app_init, app_deinit,
         .redraw = app_redraw,
         .tick = app_tick,
         .on_input = app_input,
         .on_resume = app_resume);

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

#define TT_PSRAM_BASE 0x90000000u

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

static void* tt_alloc_cached(uint32_t bytes) {
    void* p = os_malloc(bytes);
    if(!p) return NULL;
    if((uint32_t)(size_t)p >= TT_PSRAM_BASE) {
        os_free(p);
        return NULL;
    }
    memset(p, 0, bytes);
    return p;
}


/* ============================================================================ lifecycle ===== */

static bool app_init(os_app_t* app, va_list args) {
    (void)args;
    app_model_t* a = os_app_get_model(app);
    tt_model_t* m = &a->game;

    m->rng = os_tick_get() * 2654435761u + 0x9E3779B9u;
    if(m->rng == 0u) m->rng = 0x1234567u;
    m->last_ms = os_tick_get();

    ui_hints_show(false);
    ui_statusbar_show(false);

    uint16_t bc = TT_MAX_BODIES, mc = TT_MAX_CONTACTS;
    uint32_t bytes = tt_phys_bytes(bc, mc);
    void* pool = tt_alloc_cached(bytes);
    if(!pool) {
        bc = TT_MIN_BODIES;
        mc = TT_MIN_CONTACTS;
        bytes = tt_phys_bytes(bc, mc);
        pool = tt_alloc_cached(bytes);
    }
    if(!pool || !tt_game_init(m, pool, bytes, bc, mc)) {
        m->pool_failed = true;
        if(pool) os_free(pool);
        m->pool = NULL;
        return true; /* still load: redraw shows why rather than vanishing */
    }

#if TT_ENABLE_HISCORE
    tt_load_hiscore(m);
#endif
    tt_game_reset(m);
    return true;
}

static bool app_deinit(os_app_t* app) {
    app_model_t* a = os_app_get_model(app);
    tt_model_t* m = &a->game;

    if(m->score > m->hiscore) m->hiscore = m->score;
#if TT_ENABLE_HISCORE
    if(!m->pool_failed) tt_save_hiscore(m);
#endif

    if(m->pool) {
        os_free(m->pool);
        m->pool = NULL;
        m->world = NULL;
    }

    /* Nothing restores what we took: put the device back the way we found it. */
    ui_statusbar_show(true);
    ui_hints_show(true);
    return true;
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

static bool app_resume(os_app_t* app) {
    app_model_t* a = os_app_get_model(app);
    /* A system modal drains the encoder accumulator; throw away whatever is pending so coming back
     * from one cannot fling the piece. */
    (void)os_controls_encoder_get_delta();
    a->game.spin_budget = 0.0f;
    a->game.last_ms = os_tick_get();
    a->game.acc_ms = 0;
    return true;
}

/* Both of these MUST use static initialisers - the loader reads them before any code runs. */
static os_app_data_t app_data = {
    .model = NULL,
    .model_size = sizeof(app_model_t),
    .init = app_init,
    .deinit = app_deinit,
};

static os_app_t app_descriptor = {
    .name = "tapetris",
    .type = AppFullscreenType,
    .data = &app_data,
    .redraw = (os_app_redraw)app_redraw,
    .tick = app_tick,
    .on_input = app_input,
    .on_resume = app_resume,
};

os_app_t* tapp_get_descriptor(void) {
    return &app_descriptor;
}

#ifndef TT_GAME_H
#define TT_GAME_H

#include "tt_types.h"
#include "tt_phys.h" /* TT_MAX_BLOCKS — the arena is cut into blocks, and we hold them */
#include "tt_slice.h"

enum {
    TT_ST_TITLE = 0,
    TT_ST_SPAWN,
    TT_ST_FALLING,
    TT_ST_SETTLING,
    TT_ST_CLEAR_FLASH,
    TT_ST_TOPOUT,
    TT_ST_GAMEOVER,
    TT_ST_PAUSED,
};

#define TT_NO_BODY (-1)
#define TT_NPIECE 7

typedef struct {
    tt_world_t* world;
    /* The os_malloc'd blocks backing `world`. Several small ones rather than one big
     * one: the tapp heap fragments, and the largest single block it can serve is well
     * under the arena total — tt_phys_block_plan() says how to cut it. */
    void* pool[TT_MAX_BLOCKS];
    uint8_t nblocks;
    uint16_t body_cap, mf_cap;
    bool pool_failed;

    tt_slice_ctx_t slice;
    tt_band_t band[TT_MAX_BANDS];
    int nbands;

    uint8_t state;
    uint8_t prev_state; /* what to return to when unpausing */

    uint32_t last_ms, acc_ms;
    uint32_t lock_ms, touch_ms, settle_ms, quiet_ms, flash_ms, over_ms;

    int32_t active;
    uint8_t next_piece;
    uint8_t bag[TT_NPIECE];
    uint8_t bag_n;
    uint32_t rng;

    uint32_t score, hiscore, lines, level;
    float fall_v;

    /* input state, written by on_input and tick, read by the substep servo */
    bool hold_left, hold_right, soft, hard_dropped;
    uint32_t btn2_t;
    float spin_budget;

    /* feedback */
    char banner[20];
    uint32_t banner_ms;
    uint32_t clear_flash_ms; /* LED decay timer */
    uint8_t clear_count;
    uint8_t topped_body_hint;
} tt_model_t;

/* Takes OWNERSHIP of `blocks` on success: they are recorded in m->pool for tt_game_free().
 * On failure it frees nothing — the caller still owns what it allocated. */
bool tt_game_init(tt_model_t* m,
                  void* const* blocks,
                  const uint32_t* sizes,
                  uint8_t nblocks,
                  uint16_t body_cap,
                  uint16_t mf_cap);
/* Release the arena. Safe to call twice, and on a model that never came up. */
void tt_game_free(tt_model_t* m);
void tt_game_reset(tt_model_t* m);
void tt_game_tick(tt_model_t* m, uint32_t now);
void tt_game_input(tt_model_t* m, uint8_t btn, KeyStateEnum st);
void tt_game_poll_encoder(tt_model_t* m);

/* Piece geometry, shared with the NEXT preview in the renderer. Boxes are given in HALF cells. */
typedef struct {
    int8_t nb;
    int8_t b[2][4]; /* {cx2, cy2, hw2, hh2} */
} tt_piece_t;

extern const tt_piece_t tt_pieces[TT_NPIECE];

#endif /* TT_GAME_H */

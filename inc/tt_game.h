#ifndef TT_GAME_H
#define TT_GAME_H

#include "tt_types.h"
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
    void* pool; /* the single os_malloc'd block backing `world` */
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

bool tt_game_init(tt_model_t* m, void* pool, uint32_t bytes, uint16_t body_cap, uint16_t mf_cap);
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

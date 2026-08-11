/*
 * tt_config.h - every tunable in one place.
 *
 * World units are SCREEN PIXELS, +y down. Physics, coverage sampling and rendering all share one
 * coordinate system; there is no camera and no scale factor.
 */
#ifndef TT_CONFIG_H
#define TT_CONFIG_H

/* ---- well geometry ---------------------------------------------------------------------- */

#define TT_CELL 12 /* px per tetromino cell */
#define TT_COLS 10
#define TT_ROWS 19

#define TT_WELL_X0 140 /* interior, inclusive */
#define TT_WELL_X1 260 /* interior, EXCLUSIVE  -> 120 px = 10 cells */
#define TT_WELL_Y0 6 /* interior top, inclusive */
#define TT_WELL_Y1 234 /* interior floor, EXCLUSIVE -> 228 px = 19 cells */

#define TT_WELL_CX 200
#define TT_TOPOUT_Y 42 /* = Y1 - CELL*16, so 16 playable rows */

/* Drawing clips from y=0, not from the well top: a piece spawns partly above the wall tops and
 * must still be visible. The physical walls extend far above the drawn ones. */
#define TT_CLIP_Y0 0
#define TT_CLIP_Y1 TT_WELL_Y1

#define TT_SPAWN_X 200.0f
#define TT_SPAWN_Y 22.0f

/* ---- pools ------------------------------------------------------------------------------ */

#define TT_MAX_BODIES 64
#define TT_BODY_MAXPOLY 4
#define TT_POLY_MAXV 6 /* proven bound: rect + horizontal cuts only ever has <= 6 edge dirs */
#define TT_CLIP_SCRATCH 8 /* transient, pre-dedup */
#define TT_MAX_CONTACTS 192
#define TT_MAX_PAIRS 512
#define TT_HASH_BUCKETS 256 /* power of two */
#define TT_SPLIT_MAXPOLY 16

/* Halved caps, retried when the full pool cannot be served from cached RAM. */
#define TT_MIN_BODIES 32
#define TT_MIN_CONTACTS 96

/* ---- integration -------------------------------------------------------------------------- */

#define TT_STEP_MS 20u /* fixed physics step */
#define TT_STEP_DT 0.020f
#define TT_SUBSTEPS 4 /* h = 5 ms */
#define TT_MAX_STEPS_PER_TICK 4
#define TT_MAX_TICK_MS 200u /* clamp after a stall; never fast-forward */

#define TT_GRAVITY 620.0f /* px/s^2 */
#define TT_LIN_DAMP 0.20f
#define TT_ANG_DAMP 0.60f
#define TT_ACTIVE_ANG_DAMP 1.50f

/*
 * Global terminal velocity, and it is a CORRECTNESS constant, not a feel one. Narrowphase runs
 * once per step, so a body must not travel further between collision passes than the speculative
 * margin, or the contact is never created and it tunnels straight through the stack. At 300 px/s
 * and a 20 ms step that is 6 px, inside TT_SPECULATIVE below. Anything that raises this must
 * raise TT_SPECULATIVE and TT_AABB_FAT with it.
 *
 * TT_MAX_OMEGA is the same rule for rotation: a vertex 24 px out at 12 rad/s sweeps 288 px/s.
 */
#define TT_MAX_SPEED 300.0f /* px/s */
#define TT_MAX_OMEGA 12.0f /* rad/s */

/* ---- collision ---------------------------------------------------------------------------- */

#define TT_SLOP 0.30f /* allowed penetration, px */
#define TT_SPECULATIVE 7.0f /* > max travel per step (6 px at TT_MAX_SPEED) */
#define TT_AABB_FAT 9.0f /* > speculative, so the pair exists before the manifold does */
#define TT_MIN_EDGE2 0.0025f /* (0.05 px)^2 - weld threshold */

#define TT_CONTACT_HZ 30.0f
#define TT_CONTACT_ZETA 10.0f
#define TT_MAX_BIAS_VEL 150.0f /* px/s cap on positional push-out */
#define TT_RESTITUTION 0.0f /* Not Tetris blocks are inert */
#define TT_REST_THRESH 60.0f /* px/s approach below which no bounce is applied */

#define TT_MU_BLOCK 0.50f
#define TT_MU_WALL 0.40f
#define TT_MU_FLOOR 0.60f

/* ---- sleeping ----------------------------------------------------------------------------- */

#define TT_SLEEP_LIN 3.0f /* px/s  */
#define TT_SLEEP_ANG 0.12f /* rad/s */
#define TT_SLEEP_TIME 0.45f /* s     */

/* ---- geometry hygiene --------------------------------------------------------------------- */

#define TT_MIN_POLY_AREA 8.0f /* px^2 - drop slivers produced by a cut  */
#define TT_MIN_BODY_AREA 20.0f /* px^2 - drop the whole body            */
#define TT_TOUCH_EPS 0.5f /* SAT tolerance for the connectivity pass */

/* ---- line clear --------------------------------------------------------------------------- */

#define TT_SCOLS 30 /* 120 px / 4 : coverage sample columns */
#define TT_SROWS (TT_ROWS * 3) /* 3 sample rows per band               */
#define TT_SAMP_X0 (TT_WELL_X0 + 2)
#define TT_SAMP_Y0 (TT_WELL_Y0 + 2)
#define TT_SAMP_STEP 4

#define TT_FILL_PCT 80u /* percent of a band's 90 samples */
#define TT_CLEAR_FLASH_MS 220u

/* ---- game --------------------------------------------------------------------------------- */

#define TT_SETTLE_V 14.0f /* px/s  */
#define TT_SETTLE_W 0.35f /* rad/s */
#define TT_LOCK_MS 400u
#define TT_LOCK_HARD_MS 100u
#define TT_LOCK_MAX_MS 3000u
#define TT_QUIET_V 10.0f
#define TT_QUIET_W 0.30f
#define TT_QUIET_MS 250u
#define TT_WORLD_SETTLE_MAX_MS 1500u
#define TT_TOPOUT_ANIM_MS 900u

#define TT_LEVEL_LINES 8u
#define TT_LEVEL_MAX 15u
/* Terminal velocity of the active piece, i.e. the Tetris "fall speed". At level 1 the 228 px well
 * takes ~14 s to cross; level 15 is ~2.6 s. Raise TT_FALL_V0 to ~55 for a brisker level 1. */
#define TT_FALL_V0 16.0f /* px/s at level 1 */
#define TT_FALL_VSTEP 5.0f

/* ---- control feel ------------------------------------------------------------------------- */

#define TT_ENC_RAD_PER_DETENT 0.1f /* 15 deg -> 6 detents ~ a classic 90 deg turn */
#define TT_SPIN_BUDGET_MAX 1.60f /* rad, ~90 deg of debt */
#define TT_SPIN_TAU 0.10f /* s, first-order drain */
#define TT_SPIN_W_MAX 9.0f /* rad/s */
#define TT_SPIN_ALPHA 60.0f /* rad/s^2 */

#define TT_MOVE_V 70.0f /* px/s target while held */
#define TT_MOVE_ACCEL 900.0f
#define TT_MOVE_TAP 55.0f /* instant nudge on press */

#define TT_TAP_MS 220u
#define TT_SOFT_V 170.0f
#define TT_HARD_V 290.0f /* must stay under TT_MAX_SPEED, which is the anti-tunnelling bound */

/* ---- storage ------------------------------------------------------------------------------ */

/*
 * OFF by default, and deliberately so: this is the only thing tapetris does at startup that can
 * block. storage_file_exists/read go through FatFS onto the SD DMA path, and a tapp's init() runs
 * on the controls task - if that path stalls, the whole device appears to hang before the game
 * ever draws. The game itself needs no I/O at all, so the safe default is to touch none.
 *
 * Set to 1 to persist the best score across runs.
 */
#define TT_ENABLE_HISCORE 0

#define TT_SCORE_DIR "/tapetris"
#define TT_SCORE_PATH "/tapetris/hiscore.bin"
#define TT_SCORE_MAGIC 0x54544831u /* "TTH1" */

#endif /* TT_CONFIG_H */

#ifndef TT_SLICE_H
#define TT_SLICE_H

#include "tt_types.h"

#define TT_MAX_BANDS 6

typedef struct {
    float y_top, y_bot; /* the band to remove, world y */
    uint8_t pct; /* measured coverage, 0..100 */
} tt_band_t;

/* Scratch the slice path needs. Lives in the model (heap): a body of 4 polygons cut by several
 * bands yields up to TT_SPLIT_MAXPOLY fragments, and the tapp manifest gives us a 4 KB stack. */
typedef struct {
    uint32_t cov[TT_SROWS]; /* one bit per 4x4 px sample, TT_SCOLS bits per row */
    uint8_t row_pct[TT_ROWS];
    tt_wpoly_t frag[TT_SPLIT_MAXPOLY];
    tt_wpoly_t work[TT_SPLIT_MAXPOLY];
    uint8_t group[TT_SPLIT_MAXPOLY];
    uint8_t parent[TT_SPLIT_MAXPOLY];
} tt_slice_ctx_t;

/* Rasterise every alive dynamic body into the sample grid and fold it down to per-row percents.
 * Cheap enough (~0.03 ms) to run every frame for the HUD meter. */
void tt_cov_scan(const tt_world_t* w, tt_slice_ctx_t* sc);

/* Rows at or over TT_FILL_PCT, bottom-up. Returns the count (<= TT_MAX_BANDS). */
int tt_scan_bands(const tt_slice_ctx_t* sc, tt_band_t* out);

/* Cut every band out of every overlapping polygon, rebuild mass properties, split bodies the cut
 * disconnected, and wake the world. Nothing shifts down - the material above simply loses its
 * support. */
void tt_apply_clears(tt_world_t* w, tt_slice_ctx_t* sc, const tt_band_t* band, int nbands);

#endif /* TT_SLICE_H */

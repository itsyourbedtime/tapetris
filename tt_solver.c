/*
 * tt_solver.c - warm-started sequential impulses with soft (spring-damper) contact constraints.
 *
 * Soft constraints instead of raw Baumgarte because they converge to a stable stack in far fewer
 * iterations, which is what a 200 MHz budget demands. Normal is solved before friction so the
 * friction bound uses the current normal impulse.
 */

#include "tt_phys.h"
#include "tt_util.h"

void tt_solver_prepare(tt_world_t* w, float h) {
    (void)h;
    for(uint16_t m = 0; m < w->mf_count; m++) {
        tt_manifold_t* c = &w->mf[m];
        tt_body_t* A = &w->body[c->ba];
        tt_body_t* B = &w->body[c->bb];
        const vec2 n = c->n;
        const vec2 t = tt_v(n.y, -n.x);

        const tt_pc_t* pc = tt_pc_lookup(w, ((uint32_t)c->sa << 16) | (uint32_t)c->sb);

        for(int k = 0; k < c->count; k++) {
            tt_cpoint_t* p = &c->pt[k];
            const vec2 wp = p->ra; /* tt_collide_polys left the WORLD contact point here */
            const float sep = p->sep0;

            const vec2 ra = tt_sub(wp, A->p);
            const vec2 rb = tt_sub(wp, B->p);
            p->ra = ra;
            p->rb = rb;

            const float rnA = tt_cross(ra, n), rnB = tt_cross(rb, n);
            const float kn = A->im + B->im + A->ii * rnA * rnA + B->ii * rnB * rnB;
            p->nmass = (kn > 1.0e-9f) ? (1.0f / kn) : 0.0f;

            const float rtA = tt_cross(ra, t), rtB = tt_cross(rb, t);
            const float kt = A->im + B->im + A->ii * rtA * rtA + B->ii * rtB * rtB;
            p->tmass = (kt > 1.0e-9f) ? (1.0f / kt) : 0.0f;

            /* Separation base. Per substep, sep = sep0 + dot((dpB - dpA) + (rb' - ra'), n), with
             * ra'/rb' the anchors rotated by that substep's accumulated rotation delta. At prepare
             * dp = 0 and dq = identity, so this reproduces sep exactly. */
            p->sep0 = sep - tt_dot(tt_sub(rb, ra), n);

            const vec2 dv = tt_sub(tt_add(B->vel, tt_wxr(B->w, rb)), tt_add(A->vel, tt_wxr(A->w, ra)));
            p->relv0 = tt_dot(dv, n);
            p->maxni = 0.0f;

            p->nimp = 0.0f;
            p->timp = 0.0f;
            if(pc) {
                for(int j = 0; j < pc->count; j++) {
                    if(pc->id[j] != p->id) continue;
                    p->nimp = pc->nimp[j];
                    p->timp = pc->timp[j];
                    break;
                }
            }
        }
    }
}

void tt_solver_warm_start(tt_world_t* w) {
    for(uint16_t m = 0; m < w->mf_count; m++) {
        tt_manifold_t* c = &w->mf[m];
        tt_body_t* A = &w->body[c->ba];
        tt_body_t* B = &w->body[c->bb];
        const vec2 n = c->n;
        const vec2 t = tt_v(n.y, -n.x);

        for(int k = 0; k < c->count; k++) {
            const tt_cpoint_t* p = &c->pt[k];
            const vec2 ra = tt_rot(A->dq, p->ra);
            const vec2 rb = tt_rot(B->dq, p->rb);
            const vec2 P = tt_add(tt_scale(n, p->nimp), tt_scale(t, p->timp));

            A->vel = tt_sub(A->vel, tt_scale(P, A->im));
            A->w -= A->ii * tt_cross(ra, P);
            B->vel = tt_add(B->vel, tt_scale(P, B->im));
            B->w += B->ii * tt_cross(rb, P);
        }
    }
}

void tt_solver_solve(tt_world_t* w, float inv_h, bool use_bias) {
    const float bias_rate = w->bias_rate;
    const float max_bias = w->max_bias;
    const float sms = w->mass_scale;
    const float sis = w->impulse_scale;

    for(uint16_t m = 0; m < w->mf_count; m++) {
        tt_manifold_t* c = &w->mf[m];
        tt_body_t* A = &w->body[c->ba];
        tt_body_t* B = &w->body[c->bb];

        const float nx = c->n.x, ny = c->n.y;
        const float tx = ny, ty = -nx;

        float vAx = A->vel.x, vAy = A->vel.y, wA = A->w;
        float vBx = B->vel.x, vBy = B->vel.y, wB = B->w;
        const float imA = A->im, iIA = A->ii;
        const float imB = B->im, iIB = B->ii;
        const rot2 dqA = A->dq, dqB = B->dq;
        const float dpx = B->dp.x - A->dp.x, dpy = B->dp.y - A->dp.y;

        float raX[2], raY[2], rbX[2], rbY[2];
        const int cnt = c->count;

        for(int k = 0; k < cnt; k++) {
            const float ax = c->pt[k].ra.x, ay = c->pt[k].ra.y;
            const float bx = c->pt[k].rb.x, by = c->pt[k].rb.y;
            const float rax = dqA.c * ax - dqA.s * ay, ray = dqA.s * ax + dqA.c * ay;
            const float rbx = dqB.c * bx - dqB.s * by, rby = dqB.s * bx + dqB.c * by;
            raX[k] = rax;
            raY[k] = ray;
            rbX[k] = rbx;
            rbY[k] = rby;

            const float s = c->pt[k].sep0 + (dpx + rbx - rax) * nx + (dpy + rby - ray) * ny;

            const float dvx = (vBx - wB * rby) - (vAx - wA * ray);
            const float dvy = (vBy + wB * rbx) - (vAy + wA * rax);
            const float vn = dvx * nx + dvy * ny;

            float bias = 0.0f, ms = 1.0f, is = 0.0f;
            if(s > 0.0f) {
                /* Speculative: allow closing by exactly s over this substep and no more. This is
                 * what arrests a fast approach at the surface without tunnelling and without an
                 * artificial bounce. */
                bias = s * inv_h;
            } else if(use_bias) {
                bias = fmaxf(bias_rate * s, -max_bias);
                ms = sms;
                is = sis;
            }

            float imp = -c->pt[k].nmass * ms * (vn + bias) - is * c->pt[k].nimp;
            const float ni = fmaxf(c->pt[k].nimp + imp, 0.0f); /* accumulate, then clamp */
            imp = ni - c->pt[k].nimp;
            c->pt[k].nimp = ni;
            if(ni > c->pt[k].maxni) c->pt[k].maxni = ni;

            const float px = imp * nx, py = imp * ny;
            vAx -= imA * px;
            vAy -= imA * py;
            wA -= iIA * (rax * py - ray * px);
            vBx += imB * px;
            vBy += imB * py;
            wB += iIB * (rbx * py - rby * px);
        }

        for(int k = 0; k < cnt; k++) {
            const float rax = raX[k], ray = raY[k], rbx = rbX[k], rby = rbY[k];

            const float dvx = (vBx - wB * rby) - (vAx - wA * ray);
            const float dvy = (vBy + wB * rbx) - (vAy + wA * rax);
            const float vt = dvx * tx + dvy * ty;

            float imp = -c->pt[k].tmass * vt;
            const float maxF = c->mu * c->pt[k].nimp;
            const float ti = fmaxf(-maxF, fminf(c->pt[k].timp + imp, maxF));
            imp = ti - c->pt[k].timp;
            c->pt[k].timp = ti;

            const float px = imp * tx, py = imp * ty;
            vAx -= imA * px;
            vAy -= imA * py;
            wA -= iIA * (rax * py - ray * px);
            vBx += imB * px;
            vBy += imB * py;
            wB += iIB * (rbx * py - rby * px);
        }

        /* Static bodies need no branch: im = ii = 0 makes every write above a no-op, and skipping
         * the test is worth more in the inner loop than the wasted stores cost. */
        A->vel = tt_v(vAx, vAy);
        A->w = wA;
        B->vel = tt_v(vBx, vBy);
        B->w = wB;
    }
}

void tt_solver_restitution(tt_world_t* w) {
    if(TT_RESTITUTION <= 0.0f) return;

    for(uint16_t m = 0; m < w->mf_count; m++) {
        tt_manifold_t* c = &w->mf[m];
        tt_body_t* A = &w->body[c->ba];
        tt_body_t* B = &w->body[c->bb];
        const vec2 n = c->n;

        for(int k = 0; k < c->count; k++) {
            tt_cpoint_t* p = &c->pt[k];
            /* The resting-contact gate is what makes restitution safe at all: without it,
             * gravity's per-step velocity gain is handed straight back and stacks buzz forever. */
            if(p->relv0 > -TT_REST_THRESH || p->maxni == 0.0f) continue;

            const vec2 ra = tt_rot(A->dq, p->ra);
            const vec2 rb = tt_rot(B->dq, p->rb);
            const vec2 dv =
                tt_sub(tt_add(B->vel, tt_wxr(B->w, rb)), tt_add(A->vel, tt_wxr(A->w, ra)));
            const float vn = tt_dot(dv, n);

            float imp = -p->nmass * (vn + TT_RESTITUTION * p->relv0);
            const float ni = fmaxf(p->nimp + imp, 0.0f);
            imp = ni - p->nimp;
            p->nimp = ni;

            const vec2 P = tt_scale(n, imp);
            A->vel = tt_sub(A->vel, tt_scale(P, A->im));
            A->w -= A->ii * tt_cross(ra, P);
            B->vel = tt_add(B->vel, tt_scale(P, B->im));
            B->w += B->ii * tt_cross(rb, P);
        }
    }
}

void tt_solver_store(tt_world_t* w) {
    tt_pc_rebuild(w);
}

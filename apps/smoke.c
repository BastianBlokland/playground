#include "app/ecs.h"
#include "asset/manager.h"
#include "asset/register.h"
#include "cli/app.h"
#include "cli/parse.h"
#include "cli/read.h"
#include "cli/validate.h"
#include "core/alloc.h"
#include "core/bits.h"
#include "core/diag.h"
#include "core/file.h"
#include "core/float.h"
#include "core/intrinsic.h"
#include "core/math.h"
#include "core/rng.h"
#include "core/version.h"
#include "ecs/utils.h"
#include "ecs/view.h"
#include "gap/error.h"
#include "gap/input.h"
#include "gap/register.h"
#include "gap/window.h"
#include "log/logger.h"
#include "rend/error.h"
#include "rend/register.h"
#include "rend/settings.h"
#include "ui/canvas.h"
#include "ui/register.h"
#include "ui/settings.h"

/**
 * Semi-Lagrangian fluid simulation.
 * Reference: https://www.cs.ubc.ca/~rbridson/fluidsimulation/fluids_notes.pdf
 * Reference: https://www.youtube.com/watch?v=Q78wvrQ9xsU
 */

typedef struct {
  i32 x, y;
} SimCoord;

typedef struct {
  f32 x, y;
} SimCoordFrac;

static SimCoord sim_coord_round_down(const SimCoordFrac c) {
  return (SimCoord){
      .x = (i32)math_round_down_f32(c.x),
      .y = (i32)math_round_down_f32(c.y),
  };
}

static bool sim_coord_valid(const SimCoord c, const u32 width, const u32 height) {
  if (c.x < 0 || c.x > (i32)width) {
    return false;
  }
  if (c.y < 0 || c.y > (i32)height) {
    return false;
  }
  return true;
}

static u32 sim_coord_dist_manhattan(const SimCoord a, const SimCoord b) {
  return (u32)(math_abs(a.x - b.x) + math_abs(a.y - b.y));
}

typedef struct {
  u32  width, height;
  f32* values;
} SimGrid;

static SimGrid sim_grid_create(Allocator* a, const u32 width, const u32 height) {
  return (SimGrid){
      .width  = width,
      .height = height,
      .values = alloc_array_t(a, f32, height * width),
  };
}

static void sim_grid_destroy(Allocator* a, SimGrid* g) {
  alloc_free_array_t(a, g->values, g->height * g->width);
}

static u32 sim_grid_count(const SimGrid* g) { return g->height * g->width; }

static void sim_grid_copy(const SimGrid* dst, const SimGrid* src) {
  const u32 count = sim_grid_count(src);
  diag_assert(sim_grid_count(dst) == count);
  const usize size = sizeof(f32) * count;
  mem_cpy(mem_create(dst->values, size), mem_create(src->values, size));
}

static void sim_grid_clear(SimGrid* g) {
  mem_set(mem_create(g->values, sim_grid_count(g) * sizeof(f32)), 0);
}

static void sim_grid_fill(SimGrid* g, const f32 v) {
  const u32 count = sim_grid_count(g);
  for (u32 i = 0; i != count; ++i) {
    g->values[i] = v;
  }
}

static void sim_grid_rand(SimGrid* g, const f32 min, const f32 max) {
  const u32 count = sim_grid_count(g);
  for (u32 i = 0; i != count; ++i) {
    g->values[i] = rng_sample_range(g_rng, min, max);
  }
}

static f32 sim_grid_sum(const SimGrid* g) {
  const u32 count  = sim_grid_count(g);
  f32       result = 0;
  for (u32 i = 0; i != count; ++i) {
    result += g->values[i];
  }
  return result;
}

static u32 sim_grid_index(const SimGrid* g, const SimCoord c) {
  diag_assert(sim_coord_valid(c, g->width, g->height));
  return c.y * g->width + c.x;
}

static void sim_grid_set(SimGrid* g, const SimCoord c, const f32 v) {
  g->values[sim_grid_index(g, c)] = v;
}

static void sim_grid_add(SimGrid* g, const SimCoord c, const f32 v) {
  g->values[sim_grid_index(g, c)] += v;
}

static f32 sim_grid_get(const SimGrid* g, const SimCoord c) {
  return g->values[sim_grid_index(g, c)];
}

static f32 sim_grid_get_bounded(const SimGrid* g, const SimCoord c, const f32 fallback) {
  return sim_coord_valid(c, g->width, g->height) ? sim_grid_get(g, c) : fallback;
}

static f32 sim_grid_sample(const SimGrid* g, const SimCoordFrac c, const f32 fallback) {
  const SimCoord cI = sim_coord_round_down(c);

  const SimCoord c00 = {cI.x, cI.y};
  const SimCoord c10 = {cI.x + 1, cI.y};
  const SimCoord c01 = {cI.x, cI.y + 1};
  const SimCoord c11 = {cI.x + 1, cI.y + 1};

  const f32 v00 = sim_grid_get_bounded(g, c00, fallback);
  const f32 v10 = sim_grid_get_bounded(g, c10, fallback);
  const f32 v01 = sim_grid_get_bounded(g, c01, fallback);
  const f32 v11 = sim_grid_get_bounded(g, c11, fallback);

  const f32 fracX = c.x - (f32)cI.x;
  const f32 fracY = c.y - (f32)cI.y;

  const f32 x0 = math_lerp(v00, v10, fracX);
  const f32 x1 = math_lerp(v01, v11, fracX);

  return math_lerp(x0, x1, fracY);
}

typedef struct {
  SimCoord position;
  f32      smokeAmount, angle /* in radians */, force;
} SimEmitter;

typedef struct {
  u32 width, height;

  u32 solverIterations;
  f32 density;
  f32 pressureDecay;
  f32 velocityDiffusion;
  f32 smokeDiffusion;
  f32 smokeDecay;

  bool     push;
  SimCoord pushCoord;
  f32      pushPressure;

  bool         pull;
  SimCoordFrac pullCoord;
  f32          pullForce;

  SimEmitter emitters[4];
  u32        emitterCount;

  // NOTE: Velocities are stored at the edges, not the cell centers.
  SimGrid velocitiesX; // (width + 1) height
  SimGrid velocitiesY; // width * (height + 1)

  SimGrid pressure;
  SimGrid smoke;
  BitSet  solid;
} SimState;

static SimState sim_state_create(const u32 width, const u32 height) {
  SimState s = {
      .width  = width,
      .height = height,

      .solverIterations  = 32,
      .density           = 10.0f,
      .pressureDecay     = 0.5f,
      .velocityDiffusion = 0.5f,
      .smokeDiffusion    = 0.05f,
      .smokeDecay        = 0.01f,
      .pushPressure      = 500.0f,
      .pullForce         = 5.0f,

      .velocitiesX = sim_grid_create(g_allocHeap, width + 1, height),
      .velocitiesY = sim_grid_create(g_allocHeap, width, height + 1),
      .pressure    = sim_grid_create(g_allocHeap, width, height),
      .smoke       = sim_grid_create(g_allocHeap, width, height),
      .solid       = alloc_alloc(g_allocHeap, bits_to_bytes(height * width) + 1, 1),
  };

  sim_grid_clear(&s.velocitiesX);
  sim_grid_clear(&s.velocitiesY);
  sim_grid_clear(&s.pressure);
  sim_grid_clear(&s.smoke);
  mem_set(s.solid, 0);

  return s;
}

static void sim_state_destroy(SimState* s) {
  sim_grid_destroy(g_allocHeap, &s->velocitiesX);
  sim_grid_destroy(g_allocHeap, &s->velocitiesY);
  sim_grid_destroy(g_allocHeap, &s->pressure);
  sim_grid_destroy(g_allocHeap, &s->smoke);
  alloc_free(g_allocHeap, s->solid);
}

static bool sim_emitter_add(SimState* s, const SimEmitter e) {
  if (s->emitterCount == array_elems(s->emitters)) {
    return false;
  }
  s->emitters[s->emitterCount++] = e;
  return true;
}

static void sim_emitter_clear(SimState* s) { s->emitterCount = 0; }

static bool sim_solid(const SimState* s, const SimCoord c) {
  if (!sim_coord_valid(c, s->width, s->height)) {
    return false;
  }
  return bitset_test(s->solid, c.y * s->width + c.x);
}

static void sim_solid_flip(SimState* s, const SimCoord c) {
  diag_assert(sim_coord_valid(c, s->width, s->height));
  bitset_flip(s->solid, c.y * s->width + c.x);
}

static void sim_solid_set(SimState* s, const SimCoord c) {
  diag_assert(sim_coord_valid(c, s->width, s->height));
  bitset_set(s->solid, c.y * s->width + c.x);
}

static void sim_solid_set_border(SimState* s) {
  for (u32 y = 0; y != s->height; ++y) {
    for (u32 x = 0; x != s->width; ++x) {
      if (!x || !y || x == (s->width - 1) || y == (s->height - 1)) {
        bitset_set(s->solid, y * s->width + x);
      }
    }
  }
}

static void sim_solid_clear(SimState* s) { bitset_clear_all(s->solid); }

static f32 sim_pressure(const SimState* s, const SimCoord c) {
  if (sim_solid(s, c)) {
    return 0.0f;
  }
  return sim_grid_get_bounded(&s->pressure, c, 0.0f /* fallback */);
}

static void sim_pressure_clear(SimState* s) { sim_grid_clear(&s->pressure); }

static f32 sim_smoke(const SimState* s, const SimCoord c) {
  return sim_grid_get_bounded(&s->smoke, c, 0.0f /* fallback */);
}

static void sim_smoke_emit(SimState* s, const SimCoord c, const f32 smoke) {
  sim_grid_add(&s->smoke, c, smoke);
}

static f32 sim_smoke_sum(const SimState* s) { return sim_grid_sum(&s->smoke); }

static void sim_smoke_clear(SimState* s) { sim_grid_clear(&s->smoke); }

static void sim_velocity_add(SimState* s, const SimCoord c, const f32 vX, const f32 vY) {
  sim_grid_add(&s->velocitiesX, c, vX);                        // Left.
  sim_grid_add(&s->velocitiesX, (SimCoord){c.x + 1, c.y}, vX); // Right.
  sim_grid_add(&s->velocitiesY, c, vY);                        // Bottom.
  sim_grid_add(&s->velocitiesY, (SimCoord){c.x, c.y + 1}, vY); // Top.
}

static void sim_velocity_clear(SimState* s) {
  sim_grid_clear(&s->velocitiesX);
  sim_grid_clear(&s->velocitiesY);
}

static void sim_velocity_randomize(SimState* s) {
  sim_grid_rand(&s->velocitiesX, -1.0f, 1.0f);
  sim_grid_rand(&s->velocitiesY, -1.0f, 1.0f);
}

static f32 sim_velocity_bottom(const SimState* s, const SimCoord c) {
  return sim_grid_get_bounded(&s->velocitiesY, c, 0.0f /* fallback */);
}

static f32 sim_velocity_top(const SimState* s, const SimCoord c) {
  const SimCoord cAbove = {c.x, c.y + 1};
  return sim_grid_get_bounded(&s->velocitiesY, cAbove, 0.0f /* fallback */);
}

static f32 sim_velocity_left(const SimState* s, const SimCoord c) {
  return sim_grid_get_bounded(&s->velocitiesX, c, 0.0f /* fallback */);
}

static f32 sim_velocity_right(const SimState* s, const SimCoord c) {
  const SimCoord cRight = {c.x + 1, c.y};
  return sim_grid_get_bounded(&s->velocitiesY, cRight, 0.0f /* fallback */);
}

static f32 sim_velocity_x(const SimState* s, const SimCoord c) {
  return sim_grid_sample(&s->velocitiesX, (SimCoordFrac){c.x + 0.5f, c.y}, 0.0f);
}

static f32 sim_velocity_y(const SimState* s, const SimCoord c) {
  return sim_grid_sample(&s->velocitiesY, (SimCoordFrac){c.x, c.y + 0.5f}, 0.0f);
}

static f32 sim_velocity_divergence(const SimState* s, const SimCoord c) {
  const f32 vTop    = sim_velocity_top(s, c);
  const f32 vLeft   = sim_velocity_left(s, c);
  const f32 vRight  = sim_velocity_right(s, c);
  const f32 vBottom = sim_velocity_bottom(s, c);
  return (vRight - vLeft) + (vTop - vBottom);
}

static f32 sim_speed(const SimState* s, const SimCoord c) {
  const f32 vX       = sim_velocity_x(s, c);
  const f32 vY       = sim_velocity_y(s, c);
  const f32 speedSqr = vX * vX + vY * vY;
  return speedSqr != 0.0f ? intrinsic_sqrt_f32(speedSqr) : 0.0f;
}

static bool sim_pushed(const SimState* s, const SimCoord c) {
  return s->push && sim_coord_dist_manhattan(s->pushCoord, c) <= 1;
}

static f32 sim_pull_force(const SimState* s, const SimCoordFrac c) {
  const f32 smoke        = sim_grid_sample(&s->smoke, c, 0.0f);
  const f32 smokeClamped = math_clamp_f32(smoke, 0.0f, 1.0f);

  // NOTE: Arbitrary easing function at the moment.
  return 1.0f - math_pow_f32(1.0f - smokeClamped, 3.0f);
}

static void sim_pull(SimState* s, const SimCoordFrac target, const f32 force) {
  // Horizontal.
  for (u32 y = 0; y != s->velocitiesX.height; ++y) {
    for (u32 x = 0; x != s->velocitiesX.width; ++x) {
      const f32 deltaX  = target.x - x;
      const f32 deltaY  = target.y - (y + 0.5f);
      const f32 distSqr = deltaX * deltaX + deltaY * deltaY;
      if (distSqr < f32_epsilon) {
        continue;
      }
      const f32 forceMul  = sim_pull_force(s, (SimCoordFrac){x, y + 0.5f});
      const f32 veloDelta = deltaX / intrinsic_sqrt_f32(distSqr) * force * forceMul;
      sim_grid_add(&s->velocitiesX, (SimCoord){x, y}, veloDelta);
    }
  }

  // Vertical.
  for (u32 y = 0; y != s->velocitiesY.height; ++y) {
    for (u32 x = 0; x != s->velocitiesY.width; ++x) {
      const f32 deltaX  = target.x - (x + 0.5f);
      const f32 deltaY  = target.y - y;
      const f32 distSqr = deltaX * deltaX + deltaY * deltaY;
      if (distSqr < f32_epsilon) {
        continue;
      }
      const f32 forceMul  = sim_pull_force(s, (SimCoordFrac){x + 0.5f, y});
      const f32 veloDelta = deltaY / intrinsic_sqrt_f32(distSqr) * force * forceMul;
      sim_grid_add(&s->velocitiesY, (SimCoord){x, y}, veloDelta);
    }
  }
}

static void sim_diffuse_velocity_grid(SimGrid* g, const f32 diffusion, const f32 dt) {
  for (u32 y = 0; y != g->width; ++y) {
    for (u32 x = 0; x != g->height; ++x) {
      const f32 vCenter = sim_grid_get(g, (SimCoord){x, y});
      const f32 vTop    = sim_grid_get_bounded(g, (SimCoord){x + 0, y + 1}, vCenter);
      const f32 vLeft   = sim_grid_get_bounded(g, (SimCoord){x - 1, y + 0}, vCenter);
      const f32 vRight  = sim_grid_get_bounded(g, (SimCoord){x + 1, y + 0}, vCenter);
      const f32 vBottom = sim_grid_get_bounded(g, (SimCoord){x + 0, y - 1}, vCenter);

      const f32 laplacian = vLeft + vRight + vTop + vBottom - 4 * vCenter;
      const f32 vDiffused = vCenter + laplacian * diffusion * dt;
      sim_grid_set(g, (SimCoord){x, y}, vDiffused);
    }
  }
}

static void sim_diffuse_velocity(SimState* s, const f32 dt) {
  if (s->velocityDiffusion >= f32_epsilon) {
    sim_diffuse_velocity_grid(&s->velocitiesX, s->velocityDiffusion, dt);
    sim_diffuse_velocity_grid(&s->velocitiesY, s->velocityDiffusion, dt);
  }
}

static void sim_diffuse_smoke(SimState* s, const f32 dt) {
  if (s->smokeDiffusion < f32_epsilon) {
    return;
  }
  for (u32 y = 0; y != s->height; ++y) {
    for (u32 x = 0; x != s->width; ++x) {
      if (sim_solid(s, (SimCoord){x, y})) {
        continue;
      }
      const f32 vCenter = sim_grid_get(&s->smoke, (SimCoord){x, y});
      const f32 vTop    = sim_grid_get_bounded(&s->smoke, (SimCoord){x + 0, y + 1}, 0.0f);
      const f32 vLeft   = sim_grid_get_bounded(&s->smoke, (SimCoord){x - 1, y + 0}, 0.0f);
      const f32 vRight  = sim_grid_get_bounded(&s->smoke, (SimCoord){x + 1, y + 0}, 0.0f);
      const f32 vBottom = sim_grid_get_bounded(&s->smoke, (SimCoord){x + 0, y - 1}, 0.0f);

      const f32 laplacian     = vLeft + vRight + vTop + vBottom - 4 * vCenter;
      const f32 smokeDiffused = vCenter + s->smokeDiffusion * dt * laplacian;
      const f32 smokeNew      = smokeDiffused * math_exp_f32(-dt * s->smokeDecay);
      sim_grid_set(&s->smoke, (SimCoord){x, y}, smokeNew);
    }
  }
}

static void sim_advect_velocity(SimState* s, const f32 dt) {
  SimGrid velocitiesXNew = sim_grid_create(g_allocScratch, s->width + 1, s->height);
  SimGrid velocitiesYNew = sim_grid_create(g_allocScratch, s->width, s->height + 1);

  // Horizontal.
  for (u32 y = 0; y != velocitiesXNew.height; ++y) {
    for (u32 x = 0; x != velocitiesXNew.width; ++x) {
      const SimCoord cellLeft  = (SimCoord){x - 1, y};
      const SimCoord cellRight = (SimCoord){x + 0, y};
      if (sim_solid(s, cellLeft) || sim_solid(s, cellRight)) {
        sim_grid_set(&velocitiesXNew, (SimCoord){x, y}, 0.0f);
        continue;
      }
      const f32 veloX = sim_grid_get(&s->velocitiesX, (SimCoord){x, y});
      const f32 veloY = sim_grid_sample(&s->velocitiesY, (SimCoordFrac){x - 0.5f, y + 0.5f}, 0.0f);

      const f32 prevX = x - veloX * dt;
      const f32 prevY = y - veloY * dt;

      const f32 veloNew = sim_grid_sample(&s->velocitiesX, (SimCoordFrac){prevX, prevY}, 0.0f);
      sim_grid_set(&velocitiesXNew, (SimCoord){x, y}, veloNew);
    }
  }

  // Vertical.
  for (u32 y = 0; y != velocitiesYNew.height; ++y) {
    for (u32 x = 0; x != velocitiesYNew.width; ++x) {
      const SimCoord cellBottom = (SimCoord){x, y - 1};
      const SimCoord cellTop    = (SimCoord){x, y + 0};
      if (sim_solid(s, cellBottom) || sim_solid(s, cellTop)) {
        sim_grid_set(&velocitiesYNew, (SimCoord){x, y}, 0.0f);
        continue;
      }
      const f32 veloX = sim_grid_sample(&s->velocitiesX, (SimCoordFrac){x + 0.5f, y - 0.5f}, 0.0f);
      const f32 veloY = sim_grid_get(&s->velocitiesY, (SimCoord){x, y});

      const f32 prevX = x - veloX * dt;
      const f32 prevY = y - veloY * dt;

      const f32 veloNew = sim_grid_sample(&s->velocitiesY, (SimCoordFrac){prevX, prevY}, 0.0f);
      sim_grid_set(&velocitiesYNew, (SimCoord){x, y}, veloNew);
    }
  }

  sim_grid_copy(&s->velocitiesX, &velocitiesXNew);
  sim_grid_copy(&s->velocitiesY, &velocitiesYNew);
}

static void sim_advect_smoke(SimState* s, const f32 dt) {
  SimGrid smokeNew = sim_grid_create(g_allocScratch, s->width, s->height);

  for (u32 y = 0; y != s->height; ++y) {
    for (u32 x = 0; x != s->width; ++x) {
      const SimCoord cell = {x, y};
      if (sim_solid(s, cell) || sim_pushed(s, cell)) {
        sim_grid_set(&smokeNew, cell, 0);
        continue;
      }
      const f32 veloX = sim_velocity_x(s, (SimCoord){x, y});
      const f32 veloY = sim_velocity_y(s, (SimCoord){x, y});

      const f32 prevX     = x - veloX * dt;
      const f32 prevY     = y - veloY * dt;
      const f32 prevSmoke = sim_grid_sample(&s->smoke, (SimCoordFrac){prevX, prevY}, 0.0f);
      sim_grid_set(&smokeNew, cell, prevSmoke);
    }
  }

  sim_grid_copy(&s->smoke, &smokeNew);
}

static void sim_solve_pressure(SimState* s, const f32 dt) {
  if (s->density <= f32_epsilon) {
    return;
  }
  for (u32 y = 0; y != s->height; ++y) {
    for (u32 x = 0; x != s->width; ++x) {
      const bool flowTop    = !sim_solid(s, (SimCoord){x + 0, y + 1});
      const bool flowLeft   = !sim_solid(s, (SimCoord){x - 1, y + 0});
      const bool flowRight  = !sim_solid(s, (SimCoord){x + 1, y + 0});
      const bool flowBottom = !sim_solid(s, (SimCoord){x + 0, y - 1});
      const u8   flowCount  = flowLeft + flowRight + flowTop + flowBottom;

      f32 newPressure;
      if (sim_pushed(s, (SimCoord){x, y})) {
        newPressure = s->pushPressure;
      } else if (sim_solid(s, (SimCoord){x, y}) || !flowCount) {
        newPressure = 0.0f;
      } else {
        const f32 pressureTop    = sim_pressure(s, (SimCoord){x, y + 1}) * flowTop;
        const f32 pressureLeft   = sim_pressure(s, (SimCoord){x - 1, y}) * flowLeft;
        const f32 pressureRight  = sim_pressure(s, (SimCoord){x + 1, y}) * flowRight;
        const f32 pressureBottom = sim_pressure(s, (SimCoord){x, y - 1}) * flowBottom;
        const f32 pressureSum    = pressureRight + pressureLeft + pressureTop + pressureBottom;

        const f32 velTop    = sim_velocity_top(s, (SimCoord){x, y});
        const f32 velLeft   = sim_velocity_left(s, (SimCoord){x, y});
        const f32 velRight  = sim_velocity_right(s, (SimCoord){x, y});
        const f32 velBottom = sim_velocity_bottom(s, (SimCoord){x, y});
        const f32 velDelta  = velRight - velLeft + velTop - velBottom;

        newPressure = (pressureSum - s->density * velDelta / dt) / flowCount;
        newPressure -= newPressure * s->density * s->pressureDecay * dt;
      }
      sim_grid_set(&s->pressure, (SimCoord){x, y}, newPressure);
    }
  }
}

static void sim_solve_velocity(SimState* s, const f32 dt) {
  if (s->density <= f32_epsilon) {
    return;
  }
  const f32 k = dt / s->density;

  // Horizontal.
  for (u32 y = 0; y != s->velocitiesX.height; ++y) {
    for (u32 x = 0; x != s->velocitiesX.width; ++x) {
      const SimCoord cellLeft  = (SimCoord){x - 1, y};
      const SimCoord cellRight = (SimCoord){x + 0, y};
      if (sim_solid(s, cellLeft) || sim_solid(s, cellRight)) {
        sim_grid_set(&s->velocitiesX, (SimCoord){x, y}, 0.0f);
        continue;
      }
      const f32 pressureLeft  = sim_pressure(s, cellLeft);
      const f32 pressureRight = sim_pressure(s, cellRight);
      sim_grid_add(&s->velocitiesX, (SimCoord){x, y}, k * -(pressureRight - pressureLeft));
    }
  }

  // Vertical.
  for (u32 y = 0; y != s->velocitiesY.height; ++y) {
    for (u32 x = 0; x != s->velocitiesY.width; ++x) {
      const SimCoord cellBottom = (SimCoord){x, y - 1};
      const SimCoord cellTop    = (SimCoord){x, y + 0};
      if (sim_solid(s, cellBottom) || sim_solid(s, cellTop)) {
        sim_grid_set(&s->velocitiesY, (SimCoord){x, y}, 0.0f);
        continue;
      }
      const f32 pressureBottom = sim_pressure(s, cellBottom);
      const f32 pressureTop    = sim_pressure(s, cellTop);
      sim_grid_add(&s->velocitiesY, (SimCoord){x, y}, k * -(pressureTop - pressureBottom));
    }
  }
}

static bool sim_update(SimState* s, const f32 dt) {
  if (dt < f32_epsilon) {
    return false;
  }

  for (u32 i = 0; i != s->emitterCount; ++i) {
    const SimEmitter* e = &s->emitters[i];
    sim_smoke_emit(s, e->position, e->smokeAmount * dt);
    const f32 veloX = math_cos_f32(e->angle) * e->force * dt;
    const f32 veloY = math_sin_f32(e->angle) * e->force * dt;
    sim_velocity_add(s, e->position, veloX, veloY);
  }

  if (s->pull) {
    sim_pull(s, s->pullCoord, s->pullForce * dt);
  }
  sim_diffuse_velocity(s, dt);
  sim_diffuse_smoke(s, dt);
  sim_advect_velocity(s, dt);
  sim_advect_smoke(s, dt);
  for (u32 i = 0; i != s->solverIterations; ++i) {
    sim_solve_pressure(s, dt);
  }
  sim_solve_velocity(s, dt);
  return true;
}

ecs_comp_define(DemoComp) {
  EcsEntityId window;
  EcsEntityId uiCanvas;
  TimeSteady  lastTime;

  SimState sim;
};

static EcsEntityId demo_create_window(EcsWorld* world, const u16 width, const u16 height) {
  const GapVector      size           = {.width = (i32)width, .height = (i32)height};
  const GapWindowFlags flags          = GapWindowFlags_Default;
  const GapWindowMode  mode           = GapWindowMode_Windowed;
  const GapIcon        icon           = GapIcon_Main;
  const String         versionScratch = version_str_scratch(g_versionExecutable);
  const String         titleScratch = fmt_write_scratch("Smoke Demo v{}", fmt_text(versionScratch));
  return gap_window_create(world, mode, flags, size, icon, titleScratch);
}

static DemoComp* demo_create(EcsWorld* world, const u16 winWidth, const u16 winHeight) {
  const EcsEntityId global = ecs_world_global(world);

  DemoComp* demo = ecs_world_add_t(world, global, DemoComp);

  demo->window   = demo_create_window(world, winWidth, winHeight);
  demo->uiCanvas = ui_canvas_create(world, demo->window, UiCanvasCreateFlags_ToBack);

  rend_settings_window_init(world, demo->window)->flags |= RendFlags_2D;

  const u32 simWidth  = 25;
  const u32 simHeight = 25;
  demo->sim           = sim_state_create(simWidth, simHeight);

  return demo;
}

static void demo_destroy(void* data) {
  DemoComp* comp = data;
  sim_state_destroy(&comp->sim);
}

static f32 demo_time_to_seconds(const TimeDuration dur) {
  static const f64 g_toSecMul = 1.0 / (f64)time_second;
  return (f32)((f64)dur * g_toSecMul);
}

// static void sim_draw(DemoUpdateContext* ctx) {
//   ui_canvas_id_block_next(ctx->winCanvas);
//   ui_layout_push(ctx->winCanvas);
//   ui_style_push(ctx->winCanvas);

//   const u32 simWidth  = ctx->demo->simWidth;
//   const u32 simHeight = ctx->demo->simHeight;

//   const UiVector canvasSize = ui_canvas_resolution(ctx->winCanvas);
//   const UiVector cellSize   = ui_vector(40, 40);
//   const UiVector cellOrg    = {
//       canvasSize.width * 0.5f - simWidth * cellSize.width * 0.5f,
//       canvasSize.height * 0.5f - simHeight * cellSize.height * 0.5f,
//   };

//   const GapVector cursorDelta = gap_window_param(ctx->winComp, GapParam_CursorDelta);
//   const f32       distSqr     = cursorDelta.x * cursorDelta.x + cursorDelta.y * cursorDelta.y;
//   if (distSqr > f32_epsilon) {
//     const f32 dist  = intrinsic_sqrt_f32(distSqr);
//     ctx->demo->dirX = cursorDelta.x / dist;
//     ctx->demo->dirY = cursorDelta.y / dist;
//     log_i(
//         "Dir",
//         log_param("x", fmt_float(ctx->demo->dirX)),
//         log_param("y", fmt_float(ctx->demo->dirY)));
//   }

//   ctx->demo->push = false;

//   ui_layout_resize(ctx->winCanvas, UiAlign_BottomLeft, cellSize, UiBase_Absolute, Ui_XY);
//   for (u32 y = 0; y != simHeight; ++y) {
//     for (u32 x = 0; x != simWidth; ++x) {
//       const f32 divergence = sim_velocity_divergence(ctx, x, y);
//       const f32 pressure   = sim_pressure(ctx, x, y);
//       const f32 speed      = sim_speed(ctx, x, y);
//       const f32 smoke      = sim_smoke(ctx, x, y);

//       (void)speed;
//       (void)divergence;
//       (void)pressure;

//       const f32 velX =
//           sim_sample(ctx->demo->velocitiesX, simWidth + 1, simHeight, x + 0.5f, y + 0.5f);
//       const f32 velY =
//           sim_sample(ctx->demo->velocitiesY, simWidth, simHeight + 1, x + 0.5f, y + 0.5f);

//       UiColor color;
//       if (sim_solid(ctx, x, y)) {
//         color = ui_color_gray;
//       } else {
//         // const f32 frac = math_clamp_f32(pressure * 0.01f, -1.0f, 1.0f);
//         // color          = ui_color_lerp(ui_color_green, ui_color_red, (frac + 1.0f) * 0.5f);

//         const f32 frac = math_clamp_f32(smoke * 10, 0.0f, 1.0f);
//         color          = ui_color_lerp(ui_color_green, ui_color_red, frac);
//       }
//       ui_style_color(ctx->winCanvas, color);

//       const UiVector pos = ui_vector(cellOrg.x + x * cellSize.x, cellOrg.y + y * cellSize.y);
//       ui_layout_set_pos(ctx->winCanvas, UiBase_Canvas, pos, UiBase_Absolute);
//       const UiId id = ui_canvas_draw_glyph(
//           ctx->winCanvas, UiShape_Square, 5, UiFlags_Interactable | UiFlags_InteractSupportAlt);
//       const UiStatus status = ui_canvas_elem_status(ctx->winCanvas, id);
//       if (status == UiStatus_Activated) {
//         sim_solid_flip(ctx, x, y);
//       } else if (status == UiStatus_ActivatedAlt) {
//       }
//       if (status == UiStatus_Hovered /* && gap_window_key_down(ctx->winComp, GapKey_Tab) */) {
//         ctx->demo->targetX = x;
//         ctx->demo->targetY = y;

//         if (gap_window_key_down(ctx->winComp, GapKey_Tab)) {
//           sim_solid_set(ctx, x, y);
//         }
//         ctx->demo->push = gap_window_key_down(ctx->winComp, GapKey_Control);
//         // if (gap_window_key_down(ctx->winComp, GapKey_Control)) {
//         //   const f32 forceMul = 1000.0f;
//         //   sim_push(
//         //       ctx,
//         //       x,
//         //       y,
//         //       ctx->demo->dirX * forceMul * ctx->dt,
//         //       ctx->demo->dirY * forceMul * ctx->dt);
//         // }
//       }

//       {
//         const f32      lineColorFrac = math_clamp_f32(math_abs(divergence), 0.0f, 1.0f);
//         const UiColor  lineColor     = ui_color_lerp(ui_color_green, ui_color_red,
//         lineColorFrac); const UiVector posLineA      = {
//             cellOrg.x + x * cellSize.x + cellSize.x * 0.5f,
//             cellOrg.y + y * cellSize.y + cellSize.y * 0.5f,
//         };
//         const f32 lineScale = 10.0f;
//         ui_style_color(ctx->winCanvas, lineColor);
//         ui_line(
//             ctx->winCanvas,
//             posLineA,
//             ui_vector(posLineA.x + velX * lineScale, posLineA.y + velY * lineScale),
//             .base  = UiBase_Absolute,
//             .width = 4.0f);
//       }

//       ui_style_color(ctx->winCanvas, ui_color_white);
//       const String label = fmt_write_scratch(
//           "{}\n{}",
//           fmt_float(
//               pressure,
//               .minDecDigits    = 2,
//               .maxDecDigits    = 2,
//               .expThresholdPos = f64_max,
//               .expThresholdNeg = 0),
//           fmt_float(
//               divergence,
//               .minDecDigits    = 2,
//               .maxDecDigits    = 2,
//               .expThresholdPos = f64_max,
//               .expThresholdNeg = 0));
//       ui_canvas_draw_text(ctx->winCanvas, label, 10, UiAlign_MiddleCenter, UiFlags_None);
//     }
//   }

//   ui_layout_resize(ctx->winCanvas, UiAlign_BottomLeft, ui_vector(5, 5), UiBase_Absolute, Ui_XY);

//   // Horizontal.
//   // for (u32 y = 0; y != simHeight; ++y) {
//   //   for (u32 x = 0; x != (simWidth + 1); ++x) {
//   //     const f32      velo = ctx->demo->velocitiesX[y * (simWidth + 1) + x];
//   //     const UiVector pos  = {
//   //         cellOrg.x + x * cellSize.x,
//   //         cellOrg.y + y * cellSize.y + cellSize.y * 0.5f,
//   //     };
//   //     ui_style_color(ctx->winCanvas, ui_color_red);
//   //     ui_layout_set_pos(
//   //         ctx->winCanvas, UiBase_Canvas, ui_vector(pos.x - 2.5f, pos.y - 2.5f),
//   UiBase_Absolute);
//   //     ui_canvas_draw_glyph(ctx->winCanvas, UiShape_Circle, 0, UiFlags_None);

//   //     ui_line(
//   //         ctx->winCanvas,
//   //         pos,
//   //         ui_vector(pos.x + velo * cellSize.x * 0.05f, pos.y),
//   //         .base  = UiBase_Absolute,
//   //         .width = 3.0f);
//   //   }
//   // }

//   // // Vertical.
//   // for (u32 y = 0; y != (simHeight + 1); ++y) {
//   //   for (u32 x = 0; x != simWidth; ++x) {
//   //     const f32      velo = ctx->demo->velocitiesY[y * simWidth + x];
//   //     const UiVector pos  = {
//   //         cellOrg.x + x * cellSize.x + cellSize.x * 0.5f,
//   //         cellOrg.y + y * cellSize.y,
//   //     };
//   //     ui_style_color(ctx->winCanvas, ui_color_green);
//   //     ui_layout_set_pos(
//   //         ctx->winCanvas, UiBase_Canvas, ui_vector(pos.x - 2.5f, pos.y - 2.5f),
//   UiBase_Absolute);
//   //     ui_canvas_draw_glyph(ctx->winCanvas, UiShape_Circle, 0, UiFlags_None);

//   //     ui_line(
//   //         ctx->winCanvas,
//   //         pos,
//   //         ui_vector(pos.x, pos.y + velo * cellSize.y * 0.05f),
//   //         .base  = UiBase_Absolute,
//   //         .width = 3.0f);
//   //   }
//   // }

//   ui_style_pop(ctx->winCanvas);
//   ui_layout_pop(ctx->winCanvas);
//   ui_canvas_id_block_next(ctx->winCanvas);
// }

// static void demo_draw_menu_frame(const DemoUpdateContext* ctx) {
//   ui_style_push(ctx->winCanvas);
//   ui_style_outline(ctx->winCanvas, 5);
//   ui_style_color(ctx->winCanvas, ui_color(0, 0, 0, 200));
//   ui_canvas_draw_glyph(ctx->winCanvas, UiShape_Circle, 10, UiFlags_None);
//   ui_style_pop(ctx->winCanvas);
// }

// static void demo_draw_numbox_f32(const DemoUpdateContext* ctx, const String label, f32* value) {
//   demo_draw_menu_frame(ctx);
//   ui_layout_push(ctx->winCanvas);
//   static const UiVector g_frameInset = {-30, -20};
//   ui_layout_grow(ctx->winCanvas, UiAlign_MiddleCenter, g_frameInset, UiBase_Absolute, Ui_XY);
//   ui_label(ctx->winCanvas, label);
//   ui_layout_inner(
//       ctx->winCanvas, UiBase_Current, UiAlign_MiddleRight, ui_vector(0.5f, 1), UiBase_Current);
//   f64 val = *value;
//   if (ui_numbox(ctx->winCanvas, &val)) {
//     *value = (f32)val;
//   }
//   ui_layout_pop(ctx->winCanvas);
// }

// static void demo_draw_numbox_u32(const DemoUpdateContext* ctx, const String label, u32* value) {
//   demo_draw_menu_frame(ctx);
//   ui_layout_push(ctx->winCanvas);
//   static const UiVector g_frameInset = {-30, -20};
//   ui_layout_grow(ctx->winCanvas, UiAlign_MiddleCenter, g_frameInset, UiBase_Absolute, Ui_XY);
//   ui_label(ctx->winCanvas, label);
//   ui_layout_inner(
//       ctx->winCanvas, UiBase_Current, UiAlign_MiddleRight, ui_vector(0.5f, 1), UiBase_Current);
//   f64 val = *value;
//   if (ui_numbox(ctx->winCanvas, &val, .step = 1.0)) {
//     *value = (u32)val;
//   }
//   ui_layout_pop(ctx->winCanvas);
// }

// static void demo_draw_menu(const DemoUpdateContext* ctx) {
//   const UiVector size    = {250, 40};
//   const UiVector spacing = {5, 5};

//   ui_layout_inner(ctx->winCanvas, UiBase_Canvas, UiAlign_BottomLeft, size, UiBase_Absolute);
//   ui_layout_move(ctx->winCanvas, ui_vector(spacing.x, spacing.y), UiBase_Absolute, Ui_XY);

//   demo_draw_numbox_f32(ctx, string_lit("Density"), &ctx->demo->density);
//   ui_layout_next(ctx->winCanvas, Ui_Up, spacing.y);
//   demo_draw_numbox_f32(ctx, string_lit("Velocity diff"), &ctx->demo->velDiffusion);
//   ui_layout_next(ctx->winCanvas, Ui_Up, spacing.y);
//   demo_draw_numbox_f32(ctx, string_lit("Smoke diff"), &ctx->demo->smokeDiffusion);
//   ui_layout_next(ctx->winCanvas, Ui_Up, spacing.y);
//   demo_draw_numbox_f32(ctx, string_lit("Smoke decay"), &ctx->demo->smokeDecay);
//   ui_layout_next(ctx->winCanvas, Ui_Up, spacing.y);
//   demo_draw_numbox_u32(ctx, string_lit("Solver itrs"), &ctx->demo->solverIterations);
// }

ecs_view_define(FrameUpdateView) { ecs_access_write(RendSettingsGlobalComp); }

ecs_view_define(ErrorView) {
  ecs_access_maybe_read(GapErrorComp);
  ecs_access_maybe_read(RendErrorComp);
}

ecs_view_define(UpdateView) { ecs_access_write(DemoComp); }
ecs_view_define(WindowView) { ecs_access_write(GapWindowComp); }

ecs_view_define(UiCanvasView) {
  ecs_view_flags(EcsViewFlags_Exclusive); // Only access the canvas's we create.
  ecs_access_write(UiCanvasComp);
}

ecs_system_define(DemoUpdateSys) {
  EcsView*     globalView = ecs_world_view_t(world, UpdateView);
  EcsIterator* globalItr  = ecs_view_maybe_at(globalView, ecs_world_global(world));
  if (!globalItr) {
    return;
  }
  DemoComp* demo = ecs_view_write_t(globalItr, DemoComp);

  const TimeSteady timeNew   = time_steady_clock();
  TimeDuration     timeDelta = 0;
  if (demo->lastTime) {
    timeDelta = time_steady_duration(demo->lastTime, timeNew);
    timeDelta = math_min(timeDelta, time_second); // Avoid huge delta's when process was paused.
  }
  demo->lastTime = timeNew;

  sim_update(&demo->sim, demo_time_to_seconds(timeDelta));

  EcsIterator* canvasItr = ecs_view_itr(ecs_world_view_t(world, UiCanvasView));
  EcsIterator* winItr    = ecs_view_maybe_at(ecs_world_view_t(world, WindowView), demo->window);

  if (winItr) {
    GapWindowComp* winComp = ecs_view_write_t(winItr, GapWindowComp);

    if (gap_window_key_down(winComp, GapKey_Alt) && gap_window_key_pressed(winComp, GapKey_F4)) {
      gap_window_close(winComp);
    }

    UiCanvasComp* uiCanvas = null;
    if (ecs_view_maybe_jump(canvasItr, demo->uiCanvas)) {
      uiCanvas = ecs_view_write_t(canvasItr, UiCanvasComp);
      ui_canvas_reset(uiCanvas);
    }

    // TODO: Draw visualization.
    (void)uiCanvas;
  }
}

ecs_module_init(demo_module) {
  ecs_register_comp(DemoComp, .destructor = demo_destroy);

  ecs_register_view(FrameUpdateView);
  ecs_register_view(ErrorView);
  ecs_register_view(UpdateView);
  ecs_register_view(WindowView);
  ecs_register_view(UiCanvasView);

  ecs_register_system(
      DemoUpdateSys, ecs_view_id(UpdateView), ecs_view_id(WindowView), ecs_view_id(UiCanvasView));
}

static CliId g_optAssets, g_optWidth, g_optHeight;

AppType app_ecs_configure(CliApp* app) {
  cli_app_register_desc(app, string_lit("Smoke Demo"));

  g_optAssets = cli_register_flag(app, 'a', string_lit("assets"), CliOptionFlags_Value);
  cli_register_desc(app, g_optAssets, string_lit("Path to asset directory / pack file."));
  cli_register_validator(app, g_optAssets, cli_validate_file);

  g_optWidth = cli_register_flag(app, '\0', string_lit("width"), CliOptionFlags_Value);
  cli_register_desc(app, g_optWidth, string_lit("Window width in pixels."));
  cli_register_validator(app, g_optWidth, cli_validate_u16);

  g_optHeight = cli_register_flag(app, '\0', string_lit("height"), CliOptionFlags_Value);
  cli_register_desc(app, g_optHeight, string_lit("Window height in pixels."));
  cli_register_validator(app, g_optHeight, cli_validate_u16);

  return AppType_Gui;
}

static void game_crash_handler(const String message, void* ctx) {
  (void)ctx;
  /**
   * Application has crashed.
   * NOTE: Crashes are always fatal, this handler cannot prevent application shutdown. Care must be
   * taken while writing this handler as the application is in an unknown state.
   */
  gap_window_modal_error(message);
}

void app_ecs_register(EcsDef* def, const CliInvocation* invoc) {
  (void)invoc;
  diag_crash_handler(game_crash_handler, null); // Register a crash handler.

  asset_register(def, &(AssetRegisterContext){.devSupport = false});
  gap_register(def);
  rend_register(def, &(RendRegisterContext){.enableStats = false});
  ui_register(def);
  ecs_register_module(def, demo_module);
}

static AssetManagerComp* demo_init_assets(EcsWorld* world, const CliInvocation* invoc) {
  const AssetManagerFlags flags        = AssetManagerFlags_PortableCache;
  const String            overridePath = cli_read_string(invoc, g_optAssets, string_empty);
  if (!string_is_empty(overridePath)) {
    const FileInfo overrideInfo = file_stat_path_sync(overridePath);
    switch (overrideInfo.type) {
    case FileType_Regular:
      return asset_manager_create_pack(world, flags, overridePath);
    case FileType_Directory:
      return asset_manager_create_fs(world, flags | AssetManagerFlags_TrackChanges, overridePath);
    default:
      log_e("Asset directory / pack file not found", log_param("path", fmt_path(overridePath)));
      return null;
    }
  }
  const String pathPackDefault = string_lit("assets.blob");
  if (file_stat_path_sync(pathPackDefault).type == FileType_Regular) {
    return asset_manager_create_pack(world, flags, pathPackDefault);
  }
  const String pathFsDefault = string_lit("assets");
  if (file_stat_path_sync(pathFsDefault).type == FileType_Directory) {
    return asset_manager_create_fs(world, flags | AssetManagerFlags_TrackChanges, pathFsDefault);
  }
  log_e("No asset source found");
  return null;
}

bool app_ecs_init(EcsWorld* world, const CliInvocation* invoc) {
  AssetManagerComp* assets = demo_init_assets(world, invoc);
  if (UNLIKELY(!assets)) {
    gap_window_modal_error(string_lit("No (valid) assets found"));
    return false; // Initialization failed.
  }

  rend_settings_global_init(world, false /* devSupport */);
  ui_settings_global_init(world);

  const u16 windowWidth  = (u16)cli_read_u64(invoc, g_optWidth, 1200);
  const u16 windowHeight = (u16)cli_read_u64(invoc, g_optHeight, 1200);

  demo_create(world, windowWidth, windowHeight);

  return true; // Initialization succeeded.
}

AppEcsStatus app_ecs_status(EcsWorld* world) {
  /**
   * Detect any fatal errors.
   */
  EcsView*            errView    = ecs_world_view_t(world, ErrorView);
  EcsIterator*        errItr     = ecs_view_at(errView, ecs_world_global(world));
  const GapErrorComp* errGapComp = ecs_view_read_t(errItr, GapErrorComp);
  if (errGapComp) {
    log_e("Fatal platform error", log_param("error", fmt_text(gap_error_str(errGapComp->type))));
    gap_window_modal_error(gap_error_str(errGapComp->type));
    return AppEcsStatus_Failed;
  }
  const RendErrorComp* errRendComp = ecs_view_read_t(errItr, RendErrorComp);
  if (errRendComp) {
    log_e("Fatal renderer error", log_param("error", fmt_text(rend_error_str(errRendComp->type))));
    gap_window_modal_error(rend_error_str(errRendComp->type));
    return AppEcsStatus_Failed;
  }
  /**
   * Run until the window has closed.
   */
  if (!ecs_utils_any(world, WindowView)) {
    return AppEcsStatus_Finished;
  }
  return AppEcsStatus_Running;
}

void app_ecs_set_frame(EcsWorld* world, const u64 frameIdx) {
  EcsView*     view = ecs_world_view_t(world, FrameUpdateView);
  EcsIterator* itr  = ecs_view_maybe_at(view, ecs_world_global(world));
  if (LIKELY(itr)) {
    ecs_view_write_t(itr, RendSettingsGlobalComp)->frameIdx = frameIdx;
  }
}

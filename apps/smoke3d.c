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
#include "core/dynstring.h"
#include "core/file.h"
#include "core/float.h"
#include "core/format.h"
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
#include "ui/layout.h"
#include "ui/register.h"
#include "ui/settings.h"
#include "ui/shape.h"
#include "ui/style.h"
#include "ui/widget.h"

/**
 * 3D Eulerian fluid simulation with Semi-Lagrangian advection.
 * Reference: https://www.cs.ubc.ca/~rbridson/fluidsimulation/fluids_notes.pdf
 * Reference: https://www.youtube.com/watch?v=Q78wvrQ9xsU
 */

typedef struct {
  i32 x, y, z;
} SimCoord;

typedef struct {
  f32 x, y, z;
} SimCoordFrac;

static SimCoord sim_coord_round_down(const SimCoordFrac c) {
  return (SimCoord){
      .x = (i32)math_round_down_f32(c.x),
      .y = (i32)math_round_down_f32(c.y),
      .z = (i32)math_round_down_f32(c.z),
  };
}

static bool sim_coord_equal(const SimCoord a, const SimCoord b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

static bool sim_coord_valid(const SimCoord c, const u32 width, const u32 height, const u32 depth) {
  if (c.x < 0 || c.x >= (i32)width) {
    return false;
  }
  if (c.y < 0 || c.y >= (i32)height) {
    return false;
  }
  if (c.z < 0 || c.z >= (i32)depth) {
    return false;
  }
  return true;
}

static u32 sim_coord_dist_manhattan(const SimCoord a, const SimCoord b) {
  return (u32)(math_abs(a.x - b.x) + math_abs(a.y - b.y) + math_abs(a.z - b.z));
}

typedef struct {
  u32  width, height, depth;
  f32* values;
} SimGrid;

static SimGrid sim_grid_create(Allocator* a, const u32 width, const u32 height, const u32 depth) {
  return (SimGrid){
      .width  = width,
      .height = height,
      .depth  = depth,
      .values = alloc_array_t(a, f32, height * width * depth),
  };
}

static void sim_grid_destroy(Allocator* a, SimGrid* g) {
  alloc_free_array_t(a, g->values, g->height * g->width * g->depth);
}

static u32 sim_grid_count(const SimGrid* g) { return g->height * g->width * g->depth; }

static void sim_grid_copy(const SimGrid* dst, const SimGrid* src) {
  const u32 count = sim_grid_count(src);
  diag_assert(sim_grid_count(dst) == count);
  const usize size = sizeof(f32) * count;
  mem_cpy(mem_create(dst->values, size), mem_create(src->values, size));
}

static void sim_grid_clear(SimGrid* g) {
  mem_set(mem_create(g->values, sim_grid_count(g) * sizeof(f32)), 0);
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
  diag_assert(sim_coord_valid(c, g->width, g->height, g->depth));
  return c.z * g->height * g->width + c.y * g->width + c.x;
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
  return sim_coord_valid(c, g->width, g->height, g->depth) ? sim_grid_get(g, c) : fallback;
}

static f32 sim_grid_sample(const SimGrid* g, const SimCoordFrac c, const f32 fallback) {
  const SimCoord cI = sim_coord_round_down(c);

  const SimCoord c000 = {cI.x, cI.y, cI.z};
  const SimCoord c100 = {cI.x + 1, cI.y, cI.z};
  const SimCoord c010 = {cI.x, cI.y + 1, cI.z};
  const SimCoord c110 = {cI.x + 1, cI.y + 1, cI.z};
  const SimCoord c001 = {cI.x, cI.y, cI.z + 1};
  const SimCoord c101 = {cI.x + 1, cI.y, cI.z + 1};
  const SimCoord c011 = {cI.x, cI.y + 1, cI.z + 1};
  const SimCoord c111 = {cI.x + 1, cI.y + 1, cI.z + 1};

  const f32 v000 = sim_grid_get_bounded(g, c000, fallback);
  const f32 v100 = sim_grid_get_bounded(g, c100, fallback);
  const f32 v010 = sim_grid_get_bounded(g, c010, fallback);
  const f32 v110 = sim_grid_get_bounded(g, c110, fallback);
  const f32 v001 = sim_grid_get_bounded(g, c001, fallback);
  const f32 v101 = sim_grid_get_bounded(g, c101, fallback);
  const f32 v011 = sim_grid_get_bounded(g, c011, fallback);
  const f32 v111 = sim_grid_get_bounded(g, c111, fallback);

  const f32 fX = c.x - (f32)cI.x;
  const f32 fY = c.y - (f32)cI.y;
  const f32 fZ = c.z - (f32)cI.z;

  const f32 x00 = math_lerp(v000, v100, fX);
  const f32 x10 = math_lerp(v010, v110, fX);
  const f32 x01 = math_lerp(v001, v101, fX);
  const f32 x11 = math_lerp(v011, v111, fX);

  const f32 y0 = math_lerp(x00, x10, fY);
  const f32 y1 = math_lerp(x01, x11, fY);

  return math_lerp(y0, y1, fZ);
}

typedef struct {
  SimCoord position;
  f32      smokeAmount, angle /* in radians */, force;
} SimEmitter;

typedef struct {
  u32 width, height, depth;

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

  bool     guide;
  SimCoord guideCoord;
  f32      guideForce;
  f32      guideAngle;

  SimEmitter emitters[4];
  u32        emitterCount;

  // NOTE: Velocities are stored at the edges, not the cell centers.
  SimGrid velocitiesX; // (width + 1) height * depth
  SimGrid velocitiesY; // width * (height + 1) * depth
  SimGrid velocitiesZ; // width * height * (depth + 1)

  SimGrid pressure;
  SimGrid smoke;
  BitSet  solid;
} SimState;

static SimState sim_state_create(const u32 width, const u32 height, const u32 depth) {
  SimState s = {
      .width  = width,
      .height = height,

      .solverIterations  = 128,
      .density           = 10.0f,
      .pressureDecay     = 0.5f,
      .velocityDiffusion = 0.5f,
      .smokeDiffusion    = 0.05f,
      .smokeDecay        = 0.01f,
      .pushPressure      = 100.0f,
      .pullForce         = 100.0f,
      .guideForce        = 500.0f,
      .guideAngle        = math_pi_f32,

      .velocitiesX = sim_grid_create(g_allocHeap, width + 1, height, depth),
      .velocitiesY = sim_grid_create(g_allocHeap, width, height + 1, depth),
      .velocitiesZ = sim_grid_create(g_allocHeap, width, height, depth + 1),
      .pressure    = sim_grid_create(g_allocHeap, width, height, depth),
      .smoke       = sim_grid_create(g_allocHeap, width, height, depth),
      .solid       = alloc_alloc(g_allocHeap, bits_to_bytes(height * width * depth) + 1, 1),
  };

  sim_grid_clear(&s.velocitiesX);
  sim_grid_clear(&s.velocitiesY);
  sim_grid_clear(&s.velocitiesZ);
  sim_grid_clear(&s.pressure);
  sim_grid_clear(&s.smoke);
  mem_set(s.solid, 0);

  return s;
}

static void sim_state_destroy(SimState* s) {
  sim_grid_destroy(g_allocHeap, &s->velocitiesX);
  sim_grid_destroy(g_allocHeap, &s->velocitiesY);
  sim_grid_destroy(g_allocHeap, &s->velocitiesZ);
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

static bool sim_emitter_add_default(SimState* s, const SimCoord c) {
  return sim_emitter_add(
      s,
      (SimEmitter){
          .angle       = math_pi_f32 * 0.75f,
          .force       = 1000.0f,
          .position    = c,
          .smokeAmount = 5.0f,
      });
}

static SimEmitter* sim_emitter_get(SimState* s, const SimCoord c) {
  for (u32 i = 0; i != s->emitterCount; ++i) {
    if (sim_coord_equal(s->emitters[i].position, c)) {
      return &s->emitters[i];
    }
  }
  return null;
}

static void sim_emitter_remove(SimState* s, const SimEmitter* e) {
  const u32 index  = e - s->emitters;
  const u32 toMove = (s->emitterCount - 1) - index;
  if (toMove) {
    const Mem dst = mem_create(&s->emitters[index], toMove * sizeof(SimEmitter));
    const Mem src = mem_create(&s->emitters[index + 1], toMove * sizeof(SimEmitter));
    mem_move(dst, src);
  }
  --s->emitterCount;
}

static bool sim_solid(const SimState* s, const SimCoord c) {
  if (!sim_coord_valid(c, s->width, s->height, s->depth)) {
    return false;
  }
  return bitset_test(s->solid, c.z * s->height * s->width + c.y * s->width + c.x);
}

static void sim_solid_flip(SimState* s, const SimCoord c) {
  diag_assert(sim_coord_valid(c, s->width, s->height, s->depth));
  bitset_flip(s->solid, c.z * s->height * s->width + c.y * s->width + c.x);
}

static void sim_solid_set(SimState* s, const SimCoord c) {
  diag_assert(sim_coord_valid(c, s->width, s->height, s->depth));
  bitset_set(s->solid, c.z * s->height * s->width + c.y * s->width + c.x);
}

static void sim_solid_set_border(SimState* s) {
  for (u32 z = 0; z != s->depth; ++z) {
    for (u32 y = 0; y != s->height; ++y) {
      for (u32 x = 0; x != s->width; ++x) {
        if (!x || !y || !z || x == (s->width - 1) || y == (s->height - 1) || z == (s->depth - 1)) {
          bitset_set(s->solid, z * s->height * s->width + y * s->width + x);
        }
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

static void
sim_velocity_add(SimState* s, const SimCoord c, const f32 vX, const f32 vY, const f32 vZ) {
  sim_grid_add(&s->velocitiesX, c, vX);                             // Left.
  sim_grid_add(&s->velocitiesX, (SimCoord){c.x + 1, c.y, c.z}, vX); // Right.
  sim_grid_add(&s->velocitiesY, c, vY);                             // Bottom.
  sim_grid_add(&s->velocitiesY, (SimCoord){c.x, c.y + 1, c.z}, vY); // Top.
  sim_grid_add(&s->velocitiesZ, c, vZ);                             // Back.
  sim_grid_add(&s->velocitiesZ, (SimCoord){c.x, c.y, c.z + 1}, vZ); // Front.
}

static void sim_velocity_clear(SimState* s) {
  sim_grid_clear(&s->velocitiesX);
  sim_grid_clear(&s->velocitiesY);
  sim_grid_clear(&s->velocitiesZ);
}

static void sim_velocity_randomize(SimState* s) {
  sim_grid_rand(&s->velocitiesX, -25.0f, 25.0f);
  sim_grid_rand(&s->velocitiesY, -25.0f, 25.0f);
  sim_grid_rand(&s->velocitiesZ, -25.0f, 25.0f);
}

static f32 sim_velocity_bottom(const SimState* s, const SimCoord c) {
  return sim_grid_get_bounded(&s->velocitiesY, c, 0.0f /* fallback */);
}

static f32 sim_velocity_top(const SimState* s, const SimCoord c) {
  const SimCoord cAbove = {c.x, c.y + 1, c.z};
  return sim_grid_get_bounded(&s->velocitiesY, cAbove, 0.0f /* fallback */);
}

static f32 sim_velocity_left(const SimState* s, const SimCoord c) {
  return sim_grid_get_bounded(&s->velocitiesX, c, 0.0f /* fallback */);
}

static f32 sim_velocity_right(const SimState* s, const SimCoord c) {
  const SimCoord cRight = {c.x + 1, c.y, c.z};
  return sim_grid_get_bounded(&s->velocitiesX, cRight, 0.0f /* fallback */);
}

static f32 sim_velocity_back(const SimState* s, const SimCoord c) {
  return sim_grid_get_bounded(&s->velocitiesZ, c, 0.0f /* fallback */);
}

static f32 sim_velocity_front(const SimState* s, const SimCoord c) {
  const SimCoord cFront = {c.x, c.y, c.z + 1};
  return sim_grid_get_bounded(&s->velocitiesZ, cFront, 0.0f /* fallback */);
}

static f32 sim_velocity_x(const SimState* s, const SimCoord c) {
  return sim_grid_sample(&s->velocitiesX, (SimCoordFrac){c.x + 0.5f, c.y, c.z}, 0.0f);
}

static f32 sim_velocity_y(const SimState* s, const SimCoord c) {
  return sim_grid_sample(&s->velocitiesY, (SimCoordFrac){c.x, c.y + 0.5f, c.z}, 0.0f);
}

static f32 sim_velocity_z(const SimState* s, const SimCoord c) {
  return sim_grid_sample(&s->velocitiesZ, (SimCoordFrac){c.x, c.y, c.z + 0.5f}, 0.0f);
}

static f32 sim_velocity_divergence(const SimState* s, const SimCoord c) {
  const f32 vTop    = sim_velocity_top(s, c);
  const f32 vLeft   = sim_velocity_left(s, c);
  const f32 vRight  = sim_velocity_right(s, c);
  const f32 vBottom = sim_velocity_bottom(s, c);
  const f32 vBack   = sim_velocity_back(s, c);
  const f32 vFront  = sim_velocity_front(s, c);
  return (vRight - vLeft) + (vTop - vBottom) + (vFront - vBack);
}

static f32 sim_velocity_divergence_sum(const SimState* s) {
  f32 result = 0;
  for (u32 z = 0; z != s->depth; ++z) {
    for (u32 y = 0; y != s->height; ++y) {
      for (u32 x = 0; x != s->width; ++x) {
        const f32 divergence = sim_velocity_divergence(s, (SimCoord){x, y, z});
        result += math_abs(divergence);
      }
    }
  }
  return result;
}

static f32 sim_speed(const SimState* s, const SimCoord c) {
  const f32 vX       = sim_velocity_x(s, c);
  const f32 vY       = sim_velocity_y(s, c);
  const f32 vZ       = sim_velocity_z(s, c);
  const f32 speedSqr = vX * vX + vY * vY + vZ * vZ;
  return speedSqr != 0.0f ? intrinsic_sqrt_f32(speedSqr) : 0.0f;
}

/**
 * NOTE: Returns the 2d angle in the xz plane.
 */
static f32 sim_angle(const SimState* s, const SimCoord c) {
  const f32 vX = sim_velocity_x(s, c);
  const f32 vZ = sim_velocity_z(s, c);
  if (math_abs(vX) < f32_epsilon && math_abs(vZ) < f32_epsilon) {
    return 0.0f;
  }
  return math_atan2_f32(vZ, vX);
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
  for (u32 z = 0; z != s->velocitiesX.depth; ++z) {
    for (u32 y = 0; y != s->velocitiesX.height; ++y) {
      for (u32 x = 0; x != s->velocitiesX.width; ++x) {
        const f32 sampleX = (f32)x;
        const f32 sampleY = (f32)y + 0.5f;
        const f32 sampleZ = (f32)z + 0.5f;
        const f32 dx      = target.x - sampleX;
        const f32 dy      = target.y - sampleY;
        const f32 dz      = target.z - sampleZ;
        const f32 distSqr = dx * dx + dy * dy + dz * dz;
        if (distSqr < f32_epsilon) {
          continue;
        }
        const f32 forceMul  = sim_pull_force(s, (SimCoordFrac){sampleX, sampleY, sampleZ});
        const f32 veloDelta = (dx / intrinsic_sqrt_f32(distSqr)) * force * forceMul;
        sim_grid_add(&s->velocitiesX, (SimCoord){x, y, z}, veloDelta);
      }
    }
  }

  // Vertical.
  for (u32 z = 0; z != s->velocitiesY.depth; ++z) {
    for (u32 y = 0; y != s->velocitiesY.height; ++y) {
      for (u32 x = 0; x != s->velocitiesY.width; ++x) {
        const f32 sampleX = (f32)x + 0.5f;
        const f32 sampleY = (f32)y;
        const f32 sampleZ = (f32)z + 0.5f;
        const f32 dx      = target.x - sampleX;
        const f32 dy      = target.y - sampleY;
        const f32 dz      = target.z - sampleZ;
        const f32 distSqr = dx * dx + dy * dy + dz * dz;
        if (distSqr < f32_epsilon)
          continue;
        const f32 forceMul  = sim_pull_force(s, (SimCoordFrac){sampleX, sampleY, sampleZ});
        const f32 veloDelta = (dy / intrinsic_sqrt_f32(distSqr)) * force * forceMul;
        sim_grid_add(&s->velocitiesY, (SimCoord){x, y, z}, veloDelta);
      }
    }
  }

  // Depth.
  for (u32 z = 0; z != s->velocitiesZ.depth; ++z) {
    for (u32 y = 0; y != s->velocitiesZ.height; ++y) {
      for (u32 x = 0; x != s->velocitiesZ.width; ++x) {
        const f32 sampleX = (f32)x + 0.5f;
        const f32 sampleY = (f32)y + 0.5f;
        const f32 sampleZ = (f32)z;
        const f32 dx      = target.x - sampleX;
        const f32 dy      = target.y - sampleY;
        const f32 dz      = target.z - sampleZ;
        const f32 distSqr = dx * dx + dy * dy + dz * dz;
        if (distSqr < f32_epsilon)
          continue;
        const f32 forceMul  = sim_pull_force(s, (SimCoordFrac){sampleX, sampleY, sampleZ});
        const f32 veloDelta = (dz / intrinsic_sqrt_f32(distSqr)) * force * forceMul;
        sim_grid_add(&s->velocitiesZ, (SimCoord){x, y, z}, veloDelta);
      }
    }
  }
}

static void sim_diffuse_velocity_grid(SimGrid* g, const f32 diffusion, const f32 dt) {
  for (u32 z = 0; z != g->depth; ++z) {
    for (u32 y = 0; y != g->height; ++y) {
      for (u32 x = 0; x != g->width; ++x) {
        const f32 vCenter = sim_grid_get(g, (SimCoord){x, y, z});
        const f32 vLeft   = sim_grid_get_bounded(g, (SimCoord){x - 1, y, z}, vCenter);
        const f32 vRight  = sim_grid_get_bounded(g, (SimCoord){x + 1, y, z}, vCenter);
        const f32 vTop    = sim_grid_get_bounded(g, (SimCoord){x, y + 1, z}, vCenter);
        const f32 vBottom = sim_grid_get_bounded(g, (SimCoord){x, y - 1, z}, vCenter);
        const f32 vFront  = sim_grid_get_bounded(g, (SimCoord){x, y, z + 1}, vCenter);
        const f32 vBack   = sim_grid_get_bounded(g, (SimCoord){x, y, z - 1}, vCenter);

        const f32 laplacian = vLeft + vRight + vTop + vBottom + vFront + vBack - 6.0f * vCenter;
        const f32 vDiffused = vCenter + laplacian * diffusion * dt;
        sim_grid_set(g, (SimCoord){x, y, z}, vDiffused);
      }
    }
  }
}

static void sim_diffuse_velocity(SimState* s, const f32 dt) {
  if (s->velocityDiffusion >= f32_epsilon) {
    sim_diffuse_velocity_grid(&s->velocitiesX, s->velocityDiffusion, dt);
    sim_diffuse_velocity_grid(&s->velocitiesY, s->velocityDiffusion, dt);
    sim_diffuse_velocity_grid(&s->velocitiesZ, s->velocityDiffusion, dt);
  }
}

static void sim_diffuse_smoke(SimState* s, const f32 dt) {
  if (s->smokeDiffusion < f32_epsilon) {
    return;
  }
  for (u32 z = 0; z != s->depth; ++z) {
    for (u32 y = 0; y != s->height; ++y) {
      for (u32 x = 0; x != s->width; ++x) {
        if (sim_solid(s, (SimCoord){x, y, z})) {
          continue;
        }
        const f32 vCenter = sim_grid_get(&s->smoke, (SimCoord){x, y, z});
        const f32 vLeft   = sim_grid_get_bounded(&s->smoke, (SimCoord){x - 1, y, z}, 0.0f);
        const f32 vRight  = sim_grid_get_bounded(&s->smoke, (SimCoord){x + 1, y, z}, 0.0f);
        const f32 vTop    = sim_grid_get_bounded(&s->smoke, (SimCoord){x, y + 1, z}, 0.0f);
        const f32 vBottom = sim_grid_get_bounded(&s->smoke, (SimCoord){x, y - 1, z}, 0.0f);
        const f32 vFront  = sim_grid_get_bounded(&s->smoke, (SimCoord){x, y, z + 1}, 0.0f);
        const f32 vBack   = sim_grid_get_bounded(&s->smoke, (SimCoord){x, y, z - 1}, 0.0f);

        const f32 laplacian     = vLeft + vRight + vTop + vBottom + vFront + vBack - 6.0f * vCenter;
        const f32 smokeDiffused = vCenter + s->smokeDiffusion * dt * laplacian;
        const f32 smokeNew      = smokeDiffused * math_exp_f32(-dt * s->smokeDecay);
        sim_grid_set(&s->smoke, (SimCoord){x, y, z}, smokeNew);
      }
    }
  }
}

static void sim_advect_velocity(SimState* s, const f32 dt) {
  SimGrid velocitiesXNew = sim_grid_create(g_allocScratch, s->width + 1, s->height, s->depth);
  SimGrid velocitiesYNew = sim_grid_create(g_allocScratch, s->width, s->height + 1, s->depth);
  SimGrid velocitiesZNew = sim_grid_create(g_allocScratch, s->width, s->height, s->depth + 1);

  // Horizontal.
  for (u32 z = 0; z != velocitiesXNew.depth; ++z) {
    for (u32 y = 0; y != velocitiesXNew.height; ++y) {
      for (u32 x = 0; x != velocitiesXNew.width; ++x) {
        const SimCoord cellLeft  = (SimCoord){x - 1, y, z};
        const SimCoord cellRight = (SimCoord){x + 0, y, z};
        if (sim_solid(s, cellLeft) || sim_solid(s, cellRight)) {
          sim_grid_set(&velocitiesXNew, (SimCoord){x, y, z}, 0.0f);
          continue;
        }
        const f32 veloX = sim_grid_get(&s->velocitiesX, (SimCoord){x, y, z});
        const f32 veloY =
            sim_grid_sample(&s->velocitiesY, (SimCoordFrac){x - 0.5f, y + 0.5f, z}, 0.0f);
        const f32 veloZ =
            sim_grid_sample(&s->velocitiesZ, (SimCoordFrac){x - 0.5f, y, z + 0.5f}, 0.0f);

        const f32 prevX = x - veloX * dt;
        const f32 prevY = y - veloY * dt;
        const f32 prevZ = z - veloZ * dt;

        const f32 veloNew =
            sim_grid_sample(&s->velocitiesX, (SimCoordFrac){prevX, prevY, prevZ}, 0.0f);
        sim_grid_set(&velocitiesXNew, (SimCoord){x, y, z}, veloNew);
      }
    }
  }

  // Vertical.
  for (u32 z = 0; z != velocitiesYNew.depth; ++z) {
    for (u32 y = 0; y != velocitiesYNew.height; ++y) {
      for (u32 x = 0; x != velocitiesYNew.width; ++x) {
        const SimCoord cellBottom = (SimCoord){x, y - 1, z};
        const SimCoord cellTop    = (SimCoord){x, y + 0, z};
        if (sim_solid(s, cellBottom) || sim_solid(s, cellTop)) {
          sim_grid_set(&velocitiesYNew, (SimCoord){x, y, z}, 0.0f);
          continue;
        }
        const f32 veloX =
            sim_grid_sample(&s->velocitiesX, (SimCoordFrac){x + 0.5f, y - 0.5f, z}, 0.0f);
        const f32 veloY = sim_grid_get(&s->velocitiesY, (SimCoord){x, y, z});
        const f32 veloZ =
            sim_grid_sample(&s->velocitiesZ, (SimCoordFrac){x, y - 0.5f, z + 0.5f}, 0.0f);

        const f32 prevX = x - veloX * dt;
        const f32 prevY = y - veloY * dt;
        const f32 prevZ = z - veloZ * dt;

        const f32 veloNew =
            sim_grid_sample(&s->velocitiesY, (SimCoordFrac){prevX, prevY, prevZ}, 0.0f);
        sim_grid_set(&velocitiesYNew, (SimCoord){x, y, z}, veloNew);
      }
    }
  }

  // Depth.
  for (u32 z = 0; z != velocitiesZNew.depth; ++z) {
    for (u32 y = 0; y != velocitiesZNew.height; ++y) {
      for (u32 x = 0; x != velocitiesZNew.width; ++x) {
        const SimCoord cellBack  = (SimCoord){x, y, z - 1};
        const SimCoord cellFront = (SimCoord){x, y, z + 0};
        if (sim_solid(s, cellBack) || sim_solid(s, cellFront)) {
          sim_grid_set(&velocitiesZNew, (SimCoord){x, y, z}, 0.0f);
          continue;
        }
        const f32 veloX =
            sim_grid_sample(&s->velocitiesX, (SimCoordFrac){x + 0.5f, y, z - 0.5f}, 0.0f);
        const f32 veloY =
            sim_grid_sample(&s->velocitiesY, (SimCoordFrac){x, y + 0.5f, z - 0.5f}, 0.0f);
        const f32 veloZ = sim_grid_get(&s->velocitiesZ, (SimCoord){x, y, z});

        const f32 prevX = x - veloX * dt;
        const f32 prevY = y - veloY * dt;
        const f32 prevZ = z - veloZ * dt;

        const f32 veloNew =
            sim_grid_sample(&s->velocitiesZ, (SimCoordFrac){prevX, prevY, prevZ}, 0.0f);
        sim_grid_set(&velocitiesZNew, (SimCoord){x, y, z}, veloNew);
      }
    }
  }

  sim_grid_copy(&s->velocitiesX, &velocitiesXNew);
  sim_grid_copy(&s->velocitiesY, &velocitiesYNew);
  sim_grid_copy(&s->velocitiesZ, &velocitiesZNew);
}

static void sim_advect_smoke(SimState* s, const f32 dt) {
  SimGrid smokeNew = sim_grid_create(g_allocScratch, s->width, s->height, s->depth);

  for (u32 z = 0; z != s->depth; ++z) {
    for (u32 y = 0; y != s->height; ++y) {
      for (u32 x = 0; x != s->width; ++x) {
        const SimCoord cell = {x, y, z};
        if (sim_solid(s, cell) || sim_pushed(s, cell)) {
          sim_grid_set(&smokeNew, cell, 0.0f);
          continue;
        }
        const f32 veloX = sim_velocity_x(s, cell);
        const f32 veloY = sim_velocity_y(s, cell);
        const f32 veloZ = sim_velocity_z(s, cell);

        const f32 prevX = (f32)x - veloX * dt;
        const f32 prevY = (f32)y - veloY * dt;
        const f32 prevZ = (f32)z - veloZ * dt;

        const f32 prevSmoke = sim_grid_sample(&s->smoke, (SimCoordFrac){prevX, prevY, prevZ}, 0.0f);
        sim_grid_set(&smokeNew, cell, prevSmoke);
      }
    }
  }

  sim_grid_copy(&s->smoke, &smokeNew);
}

static void sim_solve_pressure(SimState* s, const f32 dt) {
  if (s->density <= f32_epsilon) {
    return;
  }

  for (u32 z = 0; z != s->depth; ++z) {
    for (u32 y = 0; y != s->height; ++y) {
      for (u32 x = 0; x != s->width; ++x) {
        const SimCoord coord = (SimCoord){x, y, z};

        const bool flowTop    = !sim_solid(s, (SimCoord){x, y + 1, z});
        const bool flowBottom = !sim_solid(s, (SimCoord){x, y - 1, z});
        const bool flowLeft   = !sim_solid(s, (SimCoord){x - 1, y, z});
        const bool flowRight  = !sim_solid(s, (SimCoord){x + 1, y, z});
        const bool flowFront  = !sim_solid(s, (SimCoord){x, y, z + 1});
        const bool flowBack   = !sim_solid(s, (SimCoord){x, y, z - 1});
        const u8   flowCount =
            (u8)(flowLeft + flowRight + flowTop + flowBottom + flowFront + flowBack);

        f32 newPressure;
        if (sim_pushed(s, coord)) {
          newPressure = s->pushPressure;
        } else if (sim_solid(s, coord) || !flowCount) {
          newPressure = 0.0f;
        } else {
          const f32 pressureLeft   = sim_pressure(s, (SimCoord){x - 1, y, z}) * flowLeft;
          const f32 pressureRight  = sim_pressure(s, (SimCoord){x + 1, y, z}) * flowRight;
          const f32 pressureTop    = sim_pressure(s, (SimCoord){x, y + 1, z}) * flowTop;
          const f32 pressureBottom = sim_pressure(s, (SimCoord){x, y - 1, z}) * flowBottom;
          const f32 pressureFront  = sim_pressure(s, (SimCoord){x, y, z + 1}) * flowFront;
          const f32 pressureBack   = sim_pressure(s, (SimCoord){x, y, z - 1}) * flowBack;
          const f32 pressureSum    = pressureLeft + pressureRight + pressureTop + pressureBottom +
                                  pressureFront + pressureBack;

          const f32 velLeft   = sim_velocity_left(s, coord);
          const f32 velRight  = sim_velocity_right(s, coord);
          const f32 velTop    = sim_velocity_top(s, coord);
          const f32 velBottom = sim_velocity_bottom(s, coord);
          const f32 velFront  = sim_velocity_front(s, coord);
          const f32 velBack   = sim_velocity_back(s, coord);

          const f32 velDelta = velRight - velLeft + velTop - velBottom + velFront - velBack;

          newPressure = (pressureSum - s->density * velDelta / dt) / (f32)flowCount;
          newPressure -= newPressure * math_min(0.0f, s->density * s->pressureDecay * dt);

          diag_assert(!float_isnan(newPressure) && !float_isinf(newPressure));
        }

        sim_grid_set(&s->pressure, coord, newPressure);
      }
    }
  }
}

static void sim_solve_velocity(SimState* s, const f32 dt) {
  if (s->density <= f32_epsilon) {
    return;
  }
  const f32 k = dt / s->density;

  // Horizontal.
  for (u32 z = 0; z != s->velocitiesX.depth; ++z) {
    for (u32 y = 0; y != s->velocitiesX.height; ++y) {
      for (u32 x = 0; x != s->velocitiesX.width; ++x) {
        const SimCoord cellLeft  = (SimCoord){x - 1, y, z};
        const SimCoord cellRight = (SimCoord){x + 0, y, z};
        if (sim_solid(s, cellLeft) || sim_solid(s, cellRight)) {
          sim_grid_set(&s->velocitiesX, (SimCoord){x, y, z}, 0.0f);
          continue;
        }
        const f32 pressureLeft  = sim_pressure(s, cellLeft);
        const f32 pressureRight = sim_pressure(s, cellRight);
        sim_grid_add(&s->velocitiesX, (SimCoord){x, y, z}, k * -(pressureRight - pressureLeft));
      }
    }
  }

  // Vertical.
  for (u32 z = 0; z != s->velocitiesY.depth; ++z) {
    for (u32 y = 0; y != s->velocitiesY.height; ++y) {
      for (u32 x = 0; x != s->velocitiesY.width; ++x) {
        const SimCoord cellBottom = (SimCoord){x, y - 1, z};
        const SimCoord cellTop    = (SimCoord){x, y + 0, z};
        if (sim_solid(s, cellBottom) || sim_solid(s, cellTop)) {
          sim_grid_set(&s->velocitiesY, (SimCoord){x, y, z}, 0.0f);
          continue;
        }
        const f32 pressureBottom = sim_pressure(s, cellBottom);
        const f32 pressureTop    = sim_pressure(s, cellTop);
        sim_grid_add(&s->velocitiesY, (SimCoord){x, y, z}, k * -(pressureTop - pressureBottom));
      }
    }
  }

  // Depth.
  for (u32 z = 0; z != s->velocitiesZ.depth; ++z) {
    for (u32 y = 0; y != s->velocitiesZ.height; ++y) {
      for (u32 x = 0; x != s->velocitiesZ.width; ++x) {
        const SimCoord cellBack  = (SimCoord){x, y, z - 1};
        const SimCoord cellFront = (SimCoord){x, y, z + 0};
        if (sim_solid(s, cellBack) || sim_solid(s, cellFront)) {
          sim_grid_set(&s->velocitiesZ, (SimCoord){x, y, z}, 0.0f);
          continue;
        }
        const f32 pressureBack  = sim_pressure(s, cellBack);
        const f32 pressureFront = sim_pressure(s, cellFront);
        sim_grid_add(&s->velocitiesZ, (SimCoord){x, y, z}, k * -(pressureFront - pressureBack));
      }
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
    const f32 veloZ = math_sin_f32(e->angle) * e->force * dt;
    sim_velocity_add(s, e->position, veloX, 0.0f, veloZ);
  }

  if (s->pull) {
    sim_pull(s, s->pullCoord, s->pullForce * dt);
  }
  if (s->guide) {
    const f32 veloX = math_cos_f32(s->guideAngle) * s->guideForce * dt;
    const f32 veloZ = math_sin_f32(s->guideAngle) * s->guideForce * dt;
    sim_velocity_add(s, s->guideCoord, veloX, 0.0f, veloZ);
  }

  sim_diffuse_velocity(s, dt);
  sim_diffuse_smoke(s, dt);
  sim_advect_velocity(s, dt);
  sim_advect_smoke(s, dt);
  for (u32 i = 0; i != s->solverIterations; ++i) {
    sim_solve_pressure(s, dt);
    sim_solve_velocity(s, dt);
  }
  return true;
}

static void sim_clear(SimState* s) {
  sim_smoke_clear(s);
  sim_velocity_clear(s);
  sim_pressure_clear(s);
}

typedef enum {
  DemoInteract_None,
  DemoInteract_Pull,
  DemoInteract_Push,
  DemoInteract_Guide,
  DemoInteract_Solid,
  DemoInteract_Emitter,

  DemoInteract_Count,
} DemoInteract;

static const String g_demoInteractNames[] = {
    string_static("None"),
    string_static("Pull"),
    string_static("Push"),
    string_static("Guide"),
    string_static("Solid"),
    string_static("Emitter"),
};
ASSERT(array_elems(g_demoInteractNames) == DemoInteract_Count, "Incorrect number of names");

typedef enum {
  DemoLayer_SmokeInterp,
  DemoLayer_Smoke,
  DemoLayer_Pressure,
  DemoLayer_Velocity,
  DemoLayer_Divergence,

  DemoLayer_Count,
} DemoLayer;

static const String g_demoLayerNames[] = {
    string_static("SmokeInterp"),
    string_static("Smoke"),
    string_static("Pressure"),
    string_static("Velocity"),
    string_static("Divergence"),
};
ASSERT(array_elems(g_demoLayerNames) == DemoLayer_Count, "Incorrect number of names");

typedef enum {
  DemoOverlay_Solid      = 1 << 0,
  DemoOverlay_Emitter    = 1 << 1,
  DemoOverlay_Velo       = 1 << 2,
  DemoOverlay_VeloCenter = 1 << 3,

  DemoOverlay_Count   = 4,
  DemoOverlay_Default = DemoOverlay_Solid | DemoOverlay_Emitter,
} DemoOverlay;

static const String g_demoOverlayNames[] = {
    string_static("Solid"),
    string_static("Emitter"),
    string_static("Velo"),
    string_static("Velo Center"),
};
ASSERT(array_elems(g_demoOverlayNames) == DemoOverlay_Count, "Incorrect number of names");

typedef enum {
  DemoLabel_None,
  DemoLabel_Smoke,
  DemoLabel_Pressure,
  DemoLabel_Speed,
  DemoLabel_Angle,
  DemoLabel_Divergence,

  DemoLabel_Count,
} DemoLabel;

static const String g_demoLabelNames[] = {
    string_static("None"),
    string_static("Smoke"),
    string_static("Pressure"),
    string_static("Speed"),
    string_static("Angle"),
    string_static("Divergence"),
};
ASSERT(array_elems(g_demoLabelNames) == DemoLabel_Count, "Incorrect number of names");

ecs_comp_define(DemoComp) {
  EcsEntityId window;
  EcsEntityId uiCanvas;
  TimeSteady  lastTime;

  SimState sim;

  bool         hideMenu;
  DemoInteract interact;
  DemoLayer    layer;
  DemoOverlay  overlay;
  DemoLabel    label;
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
  demo->overlay  = DemoOverlay_Default;

  rend_settings_window_init(world, demo->window)->flags |= RendFlags_2D;

  const u32 simWidth  = 40;
  const u32 simHeight = 30;
  demo->sim           = sim_state_create(simWidth, simHeight);

  sim_emitter_add_default(&demo->sim, (SimCoord){37, 2});

  sim_solid_set(&demo->sim, (SimCoord){34, 4});
  sim_solid_set(&demo->sim, (SimCoord){35, 5});

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

static f32 demo_cell_size(UiCanvasComp* c, const SimState* s) {
  const f32 border = 10;
  const f32 xSize  = (ui_canvas_resolution(c).width - border * 2) / (f32)s->width;
  const f32 ySize  = (ui_canvas_resolution(c).height - border * 2) / (f32)s->height;
  return math_min(xSize, ySize);
}

static UiVector demo_cell_origin(UiCanvasComp* c, const SimState* s, const f32 cellSize) {
  return (UiVector){
      ui_canvas_resolution(c).width * 0.5f - s->width * cellSize * 0.5f,
      ui_canvas_resolution(c).height * 0.5f - s->height * cellSize * 0.5f,
  };
}

static UiVector demo_cell_pos(const f32 cellSize, const UiVector cellOrigin, const SimCoord c) {
  return ui_vector(cellOrigin.x + c.x * cellSize, cellOrigin.y + c.y * cellSize);
}

static SimCoordFrac demo_input(UiCanvasComp* c, const f32 cellSize, const UiVector cellOrigin) {
  diag_assert(cellSize > f32_epsilon);
  const UiVector inputPos = ui_canvas_input_pos(c);
  return (SimCoordFrac){
      (inputPos.x - cellOrigin.x) / cellSize,
      (inputPos.y - cellOrigin.y) / cellSize,
  };
}

static void demo_interact(DemoComp* d, const GapWindowComp* w, const SimCoordFrac inputPos) {
  switch (d->interact) {
  case DemoInteract_None:
    break;
  case DemoInteract_Pull:
    d->sim.pull      = true;
    d->sim.pullCoord = inputPos;
    if (gap_window_key_pressed(w, GapKey_Plus) || gap_window_key_pressed(w, GapKey_ArrowUp)) {
      d->sim.pullForce = math_min(d->sim.pullForce * 2.0f, 1e4f);
    }
    if (gap_window_key_pressed(w, GapKey_Minus) || gap_window_key_pressed(w, GapKey_ArrowDown)) {
      d->sim.pullForce = math_max(d->sim.pullForce / 2.0f, 1.0f);
    }
    break;
  case DemoInteract_Push:
    d->sim.pushCoord = sim_coord_round_down(inputPos);
    d->sim.push      = sim_coord_valid(d->sim.pushCoord, d->sim.width, d->sim.height);
    if (gap_window_key_pressed(w, GapKey_Plus) || gap_window_key_pressed(w, GapKey_ArrowUp)) {
      d->sim.pushPressure = math_min(d->sim.pushPressure * 2.0f, 1e4f);
    }
    if (gap_window_key_pressed(w, GapKey_Minus) || gap_window_key_pressed(w, GapKey_ArrowDown)) {
      d->sim.pushPressure = math_max(d->sim.pushPressure / 2.0f, 1.0f);
    }
    break;
  case DemoInteract_Guide: {
    const f32 scroll  = gap_window_param(w, GapParam_ScrollDelta).y;
    d->sim.guideCoord = sim_coord_round_down(inputPos);
    d->sim.guide      = sim_coord_valid(d->sim.guideCoord, d->sim.width, d->sim.height);
    d->sim.guideAngle = math_mod_f32(d->sim.guideAngle + scroll * 0.1f, math_pi_f32 * 2.0f);
    if (gap_window_key_pressed(w, GapKey_Plus) || gap_window_key_pressed(w, GapKey_ArrowUp)) {
      d->sim.guideForce = math_min(d->sim.guideForce * 2.0f, 1e4f);
    }
    if (gap_window_key_pressed(w, GapKey_Minus) || gap_window_key_pressed(w, GapKey_ArrowDown)) {
      d->sim.guideForce = math_max(d->sim.guideForce / 2.0f, 1.0f);
    }
    break;
  }
  case DemoInteract_Solid: {
    const bool     click = gap_window_key_pressed(w, GapKey_MouseLeft);
    const SimCoord coord = sim_coord_round_down(inputPos);
    if (sim_coord_valid(coord, d->sim.width, d->sim.height) && click) {
      sim_solid_flip(&d->sim, coord);
    }
    break;
  }
  case DemoInteract_Emitter: {
    const bool     click   = gap_window_key_pressed(w, GapKey_MouseLeft);
    const SimCoord coord   = sim_coord_round_down(inputPos);
    SimEmitter*    emitter = sim_emitter_get(&d->sim, coord);
    if (emitter) {
      if (gap_window_key_pressed(w, GapKey_MouseLeft)) {
        sim_emitter_remove(&d->sim, emitter);
      } else {
        const f32 scroll = gap_window_param(w, GapParam_ScrollDelta).y;
        emitter->angle   = math_mod_f32(emitter->angle + scroll * 0.1f, math_pi_f32 * 2.0f);
      }
    } else if (sim_coord_valid(coord, d->sim.width, d->sim.height) && click) {
      sim_emitter_add_default(&d->sim, coord);
    }
    break;
  }
  case DemoInteract_Count:
    UNREACHABLE
  }
}

static void demo_draw_grid(
    UiCanvasComp*  c,
    const SimGrid* g,
    const f32      cellSize,
    const UiVector cellOrigin,
    const f32      minVal,
    const f32      maxVal,
    const UiColor  minColor,
    const UiColor  maxColor) {

  ui_layout_push(c);
  ui_layout_resize(c, UiAlign_BottomLeft, ui_vector(cellSize, cellSize), UiBase_Absolute, Ui_XY);
  ui_style_push(c);
  for (u32 y = 0; y != g->height; ++y) {
    for (u32 x = 0; x != g->width; ++x) {
      const SimCoord coord = {x, y};
      const f32      v     = sim_grid_get(g, coord);
      const f32      frac  = math_clamp_f32(math_unlerp(minVal, maxVal, v), 0.0f, 1.0f);

      ui_style_color(c, ui_color_lerp(minColor, maxColor, frac));
      ui_layout_set_pos(
          c, UiBase_Canvas, demo_cell_pos(cellSize, cellOrigin, coord), UiBase_Absolute);

      ui_canvas_draw_glyph(c, UiShape_Square, 5, UiFlags_None);
    }
  }
  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw_grid_sampled(
    UiCanvasComp*  c,
    const SimGrid* g,
    const f32      cellSize,
    const UiVector cellOrigin,
    const f32      minVal,
    const f32      maxVal,
    const UiColor  minColor,
    const UiColor  maxColor,
    const u32      density) {

  const u32 stepsX          = g->width * density;
  const u32 stepsY          = g->height * density;
  const f32 sampledCellSize = cellSize / density;

  ui_layout_push(c);
  ui_layout_resize(
      c,
      UiAlign_BottomLeft,
      ui_vector(sampledCellSize + 1, sampledCellSize + 1),
      UiBase_Absolute,
      Ui_XY);
  ui_style_push(c);
  ui_style_outline(c, 0);
  for (u32 y = 0; y != stepsY; ++y) {
    const f32 fracY = ((f32)y / (f32)(stepsY - 1)) * (g->height - 1);
    for (u32 x = 0; x != stepsX; ++x) {
      const f32 fracX = ((f32)x / (f32)(stepsX - 1)) * (g->width - 1);
      const f32 v     = sim_grid_sample(g, (SimCoordFrac){fracX, fracY}, 0.0f);
      const f32 frac  = math_clamp_f32(math_unlerp(minVal, maxVal, v), 0.0f, 1.0f);

      ui_style_color(c, ui_color_lerp(minColor, maxColor, frac));
      ui_layout_set_pos(
          c,
          UiBase_Canvas,
          demo_cell_pos(sampledCellSize, cellOrigin, (SimCoord){x, y}),
          UiBase_Absolute);

      ui_canvas_draw_glyph(c, UiShape_Square, 5, UiFlags_None);
    }
  }
  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw_velocity_edge(
    UiCanvasComp*   c,
    const SimState* s,
    const f32       cellSize,
    const UiVector  cellOrigin,
    const f32       velocityScale) {

  ui_layout_push(c);
  ui_style_push(c);

  const f32 dotSize   = 6.0f;
  const f32 lineWidth = 3.0f;

  ui_layout_resize(c, UiAlign_BottomLeft, ui_vector(dotSize, dotSize), UiBase_Absolute, Ui_XY);

  // Horizontal.
  ui_style_color(c, ui_color_red);
  for (u32 y = 0; y != s->velocitiesX.height; ++y) {
    for (u32 x = 0; x != s->velocitiesX.width; ++x) {
      const f32      val = sim_grid_get(&s->velocitiesX, (SimCoord){x, y});
      const UiVector pA  = {cellOrigin.x + x * cellSize, cellOrigin.y + (y + 0.5f) * cellSize};
      const UiVector pB  = {pA.x + val * cellSize * velocityScale, pA.y};

      ui_line(c, pA, pB, .base = UiBase_Absolute, .width = lineWidth);

      ui_layout_set_center(c, UiBase_Canvas, pA, UiBase_Absolute);
      ui_canvas_draw_glyph(c, UiShape_Circle, 0, UiFlags_None);
    }
  }

  // Vertical.
  ui_style_color(c, ui_color_green);
  for (u32 y = 0; y != s->velocitiesY.height; ++y) {
    for (u32 x = 0; x != s->velocitiesY.width; ++x) {
      const f32      val = sim_grid_get(&s->velocitiesY, (SimCoord){x, y});
      const UiVector pA  = {cellOrigin.x + (x + 0.5f) * cellSize, cellOrigin.y + y * cellSize};
      const UiVector pB  = {pA.x, pA.y + val * cellSize * velocityScale};

      ui_line(c, pA, pB, .base = UiBase_Absolute, .width = lineWidth);

      ui_layout_set_center(c, UiBase_Canvas, pA, UiBase_Absolute);
      ui_canvas_draw_glyph(c, UiShape_Circle, 0, UiFlags_None);
    }
  }

  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw_velocity_center(
    UiCanvasComp*   c,
    const SimState* s,
    const f32       cellSize,
    const UiVector  cellOrigin,
    const f32       velocityScale) {

  ui_layout_push(c);
  ui_style_push(c);

  const f32 dotSize   = 6.0f;
  const f32 lineWidth = 3.0f;

  ui_layout_resize(c, UiAlign_BottomLeft, ui_vector(dotSize, dotSize), UiBase_Absolute, Ui_XY);
  ui_style_color(c, ui_color_green);
  for (u32 y = 0; y != s->height; ++y) {
    for (u32 x = 0; x != s->width; ++x) {
      const f32 vX = sim_grid_sample(&s->velocitiesX, (SimCoordFrac){x + 0.5f, y + 0.5f}, 0.0f);
      const f32 vY = sim_grid_sample(&s->velocitiesY, (SimCoordFrac){x + 0.5f, y + 0.5f}, 0.0f);

      const UiVector pA = {
          cellOrigin.x + (x + 0.5f) * cellSize,
          cellOrigin.y + (y + 0.5f) * cellSize,
      };
      const UiVector pB = {
          pA.x + vX * cellSize * velocityScale,
          pA.y + vY * cellSize * velocityScale,
      };

      ui_line(c, pA, pB, .base = UiBase_Absolute, .width = lineWidth);

      ui_layout_set_center(c, UiBase_Canvas, pA, UiBase_Absolute);
      ui_canvas_draw_glyph(c, UiShape_Circle, 0, UiFlags_None);
    }
  }
  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw_velocity_divergence(
    UiCanvasComp*   c,
    const SimState* s,
    const f32       cellSize,
    const UiVector  cellOrigin,
    const f32       scale) {

  ui_layout_push(c);
  ui_layout_resize(c, UiAlign_BottomLeft, ui_vector(cellSize, cellSize), UiBase_Absolute, Ui_XY);
  ui_style_push(c);
  for (u32 y = 0; y != s->height; ++y) {
    for (u32 x = 0; x != s->width; ++x) {
      const f32 v    = sim_velocity_divergence(s, (SimCoord){x, y});
      const f32 frac = math_clamp_f32(math_abs(v) / scale, 0.0f, 1.0f);

      ui_style_color(c, ui_color_lerp(ui_color_green, ui_color_red, frac));
      ui_layout_set_pos(
          c, UiBase_Canvas, demo_cell_pos(cellSize, cellOrigin, (SimCoord){x, y}), UiBase_Absolute);

      ui_canvas_draw_glyph(c, UiShape_Square, 5, UiFlags_None);
    }
  }
  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw_velocity_color(
    UiCanvasComp*   c,
    const SimState* s,
    const f32       cellSize,
    const UiVector  cellOrigin,
    const f32       velocityScale) {

  ui_layout_push(c);
  ui_style_push(c);
  ui_layout_resize(c, UiAlign_BottomLeft, ui_vector(cellSize, cellSize), UiBase_Absolute, Ui_XY);
  for (u32 y = 0; y != s->height; ++y) {
    for (u32 x = 0; x != s->width; ++x) {
      const f32 vX = sim_grid_sample(&s->velocitiesX, (SimCoordFrac){x + 0.5f, y + 0.5f}, 0.0f);
      const f32 vY = sim_grid_sample(&s->velocitiesY, (SimCoordFrac){x + 0.5f, y + 0.5f}, 0.0f);

      const f32 vXNorm = math_clamp_f32(math_abs(vX) / velocityScale, 0.0f, 1.0f);
      const f32 vYNorm = math_clamp_f32(math_abs(vY) / velocityScale, 0.0f, 1.0f);

      const UiColor color = {.r = (u8)(vXNorm * 255.0f), .g = (u8)(vYNorm * 255.0f), 0, 255};
      ui_style_color(c, color);
      ui_layout_set_pos(
          c, UiBase_Canvas, demo_cell_pos(cellSize, cellOrigin, (SimCoord){x, y}), UiBase_Absolute);

      ui_canvas_draw_glyph(c, UiShape_Square, 5, UiFlags_None);
    }
  }
  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw_input(
    UiCanvasComp* c, const f32 cellSize, const UiVector cellOrigin, const SimCoord coord) {

  ui_layout_push(c);
  ui_layout_resize(c, UiAlign_BottomLeft, ui_vector(cellSize, cellSize), UiBase_Absolute, Ui_XY);
  ui_layout_set_pos(c, UiBase_Canvas, demo_cell_pos(cellSize, cellOrigin, coord), UiBase_Absolute);

  ui_style_push(c);
  ui_style_outline(c, 4);
  ui_style_color(c, ui_color_clear);

  ui_canvas_draw_glyph(c, UiShape_Square, 10, UiFlags_None);

  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw_solid(
    UiCanvasComp*   c,
    const SimState* s,
    const f32       cellSize,
    const UiVector  cellOrigin,
    const UiColor   color) {

  ui_layout_push(c);
  ui_layout_resize(c, UiAlign_BottomLeft, ui_vector(cellSize, cellSize), UiBase_Absolute, Ui_XY);
  ui_style_push(c);
  ui_style_color(c, color);
  ui_style_outline(c, 2);
  for (u32 y = 0; y != s->height; ++y) {
    for (u32 x = 0; x != s->width; ++x) {
      if (sim_solid(s, (SimCoord){x, y})) {
        const UiVector pos = demo_cell_pos(cellSize, cellOrigin, (SimCoord){x, y});
        ui_layout_set_pos(c, UiBase_Canvas, pos, UiBase_Absolute);
        ui_canvas_draw_glyph(c, UiShape_Circle, 4, UiFlags_None);
      }
    }
  }
  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw_emitters(
    UiCanvasComp* c, const SimState* s, const f32 cellSize, const UiVector cellOrigin) {

  ui_layout_push(c);
  ui_layout_resize(c, UiAlign_BottomLeft, ui_vector(cellSize, cellSize), UiBase_Absolute, Ui_XY);

  ui_style_push(c);
  ui_style_outline(c, 3);

  for (u32 i = 0; i != s->emitterCount; ++i) {
    const SimEmitter* emitter = &s->emitters[i];
    const UiVector    pos     = {
        cellOrigin.x + emitter->position.x * cellSize,
        cellOrigin.y + emitter->position.y * cellSize,
    };
    ui_layout_set_pos(c, UiBase_Canvas, pos, UiBase_Absolute);
    ui_canvas_draw_glyph_rotated(
        c, UiShape_ExpandLess, 0, -emitter->angle + math_pi_f32 * 0.5f, UiFlags_None);
  }

  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw_label(
    UiCanvasComp*   c,
    const SimState* s,
    const f32       cellSize,
    const UiVector  cellOrigin,
    const DemoLabel label) {

  if (label == DemoLabel_None) {
    return;
  }

  u8        textBuffer[32];
  DynString textStr = dynstring_create_over(mem_var(textBuffer));

  const FormatOptsFloat floatOpts = format_opts_float(
          .minDecDigits = 2, .maxDecDigits = 2, .expThresholdPos = f64_max, .expThresholdNeg = 0);

  ui_layout_push(c);
  ui_layout_resize(c, UiAlign_BottomLeft, ui_vector(cellSize, cellSize), UiBase_Absolute, Ui_XY);
  ui_style_push(c);
  ui_style_variation(c, UiVariation_Monospace);
  for (u32 y = 0; y != s->height; ++y) {
    for (u32 x = 0; x != s->width; ++x) {
      const SimCoord coord = {x, y};
      ui_layout_set_pos(
          c, UiBase_Canvas, demo_cell_pos(cellSize, cellOrigin, coord), UiBase_Absolute);

      dynstring_clear(&textStr);
      switch (label) {
      case DemoLabel_Smoke:
        format_write_f64(&textStr, sim_smoke(s, coord), &floatOpts);
        break;
      case DemoLabel_Pressure:
        format_write_f64(&textStr, sim_pressure(s, coord), &floatOpts);
        break;
      case DemoLabel_Speed:
        format_write_f64(&textStr, sim_speed(s, coord), &floatOpts);
        break;
      case DemoLabel_Angle:
        format_write_f64(&textStr, sim_angle(s, coord), &floatOpts);
        break;
      case DemoLabel_Divergence:
        format_write_f64(&textStr, sim_velocity_divergence(s, coord), &floatOpts);
        break;
      case DemoLabel_None:
      case DemoLabel_Count:
        UNREACHABLE;
      }
      ui_canvas_draw_text(c, dynstring_view(&textStr), 8, UiAlign_MiddleCenter, UiFlags_None);
    }
  }
  ui_style_pop(c);
  ui_layout_pop(c);
}

static void demo_draw(
    UiCanvasComp*      c,
    DemoComp*          d,
    const f32          cellSize,
    const UiVector     cellOrigin,
    const SimCoordFrac inputPos) {
  switch (d->layer) {
  case DemoLayer_SmokeInterp:
    demo_draw_grid_sampled(
        c, &d->sim.smoke, cellSize, cellOrigin, 0.0f, 0.1f, ui_color_black, ui_color_white, 4);
    break;
  case DemoLayer_Smoke:
    demo_draw_grid(
        c, &d->sim.smoke, cellSize, cellOrigin, 0.0f, 0.1f, ui_color_black, ui_color_white);
    break;
  case DemoLayer_Pressure:
    demo_draw_grid(
        c, &d->sim.pressure, cellSize, cellOrigin, -1.0f, 1.0f, ui_color_blue, ui_color_green);
    break;
  case DemoLayer_Velocity:
    demo_draw_velocity_color(c, &d->sim, cellSize, cellOrigin, 25.0f);
    break;
  case DemoLayer_Divergence:
    demo_draw_velocity_divergence(c, &d->sim, cellSize, cellOrigin, 0.01f);
    break;
  case DemoLayer_Count:
    UNREACHABLE
  }
  const SimCoord inputPosWhole = sim_coord_round_down(inputPos);
  if (d->interact && ui_canvas_status(c) == UiStatus_Idle &&
      sim_coord_valid(inputPosWhole, d->sim.width, d->sim.height)) {
    demo_draw_input(c, cellSize, cellOrigin, inputPosWhole);
  }
  if (d->overlay & DemoOverlay_Solid) {
    demo_draw_solid(c, &d->sim, cellSize, cellOrigin, ui_color_purple);
  }
  if (d->overlay & DemoOverlay_Emitter) {
    demo_draw_emitters(c, &d->sim, cellSize, cellOrigin);
  }
  if (d->overlay & DemoOverlay_Velo) {
    demo_draw_velocity_edge(c, &d->sim, cellSize, cellOrigin, 0.05f);
  }
  if (d->overlay & DemoOverlay_VeloCenter) {
    demo_draw_velocity_center(c, &d->sim, cellSize, cellOrigin, 0.05f);
  }
  demo_draw_label(c, &d->sim, cellSize, cellOrigin, d->label);
}

static const UiColor  g_demoMenuBg        = {0, 0, 0, 210};
static const UiVector g_demoMenuSize      = {280, 35};
static const UiVector g_demoMenuSpacing   = {8, 8};
static const UiVector g_demoMenuInset     = {-30, -15};
static const UiVector g_demoMenuValueSize = {0.55f, 1.0f};

static void demo_menu_frame(UiCanvasComp* c) {
  ui_style_push(c);
  ui_style_outline(c, 5);
  ui_style_color(c, g_demoMenuBg);
  ui_canvas_draw_glyph(c, UiShape_Circle, 10, UiFlags_None);
  ui_style_pop(c);
}

static void demo_menu_label(UiCanvasComp* c, const String label) {
  demo_menu_frame(c);
  ui_layout_push(c);
  ui_layout_grow(c, UiAlign_MiddleCenter, g_demoMenuInset, UiBase_Absolute, Ui_XY);
  ui_label(c, label);
  ui_layout_pop(c);
}

static void demo_menu_select(
    UiCanvasComp* c, const String label, i32* value, const String* options, const u32 optionCount) {
  demo_menu_frame(c);
  ui_layout_push(c);
  ui_layout_grow(c, UiAlign_MiddleCenter, g_demoMenuInset, UiBase_Absolute, Ui_XY);
  ui_label(c, label);
  ui_layout_inner(c, UiBase_Current, UiAlign_MiddleRight, g_demoMenuValueSize, UiBase_Current);
  ui_select(c, value, options, optionCount);
  ui_layout_pop(c);
}

static void demo_menu_select_bits(
    UiCanvasComp* c,
    const String  label,
    const BitSet  value,
    const String* options,
    const u32     optionCount) {
  demo_menu_frame(c);
  ui_layout_push(c);
  ui_layout_grow(c, UiAlign_MiddleCenter, g_demoMenuInset, UiBase_Absolute, Ui_XY);
  ui_label(c, label);
  ui_layout_inner(c, UiBase_Current, UiAlign_MiddleRight, g_demoMenuValueSize, UiBase_Current);
  ui_select_bits(c, value, options, optionCount);
  ui_layout_pop(c);
}

static void demo_menu_numbox_f32(UiCanvasComp* c, const String label, f32* value) {
  demo_menu_frame(c);
  ui_layout_push(c);
  ui_layout_grow(c, UiAlign_MiddleCenter, g_demoMenuInset, UiBase_Absolute, Ui_XY);
  ui_label(c, label);
  ui_layout_inner(c, UiBase_Current, UiAlign_MiddleRight, g_demoMenuValueSize, UiBase_Current);
  f64 editVal = *value;
  if (ui_numbox(c, &editVal)) {
    *value = (f32)editVal;
  }
  ui_layout_pop(c);
}

static void demo_menu_numbox_u32(UiCanvasComp* c, const String label, u32* value) {
  demo_menu_frame(c);
  ui_layout_push(c);
  ui_layout_grow(c, UiAlign_MiddleCenter, g_demoMenuInset, UiBase_Absolute, Ui_XY);
  ui_label(c, label);
  ui_layout_inner(c, UiBase_Current, UiAlign_MiddleRight, g_demoMenuValueSize, UiBase_Current);
  f64 editVal = *value;
  if (ui_numbox(c, &editVal, .step = 1.0)) {
    *value = (u32)editVal;
  }
  ui_layout_pop(c);
}

typedef enum {
  DemoMenuAction_None,
  DemoMenuAction_FullscreenToggle,
  DemoMenuAction_Quit,
} DemoMenuAction;

static DemoMenuAction demo_menu(UiCanvasComp* c, DemoComp* d) {
  DemoMenuAction action = 0;

  ui_layout_inner(c, UiBase_Canvas, UiAlign_BottomLeft, g_demoMenuSize, UiBase_Absolute);
  ui_layout_move(c, g_demoMenuSpacing, UiBase_Absolute, Ui_XY);

  if (ui_button(c, .label = string_lit("Quit"))) {
    action = DemoMenuAction_Quit;
  }
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  if (ui_button(c, .label = string_lit("Fullscreen"))) {
    action = DemoMenuAction_FullscreenToggle;
  }
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  if (ui_button(c, .label = string_lit("Reset"))) {
    sim_clear(&d->sim);
  }
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  if (ui_button(c, .label = string_lit("Randomize Velocity"))) {
    sim_velocity_randomize(&d->sim);
  }
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  if (ui_button(c, .label = string_lit("Solid Clear"))) {
    sim_solid_clear(&d->sim);
  }
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  if (ui_button(c, .label = string_lit("Solid Border"))) {
    sim_solid_set_border(&d->sim);
  }
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_select(c, string_lit("Label"), (i32*)&d->label, g_demoLabelNames, DemoLabel_Count);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_select_bits(
      c, string_lit("Overlay"), bitset_from_var(d->overlay), g_demoOverlayNames, DemoOverlay_Count);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_select(c, string_lit("Layer"), (i32*)&d->layer, g_demoLayerNames, DemoLayer_Count);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_select(
      c, string_lit("Interact"), (i32*)&d->interact, g_demoInteractNames, DemoInteract_Count);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_numbox_f32(c, string_lit("Density"), &d->sim.density);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_numbox_f32(c, string_lit("Pres Decay"), &d->sim.pressureDecay);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_numbox_f32(c, string_lit("Velo Diff"), &d->sim.velocityDiffusion);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_numbox_f32(c, string_lit("Smoke Diff"), &d->sim.smokeDiffusion);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_numbox_f32(c, string_lit("Smoke Decay"), &d->sim.smokeDecay);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_numbox_u32(c, string_lit("Solve Itrs"), &d->sim.solverIterations);
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_label(
      c, fmt_write_scratch("Divergence: {}", fmt_float(sim_velocity_divergence_sum(&d->sim))));
  ui_layout_next(c, Ui_Up, g_demoMenuSpacing.y);
  demo_menu_label(c, fmt_write_scratch("Smoke: {}", fmt_float(sim_smoke_sum(&d->sim))));

  return action;
}

static void demo_fullscreen_toggle(GapWindowComp* w) {
  if (gap_window_mode(w) == GapWindowMode_Fullscreen) {
    log_i("Enter windowed mode");
    const GapVector size = gap_window_param(w, GapParam_WindowSizePreFullscreen);
    gap_window_resize(w, size, GapWindowMode_Windowed);
    gap_window_flags_unset(w, GapWindowFlags_CursorConfine);
  } else {
    log_i("Enter fullscreen mode");
    gap_window_resize(w, gap_vector(0, 0), GapWindowMode_Fullscreen);
    gap_window_flags_set(w, GapWindowFlags_CursorConfine);
  }
}

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

  demo->sim.push  = false;
  demo->sim.pull  = false;
  demo->sim.guide = false;

  if (winItr) {
    GapWindowComp* winComp = ecs_view_write_t(winItr, GapWindowComp);

    if (gap_window_key_down(winComp, GapKey_Alt) && gap_window_key_pressed(winComp, GapKey_F4)) {
      gap_window_close(winComp);
    }
    if (gap_window_key_down(winComp, GapKey_Alt) &&
        gap_window_key_pressed(winComp, GapKey_Return)) {
      demo_fullscreen_toggle(winComp);
    }
    if (gap_window_key_pressed(winComp, GapKey_Tab)) {
      demo->hideMenu ^= 1;
    }
    if (gap_window_key_pressed(winComp, GapKey_R)) {
      sim_clear(&demo->sim);
    }
    for (DemoInteract interact = 0; interact != DemoInteract_Count; ++interact) {
      if (gap_window_key_pressed(winComp, GapKey_Alpha1 + interact)) {
        demo->interact = interact;
      }
    }
    for (DemoLayer layer = 0; layer != DemoLayer_Count; ++layer) {
      if (gap_window_key_pressed(winComp, GapKey_F1 + layer)) {
        demo->layer = layer;
      }
    }
    for (u32 overlayBit = 0; overlayBit != DemoOverlay_Count; ++overlayBit) {
      if (gap_window_key_pressed(winComp, GapKey_F1 + DemoLayer_Count + overlayBit)) {
        demo->overlay ^= 1 << overlayBit;
      }
    }

    if (ecs_view_maybe_jump(canvasItr, demo->uiCanvas)) {
      UiCanvasComp* uiCanvas = ecs_view_write_t(canvasItr, UiCanvasComp);
      ui_canvas_reset(uiCanvas);
      const f32 cellSize = demo_cell_size(uiCanvas, &demo->sim);
      if (cellSize > f32_epsilon) {
        const UiVector     cellOrigin = demo_cell_origin(uiCanvas, &demo->sim, cellSize);
        const SimCoordFrac inputPos   = demo_input(uiCanvas, cellSize, cellOrigin);
        if (ui_canvas_status(uiCanvas) == UiStatus_Idle) {
          demo_interact(demo, winComp, inputPos);
        }
        demo_draw(uiCanvas, demo, cellSize, cellOrigin, inputPos);
      }
      ui_canvas_id_block_next(uiCanvas);
      if (!demo->hideMenu) {
        switch (demo_menu(uiCanvas, demo)) {
        case DemoMenuAction_None:
          break;
        case DemoMenuAction_FullscreenToggle:
          demo_fullscreen_toggle(winComp);
          break;
        case DemoMenuAction_Quit:
          gap_window_close(winComp);
          break;
        }
      }
    }
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
  cli_app_register_desc(app, string_lit("3D Smoke Demo"));

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

  const u16 windowWidth  = (u16)cli_read_u64(invoc, g_optWidth, 1600);
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

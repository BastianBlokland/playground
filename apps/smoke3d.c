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
#include "ecs/entity.h"
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
 * Semi-Lagrangian fluid simulation.
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

typedef struct {
  u32  width, height, depth;
  f32* values;
} SimGrid;

static SimGrid sim_grid_create(const u32 width, const u32 height, const u32 depth) {
  return (SimGrid){
      .width  = width,
      .height = height,
      .depth  = depth,
      .values = alloc_array_t(g_allocHeap, f32, width * height * depth),
  };
}

static void sim_grid_destroy(SimGrid* g) {
  alloc_free_array_t(g_allocHeap, g->values, g->width * g->height * g->depth);
}

static u32 sim_grid_count(const SimGrid* g) { return g->depth * g->height * g->width; }

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

static bool sim_grid_valid(const SimGrid* g, const SimCoord c) {
  if (c.x < 0 || c.x > (i32)g->width) {
    return false;
  }
  if (c.y < 0 || c.y > (i32)g->height) {
    return false;
  }
  if (c.z < 0 || c.z > (i32)g->depth) {
    return false;
  }
  return true;
}

static u32 sim_grid_index(const SimGrid* g, const SimCoord c) {
  diag_assert(sim_grid_valid(g, c));
  return c.z * g->height * g->width + c.y * g->width + c.x;
}

static f32 sim_grid_get(const SimGrid* g, const SimCoord c) {
  return g->values[sim_grid_index(g, c)];
}

static f32 sim_grid_get_bounded(const SimGrid* g, const SimCoord c, const f32 fallback) {
  return sim_grid_valid(g, c) ? g->values[sim_grid_index(g, c)] : fallback;
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
  u32 width, height, depth;

  // NOTE: Velocities are stored at the edges, not the cell centers.
  SimGrid velocitiesX; // (width + 1) height * depth
  SimGrid velocitiesY; // width * (height + 1) * depth
  SimGrid velocitiesZ; // width * height * (depth + 1)

  SimGrid pressure;
  SimGrid smoke;
  BitSet  solid;
} SimState;

// static SimState sim_state_create(const u32 width, const height, )

ecs_comp_define(DemoComp) {
  EcsEntityId window;
  TimeSteady  lastTime;

  u32 targetX, targetY, targetZ;

  u32    simWidth, simHeight, simDepth;
  f32*   velocitiesX; // f32[(simWidth + 1) simHeight * simDepth]
  f32*   velocitiesY; // f32[simWidth * (simHeight + 1) * simDepth]
  f32*   velocitiesZ; // f32[simWidth * simHeight * (simDepth + 1)]
  f32*   pressure;    // f32[simWidth * simHeight * simDepth]
  f32*   smoke;       // f32[simWidth * simHeight * simDepth]
  BitSet solidMap;    // bit[simWidth * simHeight * simDepth]
};

ecs_comp_define(DemoWindowComp) { EcsEntityId uiCanvas; };

static DemoComp* demo_create(EcsWorld* world) {
  const EcsEntityId global = ecs_world_global(world);

  DemoComp* demo = ecs_world_add_t(
      world,
      global,
      DemoComp,
      .targetX   = 12,
      .targetY   = 2,
      .targetZ   = 12,
      .simWidth  = 25,
      .simHeight = 5,
      .simDepth  = 25);

  const u32 width  = demo->simWidth;
  const u32 height = demo->simHeight;
  const u32 depth  = demo->simDepth;

  demo->velocitiesX = alloc_array_t(g_allocHeap, f32, (width + 1) * height * depth);
  demo->velocitiesY = alloc_array_t(g_allocHeap, f32, width * (height + 1) * depth);
  demo->velocitiesZ = alloc_array_t(g_allocHeap, f32, width * height * (depth + 1));
  demo->pressure    = alloc_array_t(g_allocHeap, f32, width * height * depth);
  demo->smoke       = alloc_array_t(g_allocHeap, f32, width * height * depth);
  demo->solidMap    = alloc_alloc(g_allocHeap, bits_to_bytes(width * height * depth) + 1, 1);

  mem_set(mem_create(demo->velocitiesX, sizeof(f32) * (width + 1) * height * depth), 0);
  mem_set(mem_create(demo->velocitiesY, sizeof(f32) * width * (height + 1) * depth), 0);
  mem_set(mem_create(demo->velocitiesZ, sizeof(f32) * width * height * (depth + 1)), 0);
  mem_set(mem_create(demo->pressure, sizeof(f32) * width * height * depth), 0);
  mem_set(mem_create(demo->smoke, sizeof(f32) * width * height * depth), 0);
  mem_set(demo->solidMap, 0);

  return demo;
}

static void demo_destroy(void* data) {
  DemoComp* comp   = data;
  const u32 width  = comp->simWidth;
  const u32 height = comp->simHeight;
  const u32 depth  = comp->simDepth;

  alloc_free_array_t(g_allocHeap, comp->velocitiesX, (width + 1) * height * depth);
  alloc_free_array_t(g_allocHeap, comp->velocitiesY, width * (height + 1) * depth);
  alloc_free_array_t(g_allocHeap, comp->velocitiesZ, width * height * (depth + 1));
  alloc_free_array_t(g_allocHeap, comp->pressure, width * height * depth);
  alloc_free_array_t(g_allocHeap, comp->smoke, width * height * depth);
  alloc_free(g_allocHeap, comp->solidMap);
}

static EcsEntityId demo_window_create(EcsWorld* world, const u16 width, const u16 height) {
  const GapVector      size           = {.width = (i32)width, .height = (i32)height};
  const GapWindowFlags flags          = GapWindowFlags_Default;
  const GapWindowMode  mode           = GapWindowMode_Windowed;
  const GapIcon        icon           = GapIcon_Main;
  const String         versionScratch = version_str_scratch(g_versionExecutable);
  const String         titleScratch = fmt_write_scratch("Smoke Demo v{}", fmt_text(versionScratch));
  const EcsEntityId    window = gap_window_create(world, mode, flags, size, icon, titleScratch);

  const EcsEntityId uiCanvas = ui_canvas_create(world, window, UiCanvasCreateFlags_ToBack);
  ecs_world_add_t(world, window, DemoWindowComp, .uiCanvas = uiCanvas);
  return window;
}

typedef struct {
  EcsWorld* world;
  DemoComp* demo;
  f32       dt; // In seconds.

  EcsEntityId     winEntity;
  DemoWindowComp* winDemo;
  GapWindowComp*  winComp;
  UiCanvasComp*   winCanvas;
} DemoUpdateContext;

static f32 sim_time_to_seconds(const TimeDuration dur) {
  static const f64 g_toSecMul = 1.0 / (f64)time_second;
  return (f32)((f64)dur * g_toSecMul);
}

static u32 sim_index(const u32 width, const u32 height, const u32 x, const u32 y, const u32 z) {
  return z * height * width + y * width + x;
}

static u32 sim_cell_index(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  return sim_index(ctx->demo->simWidth, ctx->demo->simHeight, x, y, z);
}

static bool sim_solid(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  return bitset_test(ctx->demo->solidMap, sim_cell_index(ctx, x, y, z));
}

static void sim_solid_flip(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  bitset_flip(ctx->demo->solidMap, sim_cell_index(ctx, x, y, z));
}

static void sim_solid_set(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  bitset_set(ctx->demo->solidMap, sim_cell_index(ctx, x, y, z));
}

static f32 sim_pressure(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  if (sim_solid(ctx, x, y, z)) {
    return 0.0f;
  }
  return ctx->demo->pressure[sim_cell_index(ctx, x, y, z)];
}

static f32 sim_smoke(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  return ctx->demo->smoke[sim_cell_index(ctx, x, y, z)];
}

static void
sim_smoke_emit(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z, const f32 smoke) {
  ctx->demo->smoke[sim_cell_index(ctx, x, y, z)] += smoke;
}

static f32 sim_velocity_bottom(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  const u32 width = ctx->demo->simWidth;
  return ctx->demo->velocitiesY[y * width + x];
}

static f32 sim_velocity_top(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  const u32 width = ctx->demo->simWidth;
  return ctx->demo->velocitiesY[(y + 1) * width + x];
}

static f32 sim_velocity_left(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  const u32 width = ctx->demo->simWidth + 1;
  return ctx->demo->velocitiesX[y * width + x];
}

static f32 sim_velocity_right(DemoUpdateContext* ctx, const u32 x, const u32 y, const u32 z) {
  const u32 width = ctx->demo->simWidth + 1;
  return ctx->demo->velocitiesX[y * width + (x + 1)];
}

static f32 sim_sample(const f32* v, const u32 width, const u32 height, const f32 x, const f32 y) {
  const i32 left   = (i32)math_round_down_f32(x);
  const i32 bottom = (i32)math_round_down_f32(y);
  const i32 right  = left + 1;
  const i32 top    = bottom + 1;

  const f32 xFrac = math_clamp_f32(x - left, 0.0f, 1.0f);
  const f32 yFrac = math_clamp_f32(y - bottom, 0.0f, 1.0f);

  f32 v1, v2, v3, v4;

  // TODO: There are a bunch of redundant checks here.
  if (top >= 0 && (u32)top < height && left >= 0 && (u32)left < width) {
    v1 = v[top * width + left];
  } else {
    v1 = 0.0f;
  }
  if (top >= 0 && (u32)top < height && right >= 0 && (u32)right < width) {
    v2 = v[top * width + right];
  } else {
    v2 = 0.0f;
  }
  if (bottom >= 0 && (u32)bottom < height && left >= 0 && (u32)left < width) {
    v3 = v[bottom * width + left];
  } else {
    v3 = 0.0f;
  }
  if (bottom >= 0 && (u32)bottom < height && right >= 0 && (u32)right < width) {
    v4 = v[bottom * width + right];
  } else {
    v4 = 0.0f;
  }

  const f32 vTop    = math_lerp(v1, v2, xFrac);
  const f32 vBottom = math_lerp(v3, v4, xFrac);
  return math_lerp(vBottom, vTop, yFrac);
}

static bool sim_pushed(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  if (!ctx->demo->push) {
    return false;
  }
  if (x == ctx->demo->targetX && y == ctx->demo->targetY) {
    return true;
  }
  if (x == (ctx->demo->targetX + 1) && y == ctx->demo->targetY) {
    return true;
  }
  if (x == (ctx->demo->targetX - 1) && y == ctx->demo->targetY) {
    return true;
  }
  if (x == ctx->demo->targetX && (y + 1) == ctx->demo->targetY) {
    return true;
  }
  if (x == ctx->demo->targetX && (y - 1) == ctx->demo->targetY) {
    return true;
  }
  return false;
}

static void sim_velocity_diffuse(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  if (ctx->demo->velDiffusion < f32_epsilon) {
    return;
  }

  // Horizontal.
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != (width + 1); ++x) {
      const f32 velCenter = ctx->demo->velocitiesX[y * (width + 1) + x];
      const f32 velTop  = (y + 1) != height ? ctx->demo->velocitiesX[(y + 1) * (width + 1) + x] : 0;
      const f32 velLeft = x ? ctx->demo->velocitiesX[y * (width + 1) + (x - 1)] : 0;
      const f32 velRight  = x != width ? ctx->demo->velocitiesX[y * (width + 1) + (x + 1)] : 0;
      const f32 velBottom = y ? ctx->demo->velocitiesX[(y - 1) * (width + 1) + x] : 0;

      const f32 laplacian   = velLeft + velRight + velTop + velBottom - 4 * velCenter;
      const f32 velDiffused = velCenter + laplacian * ctx->demo->velDiffusion * ctx->dt;
      ctx->demo->velocitiesX[y * (width + 1) + x] = velDiffused;
    }
  }

  // Vertical.
  for (u32 y = 0; y != (height + 1); ++y) {
    for (u32 x = 0; x != width; ++x) {
      const f32 velCenter = ctx->demo->velocitiesY[y * width + x];
      const f32 velTop    = y != height ? ctx->demo->velocitiesY[(y + 1) * width + x] : 0;
      const f32 velLeft   = x ? ctx->demo->velocitiesY[y * width + (x - 1)] : 0;
      const f32 velRight  = (x + 1) != width ? ctx->demo->velocitiesY[y * width + (x + 1)] : 0;
      const f32 velBottom = y ? ctx->demo->velocitiesY[(y - 1) * width + x] : 0;

      const f32 laplacian   = velLeft + velRight + velTop + velBottom - 4 * velCenter;
      const f32 velDiffused = velCenter + laplacian * ctx->demo->velDiffusion * ctx->dt;
      ctx->demo->velocitiesY[y * width + x] = velDiffused;
    }
  }
}

static void sim_smoke_diffuse(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  if (ctx->demo->smokeDiffusion < f32_epsilon) {
    return;
  }

  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != width; ++x) {
      if (sim_solid(ctx, x, y)) {
        continue;
      }
      const f32 smokeCenter = ctx->demo->smoke[y * width + x];
      const f32 smokeTop    = (y + 1) != height ? ctx->demo->smoke[(y + 1) * width + x] : 0;
      const f32 smokeLeft   = x ? ctx->demo->smoke[y * width + (x - 1)] : 0;
      const f32 smokeRight  = (x + 1) != width ? ctx->demo->smoke[y * width + (x + 1)] : 0;
      const f32 smokeBottom = y ? ctx->demo->smoke[(y - 1) * width + x] : 0;

      const f32 laplacian = smokeLeft + smokeRight + smokeTop + smokeBottom - 4 * smokeCenter;

      const f32 smokeNew = smokeCenter + ctx->demo->smokeDiffusion * ctx->dt * laplacian;
      ctx->demo->smoke[y * width + x] = smokeNew * math_exp_f32(-ctx->dt * ctx->demo->smokeDecay);
    }
  }
}

static void sim_smoke_coalesce(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  const f32 k = 100.0f * ctx->dt;

  // Horizontal.
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != (width + 1); ++x) {
      const f32 smokeRight = x != width ? sim_smoke(ctx, x, y) : 0.0f;
      const f32 smokeLeft  = x ? sim_smoke(ctx, x - 1, y) : 0.0f;
      const f32 mul        = smokeRight - smokeLeft;
      ctx->demo->velocitiesX[y * (width + 1) + x] += k * mul * mul;
    }
  }

  // Vertical.
  for (u32 y = 0; y != (height + 1); ++y) {
    for (u32 x = 0; x != width; ++x) {
      const f32 smokeTop    = y != height ? sim_smoke(ctx, x, y) : 0.0f;
      const f32 smokeBottom = y ? sim_smoke(ctx, x, y - 1) : 0.0f;
      const f32 mul         = smokeTop - smokeBottom;
      ctx->demo->velocitiesY[y * width + x] += k * mul * mul;
    }
  }
}

static void sim_velocity_advect(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  f32* velocitiesX = alloc_array_t(g_allocScratch, f32, height * (width + 1));
  f32* velocitiesY = alloc_array_t(g_allocScratch, f32, (height + 1) * width);

  // Horizontal.
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != (width + 1); ++x) {
      // Check if either the left or the right cell is solid.
      if ((x && sim_solid(ctx, x - 1, y)) || (x != width && sim_solid(ctx, x, y))) {
        velocitiesX[y * (width + 1) + x] = ctx->demo->velocitiesX[y * (width + 1) + x];
        continue;
      }
      const f32 velX = ctx->demo->velocitiesX[y * (width + 1) + x];
      const f32 velY = sim_sample(ctx->demo->velocitiesY, width, height + 1, x, y + 0.5f);

      const f32 prevX = x - velX * ctx->dt;
      const f32 prevY = y - velY * ctx->dt;

      const f32 velNew = sim_sample(ctx->demo->velocitiesX, width + 1, height, prevX, prevY);

      velocitiesX[y * (width + 1) + x] = velNew;
    }
  }

  // Vertical.
  for (u32 y = 0; y != (height + 1); ++y) {
    for (u32 x = 0; x != width; ++x) {
      // Check if either the top or the bottom cell is solid.
      if ((y && sim_solid(ctx, x, y - 1)) || (y != height && sim_solid(ctx, x, y))) {
        velocitiesY[y * width + x] = ctx->demo->velocitiesY[y * width + x];
        continue;
      }
      const f32 velX = sim_sample(ctx->demo->velocitiesX, width + 1, height, x + 0.5f, y);
      const f32 velY = ctx->demo->velocitiesY[y * width + x];

      const f32 prevX = x - velX * ctx->dt;
      const f32 prevY = y - velY * ctx->dt;

      const f32 velNew = sim_sample(ctx->demo->velocitiesY, width, height + 1, prevX, prevY);
      velocitiesY[y * width + x] = velNew;
    }
  }

  mem_cpy(
      mem_create(ctx->demo->velocitiesX, sizeof(f32) * height * (width + 1)),
      mem_create(velocitiesX, sizeof(f32) * height * (width + 1)));

  mem_cpy(
      mem_create(ctx->demo->velocitiesY, sizeof(f32) * (height + 1) * width),
      mem_create(velocitiesY, sizeof(f32) * (height + 1) * width));
}

static void sim_smoke_advect(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  f32* smoke = alloc_array_t(g_allocScratch, f32, height * width);

  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != width; ++x) {
      if (sim_solid(ctx, x, y) || sim_pushed(ctx, x, y)) {
        smoke[y * width + x] = 0.0f;
        continue;
      }
      const f32 velX = sim_sample(ctx->demo->velocitiesX, width + 1, height, x + 0.5f, y + 0.5f);
      const f32 velY = sim_sample(ctx->demo->velocitiesY, width, height + 1, x + 0.5f, y + 0.5f);

      const f32 prevX = x - velX * ctx->dt;
      const f32 prevY = y - velY * ctx->dt;

      smoke[y * width + x] = sim_sample(ctx->demo->smoke, width, height, prevX, prevY);
    }
  }

  mem_cpy(
      mem_create(ctx->demo->smoke, sizeof(f32) * height * width),
      mem_create(smoke, sizeof(f32) * height * width));
}

static f32 sim_smoke_count(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  f32 smokeSum = 0;
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != width; ++x) {
      smokeSum += ctx->demo->smoke[y * width + x];
    }
  }
  return smokeSum;
}

static void
sim_push(DemoUpdateContext* ctx, const u32 x, const u32 y, const f32 forceX, const f32 forceY) {
  ctx->demo->velocitiesY[y * ctx->demo->simWidth + x] += forceY;             // Bottom.
  ctx->demo->velocitiesY[(y + 1) * ctx->demo->simWidth + x] -= forceY;       // Top.
  ctx->demo->velocitiesX[y * (ctx->demo->simWidth + 1) + x] += forceX;       // Left.
  ctx->demo->velocitiesX[y * (ctx->demo->simWidth + 1) + (x + 1)] -= forceX; // Right.
}

static f32 sim_velocity_divergence(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const f32 velTop    = sim_velocity_top(ctx, x, y);
  const f32 velLeft   = sim_velocity_left(ctx, x, y);
  const f32 velRight  = sim_velocity_right(ctx, x, y);
  const f32 velBottom = sim_velocity_bottom(ctx, x, y);
  return (velRight - velLeft) + (velTop - velBottom);
}

static f32 sim_speed(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const u32 width    = ctx->demo->simWidth;
  const u32 height   = ctx->demo->simHeight;
  const f32 velX     = sim_sample(ctx->demo->velocitiesX, width + 1, height, x + 0.5f, y + 0.5f);
  const f32 velY     = sim_sample(ctx->demo->velocitiesY, width, height + 1, x + 0.5f, y + 0.5f);
  const f32 speedSqr = velX * velX + velY * velY;
  return speedSqr != 0 ? intrinsic_sqrt_f32(speedSqr) : 0;
}

static void sim_solve_pressure(DemoUpdateContext* ctx) {
  const u32 width   = ctx->demo->simWidth;
  const u32 height  = ctx->demo->simHeight;
  const f32 density = ctx->demo->density;

  if (density < 0.01f) {
    return;
  }

  const f32 pressureDecay = ctx->demo->density * 0.5f;

  const u32 xMax = width - 1;
  const u32 yMax = height - 1;

  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != width; ++x) {

      const bool flowTop    = !sim_solid(ctx, x + 0, y + 1);
      const bool flowLeft   = !sim_solid(ctx, x - 1, y + 0);
      const bool flowRight  = !sim_solid(ctx, x + 1, y + 0);
      const bool flowBottom = !sim_solid(ctx, x + 0, y - 1);
      const u8   flowCount  = flowLeft + flowRight + flowTop + flowBottom;

      f32 newPressure;
      if (sim_pushed(ctx, x, y)) {
        newPressure = 500;
      } else if (sim_solid(ctx, x, y) || !flowCount) {
        newPressure = 0.0f;
      } else {
        const f32 pressureTop    = y != yMax ? (sim_pressure(ctx, x, y + 1) * flowTop) : 0.0f;
        const f32 pressureLeft   = x ? (sim_pressure(ctx, x - 1, y) * flowLeft) : 0.0f;
        const f32 pressureRight  = x != xMax ? (sim_pressure(ctx, x + 1, y) * flowRight) : 0.0f;
        const f32 pressureBottom = y ? (sim_pressure(ctx, x, y - 1) * flowBottom) : 0.0f;
        const f32 pressureSum    = pressureRight + pressureLeft + pressureTop + pressureBottom;

        const f32 velTop    = sim_velocity_top(ctx, x, y);
        const f32 velLeft   = sim_velocity_left(ctx, x, y);
        const f32 velRight  = sim_velocity_right(ctx, x, y);
        const f32 velBottom = sim_velocity_bottom(ctx, x, y);
        const f32 velDelta  = velRight - velLeft + velTop - velBottom;

        newPressure = (pressureSum - density * velDelta / ctx->dt) / flowCount;
        newPressure -= newPressure * pressureDecay * ctx->dt;
      }
      ctx->demo->pressure[y * width + x] = newPressure;
    }
  }
}

static void sim_velocity_update(DemoUpdateContext* ctx) {
  if (ctx->demo->density < 0.01f) {
    return;
  }
  const f32 k      = ctx->dt / ctx->demo->density;
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  // Horizontal.
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != (width + 1); ++x) {
      // Check if either the left or the right cell is solid.
      if ((x && sim_solid(ctx, x - 1, y)) || (x != width && sim_solid(ctx, x, y))) {
        ctx->demo->velocitiesX[y * (width + 1) + x] = 0;
        continue;
      }
      const f32 pressureRight = x != width ? sim_pressure(ctx, x, y) : 0.0f;
      const f32 pressureLeft  = x ? sim_pressure(ctx, x - 1, y) : 0.0f;
      ctx->demo->velocitiesX[y * (width + 1) + x] -= k * (pressureRight - pressureLeft);
    }
  }

  // Vertical.
  for (u32 y = 0; y != (height + 1); ++y) {
    for (u32 x = 0; x != width; ++x) {
      // Check if either the top or the bottom cell is solid.
      if ((y && sim_solid(ctx, x, y - 1)) || (y != height && sim_solid(ctx, x, y))) {
        ctx->demo->velocitiesY[y * width + x] = 0;
        continue;
      }
      const f32 pressureTop    = y != height ? sim_pressure(ctx, x, y) : 0.0f;
      const f32 pressureBottom = y ? sim_pressure(ctx, x, y - 1) : 0.0f;
      ctx->demo->velocitiesY[y * width + x] -= k * (pressureTop - pressureBottom);
    }
  }
}

static void sim_smoke_pull(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  const f32 force = 5.0f;

  // Horizontal.
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != (width + 1); ++x) {
      const f32 posX    = x;
      const f32 posY    = y + 0.5f;
      const f32 smoke   = sim_sample(ctx->demo->smoke, width, height, posX, posY);
      const f32 dX      = ctx->demo->targetX - posX;
      const f32 dY      = ctx->demo->targetY - posY;
      const f32 distSqr = dX * dX + dY * dY;
      if (distSqr < f32_epsilon) {
        continue;
      }
      const f32 dist         = intrinsic_sqrt_f32(distSqr);
      const f32 smokeClamped = math_clamp_f32(smoke, 0.0f, 1.0f);
      // TODO: This is very arbitrary to avoid scaling the force very high if the smoke is very
      // concentrated.
      const f32 forceMul = 1.0f - math_pow_f32(1.0f - smokeClamped, 3.0f);
      ctx->demo->velocitiesX[y * (width + 1) + x] += (dX / dist) * force * ctx->dt * forceMul;
    }
  }

  // Vertical.
  for (u32 y = 0; y != (height + 1); ++y) {
    for (u32 x = 0; x != width; ++x) {
      const f32 posX    = x + 0.5f;
      const f32 posY    = y;
      const f32 smoke   = sim_sample(ctx->demo->smoke, width, height, posX, posY);
      const f32 dX      = ctx->demo->targetX - posX;
      const f32 dY      = ctx->demo->targetY - posY;
      const f32 distSqr = dX * dX + dY * dY;
      if (distSqr < f32_epsilon) {
        continue;
      }
      const f32 dist         = intrinsic_sqrt_f32(distSqr);
      const f32 smokeClamped = math_clamp_f32(smoke, 0.0f, 1.0f);
      const f32 forceMul     = 1.0f - math_pow_f32(1.0f - smokeClamped, 3.0f);
      ctx->demo->velocitiesY[y * width + x] += (dY / dist) * force * ctx->dt * forceMul;
    }
  }
}

static void sim_update(DemoUpdateContext* ctx) {
  const f32 smokeAmount = sim_smoke_count(ctx);
  if (smokeAmount < 350.0f) {
    sim_smoke_emit(ctx, 4, 3, 25.0f * ctx->dt);
    //  sim_push(ctx, 4, 3, 0, 1000.0f * ctx->dt);
    //  sim_smoke_emit(ctx, 10, 3, 2.0f * ctx->dt);
    //  sim_push(ctx, 10, 3, 0, 1000.0f * ctx->dt);
  }
  // log_i("Update", log_param("smoke", fmt_float(smokeAmount)));

  (void)sim_smoke_coalesce;
  // sim_smoke_coalesce(ctx);
  sim_smoke_pull(ctx);
  sim_velocity_diffuse(ctx);
  sim_smoke_diffuse(ctx);
  sim_smoke_advect(ctx);
  sim_velocity_advect(ctx);
  if (ctx->demo->solve) {
    for (u32 i = 0; i != ctx->demo->solverIterations; ++i) {
      sim_solve_pressure(ctx);
    }
  }
  if (ctx->demo->updateVelocities) {
    sim_velocity_update(ctx);
  }
}

static void sim_velocity_randomize(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  // Horizontal.
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != (width + 1); ++x) {
      ctx->demo->velocitiesX[y * (width + 1) + x] = rng_sample_f32(g_rng) * 2.0f - 1.0f;
    }
  }

  // Vertical.
  for (u32 y = 0; y != (height + 1); ++y) {
    for (u32 x = 0; x != width; ++x) {
      ctx->demo->velocitiesY[y * width + x] = rng_sample_f32(g_rng) * 2.0f - 1.0f;
    }
  }
}

static void sim_velocity_clear(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  mem_set(mem_create(ctx->demo->velocitiesX, sizeof(f32) * height * (width + 1)), 0);
  mem_set(mem_create(ctx->demo->velocitiesY, sizeof(f32) * (height + 1) * width), 0);
}

static void sim_pressure_clear(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;
  mem_set(mem_create(ctx->demo->pressure, sizeof(f32) * height * width), 0);
}

static void sim_solid_set_border(DemoUpdateContext* ctx) {
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != width; ++x) {
      const bool solid = !x || !y || x == (width - 1) || y == (height - 1);
      if (solid) {
        bitset_set(ctx->demo->solidMap, y * width + x);
      } else {
        bitset_clear(ctx->demo->solidMap, y * width + x);
      }
    }
  }
}

static void sim_draw(DemoUpdateContext* ctx) {
  ui_canvas_id_block_next(ctx->winCanvas);
  ui_layout_push(ctx->winCanvas);
  ui_style_push(ctx->winCanvas);

  const u32 simWidth  = ctx->demo->simWidth;
  const u32 simHeight = ctx->demo->simHeight;

  const UiVector canvasSize = ui_canvas_resolution(ctx->winCanvas);
  const UiVector cellSize   = ui_vector(40, 40);
  const UiVector cellOrg    = {
      canvasSize.width * 0.5f - simWidth * cellSize.width * 0.5f,
      canvasSize.height * 0.5f - simHeight * cellSize.height * 0.5f,
  };

  const GapVector cursorDelta = gap_window_param(ctx->winComp, GapParam_CursorDelta);
  const f32       distSqr     = cursorDelta.x * cursorDelta.x + cursorDelta.y * cursorDelta.y;
  if (distSqr > f32_epsilon) {
    const f32 dist  = intrinsic_sqrt_f32(distSqr);
    ctx->demo->dirX = cursorDelta.x / dist;
    ctx->demo->dirY = cursorDelta.y / dist;
    log_i(
        "Dir",
        log_param("x", fmt_float(ctx->demo->dirX)),
        log_param("y", fmt_float(ctx->demo->dirY)));
  }

  ctx->demo->push = false;

  ui_layout_resize(ctx->winCanvas, UiAlign_BottomLeft, cellSize, UiBase_Absolute, Ui_XY);
  for (u32 y = 0; y != simHeight; ++y) {
    for (u32 x = 0; x != simWidth; ++x) {
      const f32 divergence = sim_velocity_divergence(ctx, x, y);
      const f32 pressure   = sim_pressure(ctx, x, y);
      const f32 speed      = sim_speed(ctx, x, y);
      const f32 smoke      = sim_smoke(ctx, x, y);

      (void)speed;
      (void)divergence;
      (void)pressure;

      const f32 velX =
          sim_sample(ctx->demo->velocitiesX, simWidth + 1, simHeight, x + 0.5f, y + 0.5f);
      const f32 velY =
          sim_sample(ctx->demo->velocitiesY, simWidth, simHeight + 1, x + 0.5f, y + 0.5f);

      UiColor color;
      if (sim_solid(ctx, x, y)) {
        color = ui_color_gray;
      } else {
        // const f32 frac = math_clamp_f32(pressure * 0.01f, -1.0f, 1.0f);
        // color          = ui_color_lerp(ui_color_green, ui_color_red, (frac + 1.0f) * 0.5f);

        const f32 frac = math_clamp_f32(smoke * 10, 0.0f, 1.0f);
        color          = ui_color_lerp(ui_color_green, ui_color_red, frac);
      }
      ui_style_color(ctx->winCanvas, color);

      const UiVector pos = ui_vector(cellOrg.x + x * cellSize.x, cellOrg.y + y * cellSize.y);
      ui_layout_set_pos(ctx->winCanvas, UiBase_Canvas, pos, UiBase_Absolute);
      const UiId id = ui_canvas_draw_glyph(
          ctx->winCanvas, UiShape_Square, 5, UiFlags_Interactable | UiFlags_InteractSupportAlt);
      const UiStatus status = ui_canvas_elem_status(ctx->winCanvas, id);
      if (status == UiStatus_Activated) {
        sim_solid_flip(ctx, x, y);
      } else if (status == UiStatus_ActivatedAlt) {
      }
      if (status == UiStatus_Hovered /* && gap_window_key_down(ctx->winComp, GapKey_Tab) */) {
        ctx->demo->targetX = x;
        ctx->demo->targetY = y;

        if (gap_window_key_down(ctx->winComp, GapKey_Tab)) {
          sim_solid_set(ctx, x, y);
        }
        ctx->demo->push = gap_window_key_down(ctx->winComp, GapKey_Control);
        // if (gap_window_key_down(ctx->winComp, GapKey_Control)) {
        //   const f32 forceMul = 1000.0f;
        //   sim_push(
        //       ctx,
        //       x,
        //       y,
        //       ctx->demo->dirX * forceMul * ctx->dt,
        //       ctx->demo->dirY * forceMul * ctx->dt);
        // }
      }

      {
        const f32      lineColorFrac = math_clamp_f32(math_abs(divergence), 0.0f, 1.0f);
        const UiColor  lineColor     = ui_color_lerp(ui_color_green, ui_color_red, lineColorFrac);
        const UiVector posLineA      = {
            cellOrg.x + x * cellSize.x + cellSize.x * 0.5f,
            cellOrg.y + y * cellSize.y + cellSize.y * 0.5f,
        };
        const f32 lineScale = 10.0f;
        ui_style_color(ctx->winCanvas, lineColor);
        ui_line(
            ctx->winCanvas,
            posLineA,
            ui_vector(posLineA.x + velX * lineScale, posLineA.y + velY * lineScale),
            .base  = UiBase_Absolute,
            .width = 4.0f);
      }

      ui_style_color(ctx->winCanvas, ui_color_white);
      const String label = fmt_write_scratch(
          "{}\n{}",
          fmt_float(
              pressure,
              .minDecDigits    = 2,
              .maxDecDigits    = 2,
              .expThresholdPos = f64_max,
              .expThresholdNeg = 0),
          fmt_float(
              divergence,
              .minDecDigits    = 2,
              .maxDecDigits    = 2,
              .expThresholdPos = f64_max,
              .expThresholdNeg = 0));
      ui_canvas_draw_text(ctx->winCanvas, label, 10, UiAlign_MiddleCenter, UiFlags_None);
    }
  }

  ui_layout_resize(ctx->winCanvas, UiAlign_BottomLeft, ui_vector(5, 5), UiBase_Absolute, Ui_XY);

  // Horizontal.
  // for (u32 y = 0; y != simHeight; ++y) {
  //   for (u32 x = 0; x != (simWidth + 1); ++x) {
  //     const f32      velo = ctx->demo->velocitiesX[y * (simWidth + 1) + x];
  //     const UiVector pos  = {
  //         cellOrg.x + x * cellSize.x,
  //         cellOrg.y + y * cellSize.y + cellSize.y * 0.5f,
  //     };
  //     ui_style_color(ctx->winCanvas, ui_color_red);
  //     ui_layout_set_pos(
  //         ctx->winCanvas, UiBase_Canvas, ui_vector(pos.x - 2.5f, pos.y - 2.5f), UiBase_Absolute);
  //     ui_canvas_draw_glyph(ctx->winCanvas, UiShape_Circle, 0, UiFlags_None);

  //     ui_line(
  //         ctx->winCanvas,
  //         pos,
  //         ui_vector(pos.x + velo * cellSize.x * 0.05f, pos.y),
  //         .base  = UiBase_Absolute,
  //         .width = 3.0f);
  //   }
  // }

  // // Vertical.
  // for (u32 y = 0; y != (simHeight + 1); ++y) {
  //   for (u32 x = 0; x != simWidth; ++x) {
  //     const f32      velo = ctx->demo->velocitiesY[y * simWidth + x];
  //     const UiVector pos  = {
  //         cellOrg.x + x * cellSize.x + cellSize.x * 0.5f,
  //         cellOrg.y + y * cellSize.y,
  //     };
  //     ui_style_color(ctx->winCanvas, ui_color_green);
  //     ui_layout_set_pos(
  //         ctx->winCanvas, UiBase_Canvas, ui_vector(pos.x - 2.5f, pos.y - 2.5f), UiBase_Absolute);
  //     ui_canvas_draw_glyph(ctx->winCanvas, UiShape_Circle, 0, UiFlags_None);

  //     ui_line(
  //         ctx->winCanvas,
  //         pos,
  //         ui_vector(pos.x, pos.y + velo * cellSize.y * 0.05f),
  //         .base  = UiBase_Absolute,
  //         .width = 3.0f);
  //   }
  // }

  ui_style_pop(ctx->winCanvas);
  ui_layout_pop(ctx->winCanvas);
  ui_canvas_id_block_next(ctx->winCanvas);
}

static void demo_draw_menu_frame(const DemoUpdateContext* ctx) {
  ui_style_push(ctx->winCanvas);
  ui_style_outline(ctx->winCanvas, 5);
  ui_style_color(ctx->winCanvas, ui_color(0, 0, 0, 200));
  ui_canvas_draw_glyph(ctx->winCanvas, UiShape_Circle, 10, UiFlags_None);
  ui_style_pop(ctx->winCanvas);
}

static void demo_draw_numbox_f32(const DemoUpdateContext* ctx, const String label, f32* value) {
  demo_draw_menu_frame(ctx);
  ui_layout_push(ctx->winCanvas);
  static const UiVector g_frameInset = {-30, -20};
  ui_layout_grow(ctx->winCanvas, UiAlign_MiddleCenter, g_frameInset, UiBase_Absolute, Ui_XY);
  ui_label(ctx->winCanvas, label);
  ui_layout_inner(
      ctx->winCanvas, UiBase_Current, UiAlign_MiddleRight, ui_vector(0.5f, 1), UiBase_Current);
  f64 val = *value;
  if (ui_numbox(ctx->winCanvas, &val)) {
    *value = (f32)val;
  }
  ui_layout_pop(ctx->winCanvas);
}

static void demo_draw_numbox_u32(const DemoUpdateContext* ctx, const String label, u32* value) {
  demo_draw_menu_frame(ctx);
  ui_layout_push(ctx->winCanvas);
  static const UiVector g_frameInset = {-30, -20};
  ui_layout_grow(ctx->winCanvas, UiAlign_MiddleCenter, g_frameInset, UiBase_Absolute, Ui_XY);
  ui_label(ctx->winCanvas, label);
  ui_layout_inner(
      ctx->winCanvas, UiBase_Current, UiAlign_MiddleRight, ui_vector(0.5f, 1), UiBase_Current);
  f64 val = *value;
  if (ui_numbox(ctx->winCanvas, &val, .step = 1.0)) {
    *value = (u32)val;
  }
  ui_layout_pop(ctx->winCanvas);
}

static void demo_draw_menu(const DemoUpdateContext* ctx) {
  const UiVector size    = {250, 40};
  const UiVector spacing = {5, 5};

  ui_layout_inner(ctx->winCanvas, UiBase_Canvas, UiAlign_BottomLeft, size, UiBase_Absolute);
  ui_layout_move(ctx->winCanvas, ui_vector(spacing.x, spacing.y), UiBase_Absolute, Ui_XY);

  demo_draw_numbox_f32(ctx, string_lit("Density"), &ctx->demo->density);
  ui_layout_next(ctx->winCanvas, Ui_Up, spacing.y);
  demo_draw_numbox_f32(ctx, string_lit("Velocity diff"), &ctx->demo->velDiffusion);
  ui_layout_next(ctx->winCanvas, Ui_Up, spacing.y);
  demo_draw_numbox_f32(ctx, string_lit("Smoke diff"), &ctx->demo->smokeDiffusion);
  ui_layout_next(ctx->winCanvas, Ui_Up, spacing.y);
  demo_draw_numbox_f32(ctx, string_lit("Smoke decay"), &ctx->demo->smokeDecay);
  ui_layout_next(ctx->winCanvas, Ui_Up, spacing.y);
  demo_draw_numbox_u32(ctx, string_lit("Solver itrs"), &ctx->demo->solverIterations);
}

ecs_view_define(FrameUpdateView) { ecs_access_write(RendSettingsGlobalComp); }

ecs_view_define(ErrorView) {
  ecs_access_maybe_read(GapErrorComp);
  ecs_access_maybe_read(RendErrorComp);
}

ecs_view_define(UpdateGlobalView) { ecs_access_write(DemoComp); }

ecs_view_define(WindowView) {
  ecs_access_write(DemoWindowComp);
  ecs_access_write(GapWindowComp);
}

ecs_view_define(UiCanvasView) {
  ecs_view_flags(EcsViewFlags_Exclusive); // Only access the canvas's we create.
  ecs_access_write(UiCanvasComp);
}

ecs_system_define(DemoUpdateSys) {
  EcsView*     globalView = ecs_world_view_t(world, UpdateGlobalView);
  EcsIterator* globalItr  = ecs_view_maybe_at(globalView, ecs_world_global(world));
  if (!globalItr) {
    return;
  }

  DemoUpdateContext ctx = {
      .world = world,
      .demo  = ecs_view_write_t(globalItr, DemoComp),
  };

  const TimeSteady timeNew   = time_steady_clock();
  TimeDuration     timeDelta = 0;
  if (ctx.demo->lastTime) {
    timeDelta = time_steady_duration(ctx.demo->lastTime, timeNew);
    timeDelta = math_min(timeDelta, time_second); // Avoid huge delta's when process was paused.
  }
  ctx.demo->lastTime = timeNew;
  ctx.dt             = sim_time_to_seconds(timeDelta);

  EcsIterator* canvasItr = ecs_view_itr(ecs_world_view_t(world, UiCanvasView));
  EcsIterator* winItr    = ecs_view_maybe_at(ecs_world_view_t(world, WindowView), ctx.demo->window);

  if (winItr) {
    ctx.winEntity = ecs_view_entity(winItr);
    ctx.winDemo   = ecs_view_write_t(winItr, DemoWindowComp);
    ctx.winComp   = ecs_view_write_t(winItr, GapWindowComp);

    if (gap_window_key_down(ctx.winComp, GapKey_Alt) &&
        gap_window_key_pressed(ctx.winComp, GapKey_F4)) {
      gap_window_close(ctx.winComp);
    }

    if (ecs_view_maybe_jump(canvasItr, ctx.winDemo->uiCanvas)) {
      ctx.winCanvas = ecs_view_write_t(canvasItr, UiCanvasComp);
      ui_canvas_reset(ctx.winCanvas);
    }

    const bool textEditor = ctx.winCanvas && ui_canvas_text_editor_active_any(ctx.winCanvas);
    if (!textEditor && gap_window_key_pressed(ctx.winComp, GapKey_Alpha1)) {
      ctx.demo->solve ^= true;
    }
    if (!textEditor && gap_window_key_pressed(ctx.winComp, GapKey_Alpha2)) {
      ctx.demo->updateVelocities ^= true;
    }
    if (!textEditor && gap_window_key_pressed(ctx.winComp, GapKey_Alpha3)) {
      sim_velocity_randomize(&ctx);
    }
    if (!textEditor && gap_window_key_pressed(ctx.winComp, GapKey_Alpha4)) {
      sim_velocity_clear(&ctx);
    }
    if (!textEditor && gap_window_key_pressed(ctx.winComp, GapKey_Alpha5)) {
      sim_pressure_clear(&ctx);
    }
    if (!textEditor && gap_window_key_pressed(ctx.winComp, GapKey_Alpha6)) {
      sim_solid_set_border(&ctx);
    }

    if (ctx.dt > f32_epsilon) {
      sim_update(&ctx);
    }
    sim_draw(&ctx);
    demo_draw_menu(&ctx);
  }
}

ecs_module_init(demo_module) {
  ecs_register_comp(DemoComp, .destructor = demo_destroy);
  ecs_register_comp(DemoWindowComp);

  ecs_register_view(FrameUpdateView);
  ecs_register_view(ErrorView);
  ecs_register_view(UpdateGlobalView);
  ecs_register_view(WindowView);
  ecs_register_view(UiCanvasView);

  ecs_register_system(
      DemoUpdateSys,
      ecs_view_id(UpdateGlobalView),
      ecs_view_id(WindowView),
      ecs_view_id(UiCanvasView));
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

  const u16 windowWidth  = (u16)cli_read_u64(invoc, g_optWidth, 1600);
  const u16 windowHeight = (u16)cli_read_u64(invoc, g_optHeight, 1600);

  DemoComp* demo = demo_create(world);

  demo->window = demo_window_create(world, windowWidth, windowHeight);
  rend_settings_window_init(world, demo->window)->flags |= RendFlags_2D;

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

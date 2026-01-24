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
#include "core/math.h"
#include "core/version.h"
#include "ecs/entity.h"
#include "ecs/utils.h"
#include "ecs/view.h"
#include "gap/error.h"
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

ecs_comp_define(DemoComp) {
  EcsEntityId window;
  TimeSteady  lastTime;

  u32    solverIterations;
  f32    density;
  u32    simWidth, simHeight;
  f32*   velocitiesX; // f32[simHeight * (simWidth + 1)]
  f32*   velocitiesY; // f32[(simHeight + 1) * simWidth]
  f32*   pressure;    // f32[simHeight * simWidth]
  BitSet solidMap;    // bit[simHeight * simWidth]
};

ecs_comp_define(DemoWindowComp) { EcsEntityId uiCanvas; };

static DemoComp* demo_create(EcsWorld* world) {
  const EcsEntityId global = ecs_world_global(world);

  DemoComp* demo = ecs_world_add_t(
      world,
      global,
      DemoComp,
      .solverIterations = 16,
      .density          = 1.0f,
      .simWidth         = 10,
      .simHeight        = 10);

  const u32 width  = demo->simWidth;
  const u32 height = demo->simHeight;

  demo->velocitiesX = alloc_array_t(g_allocHeap, f32, height * (width + 1));
  demo->velocitiesY = alloc_array_t(g_allocHeap, f32, (height + 1) * width);
  demo->pressure    = alloc_array_t(g_allocHeap, f32, height * width);
  demo->solidMap    = alloc_alloc(g_allocHeap, bits_to_bytes(height * width) + 1, 1);

  mem_set(mem_create(demo->velocitiesX, sizeof(f32) * height * (width + 1)), 0);
  mem_set(mem_create(demo->velocitiesY, sizeof(f32) * (height + 1) * width), 0);
  mem_set(mem_create(demo->pressure, sizeof(f32) * height * width), 0);
  mem_set(demo->solidMap, 0);

  return demo;
}

static void demo_destroy(void* data) {
  DemoComp* comp   = data;
  const u32 width  = comp->simWidth;
  const u32 height = comp->simHeight;

  alloc_free_array_t(g_allocHeap, comp->velocitiesX, height * (width + 1));
  alloc_free_array_t(g_allocHeap, comp->velocitiesY, (height + 1) * width);
  alloc_free_array_t(g_allocHeap, comp->pressure, height * width);
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

static bool sim_solid(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const u32 width = ctx->demo->simWidth;
  return bitset_test(ctx->demo->solidMap, y * width + x);
}

static void sim_solid_flip(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const u32 width = ctx->demo->simWidth;
  bitset_flip(ctx->demo->solidMap, y * width + x);
}

static f32 sim_pressure(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const u32 width = ctx->demo->simWidth;
  return ctx->demo->pressure[y * width + x];
}

static f32 sim_velocity_bottom(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const u32 width = ctx->demo->simWidth;
  return ctx->demo->velocitiesY[y * width + x];
}

static f32 sim_velocity_top(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const u32 width = ctx->demo->simWidth;
  return ctx->demo->velocitiesY[(y + 1) * width + x];
}

static f32 sim_velocity_left(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const u32 width = ctx->demo->simWidth + 1;
  return ctx->demo->velocitiesX[y * width + x];
}

static f32 sim_velocity_right(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const u32 width = ctx->demo->simWidth + 1;
  return ctx->demo->velocitiesX[y * width + (x + 1)];
}

static void sim_attract(DemoUpdateContext* ctx, const u32 x, const u32 y, const f32 force) {
  ctx->demo->velocitiesY[y * ctx->demo->simWidth + x] -= force * ctx->dt;
  ctx->demo->velocitiesY[(y + 1) * ctx->demo->simWidth + x] += force * ctx->dt;
  ctx->demo->velocitiesX[y * (ctx->demo->simWidth + 1) + x] -= force * ctx->dt;
  ctx->demo->velocitiesX[y * (ctx->demo->simWidth + 1) + (x + 1)] += force * ctx->dt;
}

static f32 sim_velocity_divergence(DemoUpdateContext* ctx, const u32 x, const u32 y) {
  const f32 velTop    = sim_velocity_top(ctx, x, y);
  const f32 velLeft   = sim_velocity_left(ctx, x, y);
  const f32 velRight  = sim_velocity_right(ctx, x, y);
  const f32 velBottom = sim_velocity_bottom(ctx, x, y);
  return (velRight - velLeft) + (velTop - velBottom);
}

static void sim_solve_pressure(DemoUpdateContext* ctx) {
  const u32 width   = ctx->demo->simWidth;
  const u32 height  = ctx->demo->simHeight;
  const f32 density = ctx->demo->density;
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != width; ++x) {

      const bool flowTop    = !sim_solid(ctx, x + 0, y + 1);
      const bool flowLeft   = !sim_solid(ctx, x - 1, y + 0);
      const bool flowRight  = !sim_solid(ctx, x + 1, y + 0);
      const bool flowBottom = !sim_solid(ctx, x + 0, y - 1);
      const u8   flowCount  = flowLeft + flowRight + flowTop + flowBottom;

      f32 newPressure;
      if (sim_solid(ctx, x, y) || !flowCount) {
        newPressure = 0.0f;
      } else {
        const f32 pressureTop    = sim_pressure(ctx, x, math_min(y + 1, height - 1)) * flowTop;
        const f32 pressureLeft   = sim_pressure(ctx, x ? (x - 1) : 0, y) * flowLeft;
        const f32 pressureRight  = sim_pressure(ctx, math_min(x + 1, width - 1), y) * flowRight;
        const f32 pressureBottom = sim_pressure(ctx, x, y ? (y - 1) : 0) * flowBottom;
        const f32 pressureSum    = pressureRight + pressureLeft + pressureTop + pressureBottom;

        const f32 velTop    = sim_velocity_top(ctx, x, y);
        const f32 velLeft   = sim_velocity_left(ctx, x, y);
        const f32 velRight  = sim_velocity_right(ctx, x, y);
        const f32 velBottom = sim_velocity_bottom(ctx, x, y);
        const f32 velTerm   = (velRight - velLeft + velTop - velBottom) / ctx->dt;

        newPressure = (pressureSum - density * velTerm) / flowCount;
      }
      ctx->demo->pressure[y * width + x] = newPressure;
    }
  }
}

static void sim_update_velocities(DemoUpdateContext* ctx) {
  const f32 k      = ctx->dt / ctx->demo->density;
  const u32 width  = ctx->demo->simWidth;
  const u32 height = ctx->demo->simHeight;

  // Horizontal.
  for (u32 y = 0; y != height; ++y) {
    for (u32 x = 0; x != (width + 1); ++x) {
      if (sim_solid(ctx, x, y) || (x && sim_solid(ctx, x - 1, y))) {
        ctx->demo->velocitiesX[y * (width + 1) + x] = 0;
        continue;
      }
      const f32 pressureRight = sim_pressure(ctx, math_min(x, width - 1), y);
      const f32 pressureLeft  = x ? sim_pressure(ctx, x - 1, y) : pressureRight;
      ctx->demo->velocitiesX[y * (width + 1) + x] -= k * (pressureRight - pressureLeft);
    }
  }

  // Vertical.
  for (u32 y = 0; y != (height + 1); ++y) {
    for (u32 x = 0; x != width; ++x) {
      if (sim_solid(ctx, x, y) || (y && sim_solid(ctx, x, y - 1))) {
        ctx->demo->velocitiesY[y * width + x] = 0;
        continue;
      }
      const f32 pressureTop    = sim_pressure(ctx, x, math_min(y, height - 1));
      const f32 pressureBottom = y ? sim_pressure(ctx, x, y - 1) : pressureTop;
      ctx->demo->velocitiesY[y * width + x] -= k * (pressureTop - pressureBottom);
    }
  }
}

static void sim_update(DemoUpdateContext* ctx) {
  for (u32 i = 0; i != ctx->demo->solverIterations; ++i) {
    sim_solve_pressure(ctx);
  }
  sim_update_velocities(ctx);
}

static void sim_draw(DemoUpdateContext* ctx) {
  ui_canvas_id_block_next(ctx->winCanvas);
  ui_layout_push(ctx->winCanvas);
  ui_style_push(ctx->winCanvas);

  const UiVector canvasSize = ui_canvas_resolution(ctx->winCanvas);
  const UiVector cellSize   = ui_vector(50, 50);
  const UiVector cellOrg    = {
      canvasSize.width * 0.5f - ctx->demo->simWidth * cellSize.width * 0.5f,
      canvasSize.height * 0.5f - ctx->demo->simHeight * cellSize.height * 0.5f,
  };

  ui_layout_resize(ctx->winCanvas, UiAlign_BottomLeft, cellSize, UiBase_Absolute, Ui_XY);
  for (u32 y = 0; y != ctx->demo->simHeight; ++y) {
    for (u32 x = 0; x != ctx->demo->simWidth; ++x) {
      UiColor color;
      if (sim_solid(ctx, x, y)) {
        color = ui_color_gray;
      } else {
        const f32 divergence = sim_velocity_divergence(ctx, x, y);
        const f32 pressure   = sim_pressure(ctx, x, y);
        const f32 frac       = math_clamp_f32(divergence, 0.0f, 1.0f);
        color                = ui_color_lerp(ui_color_green, ui_color_red, frac);
      }
      ui_style_color(ctx->winCanvas, color);

      const UiVector pos = ui_vector(cellOrg.x + x * cellSize.x, cellOrg.y + y * cellSize.y);
      ui_layout_set_pos(ctx->winCanvas, UiBase_Canvas, pos, UiBase_Absolute);
      const UiId id = ui_canvas_draw_glyph(ctx->winCanvas, UiShape_Square, 5, UiFlags_Interactable);
      const UiStatus status = ui_canvas_elem_status(ctx->winCanvas, id);
      if (status == UiStatus_Activated) {
        sim_solid_flip(ctx, x, y);
      }
      if (status == UiStatus_Hovered) {
        sim_attract(ctx, x, y, 10.f);
      }

      const f32 velTop    = sim_velocity_top(ctx, x, y);
      const f32 velLeft   = sim_velocity_left(ctx, x, y);
      const f32 velRight  = sim_velocity_right(ctx, x, y);
      const f32 velBottom = sim_velocity_bottom(ctx, x, y);

      ui_style_color(ctx->winCanvas, ui_color_white);
      const UiVector posCenter = {
          pos.x + cellSize.x * 0.5f,
          pos.y + cellSize.y * 0.5f,
      };
      const f32 yTop   = pos.y + cellSize.y;
      const f32 xRight = pos.x + cellSize.x;
      ui_line(
          ctx->winCanvas,
          ui_vector(posCenter.x, yTop),
          ui_vector(posCenter.x, yTop - math_clamp_f32(velTop, -100, 100)),
          .base  = UiBase_Absolute,
          .width = 3);
      ui_line(
          ctx->winCanvas,
          ui_vector(posCenter.x, pos.y),
          ui_vector(posCenter.x, pos.y + math_clamp_f32(velBottom, -100, 100)),
          .base  = UiBase_Absolute,
          .width = 3);
      ui_line(
          ctx->winCanvas,
          ui_vector(pos.x, posCenter.y),
          ui_vector(pos.x + math_clamp_f32(velLeft, -100, 100), posCenter.y),
          .base  = UiBase_Absolute,
          .width = 3);
      ui_line(
          ctx->winCanvas,
          ui_vector(xRight, posCenter.y),
          ui_vector(xRight - math_clamp_f32(velRight, -100, 100), posCenter.y),
          .base  = UiBase_Absolute,
          .width = 3);
    }
  }

  ui_style_pop(ctx->winCanvas);
  ui_layout_pop(ctx->winCanvas);
  ui_canvas_id_block_next(ctx->winCanvas);
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
    if (ctx.dt > f32_epsilon) {
      sim_update(&ctx);
    }
    sim_draw(&ctx);
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

  const u16 windowWidth  = (u16)cli_read_u64(invoc, g_optWidth, 800);
  const u16 windowHeight = (u16)cli_read_u64(invoc, g_optHeight, 800);

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

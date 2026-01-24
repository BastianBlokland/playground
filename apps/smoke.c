#include "app/ecs.h"
#include "asset/manager.h"
#include "asset/register.h"
#include "cli/app.h"
#include "cli/parse.h"
#include "cli/read.h"
#include "cli/validate.h"
#include "core/diag.h"
#include "core/file.h"
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

ecs_comp_define(DemoComp) {
  EcsEntityId window;

  u32 simWidth, simHeight;
};

ecs_comp_define(DemoWindowComp) { EcsEntityId uiCanvas; };

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

  EcsEntityId     winEntity;
  DemoWindowComp* winDemo;
  GapWindowComp*  winComp;
  UiCanvasComp*   winCanvas;
} DemoUpdateContext;

static void sim_update(DemoUpdateContext* ctx) { (void)ctx; }

static void sim_draw(DemoUpdateContext* ctx) {
  ui_canvas_id_block_next(ctx->winCanvas);
  ui_layout_push(ctx->winCanvas);
  ui_style_push(ctx->winCanvas);

  const UiVector canvasSize = ui_canvas_resolution(ctx->winCanvas);
  const UiVector cellSize   = ui_vector(15, 15);
  const UiVector cellOrg    = {
      canvasSize.width * 0.5f - ctx->demo->simWidth * cellSize.width * 0.5f,
      canvasSize.height * 0.5f - ctx->demo->simHeight * cellSize.height * 0.5f,
  };

  ui_layout_resize(ctx->winCanvas, UiAlign_BottomLeft, cellSize, UiBase_Absolute, Ui_XY);
  for (u32 y = 0; y != ctx->demo->simHeight; ++y) {
    for (u32 x = 0; x != ctx->demo->simWidth; ++x) {
      const UiVector pos = ui_vector(cellOrg.x + x * cellSize.x, cellOrg.y + y * cellSize.y);
      ui_layout_set_pos(ctx->winCanvas, UiBase_Canvas, pos, UiBase_Absolute);
      ui_canvas_draw_glyph(ctx->winCanvas, UiShape_Square, 5, UiFlags_None);
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
    sim_update(&ctx);
    sim_draw(&ctx);
  }
}

static void ecs_destruct_demo_comp(void* data) {
  DemoComp* comp = data;
  (void)comp;
}

ecs_module_init(demo_module) {
  ecs_register_comp(DemoComp, .destructor = ecs_destruct_demo_comp);
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

  const EcsEntityId window             = demo_window_create(world, windowWidth, windowHeight);
  RendSettingsComp* rendSettingsWindow = rend_settings_window_init(world, window);
  rendSettingsWindow->flags |= RendFlags_2D;

  ecs_world_add_t(
      world, ecs_world_global(world), DemoComp, .window = window, .simWidth = 50, .simHeight = 50);
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

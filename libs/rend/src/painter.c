#include "core/bits.h"
#include "core/diag.h"
#include "core/float.h"
#include "core/math.h"
#include "ecs/entity.h"
#include "ecs/utils.h"
#include "ecs/view.h"
#include "log/logger.h"
#include "rend/camera.h"
#include "rend/register.h"
#include "rvk/canvas.h"
#include "rvk/graphic.h"
#include "rvk/mesh.h"
#include "rvk/repository.h"
#include "rvk/texture.h"

#include "builder.h"
#include "light.h"
#include "object.h"
#include "painter.h"
#include "platform.h"
#include "reset.h"
#include "resource.h"
#include "view.h"

ecs_comp_define(RendPainterComp);

static void ecs_destruct_painter(void* data) {
  RendPainterComp* comp = data;
  rvk_canvas_destroy(comp->canvas);
}

ecs_view_define(GlobalView) {
  ecs_access_read(RendLightRendererComp);
  ecs_access_read(RendSettingsGlobalComp);
  ecs_access_without(RendResetComp);
  ecs_access_write(RendPlatformComp);
}

ecs_view_define(ObjView) { ecs_access_read(RendObjectComp); }

ecs_view_define(ResourceView) {
  ecs_access_maybe_read(RendResGraphicComp);
  ecs_access_maybe_read(RendResMeshComp);
  ecs_access_maybe_read(RendResTextureComp);
  ecs_access_with(RendResFinishedComp);
  ecs_access_without(RendResUnloadComp);
  ecs_access_read(RendResComp);
}

ecs_view_define(PainterCreateView) {
  ecs_access_read(GapWindowComp);
  ecs_access_without(RendPainterComp);
}

ecs_view_define(PainterUpdateView) {
  ecs_access_read(GapWindowComp);
  ecs_access_write(RendPainterComp);
  ecs_access_read(RendSettingsComp);

  ecs_access_maybe_read(RendCameraComp);
}

static RvkSize painter_win_size(const GapWindowComp* win) {
  const GapVector winSize = gap_window_param(win, GapParam_WindowSize);
  return rvk_size((u16)winSize.width, (u16)winSize.height);
}

static RendView painter_view_2d_create(const EcsEntityId sceneCameraEntity) {
  const GeoVector     cameraPosition = geo_vector(0);
  const GeoMatrix     viewProjMatrix = geo_matrix_ident();
  const RendTagFilter sceneFilter    = {0};
  return rend_view_create(sceneCameraEntity, cameraPosition, &viewProjMatrix, sceneFilter);
}

static RendView painter_view_3d_create(
    const GeoMatrix*    cameraMatrix,
    const GeoMatrix*    projMatrix,
    const EcsEntityId   sceneCameraEntity,
    const RendTagFilter sceneFilter) {
  const GeoVector cameraPosition = geo_matrix_to_translation(cameraMatrix);
  const GeoMatrix viewMatrix     = geo_matrix_inverse(cameraMatrix);
  const GeoMatrix viewProjMatrix = geo_matrix_mul(projMatrix, &viewMatrix);
  return rend_view_create(sceneCameraEntity, cameraPosition, &viewProjMatrix, sceneFilter);
}

typedef struct {
  RendBuilder*                  builder;
  const RendSettingsComp*       set;
  const RendSettingsGlobalComp* setGlobal;
  RendView                      view;
} RendPaintContext;

static RendPaintContext painter_context(
    RendBuilder*                  builder,
    const RendSettingsComp*       set,
    const RendSettingsGlobalComp* setGlobal,
    const RendView                view) {

  return (RendPaintContext){
      .builder   = builder,
      .set       = set,
      .setGlobal = setGlobal,
      .view      = view,
  };
}

typedef enum {
  RendViewType_Main,
  RendViewType_Shadow,
} RendViewType;

static void painter_set_global_data(
    RendPaintContext*             ctx,
    const GeoMatrix*              cameraMatrix,
    const GeoMatrix*              projMatrix,
    const RvkSize                 size,
    const RendSettingsGlobalComp* setGlobal,
    const RendViewType            viewType) {
  const f32 aspect = (f32)size.width / (f32)size.height;

  typedef struct {
    ALIGNAS(16)
    GeoMatrix view, viewInv;
    GeoMatrix proj, projInv;
    GeoMatrix viewProj, viewProjInv;
    GeoVector camPosition;
    GeoQuat   camRotation;
    GeoVector resolution; // x: width, y: height, z: aspect ratio (width / height), w: unused.
    GeoVector time;       // x: time seconds, y: real-time seconds, z, w: unused.
  } RendPainterGlobalData;
  ASSERT(sizeof(RendPainterGlobalData) == 448, "Size needs to match the size defined in glsl");

  const u32              dataSize = sizeof(RendPainterGlobalData);
  RendPainterGlobalData* data     = rend_builder_global_data(ctx->builder, dataSize, 0).ptr;

  *data = (RendPainterGlobalData){
      .resolution.x = size.width,
      .resolution.y = size.height,
      .resolution.z = aspect,
      .time.x       = rend_settings_time_seconds(setGlobal),
      .time.y       = rend_settings_real_time_seconds(setGlobal),
  };

  if (viewType == RendViewType_Main && ctx->set->flags & RendFlags_DebugCamera) {
    static const f32 g_size     = 300;
    static const f32 g_depthMin = -200;
    static const f32 g_depthMax = 200;

    data->viewInv     = geo_matrix_rotate_x(math_pi_f32 * 0.5f);
    data->view        = geo_matrix_inverse(&data->viewInv);
    data->proj        = geo_matrix_proj_ortho_hor(g_size, aspect, g_depthMin, g_depthMax);
    data->projInv     = geo_matrix_inverse(&data->proj);
    data->viewProj    = geo_matrix_mul(&data->proj, &data->view);
    data->viewProjInv = geo_matrix_inverse(&data->viewProj);
    data->camPosition = geo_vector(0, 0, 0);
    data->camRotation = geo_quat_forward_to_down;
  } else {
    if (cameraMatrix) {
      data->viewInv     = *cameraMatrix;
      data->view        = geo_matrix_inverse(cameraMatrix);
      data->camPosition = geo_matrix_to_translation(cameraMatrix);
      data->camRotation = geo_matrix_to_quat(cameraMatrix);
    } else {
      data->viewInv     = geo_matrix_ident();
      data->view        = geo_matrix_ident();
      data->camPosition = geo_vector(0);
      data->camRotation = geo_quat_ident;
    }
    if (projMatrix) {
      data->proj    = *projMatrix;
      data->projInv = geo_matrix_inverse(projMatrix);
    } else {
      data->proj    = geo_matrix_ident();
      data->projInv = geo_matrix_ident();
    }
    data->viewProj    = geo_matrix_mul(&data->proj, &data->view);
    data->viewProjInv = geo_matrix_inverse(&data->viewProj);
  }
}

static const RvkGraphic* painter_get_graphic(EcsIterator* resourceItr, const EcsEntityId resource) {
  if (!ecs_view_maybe_jump(resourceItr, resource)) {
    return null; // Resource not loaded yet.
  }
  const RendResComp* resComp = ecs_view_read_t(resourceItr, RendResComp);
  if (rend_res_is_failed(resComp)) {
    return null; // Failed to load.
  }
  const RendResGraphicComp* graphicRes = ecs_view_read_t(resourceItr, RendResGraphicComp);
  if (!graphicRes) {
    log_e("Invalid graphic asset", log_param("entity", ecs_entity_fmt(resource)));
    return null;
  }
  return graphicRes->graphic;
}

static const RvkTexture* painter_get_texture(EcsIterator* resourceItr, const EcsEntityId resource) {
  if (!ecs_view_maybe_jump(resourceItr, resource)) {
    return null; // Resource not loaded yet.
  }
  const RendResComp* resComp = ecs_view_read_t(resourceItr, RendResComp);
  if (rend_res_is_failed(resComp)) {
    return null; // Failed to load.
  }
  const RendResTextureComp* textureRes = ecs_view_read_t(resourceItr, RendResTextureComp);
  if (!textureRes) {
    log_e("Invalid texture asset", log_param("entity", ecs_entity_fmt(resource)));
    return null;
  }
  return textureRes->texture;
}

static bool painter_graphic_should_draw(RendPaintContext* ctx, const RvkGraphic* graphic) {
  if ((rend_builder_pass_mask(ctx->builder) & graphic->passReq) != graphic->passReq) {
    return false; // Required passes are not drawn this frame.
  }
  return true;
}

static void painter_push_simple(RendPaintContext* ctx, const RvkRepositoryId id, const Mem data) {
  const RvkRepository* repo    = rend_builder_repository(ctx->builder);
  const RvkGraphic*    graphic = rvk_repository_graphic_get(repo, id);
  if (graphic && painter_graphic_should_draw(ctx, graphic)) {
    rend_builder_draw_push(ctx->builder, graphic);
    if (data.size) {
      mem_cpy(rend_builder_draw_data(ctx->builder, (u32)data.size), data);
    }
    rend_builder_draw_instances(ctx->builder, 0 /* dataStride */, 1 /* count */);
    rend_builder_draw_flush(ctx->builder);
  }
}

static RendTags painter_push_objects_simple(
    RendPaintContext* ctx, EcsView* objView, EcsView* resView, const AssetGraphicPass passId) {
  RendTags     tagMask     = 0;
  EcsIterator* resourceItr = ecs_view_itr(resView);
  for (EcsIterator* objItr = ecs_view_itr(objView); ecs_view_walk(objItr);) {
    const RendObjectComp* obj = ecs_view_read_t(objItr, RendObjectComp);
    if (!rend_object_instance_count(obj)) {
      continue; // Object has no instances.
    }

    // Retrieve the object's graphic.
    const EcsEntityId graphicResource = rend_object_resource(obj, RendObjectRes_Graphic);
    const RvkGraphic* graphic         = painter_get_graphic(resourceItr, graphicResource);
    if (!graphic || graphic->passId != passId || !painter_graphic_should_draw(ctx, graphic)) {
      continue; // Graphic not loaded or not valid for this pass.
    }

    // If the object uses a 'per draw' texture then retrieve it.
    const EcsEntityId textureResource = rend_object_resource(obj, RendObjectRes_Texture);
    const RvkTexture* texture         = null;
    if (textureResource && !(texture = painter_get_texture(resourceItr, textureResource))) {
      continue; // Object uses a 'per draw' texture which is not loaded (yet).
    }

    rend_builder_draw_push(ctx->builder, graphic);
    if (texture) {
      rend_builder_draw_image_frozen(ctx->builder, &texture->image);
    }
    rend_object_draw(obj, &ctx->view, ctx->set, ctx->builder);
    rend_builder_draw_flush(ctx->builder);

    tagMask |= rend_object_tag_mask(obj);
  }

  return tagMask;
}

static void painter_push_shadow(RendPaintContext* ctx, EcsView* objView, EcsView* resView) {
  const RvkRepository* repo     = rend_builder_repository(ctx->builder);
  const RvkTexture*    whiteTex = rvk_repository_texture_get(repo, RvkRepositoryId_WhiteTexture);
  if (!whiteTex) {
    return; // Texture not loaded (yet).
  }
  EcsIterator* resourceItr = ecs_view_itr(resView);
  for (EcsIterator* objItr = ecs_view_itr(objView); ecs_view_walk(objItr);) {
    const RendObjectComp* obj = ecs_view_read_t(objItr, RendObjectComp);
    if (!rend_object_instance_count(obj)) {
      continue; // Object has no instances.
    }
    const EcsEntityId graphicRes = rend_object_resource(obj, RendObjectRes_GraphicShadow);
    if (!graphicRes) {
      continue; // Object has no shadow graphic.
    }
    const RvkGraphic* graphic = painter_get_graphic(resourceItr, graphicRes);
    if (!graphic || !painter_graphic_should_draw(ctx, graphic)) {
      continue; // Shadow graphic is not loaded or has unmet dependencies.
    }
    if (UNLIKELY(graphic->passId != AssetGraphicPass_Shadow)) {
      log_e("Shadow's can only be drawn from the shadow pass");
      continue;
    }

    const EcsEntityId graphicOrgRes = rend_object_resource(obj, RendObjectRes_Graphic);
    const RvkGraphic* graphicOrg    = painter_get_graphic(resourceItr, graphicOrgRes);
    if (!graphicOrg) {
      continue; // Graphic is not loaded.
    }

    rend_builder_draw_push(ctx->builder, graphic);
    rend_builder_draw_mesh(ctx->builder, graphicOrg->mesh);

    const RvkTexture* alphaTex;
    const u8          alphaTexIndex = rend_object_alpha_tex_index(obj);
    if (sentinel_check(alphaTexIndex) || !(graphicOrg->samplerMask & (1 << alphaTexIndex))) {
      alphaTex = whiteTex;
    } else {
      alphaTex = graphicOrg->samplerTextures[alphaTexIndex];
    }
    rend_builder_draw_image_frozen(ctx->builder, &alphaTex->image);
    rend_builder_draw_sampler(ctx->builder, (RvkSamplerSpec){.aniso = RvkSamplerAniso_x8});

    rend_object_draw(obj, &ctx->view, ctx->set, ctx->builder);
    rend_builder_draw_flush(ctx->builder);
  }
}

static void painter_push_ambient(RendPaintContext* ctx, const GeoColor radiance) {
  typedef enum {
    AmbientFlags_AmbientOcclusion     = 1 << 0,
    AmbientFlags_AmbientOcclusionBlur = 1 << 1,
  } AmbientFlags;

  struct {
    ALIGNAS(16)
    GeoColor radiance;  // rgb: radiance, w: unused.
    u32      packed[4]; // x: mode, y: flags, zw: unused.
  } data;

  AmbientFlags flags = 0;
  if (ctx->set->flags & RendFlags_AmbientOcclusion) {
    flags |= AmbientFlags_AmbientOcclusion;
  }
  if (ctx->set->flags & RendFlags_AmbientOcclusionBlur) {
    flags |= AmbientFlags_AmbientOcclusionBlur;
  }

  data.radiance  = radiance;
  data.packed[0] = ctx->set->ambientMode;
  data.packed[1] = flags;

  RvkRepositoryId graphicId;
  if (ctx->set->ambientMode >= RendAmbientMode_DebugStart) {
    graphicId = RvkRepositoryId_AmbientDebugGraphic;
  } else {
    graphicId = RvkRepositoryId_AmbientGraphic;
  }
  painter_push_simple(ctx, graphicId, mem_var(data));
}

static void painter_push_ambient_occlusion(RendPaintContext* ctx) {
  struct {
    ALIGNAS(16)
    f32       radius;
    f32       power;
    GeoVector kernel[rend_ao_kernel_size];
  } data;

  data.radius = ctx->set->aoRadius;
  data.power  = ctx->set->aoPower;

  const Mem kernel = mem_create(ctx->set->aoKernel, sizeof(GeoVector) * rend_ao_kernel_size);
  mem_cpy(array_mem(data.kernel), kernel);

  painter_push_simple(ctx, RvkRepositoryId_AmbientOcclusionGraphic, mem_var(data));
}

static void painter_push_tonemapping(RendPaintContext* ctx) {
  struct {
    ALIGNAS(16)
    f32 exposure;
    u32 mode;
    f32 grayscaleFrac;
    f32 unused;
  } data;

  data.exposure      = ctx->set->exposure;
  data.mode          = ctx->set->tonemapper;
  data.grayscaleFrac = ctx->set->grayscaleFrac;

  painter_push_simple(ctx, RvkRepositoryId_TonemapperGraphic, mem_var(data));
}

static void
painter_push_debug_image_viewer(RendPaintContext* ctx, RvkImage* image, const f32 exposure) {
  const RvkRepository* repo = rend_builder_repository(ctx->builder);
  const RvkGraphic*    graphic;
  switch (image->type) {
  case RvkImageType_ColorSourceArray:
    graphic = rvk_repository_graphic_get(repo, RvkRepositoryId_DebugImageViewerArrayGraphic);
    break;
  case RvkImageType_ColorSourceCube:
    graphic = rvk_repository_graphic_get(repo, RvkRepositoryId_DebugImageViewerCubeGraphic);
    break;
  default:
    graphic = rvk_repository_graphic_get(repo, RvkRepositoryId_DebugImageViewerGraphic);
    break;
  }
  if (graphic) {
    enum {
      ImageViewerFlags_FlipY       = 1 << 0,
      ImageViewerFlags_AlphaIgnore = 1 << 1,
      ImageViewerFlags_AlphaOnly   = 1 << 2,
    };

    u32 flags = 0;
    if (image->type != RvkImageType_ColorSource && image->type != RvkImageType_ColorSourceCube) {
      /**
       * Volo is using source textures with the image origin at the bottom left (as opposed to the
       * conventional top left). This is an historical mistake that should be corrected but until
       * that time we need to flip non-source (attachments) images as they are using top-left.
       */
      flags |= ImageViewerFlags_FlipY;
    }
    if (ctx->set->debugViewerFlags & RendDebugViewer_AlphaIgnore) {
      flags |= ImageViewerFlags_AlphaIgnore;
    }
    if (ctx->set->debugViewerFlags & RendDebugViewer_AlphaOnly) {
      flags |= ImageViewerFlags_AlphaOnly;
    }

    struct {
      ALIGNAS(16)
      u32 flags;
      u32 imageChannels;
      f32 lod;
      f32 layer;
      f32 exposure;
      f32 aspect;
    } data = {
        .flags         = flags,
        .imageChannels = vkFormatComponents(image->vkFormat),
        .lod           = ctx->set->debugViewerLod,
        .layer         = ctx->set->debugViewerLayer,
        .exposure      = exposure,
        .aspect        = (f32)image->size.width / (f32)image->size.height,
    };

    rend_builder_draw_push(ctx->builder, graphic);
    mem_cpy(rend_builder_draw_data(ctx->builder, sizeof(data)), mem_var(data));

    RvkSamplerSpec sampler = {.filter = RvkSamplerFilter_Nearest};
    if (ctx->set->debugViewerFlags & RendDebugViewer_Interpolate) {
      sampler.filter = RvkSamplerFilter_Linear;
    }
    rend_builder_draw_image(ctx->builder, image);
    rend_builder_draw_sampler(ctx->builder, sampler);
    rend_builder_draw_instances(ctx->builder, 0 /* dataStride */, 1 /* count */);
    rend_builder_draw_flush(ctx->builder);
  }
}

static void
painter_push_debug_mesh_viewer(RendPaintContext* ctx, const f32 aspect, const RvkMesh* mesh) {
  const RvkRepository*  repo      = rend_builder_repository(ctx->builder);
  const RvkRepositoryId graphicId = RvkRepositoryId_DebugMeshViewerGraphic;
  const RvkGraphic*     graphic   = rvk_repository_graphic_get(repo, graphicId);
  if (graphic) {
    const GeoVector meshCenter = geo_box_center(&mesh->bounds);
    const f32       meshSize   = math_max(1.0f, geo_box_size(&mesh->bounds).y);

    const f32 timeSeconds = rend_settings_real_time_seconds(ctx->setGlobal);

    const GeoVector pos       = geo_vector(0, -meshCenter.y + meshSize * 0.15f);
    const f32       orthoSize = meshSize * 1.75f;
    const f32       rotY      = timeSeconds * math_deg_to_rad * 10.0f;
    const f32       rotX      = -10.0f * math_deg_to_rad;
    const GeoMatrix projMat   = geo_matrix_proj_ortho_hor(orthoSize, aspect, -100.0f, 100.0f);
    const GeoMatrix rotYMat   = geo_matrix_rotate_y(rotY);
    const GeoMatrix rotXMat   = geo_matrix_rotate_x(rotX);
    const GeoMatrix rotMat    = geo_matrix_mul(&rotXMat, &rotYMat);
    const GeoMatrix posMat    = geo_matrix_translate(pos);
    const GeoMatrix viewMat   = geo_matrix_mul(&posMat, &rotMat);

    struct {
      ALIGNAS(16)
      GeoMatrix viewProj;
    } data = {.viewProj = geo_matrix_mul(&projMat, &viewMat)};

    rend_builder_draw_push(ctx->builder, graphic);
    mem_cpy(rend_builder_draw_data(ctx->builder, sizeof(data)), mem_var(data));
    rend_builder_draw_mesh(ctx->builder, mesh);
    rend_builder_draw_instances(ctx->builder, 0 /* dataStride */, 1 /* count */);
    rend_builder_draw_flush(ctx->builder);
  }
}

static void painter_push_debug_resource_viewer(
    EcsWorld*         world,
    RendPaintContext* ctx,
    const f32         aspect,
    EcsView*          resView,
    const EcsEntityId resEntity) {

  rend_res_request(world, resEntity);

  EcsIterator* itr = ecs_view_maybe_at(resView, resEntity);
  if (itr) {
    const RendResTextureComp* textureComp = ecs_view_read_t(itr, RendResTextureComp);
    if (textureComp) {
      const f32 exposure = 1.0f;
      diag_assert(textureComp->texture->image.frozen);
      // NOTE: The following cast is questionable but safe as frozen images are fully immutable.
      painter_push_debug_image_viewer(ctx, (RvkImage*)&textureComp->texture->image, exposure);
    }
    const RendResMeshComp* meshComp = ecs_view_read_t(itr, RendResMeshComp);
    if (meshComp) {
      painter_push_debug_mesh_viewer(ctx, aspect, meshComp->mesh);
    }
  }
}

static bool rend_canvas_paint_2d(
    EcsWorld*                     world,
    RendPainterComp*              painter,
    RendPlatformComp*             platform,
    const RendSettingsComp*       set,
    const RendSettingsGlobalComp* setGlobal,
    const GapWindowComp*          win,
    const EcsEntityId             camEntity,
    EcsView*                      objView,
    EcsView*                      resView,
    const u16                     presentFrequency) {

  const RvkSize winSize   = painter_win_size(win);
  const f32     winAspect = winSize.height ? ((f32)winSize.width / (f32)winSize.height) : 1.0f;

  RendBuilder* b = rend_builder(platform->builderContainer);
  if (!rend_builder_canvas_push(b, painter->canvas, set, setGlobal->frameIdx, winSize)) {
    return false; // Canvas not ready for rendering.
  }

  rend_builder_phase_output(b); // Acquire swapchain image.

  RvkImage* swapchainImage = rend_builder_img_swapchain(b);
  if (swapchainImage) {
    rend_builder_img_clear_color(b, swapchainImage, geo_color_black);

    rend_builder_pass_push(b, platform->passes[AssetGraphicPass_Post]);
    {
      const RendView   mainView = painter_view_2d_create(camEntity);
      RendPaintContext ctx      = painter_context(b, set, setGlobal, mainView);
      rend_builder_attach_color(b, swapchainImage, 0);
      painter_set_global_data(&ctx, null, null, winSize, setGlobal, RendViewType_Main);
      painter_push_objects_simple(&ctx, objView, resView, AssetGraphicPass_Post);
      if (set->debugViewerResource) {
        painter_push_debug_resource_viewer(
            world, &ctx, winAspect, resView, set->debugViewerResource);
      }
    }
    rend_builder_pass_flush(b);
  }

  rend_builder_canvas_flush(b, presentFrequency);
  return true;
}

static bool rend_canvas_paint_3d(
    EcsWorld*                     world,
    RendPainterComp*              painter,
    RendPlatformComp*             platform,
    const RendSettingsComp*       set,
    const RendSettingsGlobalComp* setGlobal,
    const RendLightRendererComp*  light,
    const GapWindowComp*          win,
    const EcsEntityId             camEntity,
    const RendCameraComp*         cam,
    EcsView*                      objView,
    EcsView*                      resView,
    const u16                     presentFrequency) {

  const RvkSize winSize   = painter_win_size(win);
  const f32     winAspect = winSize.height ? ((f32)winSize.width / (f32)winSize.height) : 1.0f;

  RendBuilder* b = rend_builder(platform->builderContainer);
  if (!rend_builder_canvas_push(b, painter->canvas, set, setGlobal->frameIdx, winSize)) {
    return false; // Canvas not ready for rendering.
  }
  const GeoMatrix camMat  = rend_cam_transform_matrix(cam);
  const GeoMatrix projMat = rend_camera_proj(cam, winAspect);
  const RendView  mainView =
      painter_view_3d_create(&camMat, &projMat, camEntity, (RendTagFilter){0});

  // Geometry pass.
  const RvkSize geoSize      = rvk_size_scale(winSize, set->resolutionScale);
  RvkPass*      geoPass      = platform->passes[AssetGraphicPass_Geometry];
  RvkImage*     geoBase      = rend_builder_attach_acquire_color(b, geoPass, 0, geoSize);
  RvkImage*     geoNormal    = rend_builder_attach_acquire_color(b, geoPass, 1, geoSize);
  RvkImage*     geoAttribute = rend_builder_attach_acquire_color(b, geoPass, 2, geoSize);
  RvkImage*     geoEmissive  = rend_builder_attach_acquire_color(b, geoPass, 3, geoSize);
  RvkImage*     geoDepth     = rend_builder_attach_acquire_depth(b, geoPass, geoSize);
  RendTags      geoTagMask;
  {
    rend_builder_pass_push(b, geoPass);

    RendPaintContext ctx = painter_context(b, set, setGlobal, mainView);
    rend_builder_attach_color(b, geoBase, 0);
    rend_builder_attach_color(b, geoNormal, 1);
    rend_builder_attach_color(b, geoAttribute, 2);
    rend_builder_attach_color(b, geoEmissive, 3);
    rend_builder_attach_depth(b, geoDepth);
    painter_set_global_data(&ctx, &camMat, &projMat, geoSize, setGlobal, RendViewType_Main);
    geoTagMask = painter_push_objects_simple(&ctx, objView, resView, AssetGraphicPass_Geometry);

    rend_builder_pass_flush(b);
  }

  // Shadow pass.
  const bool    shadActive = (set->flags & RendFlags_Shadows) && rend_light_has_shadow(light);
  const RvkSize shadSize   = shadActive ? rvk_size_square(set->shadowResolution) : rvk_size_one;
  RvkPass*      shadPass   = platform->passes[AssetGraphicPass_Shadow];
  RvkImage*     shadDepth  = rend_builder_attach_acquire_depth(b, shadPass, shadSize);
  if (shadActive) {
    rend_builder_pass_push(b, shadPass);

    const GeoMatrix*    shadTrans  = rend_light_shadow_trans(light);
    const GeoMatrix*    shadProj   = rend_light_shadow_proj(light);
    const RendTagFilter shadFilter = {.required = RendTags_ShadowCaster, .illegal = 0};
    const RendView   shadView = painter_view_3d_create(shadTrans, shadProj, camEntity, shadFilter);
    RendPaintContext ctx      = painter_context(b, set, setGlobal, shadView);
    rend_builder_attach_depth(b, shadDepth);
    painter_set_global_data(&ctx, shadTrans, shadProj, shadSize, setGlobal, RendViewType_Shadow);
    painter_push_shadow(&ctx, objView, resView);

    rend_builder_pass_flush(b);
  } else {
    rend_builder_img_clear_depth(b, shadDepth, 0);
  }

  // Ambient occlusion.
  const bool    aoActive = (set->flags & RendFlags_AmbientOcclusion) != 0;
  const RvkSize aoSize = aoActive ? rvk_size_scale(geoSize, set->aoResolutionScale) : rvk_size_one;
  RvkPass*      aoPass = platform->passes[AssetGraphicPass_AmbientOcclusion];
  RvkImage*     aoBuffer = rend_builder_attach_acquire_color(b, aoPass, 0, aoSize);
  if (aoActive) {
    rend_builder_pass_push(b, aoPass);

    RendPaintContext ctx = painter_context(b, set, setGlobal, mainView);
    rend_builder_global_image(b, geoNormal, 0);
    rend_builder_global_image(b, geoDepth, 1);
    rend_builder_attach_color(b, aoBuffer, 0);
    painter_set_global_data(&ctx, &camMat, &projMat, aoSize, setGlobal, RendViewType_Main);
    painter_push_ambient_occlusion(&ctx);

    rend_builder_pass_flush(b);
  } else {
    rend_builder_img_clear_color(b, aoBuffer, geo_color_white);
  }

  // Forward pass.
  RvkPass*  fwdPass  = platform->passes[AssetGraphicPass_Forward];
  RvkImage* fwdColor = rend_builder_attach_acquire_color(b, fwdPass, 0, geoSize);
  {
    rend_builder_pass_push(b, fwdPass);

    // Copy the geometry depth to both bind it as an attachment as well as a global image.
    // TODO: This copy can potentially be avoided by support a read-only depth attachment.
    RvkImage* geoDepthRead = rend_builder_attach_acquire_copy(b, geoDepth);

    if (set->skyMode == RendSkyMode_None) {
      rend_builder_img_clear_color(b, fwdColor, geo_color_black);
    }
    RendPaintContext ctx = painter_context(b, set, setGlobal, mainView);
    if (ctx.set->ambientMode >= RendAmbientMode_DebugStart) {
      // Disable lighting when using any of the debug ambient modes.
      ctx.view.filter.illegal |= RendTags_Light;
    }
    rend_builder_global_image(b, geoBase, 0);
    rend_builder_global_image(b, geoNormal, 1);
    rend_builder_global_image(b, geoAttribute, 2);
    rend_builder_global_image(b, geoEmissive, 3);
    rend_builder_global_image(b, geoDepthRead, 4);
    rend_builder_global_image(b, aoBuffer, 5);
    rend_builder_global_shadow(b, shadDepth, 6);
    rend_builder_attach_color(b, fwdColor, 0);
    rend_builder_attach_depth(b, geoDepth);
    painter_set_global_data(&ctx, &camMat, &projMat, geoSize, setGlobal, RendViewType_Main);
    painter_push_ambient(&ctx, rend_light_ambient_radiance(light));
    switch ((u32)set->skyMode) {
    case RendSkyMode_Gradient:
      painter_push_simple(&ctx, RvkRepositoryId_SkyGradientGraphic, mem_empty);
      break;
    case RendSkyMode_CubeMap:
      painter_push_simple(&ctx, RvkRepositoryId_SkyCubeMapGraphic, mem_empty);
      break;
    }
    if (geoTagMask & RendTags_Outline) {
      painter_push_simple(&ctx, RvkRepositoryId_OutlineGraphic, mem_empty);
    }
    painter_push_objects_simple(&ctx, objView, resView, AssetGraphicPass_Forward);

    rend_builder_pass_flush(b);
    rend_builder_attach_release(b, geoDepthRead);
  }

  rend_builder_attach_release(b, geoBase);
  rend_builder_attach_release(b, geoNormal);
  rend_builder_attach_release(b, geoAttribute);
  rend_builder_attach_release(b, geoEmissive);
  rend_builder_attach_release(b, geoDepth);
  rend_builder_attach_release(b, aoBuffer);

  rend_builder_phase_output(b); // Acquire swapchain image.

  // Post pass.
  RvkImage* swapchainImage = rend_builder_img_swapchain(b);
  if (swapchainImage) {
    rend_builder_pass_push(b, platform->passes[AssetGraphicPass_Post]);

    RendPaintContext ctx = painter_context(b, set, setGlobal, mainView);
    rend_builder_global_image(b, fwdColor, 0);
    rend_builder_attach_color(b, swapchainImage, 0);
    painter_set_global_data(&ctx, &camMat, &projMat, winSize, setGlobal, RendViewType_Main);
    painter_push_tonemapping(&ctx);
    painter_push_objects_simple(&ctx, objView, resView, AssetGraphicPass_Post);
    if (set->flags & RendFlags_DebugShadow) {
      const f32 exposure = 0.5f;
      painter_push_debug_image_viewer(&ctx, shadDepth, exposure);
    } else if (set->debugViewerResource) {
      painter_push_debug_resource_viewer(world, &ctx, winAspect, resView, set->debugViewerResource);
    }
    rend_builder_pass_flush(b);
  }

  rend_builder_attach_release(b, fwdColor);
  rend_builder_attach_release(b, shadDepth);

  rend_builder_canvas_flush(b, presentFrequency);
  return true;
}

ecs_system_define(RendPainterCreateSys) {
  EcsView*     globalView = ecs_world_view_t(world, GlobalView);
  EcsIterator* globalItr  = ecs_view_maybe_at(globalView, ecs_world_global(world));
  if (!globalItr) {
    return;
  }
  RendPlatformComp* plat = ecs_view_write_t(globalItr, RendPlatformComp);

  EcsView* painterView = ecs_world_view_t(world, PainterCreateView);
  for (EcsIterator* itr = ecs_view_itr(painterView); ecs_view_walk(itr);) {
    const EcsEntityId    entity = ecs_view_entity(itr);
    const GapWindowComp* win    = ecs_view_read_t(itr, GapWindowComp);
    if (gap_window_events(win) & GapWindowEvents_Initializing) {
      continue;
    }
    ecs_world_add_t(
        world, entity, RendPainterComp, .canvas = rvk_canvas_create(plat->lib, plat->device, win));

    if (!ecs_world_has_t(world, entity, RendSettingsComp)) {
      RendSettingsComp* settings = ecs_world_add_t(world, entity, RendSettingsComp);
      rend_settings_to_default(settings);
    }
  }
}

ecs_system_define(RendPainterDrawSys) {
  EcsView*     globalView = ecs_world_view_t(world, GlobalView);
  EcsIterator* globalItr  = ecs_view_maybe_at(globalView, ecs_world_global(world));
  if (!globalItr) {
    return;
  }
  RendPlatformComp*             platform  = ecs_view_write_t(globalItr, RendPlatformComp);
  const RendSettingsGlobalComp* setGlobal = ecs_view_read_t(globalItr, RendSettingsGlobalComp);
  const RendLightRendererComp*  light     = ecs_view_read_t(globalItr, RendLightRendererComp);

  EcsView* painterView = ecs_world_view_t(world, PainterUpdateView);
  EcsView* objView     = ecs_world_view_t(world, ObjView);
  EcsView* resView     = ecs_world_view_t(world, ResourceView);

  const u16 presentFrequency = setGlobal ? setGlobal->limiterFreq : 0;

  for (EcsIterator* itr = ecs_view_itr(painterView); ecs_view_walk(itr);) {
    const EcsEntityId       entity  = ecs_view_entity(itr);
    const GapWindowComp*    win     = ecs_view_read_t(itr, GapWindowComp);
    RendPainterComp*        painter = ecs_view_write_t(itr, RendPainterComp);
    const RendSettingsComp* set     = ecs_view_read_t(itr, RendSettingsComp);
    const RendCameraComp*   cam     = ecs_view_read_t(itr, RendCameraComp);

    const RvkRepository* repo = rvk_canvas_repository(painter->canvas);
    if (cam && rvk_repository_all_set(repo) && !(set->flags & RendFlags_2D)) {
      rend_canvas_paint_3d(
          world,
          painter,
          platform,
          set,
          setGlobal,
          light,
          win,
          entity,
          cam,
          objView,
          resView,
          presentFrequency);
    } else {
      rend_canvas_paint_2d(
          world,
          painter,
          platform,
          set,
          setGlobal,
          win,
          entity,
          objView,
          resView,
          presentFrequency);
    }
  }
}

ecs_module_init(rend_painter_module) {
  ecs_register_comp(RendPainterComp, .destructor = ecs_destruct_painter);

  ecs_register_view(GlobalView);
  ecs_register_view(ObjView);
  ecs_register_view(ResourceView);
  ecs_register_view(PainterCreateView);
  ecs_register_view(PainterUpdateView);

  ecs_register_system(
      RendPainterCreateSys, ecs_view_id(GlobalView), ecs_view_id(PainterCreateView));

  ecs_register_system(
      RendPainterDrawSys,
      ecs_view_id(GlobalView),
      ecs_view_id(PainterUpdateView),
      ecs_view_id(ObjView),
      ecs_view_id(ResourceView));

  ecs_order(RendPainterDrawSys, RendOrder_Draw);
}

void rend_painter_teardown(EcsWorld* world, const EcsEntityId entity) {
  ecs_world_remove_t(world, entity, RendPainterComp);
}

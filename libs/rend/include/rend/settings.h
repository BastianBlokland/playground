#pragma once
#include "ecs/module.h"
#include "geo/forward.h"

#define rend_ao_kernel_size 16

typedef enum eRendFlags {
  RendFlags_FrustumCulling       = 1 << 0,
  RendFlags_AmbientOcclusion     = 1 << 1,
  RendFlags_AmbientOcclusionBlur = 1 << 2,
  RendFlags_Shadows              = 1 << 3,
  RendFlags_2D                   = 1 << 4, // Disable 3d rendering.
  RendFlags_DebugCamera          = 1 << 5,
  RendFlags_DebugShadow          = 1 << 6,

  RendFlags_DebugOverlay = RendFlags_DebugShadow,
} RendFlags;

typedef enum eRendSyncMode {
  RendSyncMode_Immediate,
  RendSyncMode_VSync,
} RendSyncMode;

typedef enum eRendAmbientMode {
  RendAmbientMode_Solid,
  RendAmbientMode_DiffuseIrradiance,
  RendAmbientMode_SpecularIrradiance,

  // Debug modes.
  RendAmbientMode_DebugStart,
  RendAmbientMode_DebugColor = RendAmbientMode_DebugStart,
  RendAmbientMode_DebugRoughness,
  RendAmbientMode_DebugMetalness,
  RendAmbientMode_DebugEmissive,
  RendAmbientMode_DebugNormal,
  RendAmbientMode_DebugDepth,
  RendAmbientMode_DebugTags,
  RendAmbientMode_DebugAmbientOcclusion,
  RendAmbientMode_DebugFresnel,
  RendAmbientMode_DebugDiffuseIrradiance,
  RendAmbientMode_DebugSpecularIrradiance,
} RendAmbientMode;

typedef enum eRendSkyMode {
  RendSkyMode_None,
  RendSkyMode_Gradient,
  RendSkyMode_CubeMap,
} RendSkyMode;

typedef enum eRendTonemapper {
  RendTonemapper_Linear,
  RendTonemapper_LinearSmooth,
  RendTonemapper_Reinhard,
  RendTonemapper_ReinhardJodie,
  RendTonemapper_Aces,
} RendTonemapper;

typedef enum {
  RendDebugViewer_Interpolate = 1 << 0, // Enable linear interpolation for textures in the viewer.
  RendDebugViewer_AlphaIgnore = 1 << 1, // Ignore the alpha when viewing textures in the viewer.
  RendDebugViewer_AlphaOnly   = 1 << 2, // Show only alpha when viewing textures in the viewer.
} RendDebugViewerFlags;

ecs_comp_extern_public(RendSettingsComp) {
  RendFlags            flags;
  RendSyncMode         syncMode;
  RendAmbientMode      ambientMode;
  RendSkyMode          skyMode;
  f32                  exposure;
  RendTonemapper       tonemapper;
  f32                  resolutionScale;
  u16                  shadowResolution;
  f32                  aoAngle, aoRadius, aoRadiusPower, aoPower, aoResolutionScale;
  GeoVector*           aoKernel; // GeoVector[rend_ao_kernel_size];
  f32                  grayscaleFrac;
  EcsEntityId          debugViewerResource; // Resource entity to visualize for debug purposes.
  f32                  debugViewerLod;      // Level-of-detail to use for the debug-viewer.
  f32                  debugViewerLayer;    // Layer to show in the debug-viewer.
  RendDebugViewerFlags debugViewerFlags;    // Flags to use for the debug-viewer.
};

typedef enum eRendGlobalFlags {
  RendGlobalFlags_Validation       = 1 << 0,
  RendGlobalFlags_Profiling        = 1 << 1,
  RendGlobalFlags_Verbose          = 1 << 2,
  RendGlobalFlags_DebugGpu         = 1 << 3,
  RendGlobalFlags_DebugLight       = 1 << 4,
  RendGlobalFlags_DebugLightFreeze = 1 << 5
} RendGlobalFlags;

ecs_comp_extern_public(RendSettingsGlobalComp) {
  u64          frameIdx;
  TimeDuration time, timeReal;

  RendGlobalFlags flags;
  u16             limiterFreq;
  f32             shadowFilterSize; // In world space.
};

RendSettingsGlobalComp* rend_settings_global_init(EcsWorld*, bool devSupport);
RendSettingsComp*       rend_settings_window_init(EcsWorld*, EcsEntityId window);

void rend_settings_to_default(RendSettingsComp*);
void rend_settings_global_to_default(RendSettingsGlobalComp*, bool devSupport);

void rend_settings_generate_ao_kernel(RendSettingsComp*);

f32 rend_settings_time_seconds(const RendSettingsGlobalComp*);
f32 rend_settings_real_time_seconds(const RendSettingsGlobalComp*);

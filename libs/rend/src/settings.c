#include "core/alloc.h"
#include "core/math.h"
#include "core/rng.h"
#include "core/time.h"
#include "ecs/world.h"
#include "geo/vector.h"
#include "rend/settings.h"

#define VOLO_REND_VALIDATION 0

ecs_comp_define(RendSettingsComp);
ecs_comp_define(RendSettingsGlobalComp);

static void ecs_destruct_rend_settings_comp(void* data) {
  RendSettingsComp* comp = data;
  alloc_free_array_t(g_allocHeap, comp->aoKernel, rend_ao_kernel_size);
}

static f32 rend_time_to_seconds(const TimeDuration dur) {
  static const f64 g_toSecMul = 1.0 / (f64)time_second;
  // NOTE: Potentially can be done in 32 bit but with nano-seconds its at the edge of f32 precision.
  return (f32)((f64)dur * g_toSecMul);
}

ecs_module_init(rend_settings_module) {
  ecs_register_comp(RendSettingsComp, .destructor = ecs_destruct_rend_settings_comp);
  ecs_register_comp(RendSettingsGlobalComp);
}

RendSettingsGlobalComp* rend_settings_global_init(EcsWorld* world, const bool devSupport) {
  const EcsEntityId       global   = ecs_world_global(world);
  RendSettingsGlobalComp* settings = ecs_world_add_t(world, global, RendSettingsGlobalComp);
  rend_settings_global_to_default(settings, devSupport);
  return settings;
}

RendSettingsComp* rend_settings_window_init(EcsWorld* world, const EcsEntityId window) {
  RendSettingsComp* settings = ecs_world_add_t(world, window, RendSettingsComp);
  rend_settings_to_default(settings);
  return settings;
}

void rend_settings_to_default(RendSettingsComp* s) {
  // clang-format off
  s->flags = RendFlags_FrustumCulling       |
             RendFlags_AmbientOcclusion     |
             RendFlags_AmbientOcclusionBlur;

  // clang-format on
  s->syncMode            = RendSyncMode_VSync;
  s->ambientMode         = RendAmbientMode_SpecularIrradiance;
  s->skyMode             = RendSkyMode_Gradient;
  s->exposure            = 1.0f;
  s->tonemapper          = RendTonemapper_LinearSmooth;
  s->resolutionScale     = 1.0f;
  s->aoAngle             = 80 * math_deg_to_rad;
  s->aoRadius            = 0.5f;
  s->aoRadiusPower       = 2.5f;
  s->aoPower             = 1.25f;
  s->aoResolutionScale   = 0.75f;
  s->shadowResolution    = 2048;
  s->grayscaleFrac       = 0.0f;
  s->debugViewerResource = 0;
  s->debugViewerLod      = 0.0f;
  s->debugViewerFlags    = 0;

  rend_settings_generate_ao_kernel(s);
}

void rend_settings_global_to_default(RendSettingsGlobalComp* s, const bool devSupport) {
  s->flags       = 0;
  s->limiterFreq = 0;

  if (devSupport) {
    s->flags |= RendGlobalFlags_DebugGpu;
    s->flags |= RendGlobalFlags_Profiling;
#if VOLO_REND_VALIDATION
    s->flags |= RendGlobalFlags_Validation;
#endif
  }

  s->shadowFilterSize = 0.025f;
}

void rend_settings_generate_ao_kernel(RendSettingsComp* s) {
  if (!s->aoKernel) {
    s->aoKernel = alloc_array_t(g_allocHeap, GeoVector, rend_ao_kernel_size);
  }
  Rng* rng = rng_create_xorwow(g_allocScratch, 42);
  for (u32 i = 0; i != rend_ao_kernel_size; ++i) {
    const GeoVector randInCone = geo_vector_rand_in_cone3(rng, s->aoAngle);
    const f32       rand       = rng_sample_f32(rng);
    const f32       mag = math_lerp(0.1f, 1.0f, math_pow_f32(rand, s->aoRadiusPower)) * s->aoRadius;
    s->aoKernel[i]      = geo_vector_mul(randInCone, mag);
  }
}

f32 rend_settings_time_seconds(const RendSettingsGlobalComp* setGlobal) {
  return rend_time_to_seconds(setGlobal->time);
}

f32 rend_settings_real_time_seconds(const RendSettingsGlobalComp* setGlobal) {
  return rend_time_to_seconds(setGlobal->timeReal);
}

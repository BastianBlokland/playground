#include "core/diag.h"
#include "core/math.h"
#include "geo/matrix.h"
#include "rend/camera.h"

static const f32 g_camOrthoNear = -100.0f;
static const f32 g_camOrthoFar  = +100.0f;

static GeoVector cam_world_from_ndc(const GeoMatrix* invViewProj, const GeoVector pos) {
  const GeoVector ndc = geo_vector(pos.x, pos.y, pos.z, 1);
  return geo_vector_perspective_div(geo_matrix_transform(invViewProj, ndc));
}

static GeoVector cam_world_from_screen_near(const GeoMatrix* invViewProj, const GeoVector normPos) {
  const f32 ndcX    = normPos.x * 2 - 1;
  const f32 ndcY    = -normPos.y * 2 + 1;
  const f32 ndcNear = 1.0f;
  return cam_world_from_ndc(invViewProj, geo_vector(ndcX, ndcY, ndcNear));
}

static GeoVector cam_world_from_screen_far(const GeoMatrix* invViewProj, const GeoVector normPos) {
  const f32 ndcX   = normPos.x * 2 - 1;
  const f32 ndcY   = -normPos.y * 2 + 1;
  const f32 ndcFar = 1e-4f; // NOTE: Using an infinitely far depth plane so avoid 0.
  return cam_world_from_ndc(invViewProj, geo_vector(ndcX, ndcY, ndcFar));
}

ecs_comp_define(RendCameraComp);

ecs_module_init(rend_camera_module) { ecs_register_comp(RendCameraComp); }

GeoMatrix rend_cam_transform_matrix(const RendCameraComp* cam) {
  const GeoMatrix pos = geo_matrix_translate(cam->position);
  const GeoMatrix rot = geo_matrix_from_quat(cam->rotation);
  return geo_matrix_mul(&pos, &rot);
}

GeoMatrix rend_cam_transform_matrix_inv(const RendCameraComp* cam) {
  const GeoMatrix rot = geo_matrix_from_quat(geo_quat_inverse(cam->rotation));
  const GeoMatrix pos = geo_matrix_translate(geo_vector_mul(cam->position, -1));
  return geo_matrix_mul(&rot, &pos);
}

f32 rend_camera_near(const RendCameraComp* cam) {
  return (cam->flags & RendCameraFlags_Orthographic) ? g_camOrthoNear : cam->persNear;
}

f32 rend_camera_far(const RendCameraComp* cam) {
  // NOTE: For perspective projections the far plane is infinitely far away so we return an
  // arbitrarily large number.
  static const f32 g_persFar = 1e8;
  return (cam->flags & RendCameraFlags_Orthographic) ? g_camOrthoFar : g_persFar;
}

GeoMatrix rend_camera_proj(const RendCameraComp* cam, const f32 aspect) {
  const bool useVerticalFov = aspect < 1.0f;
  if (cam->flags & RendCameraFlags_Orthographic) {
    if (useVerticalFov) {
      return geo_matrix_proj_ortho_ver(cam->orthoSize, aspect, g_camOrthoNear, g_camOrthoFar);
    }
    return geo_matrix_proj_ortho_hor(cam->orthoSize, aspect, g_camOrthoNear, g_camOrthoFar);
  }
  if (useVerticalFov) {
    return geo_matrix_proj_pers_ver(cam->persFov, aspect, cam->persNear);
  }
  return geo_matrix_proj_pers_hor(cam->persFov, aspect, cam->persNear);
}

GeoMatrix rend_camera_view_proj(const RendCameraComp* cam, const f32 aspect) {
  const GeoMatrix p = rend_camera_proj(cam, aspect);
  const GeoMatrix v = rend_cam_transform_matrix_inv(cam);
  return geo_matrix_mul(&p, &v);
}

void rend_camera_frustum4(const RendCameraComp* cam, const f32 aspect, GeoPlane out[4]) {
  const GeoMatrix viewProj = rend_camera_view_proj(cam, aspect);
  geo_matrix_frustum4(&viewProj, out);
}

void rend_camera_frustum_corners(
    const RendCameraComp* cam,
    const f32             aspect,
    const GeoVector       rectMin,
    const GeoVector       rectMax,
    GeoVector             out[8]) {
  diag_assert(rectMin.x < rectMax.x && rectMin.y < rectMax.y);

  const GeoMatrix viewProj    = rend_camera_view_proj(cam, aspect);
  const GeoMatrix invViewProj = geo_matrix_inverse(&viewProj);

  out[0] = cam_world_from_screen_near(&invViewProj, geo_vector(rectMin.x, rectMin.y));
  out[1] = cam_world_from_screen_near(&invViewProj, geo_vector(rectMin.x, rectMax.y));
  out[2] = cam_world_from_screen_near(&invViewProj, geo_vector(rectMax.x, rectMax.y));
  out[3] = cam_world_from_screen_near(&invViewProj, geo_vector(rectMax.x, rectMin.y));
  out[4] = cam_world_from_screen_far(&invViewProj, geo_vector(rectMin.x, rectMin.y));
  out[5] = cam_world_from_screen_far(&invViewProj, geo_vector(rectMin.x, rectMax.y));
  out[6] = cam_world_from_screen_far(&invViewProj, geo_vector(rectMax.x, rectMax.y));
  out[7] = cam_world_from_screen_far(&invViewProj, geo_vector(rectMax.x, rectMin.y));
}

GeoRay rend_camera_ray(const RendCameraComp* cam, const f32 aspect, const GeoVector normScreenPos) {
  const GeoMatrix viewProj    = rend_camera_view_proj(cam, aspect);
  const GeoMatrix invViewProj = geo_matrix_inverse(&viewProj);

  const GeoVector posNear = cam_world_from_screen_near(&invViewProj, normScreenPos);
  const GeoVector posFar  = cam_world_from_screen_far(&invViewProj, normScreenPos);
  return (GeoRay){.point = posNear, .dir = geo_vector_norm(geo_vector_sub(posFar, posNear))};
}

void rend_camera_to_default(RendCameraComp* cam) {
  cam->persFov   = 60.0f * math_deg_to_rad;
  cam->orthoSize = 5.0f;
  cam->persNear  = 0.1f;
}

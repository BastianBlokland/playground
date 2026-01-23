#pragma once
#include "ecs/module.h"
#include "geo/matrix.h"
#include "geo/quat.h"
#include "geo/vector.h"

typedef enum {
  RendCameraFlags_None         = 0,
  RendCameraFlags_Orthographic = 1 << 1,
} RendCameraFlags;

ecs_comp_extern_public(RendCameraComp) {
  GeoVector       position;
  GeoQuat         rotation;
  f32             persFov;
  f32             persNear;
  f32             orthoSize;
  RendCameraFlags flags;
};

GeoMatrix rend_cam_transform_matrix(const RendCameraComp*);
GeoMatrix rend_cam_transform_matrix_inv(const RendCameraComp*);

/**
 * Retrieve the camera's near and far plane distances.
 */
f32 rend_camera_near(const RendCameraComp*);
f32 rend_camera_far(const RendCameraComp*);

/**
 * Compute the projection matrix at the given aspect.
 */
GeoMatrix rend_camera_proj(const RendCameraComp*, f32 aspect);

/**
 * Compute the view-projection matrix at the given aspect.
 */
GeoMatrix rend_camera_view_proj(const RendCameraComp*, f32 aspect);

/**
 * Compute 4 frustum planes.
 * NOTE: Plane normals point towards the inside of the frustum.
 *
 * [0] = Left plane.
 * [1] = Right plane.
 * [2] = Top plane.
 * [3] = Bottom plane.
 */
void rend_camera_frustum4(const RendCameraComp*, f32 aspect, GeoPlane out[4]);

/**
 * Compute the world-space corner points of a rectangle inside the camera view.
 * NOTE: Rect coordinates are in normalized screen positions (x: 0 - 1, y: 0 - 1).
 *
 * Pre-condition: Given rectangle is not inverted.
 * Pre-condition: Given rectangle is not infinitely small.
 */
void rend_camera_frustum_corners(
    const RendCameraComp*, f32 aspect, GeoVector rectMin, GeoVector rectMax, GeoVector out[8]);

/**
 * Compute a world-space ray through the given normalized screen position (x: 0 - 1, y: 0 - 1).
 */
GeoRay rend_camera_ray(const RendCameraComp*, f32 aspect, GeoVector normScreenPos);

void rend_camera_to_default(RendCameraComp*);

#ifndef INCLUDE_STANDARD
#define INCLUDE_STANDARD

#include "color.glsl"
#include "object.glsl"
#include "types.glsl"

const u32 c_maxJoints = 75; // Needs to match the maximum in renderable.c

struct StandardData {
  f32v4 posAndScale; // x, y, z position, w scale
  f32v4 rot;         // x, y, z, w rotation quaternion
  f32v4 data;        // x tag bits, y color, z emissive
};

struct StandardSkinnedData {
  f32v4 posAndScale; // x, y, z position, w scale
  f32v4 rot;         // x, y, z, w rotation quaternion
  f32v4 data;        // x tag bits, y color, z emissive

  // Transformation matrices relative to the bind pose.
  // NOTE: Transposed to 3x4 to save bandwidth.
  f32m3x4 jointDelta[c_maxJoints];
};

/**
 * Compute a skinning matrix that blends between 4 joints.
 */
f32m4x3 standard_skin_mat(
    const StandardSkinnedData data, const u32v4 jointIndices, const f32v4 jointWeights) {
  return jointWeights.x * transpose(data.jointDelta[jointIndices.x]) +
         jointWeights.y * transpose(data.jointDelta[jointIndices.y]) +
         jointWeights.z * transpose(data.jointDelta[jointIndices.z]) +
         jointWeights.w * transpose(data.jointDelta[jointIndices.w]);
}

/**
 * Retrieve the instance color.
 */
f32v4 standard_color(const f32v4 StandardData) {
  return color_from_u32(floatBitsToUint(StandardData.y));
}

/**
 * Retrieve the instance emissive.
 */
f32v4 standard_emissive(const f32v4 StandardData) {
  return color_from_u32(floatBitsToUint(StandardData.z));
}

#endif // INCLUDE_STANDARD

#include "binding.glsl"
#include "global.glsl"
#include "object.glsl"
#include "vertex.glsl"

struct InstanceData {
  f32v4 pos; // x, y, z: position
};

bind_global_data(0) readonly uniform Global { GlobalData u_global; };
bind_graphic_data(0) readonly buffer Mesh { VertexPacked[] u_vertices; };
bind_instance_data(0) readonly uniform Instance { InstanceData[c_objMaxInstances] u_instances; };

void main() {
  const Vertex vert = vert_unpack(u_vertices[in_vertexIndex]);

  const f32v3 instancePos = u_instances[in_instanceIndex].pos.xyz;
  const f32v3 worldPos = vert.position + instancePos;

  out_vertexPosition = u_global.viewProj * f32v4(worldPos, 1);
}

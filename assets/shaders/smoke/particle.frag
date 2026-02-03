#include "binding.glsl"

bind_internal(0) in flat f32v3 in_color;

bind_internal(0) out f32v3 out_color;

void main() {
  out_color = in_color;
}

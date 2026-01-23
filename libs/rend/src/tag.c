#include "core/array.h"
#include "core/bits.h"
#include "core/diag.h"
#include "ecs/world.h"
#include "rend/tag.h"

ecs_comp_define(RendTagComp);

static void ecs_combine_tags(void* dataA, void* dataB) {
  RendTagComp* compA = dataA;
  RendTagComp* compB = dataB;
  compA->tags |= compB->tags;
}

ecs_module_init(rend_tag_module) { ecs_register_comp(RendTagComp, .combinator = ecs_combine_tags); }

String rend_tag_name(const RendTags tags) {
  diag_assert_msg(bits_popcnt((u32)tags) == 1, "Exactly one tag should be set");
  const u32           index     = bits_ctz_32(tags);
  static const String g_names[] = {
      string_static("Selected"),
      string_static("Geometry"),
      string_static("Transparent"),
      string_static("Unlit"),
      string_static("Debug"),
      string_static("Light"),
      string_static("ShadowCaster"),
  };
  ASSERT(array_elems(g_names) == RendTags_Count, "Incorrect number of tag names");
  return g_names[index];
}

void rend_tag_add(EcsWorld* world, const EcsEntityId entity, const RendTags tags) {
  ecs_world_add_t(world, entity, RendTagComp, .tags = tags);
}

bool rend_tag_filter(const RendTagFilter filter, const RendTags tags) {
  return ((tags & filter.required) == filter.required) && ((tags & filter.illegal) == 0);
}

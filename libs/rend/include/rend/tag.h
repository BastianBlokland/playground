#pragma once
#include "ecs/module.h"

typedef enum eRendTags {
  RendTags_None         = 0,
  RendTags_Outline      = 1 << 0,
  RendTags_Geometry     = 1 << 1,
  RendTags_Transparent  = 1 << 2,
  RendTags_Unlit        = 1 << 3,
  RendTags_Debug        = 1 << 4,
  RendTags_Light        = 1 << 5,
  RendTags_ShadowCaster = 1 << 6,

  RendTags_Count   = 7,
  RendTags_Default = RendTags_Geometry | RendTags_ShadowCaster,
} RendTags;

typedef struct sRendTagFilter {
  RendTags required, illegal;
} RendTagFilter;

ecs_comp_extern_public(RendTagComp) { RendTags tags; };

/**
 * Lookup the name of the given tag.
 * Pre-condition: Only a single bit is set.
 */
String rend_tag_name(RendTags);

void rend_tag_add(EcsWorld* world, EcsEntityId, RendTags);

bool rend_tag_filter(RendTagFilter filter, RendTags);

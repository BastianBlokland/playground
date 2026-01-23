#pragma once
#include "asset/forward.h"
#include "ecs/forward.h"

/**
 * Reference to an asset.
 */
typedef struct sAssetRef {
  StringHash  id;
  EcsEntityId entity;
} AssetRef;

EcsEntityId asset_ref_resolve(EcsWorld*, AssetManagerComp*, const AssetRef*);

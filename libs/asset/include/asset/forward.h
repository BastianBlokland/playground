#pragma once
#include "core/forward.h"
#include "ecs/module.h"

/**
 * Forward header for the asset library.
 */

ecs_comp_extern(AssetAtlasComp);
ecs_comp_extern(AssetComp);
ecs_comp_extern(AssetFailedComp);
ecs_comp_extern(AssetImportEnvComp);
ecs_comp_extern(AssetLoadedComp);
ecs_comp_extern(AssetManagerComp);

typedef enum eAssetGraphicPass       AssetGraphicPass;
typedef struct sAssetGraphicOverride AssetGraphicOverride;
typedef struct sAssetRef             AssetRef;

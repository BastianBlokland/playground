#include "core/alloc.h"
#include "core/array.h"
#include "core/diag.h"

#include "repository.h"

typedef enum {
  RvkRepositoryType_None,
  RvkRepositoryType_Texture,
  RvkRepositoryType_Mesh,
  RvkRepositoryType_Graphic,
} RvkRepositoryType;

typedef struct {
  RvkRepositoryType type;
  union {
    const RvkTexture* texture;
    const RvkMesh*    mesh;
    const RvkGraphic* graphic;
  };
} RvkRepositoryEntry;

struct sRvkRepository {
  RvkRepositoryEntry entries[RvkRepositoryId_Count];
};

// clang-format off
String rvk_repository_id_str(const RvkRepositoryId id) {
  static const String g_names[] = {
      [RvkRepositoryId_AmbientDebugGraphic]          = string_static("AmbientDebugGraphic"),
      [RvkRepositoryId_AmbientGraphic]               = string_static("AmbientGraphic"),
      [RvkRepositoryId_AmbientOcclusionGraphic]      = string_static("AmbientOcclusionGraphic"),
      [RvkRepositoryId_DebugImageViewerArrayGraphic] = string_static("DebugImageViewerArrayGraphic"),
      [RvkRepositoryId_DebugImageViewerCubeGraphic]  = string_static("DebugImageViewerCubeGraphic"),
      [RvkRepositoryId_DebugImageViewerGraphic]      = string_static("DebugImageViewerGraphic"),
      [RvkRepositoryId_DebugMeshViewerGraphic]       = string_static("DebugMeshViewerGraphic"),
      [RvkRepositoryId_MissingMesh]                  = string_static("MissingMesh"),
      [RvkRepositoryId_MissingTexture]               = string_static("MissingTexture"),
      [RvkRepositoryId_MissingTextureArray]          = string_static("MissingTextureArray"),
      [RvkRepositoryId_MissingTextureCube]           = string_static("MissingTextureCube"),
      [RvkRepositoryId_OutlineGraphic]               = string_static("OutlineGraphic"),
      [RvkRepositoryId_SkyCubeMapGraphic]            = string_static("SkyCubeMapGraphic"),
      [RvkRepositoryId_SkyGradientGraphic]           = string_static("SkyGradientGraphic"),
      [RvkRepositoryId_TonemapperGraphic]            = string_static("TonemapperGraphic"),
      [RvkRepositoryId_WhiteTexture]                 = string_static("WhiteTexture"),
  };
  ASSERT(array_elems(g_names) == RvkRepositoryId_Count, "Incorrect number of names");
  return g_names[id];
}
// clang-format on

RvkRepository* rvk_repository_create(void) {
  RvkRepository* repo = alloc_alloc_t(g_allocHeap, RvkRepository);
  *repo               = (RvkRepository){0};
  return repo;
}

void rvk_repository_destroy(RvkRepository* repo) { alloc_free_t(g_allocHeap, repo); }

void rvk_repository_texture_set(RvkRepository* r, const RvkRepositoryId id, const RvkTexture* tex) {
  r->entries[id].type    = RvkRepositoryType_Texture;
  r->entries[id].texture = tex;
}

void rvk_repository_mesh_set(RvkRepository* r, const RvkRepositoryId id, const RvkMesh* mesh) {
  r->entries[id].type = RvkRepositoryType_Mesh;
  r->entries[id].mesh = mesh;
}

void rvk_repository_graphic_set(RvkRepository* r, const RvkRepositoryId id, const RvkGraphic* gra) {
  r->entries[id].type    = RvkRepositoryType_Graphic;
  r->entries[id].graphic = gra;
}

void rvk_repository_unset(RvkRepository* r, const RvkRepositoryId id) {
  r->entries[id].type = RvkRepositoryType_None;
}

bool rvk_repository_is_set(const RvkRepository* r, const RvkRepositoryId id) {
  return r->entries[id].type != RvkRepositoryType_None;
}

bool rvk_repository_all_set(const RvkRepository* r) {
  for (RvkRepositoryId id = 0; id != RvkRepositoryId_Count; ++id) {
    if (r->entries[id].type == RvkRepositoryType_None) {
      return false;
    }
  }
  return true;
}

const RvkTexture* rvk_repository_texture_get(const RvkRepository* r, const RvkRepositoryId id) {
  if (UNLIKELY(r->entries[id].type != RvkRepositoryType_Texture)) {
    return null;
  }
  return r->entries[id].texture;
}

const RvkMesh* rvk_repository_mesh_get(const RvkRepository* r, const RvkRepositoryId id) {
  if (UNLIKELY(r->entries[id].type != RvkRepositoryType_Mesh)) {
    return null;
  }
  return r->entries[id].mesh;
}

const RvkGraphic* rvk_repository_graphic_get(const RvkRepository* r, const RvkRepositoryId id) {
  if (UNLIKELY(r->entries[id].type != RvkRepositoryType_Graphic)) {
    return null;
  }
  return r->entries[id].graphic;
}

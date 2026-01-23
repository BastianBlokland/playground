#include "core/alloc.h"
#include "core/array.h"
#include "core/diag.h"
#include "core/format.h"
#include "core/sort.h"
#include "core/stringtable.h"
#include "log/logger.h"
#include "script/args.h"
#include "script/binder.h"
#include "script/enum.h"
#include "script/sig.h"
#include "script/val.h"

#include "import_mesh.h"

ScriptBinder* g_assetScriptImportMeshBinder;

static ScriptEnum g_importAnimFlags;

static void import_init_enum_anim_flags(void) {
#define ENUM_PUSH(_ENUM_, _NAME_)                                                                  \
  script_enum_push((_ENUM_), string_hash_lit(#_NAME_), AssetMeshAnimFlags_##_NAME_);

  ENUM_PUSH(&g_importAnimFlags, Active);
  ENUM_PUSH(&g_importAnimFlags, Loop);
  ENUM_PUSH(&g_importAnimFlags, FadeIn);
  ENUM_PUSH(&g_importAnimFlags, FadeOut);
  ENUM_PUSH(&g_importAnimFlags, RandomTime);

#undef ENUM_PUSH
}

static i8 import_compare_anim_layer(const void* a, const void* b) {
  return compare_i32(field_ptr(a, AssetImportAnim, layer), field_ptr(b, AssetImportAnim, layer));
}

static bool import_mesh_joint_find_duplicate(AssetImportMesh* data, StringHash* outDuplicate) {
  if (!data->jointCount) {
    return false;
  }
  for (u32 i = 0; i != data->jointCount - 1; ++i) {
    for (u32 j = i + 1; j != data->jointCount; ++j) {
      if (data->joints[i].nameHash == data->joints[j].nameHash) {
        return *outDuplicate = data->joints[i].nameHash, true; // Duplicate found.
      }
    }
  }
  return false;
}

static bool import_mesh_anim_find_duplicate(AssetImportMesh* data, StringHash* outDuplicate) {
  if (!data->animCount) {
    return false;
  }
  for (u32 i = 0; i != data->animCount - 1; ++i) {
    for (u32 j = i + 1; j != data->animCount; ++j) {
      if (data->anims[i].nameHash == data->anims[j].nameHash) {
        return *outDuplicate = data->anims[i].nameHash, true; // Duplicate found.
      }
    }
  }
  return false;
}

static ScriptVal import_eval_flat_normals(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data = ctx->data;
  if (call->argCount < 1) {
    return script_bool(data->flatNormals);
  }
  data->flatNormals = script_arg_bool(call, 0);
  return script_null();
}

static ScriptVal import_eval_joint_count(AssetImportContext* ctx, ScriptBinderCall* call) {
  (void)call;
  AssetImportMesh* data = ctx->data;
  return script_num(data->jointCount);
}

static ScriptVal import_eval_joint_parent(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data  = ctx->data;
  const u32        index = (u32)script_arg_num_clamped(call, 0, 0, data->jointCount - 1);
  diag_assert(index < data->jointCount);
  return script_num(data->joints[index].parentIndex);
}

static ScriptVal import_eval_joint_find(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data      = ctx->data;
  const StringHash jointName = script_arg_str(call, 0);
  for (u32 jointIndex = 0; jointIndex != data->jointCount; ++jointIndex) {
    if (data->joints[jointIndex].nameHash == jointName) {
      return script_num(jointIndex);
    }
  }
  return script_null();
}

static ScriptVal import_eval_joint_name(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data  = ctx->data;
  const u32        index = (u32)script_arg_num_clamped(call, 0, 0, data->jointCount - 1);
  diag_assert(index < data->jointCount);
  if (call->argCount < 2) {
    return script_str(data->joints[index].nameHash);
  }
  data->joints[index].nameHash = script_arg_str(call, 1);
  return script_null();
}

static ScriptVal import_eval_joint_name_match(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data  = ctx->data;
  const u32        index = (u32)script_arg_num_clamped(call, 0, 0, data->jointCount - 1);
  diag_assert(index < data->jointCount);

  const String name = stringtable_lookup(g_stringtable, data->joints[index].nameHash);

  const StringHash patternHash = script_arg_str(call, 1);
  const String     patternStr  = stringtable_lookup(g_stringtable, patternHash);
  return script_bool(string_match_glob(name, patternStr, StringMatchFlags_IgnoreCase));
}

static ScriptVal import_eval_joint_name_trim(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data       = ctx->data;
  const u32        index      = (u32)script_arg_num_clamped(call, 0, 0, data->jointCount - 1);
  const StringHash prefixHash = script_arg_str(call, 1);
  const StringHash suffixHash = script_arg_opt_str(call, 2, 0);
  if (!data->joints[index].nameHash) {
    return script_str_empty();
  }
  String name = stringtable_lookup(g_stringtable, data->joints[index].nameHash);

  const String prefix = stringtable_lookup(g_stringtable, prefixHash);
  if (string_starts_with(name, prefix)) {
    name = string_slice(name, prefix.size, name.size - prefix.size);
  }

  const String suffix = suffixHash ? stringtable_lookup(g_stringtable, suffixHash) : string_empty;
  if (string_ends_with(name, suffix)) {
    name = string_slice(name, 0, name.size - suffix.size);
  }

  data->joints[index].nameHash = stringtable_add(g_stringtable, name);
  return script_str(data->joints[index].nameHash);
}

static ScriptVal import_eval_anim_count(AssetImportContext* ctx, ScriptBinderCall* call) {
  (void)call;
  AssetImportMesh* data = ctx->data;
  return script_num(data->animCount);
}

static ScriptVal import_eval_anim_find(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data     = ctx->data;
  const StringHash animName = script_arg_str(call, 0);
  for (u32 animIndex = 0; animIndex != data->animCount; ++animIndex) {
    if (data->anims[animIndex].nameHash == animName) {
      return script_num(animIndex);
    }
  }
  return script_null();
}

static ScriptVal import_eval_anim_layer(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data  = ctx->data;
  const u32        index = (u32)script_arg_num_clamped(call, 0, 0, data->animCount - 1);
  diag_assert(index < data->animCount);
  if (call->argCount < 2) {
    return script_num(data->anims[index].layer);
  }
  data->anims[index].layer = (i32)script_arg_num(call, 1);
  return script_null();
}

static ScriptVal import_eval_anim_flag(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data  = ctx->data;
  const u32        index = (u32)script_arg_num_clamped(call, 0, 0, data->animCount - 1);
  diag_assert(index < data->animCount);
  const i32 flag = script_arg_enum(call, 1, &g_importAnimFlags);
  if (call->argCount < 3) {
    return script_bool((data->anims[index].flags & flag) != 0);
  }
  const bool enabled = script_arg_bool(call, 2);
  if (enabled != !!(data->anims[index].flags & flag)) {
    data->anims[index].flags ^= flag;
  }
  return script_null();
}

static ScriptVal import_eval_anim_name(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data  = ctx->data;
  const u32        index = (u32)script_arg_num_clamped(call, 0, 0, data->animCount - 1);
  diag_assert(index < data->animCount);
  if (call->argCount < 2) {
    return script_str(data->anims[index].nameHash);
  }
  data->anims[index].nameHash = script_arg_str(call, 1);
  return script_null();
}

static ScriptVal import_eval_anim_name_match(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data  = ctx->data;
  const u32        index = (u32)script_arg_num_clamped(call, 0, 0, data->animCount - 1);
  diag_assert(index < data->animCount);

  const String name = stringtable_lookup(g_stringtable, data->anims[index].nameHash);

  const StringHash patternHash = script_arg_str(call, 1);
  const String     patternStr  = stringtable_lookup(g_stringtable, patternHash);
  return script_bool(string_match_glob(name, patternStr, StringMatchFlags_IgnoreCase));
}

static ScriptVal import_eval_anim_name_trim(AssetImportContext* ctx, ScriptBinderCall* call) {
  AssetImportMesh* data       = ctx->data;
  const u32        index      = (u32)script_arg_num_clamped(call, 0, 0, data->animCount - 1);
  const StringHash prefixHash = script_arg_str(call, 1);
  const StringHash suffixHash = script_arg_opt_str(call, 2, 0);
  if (!data->anims[index].nameHash) {
    return script_str_empty();
  }
  String name = stringtable_lookup(g_stringtable, data->anims[index].nameHash);

  const String prefix = stringtable_lookup(g_stringtable, prefixHash);
  if (string_starts_with(name, prefix)) {
    name = string_slice(name, prefix.size, name.size - prefix.size);
  }

  const String suffix = suffixHash ? stringtable_lookup(g_stringtable, suffixHash) : string_empty;
  if (string_ends_with(name, suffix)) {
    name = string_slice(name, 0, name.size - suffix.size);
  }

  data->anims[index].nameHash = stringtable_add(g_stringtable, name);
  return script_str(data->anims[index].nameHash);
}

void asset_data_init_import_mesh(const bool devSupport) {
  import_init_enum_anim_flags();

  ScriptBinderFlags flags = ScriptBinderFlags_DisallowMemoryAccess;
  if (devSupport) {
    flags |= ScriptBinderFlags_DevSupport;
  }
  ScriptBinder* binder = script_binder_create(g_allocPersist, string_lit("import-mesh"), flags);
  script_binder_filter_set(binder, string_lit("import/mesh/*.script"));

  // clang-format off
  static const String g_animFlagsDoc   = string_static("Supported flags:\n\n-`Active`\n\n-`Loop`\n\n-`FadeIn`\n\n-`FadeOut`\n\n-`RandomTime`");
  static const String g_globPatternDoc = string_static("Supported pattern syntax:\n- '?' matches any single character.\n- '*' matches any number of any characters including none.\n- '!' inverts the entire match (not per segment and cannot be disabled after enabling).");
  {
    const String       name   = string_lit("flat_normals");
    const String       doc    = string_lit("Import flat (per face) normals (ignore per-vertex normals).");
    const ScriptMask   ret    = script_mask_bool | script_mask_null;
    const ScriptSigArg args[] = {
        {string_lit("flatNormals"), script_mask_bool | script_mask_null},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_flat_normals);
  }
  {
    const String       name   = string_lit("joint_count");
    const String       doc    = string_lit("Query the amount of joints in the mesh.\nThe joints are topologically sorted so the root is always at index 0.");
    const ScriptMask   ret    = script_mask_num | script_mask_null;
    asset_import_bind(binder, name, doc, ret, null, 0, import_eval_joint_count);
  }
  {
    const String       name   = string_lit("joint_parent");
    const String       doc    = string_lit("Query the index of the joint's parent (same as the input for the root).");
    const ScriptMask   ret    = script_mask_num;
    const ScriptSigArg args[] = {
        {string_lit("index"), script_mask_num},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_joint_parent);
  }
  {
    const String       name   = string_lit("joint_find");
    const String       doc    = string_lit("Find a joint with the given name, returns the index of the joint or null if none was found.");
    const ScriptMask   ret    = script_mask_num | script_mask_null;
    const ScriptSigArg args[] = {
        {string_lit("jointName"), script_mask_str},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_joint_find);
  }
  {
    const String       name   = string_lit("joint_name");
    const String       doc    = string_lit("Query or change the name of the joint at the given index.");
    const ScriptMask   ret    = script_mask_str | script_mask_null;
    const ScriptSigArg args[] = {
        {string_lit("index"), script_mask_num},
        {string_lit("newName"), script_mask_str | script_mask_null},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_joint_name);
  }
  {
    const String       name   = string_lit("joint_name_match");
    const String       doc    = fmt_write_scratch("Check if the joint name matches the given pattern.\n\n{}", fmt_text(g_globPatternDoc));
    const ScriptMask   ret    = script_mask_bool;
    const ScriptSigArg args[] = {
        {string_lit("index"), script_mask_num},
        {string_lit("pattern"), script_mask_str},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_joint_name_match);
  }
  {
    const String       name   = string_lit("joint_name_trim");
    const String       doc    = string_lit("Remove a prefix (and optionally suffix) from the joint name at the given index. Returns the new name.");
    const ScriptMask   ret    = script_mask_str;
    const ScriptSigArg args[] = {
        {string_lit("index"), script_mask_num},
        {string_lit("prefix"), script_mask_str},
        {string_lit("suffix"), script_mask_str | script_mask_null},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_joint_name_trim);
  }
  {
    const String       name   = string_lit("anim_count");
    const String       doc    = string_lit("Query the amount of animations in the mesh.");
    const ScriptMask   ret    = script_mask_num | script_mask_null;
    asset_import_bind(binder, name, doc, ret, null, 0, import_eval_anim_count);
  }
  {
    const String       name   = string_lit("anim_find");
    const String       doc    = string_lit("Find an animation with the given name, returns the index of the animation or null if none was found.");
    const ScriptMask   ret    = script_mask_num | script_mask_null;
    const ScriptSigArg args[] = {
        {string_lit("animName"), script_mask_str},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_anim_find);
  }
  {
    const String       name   = string_lit("anim_layer");
    const String       doc    = string_lit("Query or change the layer (sorting index) of the animation at the given index.");
    const ScriptMask   ret    = script_mask_num | script_mask_null;
    const ScriptSigArg args[] = {
        {string_lit("index"), script_mask_num},
        {string_lit("newLayer"), script_mask_num | script_mask_null},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_anim_layer);
  }
  {
    const String       name   = string_lit("anim_flag");
    const String       doc    = fmt_write_scratch("Query or change an animation flag.\n\n{}", fmt_text(g_animFlagsDoc));
    const ScriptMask   ret    = script_mask_bool | script_mask_null;
    const ScriptSigArg args[] = {
        {string_lit("index"), script_mask_num},
        {string_lit("flag"), script_mask_str},
        {string_lit("enable"), script_mask_bool | script_mask_null},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_anim_flag);
  }
  {
    const String       name   = string_lit("anim_name");
    const String       doc    = string_lit("Query or change the name of the animation at the given index.");
    const ScriptMask   ret    = script_mask_str | script_mask_null;
    const ScriptSigArg args[] = {
        {string_lit("index"), script_mask_num},
        {string_lit("newName"), script_mask_str | script_mask_null},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_anim_name);
  }
  {
    const String       name   = string_lit("anim_name_match");
    const String       doc    = fmt_write_scratch("Check if the animation name matches the given pattern.\n\n{}", fmt_text(g_globPatternDoc));
    const ScriptMask   ret    = script_mask_bool;
    const ScriptSigArg args[] = {
        {string_lit("index"), script_mask_num},
        {string_lit("pattern"), script_mask_str},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_anim_name_match);
  }
  {
    const String       name   = string_lit("anim_name_trim");
    const String       doc    = string_lit("Remove a prefix (and optionally suffix) from the animation name at the given index. Returns the new name.");
    const ScriptMask   ret    = script_mask_str;
    const ScriptSigArg args[] = {
        {string_lit("index"), script_mask_num},
        {string_lit("prefix"), script_mask_str},
        {string_lit("suffix"), script_mask_str | script_mask_null},
    };
    asset_import_bind(binder, name, doc, ret, args, array_elems(args), import_eval_anim_name_trim);
  }
  // clang-format on

  asset_import_register(binder);

  script_binder_finalize(binder);
  g_assetScriptImportMeshBinder = binder;
}

bool asset_import_mesh(const AssetImportEnvComp* env, const String id, AssetImportMesh* data) {
  // Run import scripts.
  if (!asset_import_eval(env, g_assetScriptImportMeshBinder, id, data)) {
    return false;
  }

  // Check for duplicate joint names.
  StringHash duplicateJointNameHash;
  if (import_mesh_joint_find_duplicate(data, &duplicateJointNameHash)) {
    String duplicateJointName = string_lit("< unknown >");
    if (duplicateJointNameHash) {
      duplicateJointName = stringtable_lookup(g_stringtable, duplicateJointNameHash);
    }
    log_e(
        "Duplicate joint name found in mesh",
        log_param("asset", fmt_text(id)),
        log_param("joint-name", fmt_text(duplicateJointName)));
    return false;
  }

  // Check for duplicate animation names.
  StringHash duplicateAnimNameHash;
  if (import_mesh_anim_find_duplicate(data, &duplicateAnimNameHash)) {
    String duplicateAnimName = string_lit("< unknown >");
    if (duplicateAnimNameHash) {
      duplicateAnimName = stringtable_lookup(g_stringtable, duplicateAnimNameHash);
    }
    log_e(
        "Duplicate animation name found in mesh",
        log_param("asset", fmt_text(id)),
        log_param("anim-name", fmt_text(duplicateAnimName)));
    return false;
  }

  // Apply animation layer sorting.
  sort_quicksort_t(
      data->anims, data->anims + data->animCount, AssetImportAnim, import_compare_anim_layer);

  return true;
}

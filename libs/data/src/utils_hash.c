#include "core/bits.h"
#include "core/diag.h"
#include "data/utils.h"

#include "registry.h"

typedef struct {
  const DataReg* reg;
  DataHashFlags  flags;
  DataMeta       meta;
  DynArray*      pendingTypes; // DataType[].
} HashCtx;

static u32 data_hash_internal(const HashCtx*);

static u32 data_hash_struct(const HashCtx* ctx) {
  const DataDecl* decl = data_decl(ctx->reg, ctx->meta.type);
  diag_assert(decl->kind == DataKind_Struct);

  u32 hash = bits_hash_32_val((u32)decl->val_struct.fields.size);

  dynarray_for_t(&decl->val_struct.fields, DataDeclField, fieldDecl) {
    const HashCtx fieldCtx = {
        .reg          = ctx->reg,
        .flags        = ctx->flags,
        .meta         = fieldDecl->meta,
        .pendingTypes = ctx->pendingTypes,
    };
    const u32 fieldHash = data_hash_internal(&fieldCtx);

    if (!(ctx->flags & DataHashFlags_ExcludeIds)) {
      hash = bits_hash_32_combine(hash, fieldDecl->id.hash);
    }
    hash = bits_hash_32_combine(hash, fieldHash);
  }

  return hash;
}

static u32 data_hash_union(const HashCtx* ctx) {
  const DataDecl* decl = data_decl(ctx->reg, ctx->meta.type);
  diag_assert(decl->kind == DataKind_Union);

  u32 hash = bits_hash_32_val((u32)decl->val_union.choices.size);

  const DataUnionNameType nameType = data_union_name_type(&decl->val_union);
  hash                             = bits_hash_32_combine(hash, bits_hash_32_val(nameType));

  dynarray_for_t(&decl->val_union.choices, DataDeclChoice, choiceDecl) {
    const bool emptyChoice = choiceDecl->meta.type == 0;

    const u32 choiceTagHash = bits_hash_32_val(choiceDecl->tag);

    u32 choiceValHash;
    if (emptyChoice) {
      choiceValHash = bits_hash_32_val(42);
    } else {
      const HashCtx choiceCtx = {
          .reg          = ctx->reg,
          .flags        = ctx->flags,
          .meta         = choiceDecl->meta,
          .pendingTypes = ctx->pendingTypes,
      };
      choiceValHash = data_hash_internal(&choiceCtx);
    }

    if (!(ctx->flags & DataHashFlags_ExcludeIds)) {
      hash = bits_hash_32_combine(hash, choiceDecl->id.hash);
    }
    hash = bits_hash_32_combine(hash, choiceTagHash);
    hash = bits_hash_32_combine(hash, choiceValHash);
  }

  return hash;
}

static u32 data_hash_enum(const HashCtx* ctx) {
  const DataDecl* decl = data_decl(ctx->reg, ctx->meta.type);

  u32 hash = bits_hash_32_val((u32)decl->val_enum.consts.size);
  hash     = bits_hash_32_combine(hash, bits_hash_32_val(decl->val_enum.multi));

  dynarray_for_t(&decl->val_enum.consts, DataDeclConst, constDecl) {
    const u32 constValHash = bits_hash_32_val((u32)constDecl->value);

    if (!(ctx->flags & DataHashFlags_ExcludeIds)) {
      hash = bits_hash_32_combine(hash, constDecl->id.hash);
    }
    hash = bits_hash_32_combine(hash, constValHash);
  }

  return hash;
}

static u32 data_hash_opaque(const HashCtx* ctx) {
  const DataDecl* decl = data_decl(ctx->reg, ctx->meta.type);

  u32 hash = bits_hash_32_val(DataKind_Opaque);
  hash     = bits_hash_32_combine(hash, bits_hash_32_val((u32)decl->size));
  return hash;
}

static u32 data_hash_single(const HashCtx* ctx) {
  if (dynarray_search_linear(ctx->pendingTypes, compare_u32, &ctx->meta.type)) {
    return u32_max; // Type is used recursively, use a sentinel hash value.
  }
  *dynarray_push_t(ctx->pendingTypes, DataType) = ctx->meta.type;

  const DataKind kind = data_decl(ctx->reg, ctx->meta.type)->kind;
  u32            result;
  switch (kind) {
  case DataKind_bool:
  case DataKind_i8:
  case DataKind_i16:
  case DataKind_i32:
  case DataKind_i64:
  case DataKind_u8:
  case DataKind_u16:
  case DataKind_u32:
  case DataKind_u64:
  case DataKind_f16:
  case DataKind_f32:
  case DataKind_f64:
  case DataKind_TimeDuration:
  case DataKind_Angle:
  case DataKind_String:
  case DataKind_StringHash:
  case DataKind_DataMem:
    result = bits_hash_32_val(kind);
    break;
  case DataKind_Struct:
    result = data_hash_struct(ctx);
    break;
  case DataKind_Union:
    result = data_hash_union(ctx);
    break;
  case DataKind_Enum:
    result = data_hash_enum(ctx);
    break;
  case DataKind_Opaque:
    result = data_hash_opaque(ctx);
    break;
  case DataKind_Invalid:
  case DataKind_Count:
#ifdef VOLO_RELEASE
  default:
#endif
    diag_crash();
  }

  diag_assert(*(dynarray_end_t(ctx->pendingTypes, DataType) - 1) == ctx->meta.type);
  dynarray_pop(ctx->pendingTypes, 1);

  return result;
}

static u32 data_hash_flags(const DataFlags flags) {
  return bits_hash_32_val(flags & DataFlags_Hash);
}

static u32 data_hash_internal(const HashCtx* ctx) {
  const u32 containerHash  = bits_hash_32_val(ctx->meta.container);
  const u32 flagsHash      = data_hash_flags(ctx->meta.flags);
  const u32 fixedCountHash = bits_hash_32_val(ctx->meta.fixedCount);

  u32 res = data_hash_single(ctx);
  res     = bits_hash_32_combine(res, containerHash);
  res     = bits_hash_32_combine(res, flagsHash);
  res     = bits_hash_32_combine(res, fixedCountHash);
  return res;
}

u32 data_hash(const DataReg* reg, const DataMeta meta, const DataHashFlags flags) {
  DynArray pendingTypes = dynarray_create_over_t(mem_stack(1024), DataType);

  const HashCtx ctx = {
      .reg          = reg,
      .flags        = flags,
      .meta         = meta,
      .pendingTypes = &pendingTypes,
  };
  const u32 result = data_hash_internal(&ctx);
  diag_assert(dynarray_size(&pendingTypes) == 0);
  return result;
}

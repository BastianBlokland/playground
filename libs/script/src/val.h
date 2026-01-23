#pragma once
#include "script/val.h"

/**
 * ScriptVal's are 64bit values with 64bit alignment.
 *
 * | Type     | Byte 0  | Byte 1  | Byte 2  | Byte 3  | Byte 4  | Byte 5  | Byte 6  | Byte 7  |
 * |----------|---------|---------|---------|---------|---------|---------|---------|---------|
 * | null     | unused  | unused  | unused  | unused  | unused  | unused  | unused  | tag 0   |
 * | num      | i32 1/4 | i32 2/4 | i32 3/4 | i32 4/4 | unused  | unused  | unused  | tag 1   |
 * | numRange | i32 1/4 | i32 2/4 | i32 3/4 | i32 4/4 | u16 1/2 | u16 2/2 | unused  | tag 2   |
 * | bool     | u1      | unused  | unused  | unused  | unused  | unused  | unused  | tag 3   |
 * | str      | u32 1/4 | u32 2/4 | u32 3/4 | u32 4/4 | unused  | unused  | unused  | tag 4   |
 * | id       | u56 1/7 | u56 2/7 | u56 3/7 | u56 4/7 | u56 5/7 | u56 6/7 | u56 7/7 | tag 5   |
 *
 * NOTE: Assumes little-endian byte order.
 */

/**
 * Index of the type byte inside a ScriptVal.
 */
#define val_type_byte_index 7

MAYBE_UNUSED INLINE_HINT static ScriptType val_type(const ScriptVal value) {
  return (ScriptType)value.bytes[val_type_byte_index];
}

MAYBE_UNUSED INLINE_HINT static bool val_type_check(const ScriptVal v, const ScriptMask mask) {
  return (mask & (1 << val_type(v))) != 0;
}

MAYBE_UNUSED INLINE_HINT static ScriptVal val_null(void) {
  ASSERT(ScriptType_Null == 0, "ScriptType_Null should be initializable using zero-init");
  return (ScriptVal){0};
}

MAYBE_UNUSED INLINE_HINT static ScriptVal val_bool(const bool value) {
  ScriptVal result;
  *(bool*)result.bytes              = value;
  result.bytes[val_type_byte_index] = ScriptType_Bool;
  return result;
}

MAYBE_UNUSED INLINE_HINT static ScriptVal val_num(const i32 value) {
  ScriptVal result;
  *(i32*)result.bytes               = value;
  result.bytes[val_type_byte_index] = ScriptType_Num;
  return result;
}

MAYBE_UNUSED INLINE_HINT static ScriptVal val_num_range(const i32 base, const u16 extent) {
  ScriptVal result;
  *(i32*)result.bytes                 = base;
  *(u16*)(result.bytes + sizeof(i32)) = extent;
  result.bytes[val_type_byte_index]   = ScriptType_NumRange;
  return result;
}

MAYBE_UNUSED INLINE_HINT static ScriptVal val_str(const StringHash value) {
  ScriptVal result;
  *(StringHash*)result.bytes        = value;
  result.bytes[val_type_byte_index] = ScriptType_Str;
  return result;
}

MAYBE_UNUSED INLINE_HINT static ScriptVal val_id(const u64 value /* u56 */) {
  ScriptVal result;
  *(u64*)result.bytes               = value;
  result.bytes[val_type_byte_index] = ScriptType_Id;
  return result;
}

MAYBE_UNUSED INLINE_HINT static i32 val_as_num(const ScriptVal value) { return *(i32*)value.bytes; }

MAYBE_UNUSED INLINE_HINT static i32 val_as_num_range_base(const ScriptVal value) {
  return *(i32*)value.bytes;
}

MAYBE_UNUSED INLINE_HINT static u16 val_as_num_range_extent(const ScriptVal value) {
  return *(u16*)(value.bytes + sizeof(i32));
}

MAYBE_UNUSED INLINE_HINT static bool val_as_bool(const ScriptVal value) {
  return *(bool*)value.bytes;
}

MAYBE_UNUSED INLINE_HINT static StringHash val_as_str(const ScriptVal value) {
  return *(StringHash*)value.bytes;
}

MAYBE_UNUSED INLINE_HINT static u64 val_as_id(const ScriptVal value) {
  return (*(u64*)value.bytes) & u64_lit(0x00FFFFFFFFFFFFFF);
}

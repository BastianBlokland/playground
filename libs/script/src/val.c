#include "core/alloc.h"
#include "core/array.h"
#include "core/bits.h"
#include "core/diag.h"
#include "core/dynstring.h"
#include "core/intrinsic.h"
#include "core/math.h"
#include "core/stringtable.h"
#include "script/hash.h"
#include "script/val.h"

#include "val.h"

static i32 val_clamp_to_i32(const i64 val) {
  if (val <= i32_min) {
    return i32_min;
  }
  if (val >= i32_max) {
    return i32_max;
  }
  return (i32)val;
}

ScriptType script_type(const ScriptVal value) { return val_type(value); }

bool script_type_check(const ScriptVal value, const ScriptMask mask) {
  return val_type_check(value, mask);
}

ScriptVal script_null(void) { return val_null(); }
ScriptVal script_num(const i32 value) { return val_num(value); }
ScriptVal script_num_range(const i32 base, const u16 extent) { return val_num_range(base, extent); }
ScriptVal script_num_range_from_to(i32 min, i32 max) {
  if (min > max) {
    const i32 tmp = max;
    max           = min;
    min           = tmp;
  }
  const i32 extent = max - min;
  diag_assert(extent >= 0);
  return val_num_range(min, extent >= u16_max ? u16_max : (u16)extent);
}
ScriptVal script_bool(const bool value) { return val_bool(value); }
ScriptVal script_str(const StringHash str) { return val_str(str); }
ScriptVal script_str_empty(void) { return val_str(ScriptHash_empty); }
ScriptVal script_str_or_null(const StringHash str) { return str ? val_str(str) : val_null(); }

ScriptVal script_id(const u64 id) {
  diag_assert(id);
  return val_id(id);
}

ScriptVal script_id_or_null(const u64 id) { return id ? val_id(id) : val_null(); }

i32 script_get_num(const ScriptVal value, const i32 fallback) {
  return val_type(value) == ScriptType_Num ? val_as_num(value) : fallback;
}

i32 script_get_num_range_base(const ScriptVal value, const i32 fallback) {
  return val_type(value) == ScriptType_NumRange ? val_as_num_range_base(value) : fallback;
}

u16 script_get_num_range_extent(const ScriptVal value, const u16 fallback) {
  return val_type(value) == ScriptType_NumRange ? val_as_num_range_extent(value) : fallback;
}

i32 script_get_num_range_min(const ScriptVal value, const i32 fallback) {
  return val_type(value) == ScriptType_NumRange ? val_as_num_range_base(value) : fallback;
}

i32 script_get_num_range_max(const ScriptVal value, const i32 fallback) {
  if (val_type(value) != ScriptType_NumRange) {
    return fallback;
  }
  const i64 max = (i64)val_as_num_range_base(value) + (i64)val_as_num_range_extent(value);
  return max >= i32_max ? i32_max : (i32)max;
}

bool script_get_bool(const ScriptVal value, const bool fallback) {
  return val_type(value) == ScriptType_Bool ? val_as_bool(value) : fallback;
}

StringHash script_get_str(const ScriptVal value, const StringHash fallback) {
  return val_type(value) == ScriptType_Str ? val_as_str(value) : fallback;
}

u64 script_get_id(const ScriptVal value, const u64 fallback) {
  return val_type(value) == ScriptType_Id ? val_as_id(value) : fallback;
}

bool script_val_valid(const ScriptVal value) {
  const ScriptType type = val_type(value);
  if (type >= ScriptType_Count) {
    return false; // Invalid type.
  }
  switch (type) {
  case ScriptType_Null:
  case ScriptType_Num:
  case ScriptType_NumRange:
  case ScriptType_Str:
    return true;
  case ScriptType_Id:
    return val_as_id(value) != 0;
  case ScriptType_Bool:
    return val_as_bool(value) < 2;
  case ScriptType_Count:
    break;
  }
  UNREACHABLE
}

bool script_truthy(const ScriptVal value) {
  switch (val_type(value)) {
  case ScriptType_Null:
    return false;
  case ScriptType_Bool:
    return val_as_bool(value);
  case ScriptType_Num:
  case ScriptType_NumRange:
  case ScriptType_Str:
  case ScriptType_Id:
    return true;
  case ScriptType_Count:
    break;
  }
  UNREACHABLE
}

ScriptVal script_truthy_as_val(const ScriptVal value) { return val_bool(script_truthy(value)); }

bool script_falsy(const ScriptVal value) { return !script_truthy(value); }

ScriptVal script_falsy_as_val(const ScriptVal value) { return val_bool(!script_truthy(value)); }

bool script_non_null(const ScriptVal value) { return val_type(value) != ScriptType_Null; }

ScriptVal script_non_null_as_val(const ScriptVal value) {
  return val_bool(val_type(value) != ScriptType_Null);
}

ScriptVal script_val_or(const ScriptVal value, const ScriptVal fallback) {
  return val_type(value) ? value : fallback;
}

u32 script_hash(const ScriptVal value) {
  const u32 tHash = script_val_type_hash(val_type(value));
  switch (val_type(value)) {
  case ScriptType_Null:
    return tHash;
  case ScriptType_Num:
    return bits_hash_32_combine(tHash, bits_hash_32(mem_create(value.bytes, sizeof(i32))));
  case ScriptType_NumRange: {
    const usize size = sizeof(i32) + sizeof(u16);
    return bits_hash_32_combine(tHash, bits_hash_32(mem_create(value.bytes, size)));
  }
  case ScriptType_Bool:
    return bits_hash_32_combine(tHash, bits_hash_32(mem_create(value.bytes, sizeof(bool))));
  case ScriptType_Str:
    return bits_hash_32_combine(tHash, val_as_str(value));
  case ScriptType_Id: {
    const u32 idA = (u32)(val_as_id(value) & 0xFFFFFFFF);
    const u32 idB = (u32)(val_as_id(value) >> 32);
    return bits_hash_32_combine(tHash, bits_hash_32_combine(idA, idB));
  }
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_zero_pad(const ScriptVal v) {
  ScriptVal result                  = {0};
  result.bytes[val_type_byte_index] = val_type(v);
  switch (result.bytes[val_type_byte_index]) {
  case ScriptType_Null:
    break; // Nothing to do; null is all zeroes.
  case ScriptType_Num:
    mem_cpy(array_mem(result.bytes), mem_create(v.bytes, sizeof(i32)));
    break;
  case ScriptType_NumRange:
    mem_cpy(array_mem(result.bytes), mem_create(v.bytes, sizeof(i32) + sizeof(u16)));
    break;
  case ScriptType_Bool:
    result.bytes[0] = v.bytes[0];
    break;
  case ScriptType_Str:
    mem_cpy(array_mem(result.bytes), mem_create(v.bytes, sizeof(StringHash)));
    break;
  case ScriptType_Id:
    mem_cpy(array_mem(result.bytes), mem_create(v.bytes, 7));
    break;
  case ScriptType_Count:
    diag_assert_fail("Invalid script value");
    UNREACHABLE
  }
  return result;
}

String script_val_type_str(const ScriptType type) {
  diag_assert_msg(type < ScriptType_Count, "Invalid script value type: {}", fmt_int(type));
  static const String g_names[] = {
      string_static("null"),
      string_static("num"),
      string_static("range"),
      string_static("bool"),
      string_static("str"),
      string_static("id"),
  };
  ASSERT(array_elems(g_names) == ScriptType_Count, "Incorrect number of names");
  return g_names[type];
}

static const StringHash g_valTypeHashes[ScriptType_Count] = {
    ScriptHash_null,
    ScriptHash_num,
    ScriptHash_range,
    ScriptHash_bool,
    ScriptHash_str,
    ScriptHash_id,
};

StringHash script_val_type_hash(const ScriptType type) {
  diag_assert_msg(type < ScriptType_Count, "Invalid script value type: {}", fmt_int(type));
  return g_valTypeHashes[type];
}

ScriptType script_val_type_from_hash(const StringHash hash) {
  for (ScriptType t = 0; t != ScriptType_Count; ++t) {
    if (hash == g_valTypeHashes[t]) {
      return t;
    }
  }
  return ScriptType_Null; // TODO: Should we return a sentinel instead?
}

void script_val_write(const ScriptVal value, DynString* str) {
  switch (val_type(value)) {
  case ScriptType_Null:
    dynstring_append(str, string_lit("null"));
    return;
  case ScriptType_Num:
    format_write_i64(str, val_as_num(value), &format_opts_int());
    return;
  case ScriptType_NumRange: {
    const i32 min = val_as_num_range_base(value);
    const i64 max = min + val_as_num_range_extent(value);
    fmt_write(str, "{} to {}", fmt_int(min), fmt_int(max >= i32_max ? i32_max : max));
    return;
  }
  case ScriptType_Bool:
    format_write_bool(str, val_as_bool(value));
    return;
  case ScriptType_Str:
    format_write_stringhash(str, val_as_str(value), &format_opts_text());
    return;
  case ScriptType_Id:
    format_write_u64(str, val_as_id(value), &format_opts_int());
    return;
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

String script_val_scratch(const ScriptVal value) {
  const Mem scratchMem = alloc_alloc(g_allocScratch, 128, 1);
  DynString str        = dynstring_create_over(scratchMem);

  script_val_write(value, &str);

  const String res = dynstring_view(&str);
  dynstring_destroy(&str);
  return res;
}

void script_mask_write(ScriptMask mask, DynString* str) {
  if (mask == script_mask_any) {
    dynstring_append(str, string_lit("any"));
    return;
  }
  if (mask == script_mask_none) {
    dynstring_append(str, string_lit("none"));
    return;
  }
  if ((mask & (1 << ScriptType_Null)) && intrinsic_popcnt_32(mask) == 2) {
    mask ^= 1 << ScriptType_Null; // Clear the null bit.
    const ScriptType type = (ScriptType)bitset_next(bitset_from_var(mask), 0);
    dynstring_append(str, script_val_type_str(type));
    dynstring_append_char(str, '?');
    return;
  }

  bool first = true;
  bitset_for(bitset_from_var(mask), typeIndex) {
    if (!first) {
      dynstring_append(str, string_lit(" | "));
    }
    first = false;
    dynstring_append(str, script_val_type_str((ScriptType)typeIndex));
  }
}

String script_mask_scratch(const ScriptMask mask) {
  const Mem scratchMem = alloc_alloc(g_allocScratch, 256, 1);
  DynString str        = dynstring_create_over(scratchMem);

  script_mask_write(mask, &str);

  const String res = dynstring_view(&str);
  dynstring_destroy(&str);
  return res;
}

bool script_val_equal(const ScriptVal a, const ScriptVal b) {
  if (val_type(a) != val_type(b)) {
    return false;
  }
  switch (val_type(a)) {
  case ScriptType_Null:
    return true;
  case ScriptType_Num:
    return val_as_num(a) == val_as_num(b);
  case ScriptType_NumRange:
    return (val_as_num_range_base(a) == val_as_num_range_base(b)) &&
           (val_as_num_range_extent(a) == val_as_num_range_extent(b));
  case ScriptType_Bool:
    return val_as_bool(a) == val_as_bool(b);
  case ScriptType_Str:
    return val_as_str(a) == val_as_str(b);
  case ScriptType_Id:
    return val_as_id(a) == val_as_id(b);
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_equal_as_val(const ScriptVal a, const ScriptVal b) {
  return val_bool(script_val_equal(a, b));
}

bool script_val_less(const ScriptVal a, const ScriptVal b) {
  if (val_type(a) != val_type(b)) {
    return false;
  }
  switch (val_type(a)) {
  case ScriptType_Null:
  case ScriptType_Str:
  case ScriptType_Id:
    return false;
  case ScriptType_Num:
    return val_as_num(a) < val_as_num(b);
  case ScriptType_NumRange: {
    const i32 aMin = val_as_num_range_base(a);
    const i32 bMin = val_as_num_range_base(b);
    const i64 aMax = aMin + (i64)val_as_num_range_extent(a);
    const i64 bMax = bMin + (i64)val_as_num_range_extent(b);
    return (aMin < bMin) && (aMax < bMax);
  }
  case ScriptType_Bool:
    return val_as_bool(a) < val_as_bool(b); // NOTE: Questionable usefulness?
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_less_as_val(const ScriptVal a, const ScriptVal b) {
  return val_bool(script_val_less(a, b));
}

bool script_val_greater(const ScriptVal a, const ScriptVal b) {
  if (val_type(a) != val_type(b)) {
    return false;
  }
  switch (val_type(a)) {
  case ScriptType_Null:
  case ScriptType_Str:
  case ScriptType_Id:
    return false;
  case ScriptType_Num:
    return val_as_num(a) > val_as_num(b);
  case ScriptType_NumRange: {
    const i32 aMin = val_as_num_range_base(a);
    const i32 bMin = val_as_num_range_base(b);
    const i64 aMax = aMin + (i64)val_as_num_range_extent(a);
    const i64 bMax = bMin + (i64)val_as_num_range_extent(b);
    return (aMin > bMin) && (aMax > bMax);
  }
  case ScriptType_Bool:
    return val_as_bool(a) > val_as_bool(b);
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_greater_as_val(const ScriptVal a, const ScriptVal b) {
  return val_bool(script_val_greater(a, b));
}

ScriptVal script_val_type(const ScriptVal val) { return val_str(g_valTypeHashes[val_type(val)]); }

ScriptVal script_val_hash(const ScriptVal val) { return val_num(script_hash(val)); }

ScriptVal script_val_neg(const ScriptVal val) {
  switch (val_type(val)) {
  case ScriptType_Null:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num:
    return val_num(-val_as_num(val));
  case ScriptType_NumRange: {
    const i32 base = val_as_num_range_base(val);
    const i64 min  = (i64)base - val_as_num_range_extent(val);
    return script_num_range_from_to(min <= i32_min ? i32_min : (i32)min, base);
  }
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_inv(const ScriptVal val) { return val_bool(!script_truthy(val)); }

ScriptVal script_val_add(const ScriptVal a, const ScriptVal b) {
  if (val_type(a) != val_type(b)) {
    return val_null();
  }
  switch (val_type(a)) {
  case ScriptType_Null:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num: {
    const i64 res = (i64)val_as_num(a) + (i64)val_as_num(b);
    return val_num(res >= i32_max ? i32_max : (i32)res);
  }
  case ScriptType_NumRange: {
    const i32 aMin   = val_as_num_range_base(a);
    const i32 bMin   = val_as_num_range_base(b);
    const i32 resMin = val_clamp_to_i32(aMin + bMin);

    const i64 aMax   = aMin + (i64)val_as_num_range_extent(a);
    const i64 bMax   = bMin + (i64)val_as_num_range_extent(b);
    const i32 resMax = val_clamp_to_i32(aMax + bMax);
    return script_num_range_from_to(resMin, resMax);
  }
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_sub(const ScriptVal a, const ScriptVal b) {
  if (val_type(a) != val_type(b)) {
    return val_null();
  }
  switch (val_type(a)) {
  case ScriptType_Null:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num: {
    const i64 res = (i64)val_as_num(a) - (i64)val_as_num(b);
    return val_num(res <= i32_min ? i32_min : (i32)res);
  }
  case ScriptType_NumRange: {
    const i32 aMin   = val_as_num_range_base(a);
    const i32 bMin   = val_as_num_range_base(b);
    const i32 resMin = val_clamp_to_i32(aMin - bMin);

    const i64 aMax   = (i64)aMin + (i64)val_as_num_range_extent(a);
    const i64 bMax   = (i64)bMin + (i64)val_as_num_range_extent(b);
    const i32 resMax = val_clamp_to_i32(aMax - bMax);
    return script_num_range_from_to(resMin, resMax);
  }
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_mul(const ScriptVal a, const ScriptVal b) {
  switch (val_type(a)) {
  case ScriptType_Null:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num: {
    if (val_type(b) == ScriptType_Num) {
      return val_num(val_clamp_to_i32((i64)val_as_num(a) * (i64)val_as_num(b)));
    }
    return val_null();
  }
  case ScriptType_NumRange: {
    const i64 aMin = val_as_num_range_base(a);
    const i64 aMax = (i64)aMin + val_as_num_range_extent(a);
    if (val_type(b) == ScriptType_Num) {
      const i32 multiplier = val_as_num(b);
      return script_num_range_from_to(
          val_clamp_to_i32(aMin * multiplier), val_clamp_to_i32(aMax * multiplier));
    }
    return val_null();
  }
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_div(const ScriptVal a, const ScriptVal b) {
  switch (val_type(a)) {
  case ScriptType_Null:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num: {
    if (val_type(b) == ScriptType_Num) {
      return val_num(val_clamp_to_i32((i64)val_as_num(a) / (i64)val_as_num(b)));
    }
    return val_null();
  }
  case ScriptType_NumRange: {
    const i64 aMin = val_as_num_range_base(a);
    const i64 aMax = aMin + val_as_num_range_extent(a);
    if (val_type(b) == ScriptType_Num) {
      const i32 multiplier = val_as_num(b);
      return script_num_range_from_to(
          val_clamp_to_i32(aMin / multiplier), val_clamp_to_i32(aMax / multiplier));
    }
    return val_null();
  }
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_mod(const ScriptVal a, const ScriptVal b) {
  switch (val_type(a)) {
  case ScriptType_Null:
  case ScriptType_NumRange:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num:
    return val_type(b) == ScriptType_Num ? val_num(val_as_num(a) % val_as_num(b)) : val_null();
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_abs(const ScriptVal val) {
  switch (val_type(val)) {
  case ScriptType_Null:
  case ScriptType_NumRange:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num: {
    const i32 num = val_as_num(val);
    return val_num(num < 0 ? -num : num);
  }
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_logic_and(const ScriptVal a, const ScriptVal b) {
  return val_bool(script_truthy(a) && script_truthy(b));
}

ScriptVal script_val_logic_or(const ScriptVal a, const ScriptVal b) {
  return val_bool(script_truthy(a) || script_truthy(b));
}

ScriptVal script_val_clamp(const ScriptVal v, const ScriptVal range) {
  if (val_type(range) != ScriptType_NumRange) {
    return val_null();
  }
  const i32 min = val_as_num_range_base(range);
  const i32 max = val_clamp_to_i32((i64)min + val_as_num_range_extent(range));
  switch (val_type(v)) {
  case ScriptType_Null:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num: {
    i32 val = val_as_num(v);
    if (val <= min) {
      val = min;
    }
    if (val >= max) {
      val = max;
    }
    return val_num(val);
  }
  case ScriptType_NumRange: {
    const i32 vMin = val_as_num_range_base(v);
    const i32 vMax = val_clamp_to_i32((i64)vMin + val_as_num_range_extent(v));
    return script_num_range_from_to(math_clamp_i32(vMin, min, max), math_clamp_i32(vMax, min, max));
  }
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_contains(const ScriptVal v, const ScriptVal range) {
  if (val_type(range) != ScriptType_NumRange) {
    return val_null();
  }
  const i32 min = val_as_num_range_base(range);
  const i32 max = val_clamp_to_i32((i64)min + val_as_num_range_extent(range));
  switch (val_type(v)) {
  case ScriptType_Null:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num:
    return script_bool(val_as_num(v) >= min && val_as_num(v) <= max);
  case ScriptType_NumRange: {
    const i32 vMin = val_as_num_range_base(v);
    const i32 vMax = val_clamp_to_i32((i64)vMin + val_as_num_range_extent(v));
    return script_bool(vMin >= min && vMax <= max);
  }
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_min(const ScriptVal x, const ScriptVal y) {
  if (val_type(x) != val_type(y)) {
    return val_null();
  }
  switch (val_type(x)) {
  case ScriptType_Null:
  case ScriptType_Bool:
  case ScriptType_NumRange:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num:
    return val_num(math_min(val_as_num(x), val_as_num(y)));
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_max(const ScriptVal x, const ScriptVal y) {
  if (val_type(x) != val_type(y)) {
    return val_null();
  }
  switch (val_type(x)) {
  case ScriptType_Null:
  case ScriptType_NumRange:
  case ScriptType_Bool:
  case ScriptType_Str:
  case ScriptType_Id:
    return val_null();
  case ScriptType_Num:
    return val_num(math_max(val_as_num(x), val_as_num(y)));
  case ScriptType_Count:
    break;
  }
  diag_assert_fail("Invalid script value");
  UNREACHABLE
}

ScriptVal script_val_num_range_min(const ScriptVal val) {
  if (val_type(val) != ScriptType_NumRange) {
    return val_null();
  }
  return val_num(val_as_num_range_base(val));
}

ScriptVal script_val_num_range_max(const ScriptVal val) {
  if (val_type(val) != ScriptType_NumRange) {
    return val_null();
  }
  const i64 max = (i64)val_as_num_range_base(val) + (i64)val_as_num_range_extent(val);
  return val_num(max >= i32_max ? i32_max : (i32)max);
}

ScriptVal script_val_num_range_from_to(const ScriptVal from, const ScriptVal to) {
  if (val_type(from) != ScriptType_Num || val_type(to) != ScriptType_Num) {
    return val_null();
  }
  return script_num_range_from_to(val_as_num(from), val_as_num(to));
}

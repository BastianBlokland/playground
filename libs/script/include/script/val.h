#pragma once
#include "core/bits.h"
#include "core/forward.h"

typedef enum eScriptType {
  ScriptType_Null,
  ScriptType_Num,
  ScriptType_NumRange,
  ScriptType_Bool,
  ScriptType_Str,
  ScriptType_Id, // u56 (7 bytes).

  ScriptType_Count,
} ScriptType;

typedef u16 ScriptMask;
ASSERT(ScriptType_Count < 16, "ScriptType's have to be indexable with 16 bits");

#define script_mask(_TYPE_) ((ScriptMask)(1 << _TYPE_))
#define script_mask_none ((ScriptMask)0)
#define script_mask_any ((ScriptMask)bit_range_32(0, ScriptType_Count))
#define script_mask_null script_mask(ScriptType_Null)
#define script_mask_num script_mask(ScriptType_Num)
#define script_mask_num_range script_mask(ScriptType_NumRange)
#define script_mask_bool script_mask(ScriptType_Bool)
#define script_mask_str script_mask(ScriptType_Str)
#define script_mask_id script_mask(ScriptType_Id)

/**
 * Type-erased script value.
 */
typedef struct sScriptVal {
  ALIGNAS(8) u8 bytes[8];
} ScriptVal;

ASSERT(sizeof(ScriptVal) == 8, "Expected ScriptVal's size to be 64 bits");
ASSERT(alignof(ScriptVal) == 8, "Expected ScriptVal's alignment to be 64 bits");

/**
 * Retrieve the type of the given value.
 */
ScriptType script_type(ScriptVal);
bool       script_type_check(ScriptVal, ScriptMask);

/**
 * Type-erase a value into a ScriptVal.
 */
ScriptVal script_null(void);
ScriptVal script_num(i32);
ScriptVal script_num_range(i32 base, u16 extent);
ScriptVal script_num_range_from_to(i32 min, i32 max);
ScriptVal script_bool(bool);
ScriptVal script_str(StringHash);
ScriptVal script_str_empty(void);
ScriptVal script_str_or_null(StringHash);
ScriptVal script_id(u64 id /* u56 */);
ScriptVal script_id_or_null(u64 id /* u56 */);

/**
 * Extract a specific type.
 */
i32        script_get_num(ScriptVal, i32 fallback);
i32        script_get_num_range_base(ScriptVal, i32 fallback);
u16        script_get_num_range_extent(ScriptVal, u16 fallback);
i32        script_get_num_range_min(ScriptVal, i32 fallback);
i32        script_get_num_range_max(ScriptVal, i32 fallback);
bool       script_get_bool(ScriptVal, bool fallback);
StringHash script_get_str(ScriptVal, StringHash fallback);
u64        script_get_id(ScriptVal, u64 fallback);

/**
 * Value utilities.
 */
bool      script_val_valid(ScriptVal);
bool      script_truthy(ScriptVal);
ScriptVal script_truthy_as_val(ScriptVal);
bool      script_falsy(ScriptVal);
ScriptVal script_falsy_as_val(ScriptVal);
bool      script_non_null(ScriptVal);
ScriptVal script_non_null_as_val(ScriptVal);
ScriptVal script_val_or(ScriptVal value, ScriptVal fallback);
u32       script_hash(ScriptVal);
ScriptVal script_zero_pad(ScriptVal); // Fill padding with zeroes; usefull for serialization.

/**
 * Create a textual representation of a value.
 */
String     script_val_type_str(ScriptType);
StringHash script_val_type_hash(ScriptType);
ScriptType script_val_type_from_hash(StringHash);
void       script_val_write(ScriptVal, DynString*);
String     script_val_scratch(ScriptVal);
void       script_mask_write(ScriptMask, DynString*);
String     script_mask_scratch(ScriptMask);

/**
 * Compare values.
 */
bool      script_val_equal(ScriptVal, ScriptVal);
ScriptVal script_val_equal_as_val(ScriptVal, ScriptVal);
bool      script_val_less(ScriptVal, ScriptVal);
ScriptVal script_val_less_as_val(ScriptVal, ScriptVal);
bool      script_val_greater(ScriptVal, ScriptVal);
ScriptVal script_val_greater_as_val(ScriptVal, ScriptVal);

/**
 * Value arithmetic.
 */
ScriptVal script_val_type(ScriptVal);
ScriptVal script_val_hash(ScriptVal);
ScriptVal script_val_neg(ScriptVal);
ScriptVal script_val_inv(ScriptVal);
ScriptVal script_val_add(ScriptVal, ScriptVal);
ScriptVal script_val_sub(ScriptVal, ScriptVal);
ScriptVal script_val_mul(ScriptVal, ScriptVal);
ScriptVal script_val_div(ScriptVal, ScriptVal);
ScriptVal script_val_mod(ScriptVal, ScriptVal);
ScriptVal script_val_abs(ScriptVal);
ScriptVal script_val_logic_and(ScriptVal, ScriptVal);
ScriptVal script_val_logic_or(ScriptVal, ScriptVal);
ScriptVal script_val_clamp(ScriptVal, ScriptVal range);
ScriptVal script_val_contains(ScriptVal, ScriptVal range);
ScriptVal script_val_min(ScriptVal, ScriptVal);
ScriptVal script_val_max(ScriptVal, ScriptVal);

/**
 * Value conversions.
 */
ScriptVal script_val_num_range_min(ScriptVal);
ScriptVal script_val_num_range_max(ScriptVal);
ScriptVal script_val_num_range_from_to(ScriptVal from, ScriptVal to);

/**
 * Create a formatting argument for a script value.
 */
#define script_val_fmt(_VAL_) fmt_text(script_val_scratch(_VAL_))
#define script_mask_fmt(_MASK_) fmt_text(script_mask_scratch(_MASK_))

#include "core/diag.h"
#include "script/args.h"
#include "script/binder.h"
#include "script/enum.h"
#include "script/panic.h"
#include "script/val.h"

#include "val.h"

NORETURN static void arg_type_error(ScriptBinderCall* c, const u16 i, const ScriptMask mask) {
  script_panic_raise(
      c->panicHandler,
      (ScriptPanic){
          ScriptPanic_ArgumentTypeMismatch,
          .argIndex   = i,
          .typeMask   = mask,
          .typeActual = script_type(c->args[i]),
      });
}

INLINE_HINT static void arg_type_check(ScriptBinderCall* c, const u16 i, const ScriptMask mask) {
  if (UNLIKELY(c->argCount <= i)) {
    script_panic_raise(c->panicHandler, (ScriptPanic){ScriptPanic_ArgumentMissing, .argIndex = i});
  }
  if (UNLIKELY(!val_type_check(c->args[i], mask))) {
    script_panic_raise(
        c->panicHandler,
        (ScriptPanic){
            ScriptPanic_ArgumentTypeMismatch,
            .argIndex   = i,
            .typeMask   = mask,
            .typeActual = script_type(c->args[i]),
        });
  }
}

ScriptType script_arg_check(ScriptBinderCall* c, const u16 i, const ScriptMask mask) {
  arg_type_check(c, i, mask);
  return val_type(c->args[i]);
}

bool script_arg_has(ScriptBinderCall* c, const u16 i) {
  return c->argCount > i && val_type(c->args[i]) != ScriptType_Null;
}

void script_arg_shift(ScriptBinderCall* c) {
  diag_assert(c->argCount);
  ++c->args;
  --c->argCount;
}

ScriptVal script_arg_any(ScriptBinderCall* c, const u16 i) {
  if (UNLIKELY(c->argCount <= i)) {
    script_panic_raise(c->panicHandler, (ScriptPanic){ScriptPanic_ArgumentMissing, .argIndex = i});
  }
  return c->args[i];
}

i32 script_arg_num(ScriptBinderCall* c, const u16 i) {
  arg_type_check(c, i, script_mask_num);
  return val_as_num(c->args[i]);
}

i32 script_arg_num_clamped(ScriptBinderCall* c, const u16 i, const i32 min, const i32 max) {
  arg_type_check(c, i, script_mask_num);
  const i32 res = val_as_num(c->args[i]);
  if (LIKELY(res >= min && res <= max)) {
    return res;
  }
  script_panic_raise(c->panicHandler, (ScriptPanic){ScriptPanic_ArgumentOutOfRange, .argIndex = i});
}

ScriptArgNumRange script_arg_num_range(ScriptBinderCall* c, const u16 i) {
  arg_type_check(c, i, script_mask_num_range);
  const i32 min = val_as_num_range_base(c->args[i]);
  const i64 max = (i64)min + val_as_num_range_extent(c->args[i]);
  return (ScriptArgNumRange){.min = min, .max = max >= i32_max ? i32_max : (i32)max};
}

bool script_arg_bool(ScriptBinderCall* c, const u16 i) {
  arg_type_check(c, i, script_mask_bool);
  return val_as_bool(c->args[i]);
}

StringHash script_arg_str(ScriptBinderCall* c, const u16 i) {
  arg_type_check(c, i, script_mask_str);
  return val_as_str(c->args[i]);
}

u64 script_arg_id(ScriptBinderCall* c, const u16 i) {
  arg_type_check(c, i, script_mask_id);
  return val_as_id(c->args[i]);
}

i32 script_arg_enum(ScriptBinderCall* c, const u16 i, const ScriptEnum* e) {
  arg_type_check(c, i, script_mask_str);
  return script_enum_lookup_value_at_index(e, val_as_str(c->args[i]), i, c->panicHandler);
}

ScriptType script_arg_opt_type(ScriptBinderCall* c, const u16 i) {
  if (LIKELY(c->argCount > i)) {
    return val_type(c->args[i]);
  }
  return ScriptType_Null;
}

ScriptVal script_arg_opt_any(ScriptBinderCall* c, const u16 i, const ScriptVal def) {
  return c->argCount > i ? c->args[i] : def;
}

i32 script_arg_opt_num(ScriptBinderCall* c, const u16 i, const i32 def) {
  if (c->argCount > i) {
    if (val_type(c->args[i]) == ScriptType_Num) {
      return val_as_num(c->args[i]);
    }
    if (val_type(c->args[i]) == ScriptType_Null) {
      return def;
    }
    arg_type_error(c, i, script_mask_num | script_mask_null);
  }
  return def;
}

i32 script_arg_opt_num_clamped(
    ScriptBinderCall* c, const u16 i, const i32 min, const i32 max, const i32 def) {
  if (c->argCount > i) {
    if (val_type(c->args[i]) == ScriptType_Num) {
      const i32 res = val_as_num(c->args[i]);
      if (LIKELY(res >= min && res <= max)) {
        return res;
      }
      script_panic_raise(
          c->panicHandler, (ScriptPanic){ScriptPanic_ArgumentOutOfRange, .argIndex = i});
    }
    if (val_type(c->args[i]) == ScriptType_Null) {
      return def;
    }
    arg_type_error(c, i, script_mask_num | script_mask_null);
  }
  return def;
}

ScriptArgNumRange
script_arg_opt_num_range(ScriptBinderCall* c, const u16 i, const ScriptArgNumRange def) {
  if (c->argCount > i) {
    if (val_type(c->args[i]) == ScriptType_NumRange) {
      const i32 min = val_as_num_range_base(c->args[i]);
      const i64 max = (i64)min + val_as_num_range_extent(c->args[i]);
      return (ScriptArgNumRange){.min = min, .max = max >= i32_max ? i32_max : (i32)max};
    }
    if (val_type(c->args[i]) == ScriptType_Null) {
      return def;
    }
    arg_type_error(c, i, script_mask_num_range | script_mask_null);
  }
  return def;
}

bool script_arg_opt_bool(ScriptBinderCall* c, const u16 i, const bool def) {
  if (c->argCount > i) {
    if (val_type(c->args[i]) == ScriptType_Bool) {
      return val_as_bool(c->args[i]);
    }
    if (val_type(c->args[i]) == ScriptType_Null) {
      return def;
    }
    arg_type_error(c, i, script_mask_bool | script_mask_null);
  }
  return def;
}

StringHash script_arg_opt_str(ScriptBinderCall* c, const u16 i, const StringHash def) {
  if (c->argCount > i) {
    if (val_type(c->args[i]) == ScriptType_Str) {
      return val_as_str(c->args[i]);
    }
    if (val_type(c->args[i]) == ScriptType_Null) {
      return def;
    }
    arg_type_error(c, i, script_mask_str | script_mask_null);
  }
  return def;
}

u64 script_arg_opt_id(ScriptBinderCall* c, const u16 i, const u64 def) {
  if (c->argCount > i) {
    if (val_type(c->args[i]) == ScriptType_Id) {
      return val_as_id(c->args[i]);
    }
    if (val_type(c->args[i]) == ScriptType_Null) {
      return def;
    }
    arg_type_error(c, i, script_mask_id | script_mask_null);
  }
  return def;
}

i32 script_arg_opt_enum(ScriptBinderCall* c, const u16 i, const ScriptEnum* e, const i32 def) {
  if (c->argCount > i) {
    if (val_type(c->args[i]) == ScriptType_Str) {
      return script_enum_lookup_value(e, val_as_str(c->args[i]), c->panicHandler);
    }
    if (val_type(c->args[i]) == ScriptType_Null) {
      return def;
    }
    arg_type_error(c, i, script_mask_str | script_mask_null);
  }
  return def;
}

i32 script_arg_maybe_num(ScriptBinderCall* c, const u16 i, const i32 def) {
  if (c->argCount > i && val_type(c->args[i]) == ScriptType_Num) {
    return val_as_num(c->args[i]);
  }
  return def;
}

bool script_arg_maybe_bool(ScriptBinderCall* c, const u16 i, const bool def) {
  if (c->argCount > i && val_type(c->args[i]) == ScriptType_Bool) {
    return val_as_bool(c->args[i]);
  }
  return def;
}

StringHash script_arg_maybe_str(ScriptBinderCall* c, const u16 i, const StringHash def) {
  if (c->argCount > i && val_type(c->args[i]) == ScriptType_Str) {
    return val_as_str(c->args[i]);
  }
  return def;
}

u64 script_arg_maybe_id(ScriptBinderCall* c, const u16 i, const u64 def) {
  if (c->argCount > i && val_type(c->args[i]) == ScriptType_Id) {
    return val_as_id(c->args[i]);
  }
  return def;
}

i32 script_arg_maybe_enum(ScriptBinderCall* c, const u16 i, const ScriptEnum* e, const i32 def) {
  if (c->argCount > i && val_type(c->args[i]) == ScriptType_Str) {
    return script_enum_lookup_maybe_value(e, val_as_str(c->args[i]), def);
  }
  return def;
}

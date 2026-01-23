#pragma once
#include "script/forward.h"

/**
 * Argument check utilities.
 * NOTE: On failure the functions will not return, control-flow returns back to the script runtime.
 */

typedef struct {
  i32 min, max;
} ScriptArgNumRange;

ScriptType script_arg_check(ScriptBinderCall*, u16 i, ScriptMask);
bool       script_arg_has(ScriptBinderCall*, u16 i);

void script_arg_shift(ScriptBinderCall*);

ScriptVal         script_arg_any(ScriptBinderCall*, u16 i);
i32               script_arg_num(ScriptBinderCall*, u16 i);
i32               script_arg_num_clamped(ScriptBinderCall*, u16 i, i32 min, i32 max);
ScriptArgNumRange script_arg_num_range(ScriptBinderCall*, u16 i);
bool              script_arg_bool(ScriptBinderCall*, u16 i);
StringHash        script_arg_str(ScriptBinderCall*, u16 i);
u64               script_arg_id(ScriptBinderCall*, u16 i);
i32               script_arg_enum(ScriptBinderCall*, u16 i, const ScriptEnum*);

ScriptType        script_arg_opt_type(ScriptBinderCall*, u16 i);
ScriptVal         script_arg_opt_any(ScriptBinderCall*, u16 i, ScriptVal def);
i32               script_arg_opt_num(ScriptBinderCall*, u16 i, i32 def);
i32               script_arg_opt_num_clamped(ScriptBinderCall*, u16 i, i32 min, i32 max, i32 def);
ScriptArgNumRange script_arg_opt_num_range(ScriptBinderCall*, u16 i, ScriptArgNumRange def);
bool              script_arg_opt_bool(ScriptBinderCall*, u16 i, bool def);
StringHash        script_arg_opt_str(ScriptBinderCall*, u16 i, StringHash def);
u64               script_arg_opt_id(ScriptBinderCall*, u16 i, u64 def);
i32               script_arg_opt_enum(ScriptBinderCall*, u16 i, const ScriptEnum*, i32 def);

i32        script_arg_maybe_num(ScriptBinderCall*, u16 i, i32 def);
bool       script_arg_maybe_bool(ScriptBinderCall*, u16 i, bool def);
StringHash script_arg_maybe_str(ScriptBinderCall*, u16 i, StringHash def);
u64        script_arg_maybe_id(ScriptBinderCall*, u16 i, u64 def);
i32        script_arg_maybe_enum(ScriptBinderCall*, u16 i, const ScriptEnum*, i32 def);

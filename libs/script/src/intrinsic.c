#include "core/array.h"
#include "core/diag.h"
#include "script/intrinsic.h"

u32 script_intrinsic_arg_count(const ScriptIntrinsic i) {
  switch (i) {
  case ScriptIntrinsic_Break:
  case ScriptIntrinsic_Continue:
    return 0;
  case ScriptIntrinsic_Return:
  case ScriptIntrinsic_Assert:
  case ScriptIntrinsic_MemLoadDynamic:
  case ScriptIntrinsic_Invert:
  case ScriptIntrinsic_Absolute:
  case ScriptIntrinsic_Negate:
  case ScriptIntrinsic_Type:
  case ScriptIntrinsic_Hash:
  case ScriptIntrinsic_RangeMin:
  case ScriptIntrinsic_RangeMax:
    return 1;
  case ScriptIntrinsic_MemStoreDynamic:
  case ScriptIntrinsic_Add:
  case ScriptIntrinsic_Div:
  case ScriptIntrinsic_Equal:
  case ScriptIntrinsic_Greater:
  case ScriptIntrinsic_GreaterOrEqual:
  case ScriptIntrinsic_Less:
  case ScriptIntrinsic_LessOrEqual:
  case ScriptIntrinsic_LogicAnd:
  case ScriptIntrinsic_LogicOr:
  case ScriptIntrinsic_Mod:
  case ScriptIntrinsic_Mul:
  case ScriptIntrinsic_NotEqual:
  case ScriptIntrinsic_NullCoalescing:
  case ScriptIntrinsic_Sub:
  case ScriptIntrinsic_Min:
  case ScriptIntrinsic_Max:
  case ScriptIntrinsic_Clamp:
  case ScriptIntrinsic_Contains:
  case ScriptIntrinsic_RangeFromTo:
    return 2;
  case ScriptIntrinsic_Select:
    return 3;
  case ScriptIntrinsic_Loop:
    return 4;
  case ScriptIntrinsic_Count:
    break;
  }
  diag_assert_fail("Unknown intrinsic type");
  UNREACHABLE
}

u32 script_intrinsic_arg_count_always_reached(const ScriptIntrinsic i) {
  switch (i) {
  case ScriptIntrinsic_Select:         // Always reached args: condition.
  case ScriptIntrinsic_NullCoalescing: // Always reached args: lhs.
  case ScriptIntrinsic_LogicAnd:       // Always reached args: lhs.
  case ScriptIntrinsic_LogicOr:        // Always reached args: lhs.
    return 1;                          //
  case ScriptIntrinsic_Loop:           // Always reached args: setup, condition.
    return 2;                          //
  default:                             // Always reached args: all.
    return script_intrinsic_arg_count(i);
  }
}

bool script_intrinsic_deterministic(const ScriptIntrinsic i) {
  switch (i) {
  case ScriptIntrinsic_Continue:
  case ScriptIntrinsic_Break:
  case ScriptIntrinsic_Return:
  case ScriptIntrinsic_Assert:
  case ScriptIntrinsic_MemLoadDynamic:
  case ScriptIntrinsic_MemStoreDynamic:
    return false;
  default:
    return true;
  }
}

String script_intrinsic_str(const ScriptIntrinsic i) {
  diag_assert(i < ScriptIntrinsic_Count);
  static const String g_names[] = {
      [ScriptIntrinsic_Continue]        = string_static("continue"),
      [ScriptIntrinsic_Break]           = string_static("break"),
      [ScriptIntrinsic_Return]          = string_static("return"),
      [ScriptIntrinsic_Type]            = string_static("type"),
      [ScriptIntrinsic_Hash]            = string_static("hash"),
      [ScriptIntrinsic_Assert]          = string_static("assert"),
      [ScriptIntrinsic_MemLoadDynamic]  = string_static("mem-load-dynamic"),
      [ScriptIntrinsic_MemStoreDynamic] = string_static("mem-store-dynamic"),
      [ScriptIntrinsic_Select]          = string_static("select"),
      [ScriptIntrinsic_NullCoalescing]  = string_static("null-coalescing"),
      [ScriptIntrinsic_LogicAnd]        = string_static("logic-and"),
      [ScriptIntrinsic_LogicOr]         = string_static("logic-or"),
      [ScriptIntrinsic_Loop]            = string_static("loop"),
      [ScriptIntrinsic_Equal]           = string_static("equal"),
      [ScriptIntrinsic_NotEqual]        = string_static("not-equal"),
      [ScriptIntrinsic_Less]            = string_static("less"),
      [ScriptIntrinsic_LessOrEqual]     = string_static("less-or-equal"),
      [ScriptIntrinsic_Greater]         = string_static("greater"),
      [ScriptIntrinsic_GreaterOrEqual]  = string_static("greater-or-equal"),
      [ScriptIntrinsic_Add]             = string_static("add"),
      [ScriptIntrinsic_Sub]             = string_static("sub"),
      [ScriptIntrinsic_Mul]             = string_static("mul"),
      [ScriptIntrinsic_Div]             = string_static("div"),
      [ScriptIntrinsic_Mod]             = string_static("mod"),
      [ScriptIntrinsic_Negate]          = string_static("negate"),
      [ScriptIntrinsic_Invert]          = string_static("invert"),
      [ScriptIntrinsic_Absolute]        = string_static("absolute"),
      [ScriptIntrinsic_Min]             = string_static("min"),
      [ScriptIntrinsic_Max]             = string_static("max"),
      [ScriptIntrinsic_Clamp]           = string_static("clamp"),
      [ScriptIntrinsic_Contains]        = string_static("contains"),
      [ScriptIntrinsic_RangeMin]        = string_static("range-min"),
      [ScriptIntrinsic_RangeMax]        = string_static("range-max"),
      [ScriptIntrinsic_RangeFromTo]     = string_static("range-from-to"),
  };
  ASSERT(array_elems(g_names) == ScriptIntrinsic_Count, "Incorrect number of names");
  return g_names[i];
}

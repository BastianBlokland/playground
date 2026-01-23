#pragma once
#include "core/forward.h"

typedef enum eScriptIntrinsic {
  ScriptIntrinsic_Continue,        // Args: none.
  ScriptIntrinsic_Break,           // Args: none.
  ScriptIntrinsic_Return,          // Args: value.
  ScriptIntrinsic_Type,            // Args: value.
  ScriptIntrinsic_Hash,            // Args: value.
  ScriptIntrinsic_Assert,          // Args: condition.
  ScriptIntrinsic_MemLoadDynamic,  // Args: key.
  ScriptIntrinsic_MemStoreDynamic, // Args: key, value.
  ScriptIntrinsic_Select,          // Args: condition, if branch, else branch.
  ScriptIntrinsic_NullCoalescing,  // Args: lhs, rhs.
  ScriptIntrinsic_LogicAnd,        // Args: lhs, rhs.
  ScriptIntrinsic_LogicOr,         // Args: lhs, rhs.
  ScriptIntrinsic_Loop,            // Args: setup, condition, increment, body.
  ScriptIntrinsic_Equal,           // Args: lhs, rhs.
  ScriptIntrinsic_NotEqual,        // Args: lhs, rhs.
  ScriptIntrinsic_Less,            // Args: lhs, rhs.
  ScriptIntrinsic_LessOrEqual,     // Args: lhs, rhs.
  ScriptIntrinsic_Greater,         // Args: lhs, rhs.
  ScriptIntrinsic_GreaterOrEqual,  // Args: lhs, rhs.
  ScriptIntrinsic_Add,             // Args: lhs, rhs.
  ScriptIntrinsic_Sub,             // Args: lhs, rhs.
  ScriptIntrinsic_Mul,             // Args: lhs, rhs.
  ScriptIntrinsic_Div,             // Args: lhs, rhs.
  ScriptIntrinsic_Mod,             // Args: lhs, rhs.
  ScriptIntrinsic_Negate,          // Args: value.
  ScriptIntrinsic_Invert,          // Args: value.
  ScriptIntrinsic_Absolute,        // Args: value.
  ScriptIntrinsic_RangeMin,        // Args: value.
  ScriptIntrinsic_RangeMax,        // Args: value.
  ScriptIntrinsic_RangeFromTo,     // Args: min, max.
  ScriptIntrinsic_Clamp,           // Args: value, range.
  ScriptIntrinsic_Contains,        // Args: value, range.
  ScriptIntrinsic_Min,             // Args: x, y.
  ScriptIntrinsic_Max,             // Args: x, y.

  ScriptIntrinsic_Count,
} ScriptIntrinsic;

/**
 * Intrinsic traits.
 */
u32  script_intrinsic_arg_count(ScriptIntrinsic);
u32  script_intrinsic_arg_count_always_reached(ScriptIntrinsic);
bool script_intrinsic_deterministic(ScriptIntrinsic);

/**
 * Get a textual representation of the given intrinsic.
 */
String script_intrinsic_str(ScriptIntrinsic);

/**
 * Create a formatting argument for an intrinsic.
 */
#define script_intrinsic_fmt(_VAL_) fmt_text(script_intrinsic_str(_VAL_))

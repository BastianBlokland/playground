#include "core/alloc.h"
#include "core/array.h"
#include "core/diag.h"
#include "core/dynstring.h"

#include "panic.h"

// clang-format off
static const String g_panicStrs[] = {
    [ScriptPanic_None]                        = string_static("None"),
    [ScriptPanic_AssertionFailed]             = string_static("Script assertion failed"),
    [ScriptPanic_ExecutionFailed]             = string_static("Script execution failed"),
    [ScriptPanic_ExecutionLimitExceeded]      = string_static("Script execution limit exceeded"),
    [ScriptPanic_ArgumentInvalid]             = string_static("Argument {arg-index} invalid"),
    [ScriptPanic_ArgumentTypeMismatch]        = string_static("Argument {arg-index} expected '{type-mask}' got '{type-actual}'"),
    [ScriptPanic_ArgumentIdTypeMismatch]      = string_static("Argument {arg-index} mismatching id type"),
    [ScriptPanic_ArgumentIdExpired]           = string_static("Argument {arg-index} id expired"),
    [ScriptPanic_ArgumentMissing]             = string_static("Argument {arg-index} missing"),
    [ScriptPanic_ArgumentOutOfRange]          = string_static("Argument {arg-index} out of range"),
    [ScriptPanic_ArgumentCountExceedsMaximum] = string_static("Argument count exceeds maximum"),
    [ScriptPanic_EnumInvalidEntry]            = string_static("Invalid enum entry"),
    [ScriptPanic_UnimplementedBinding]        = string_static("Unimplemented binding"),
    [ScriptPanic_EventAlreadyFinished]        = string_static("Event already finished"),
    [ScriptPanic_EventInvalidParent]          = string_static("Event parent invalid"),
    [ScriptPanic_AttributeUnitNotFound]       = string_static("Unit not found in attributes"),
    [ScriptPanic_AttributePowerNotFound]      = string_static("Power not found in attributes"),
    [ScriptPanic_AttributePropulsionNotFound] = string_static("Propulsion not found in attributes"),
    [ScriptPanic_AttributeNotFound]           = string_static("Attribute not found"),
    [ScriptPanic_ArrayLimitReached]           = string_static("Array limit reached"),
    [ScriptPanic_ArrayIndexOutOfRange]        = string_static("Array index {context-int} is out of range"),
    [ScriptPanic_ArrayElementTypeMismatch]    = string_static("Array element {arg-index} expected '{type-mask}' got '{type-actual}'"),
    [ScriptPanic_ArrayElementIdTypeMismatch]  = string_static("Array element {arg-index} mismatching id type"),
    [ScriptPanic_IteratorLimitReached]        = string_static("Iterator limit reached"),
    [ScriptPanic_IteratorFilterLimitReached]  = string_static("Iterator filter limit reached"),
};
// clang-format on
ASSERT(array_elems(g_panicStrs) == ScriptPanicKind_Count, "Incorrect number of kind strs");

typedef enum {
  PanicReplKind_ArgIndex,
  PanicReplKind_TypeMask,
  PanicReplKind_TypeActual,
  PanicReplKind_ContextInt,
} PanicReplKind;

typedef struct {
  usize         start, end;
  PanicReplKind kind;
} PanicRepl;

static PanicReplKind panic_replacement_parse(const String str) {
  if (string_eq(str, string_lit("arg-index"))) {
    return PanicReplKind_ArgIndex;
  }
  if (string_eq(str, string_lit("type-mask"))) {
    return PanicReplKind_TypeMask;
  }
  if (string_eq(str, string_lit("type-actual"))) {
    return PanicReplKind_TypeActual;
  }
  if (string_eq(str, string_lit("context-int"))) {
    return PanicReplKind_ContextInt;
  }
  diag_crash_msg("Unsupported panic replacement");
}

static bool panic_replacement_find(String str, PanicRepl* result) {
  const usize startIdx = string_find_first_char(str, '{');
  if (sentinel_check(startIdx)) {
    return false;
  }
  const usize len = string_find_first_char(mem_consume(str, startIdx), '}');
  diag_assert(!sentinel_check(len));

  *result = (PanicRepl){
      .start = startIdx,
      .end   = startIdx + len + 1,
      .kind  = panic_replacement_parse(string_slice(str, startIdx + 1, len - 1)),
  };
  return true;
}

NORETURN void script_panic_raise(ScriptPanicHandler* handler, const ScriptPanic panic) {
  handler->result = panic;
  longjmp(handler->anchor, true /* hasPanic */);
}

void script_panic_write(
    DynString* out, const ScriptPanic* panic, const ScriptPanicOutputFlags flags) {
  diag_assert(panic->kind != ScriptPanic_None && panic->kind < ScriptPanicKind_Count);

  if (flags & ScriptPanicOutput_IncludeRange) {
    fmt_write(
        out,
        "{}:{}-{}:{}: ",
        fmt_int(panic->range.start.line + 1),
        fmt_int(panic->range.start.column + 1),
        fmt_int(panic->range.end.line + 1),
        fmt_int(panic->range.end.column + 1));
  }

  String str = g_panicStrs[panic->kind];
  while (!string_is_empty(str)) {
    PanicRepl repl;
    if (!panic_replacement_find(str, &repl)) {
      // No replacement, append the text verbatim.
      dynstring_append(out, str);
      break;
    }

    // Append the text before the replacement verbatim.
    dynstring_append(out, string_slice(str, 0, repl.start));

    switch (repl.kind) {
    case PanicReplKind_ArgIndex:
      format_write_int(out, panic->argIndex);
      break;
    case PanicReplKind_TypeMask:
      dynstring_append(out, script_mask_scratch(panic->typeMask));
      break;
    case PanicReplKind_TypeActual:
      dynstring_append(out, script_val_type_str(panic->typeActual));
      break;
    case PanicReplKind_ContextInt:
      format_write_int(out, panic->contextInt);
      break;
    }

    str = mem_consume(str, repl.end);
  }
}

String script_panic_scratch(const ScriptPanic* diag, const ScriptPanicOutputFlags flags) {
  Mem       bufferMem = alloc_alloc(g_allocScratch, usize_kibibyte, 1);
  DynString buffer    = dynstring_create_over(bufferMem);

  script_panic_write(&buffer, diag, flags);

  return dynstring_view(&buffer);
}

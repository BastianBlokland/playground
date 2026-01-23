#pragma once
#include "core/array.h"
#include "script/forward.h"
#include "script/panic.h"

#define script_prog_regs 38

// clang-format off

/**
 * Code operation.
 *
 * Doc format:
 * - '[]' represents data part of the operation itself.
 * - '()' represents registers that are read or written by the operation.
 *
 * Operation data:
 * - op-code:        1 byte(s).
 * - instruction:    2 byte(s).
 * - register-id:    1 byte(s).
 * - register-count: 1 byte(s).
 * - extern-func:    2 byte(s).
 * - value-id:       1 byte(s).
 * - boolean         1 byte(s).
 * - small-int       1 byte(s).
 * - memory-key:     4 byte(s).
 *
 * NOTE: Multi-byte operation data is encoded as little-endian.
 * NOTE: There is no alignment requirement for operation data.
 * NOTE: Instruction values are 2 byte offsets from the start of the code memory.
 */
typedef enum eScriptOp {
  ScriptOp_Fail              = 0,  // [       ] (       ) -> ( ) Terminate the execution.
  ScriptOp_Assert            = 1,  // [s      ] (s      ) -> ( ) Terminate the execution if register 's' is falsy.
  ScriptOp_Return            = 2,  // [s      ] (s      ) -> ( ) Return register 's'.
  ScriptOp_ReturnNull        = 3,  // [       ] (       ) -> ( ) Return value null.
  ScriptOp_Move              = 4,  // [d,s    ] (s      ) -> (d) Load value at register 's' into register 'd'.
  ScriptOp_Jump              = 5,  // [i      ] (       ) -> ( ) Jump to instruction 'i'.
  ScriptOp_JumpIfTruthy      = 6,  // [r,i    ] (r      ) -> ( ) Jump to instruction 'i' if register 'r' is truthy.
  ScriptOp_JumpIfFalsy       = 7,  // [r,i    ] (r      ) -> ( ) Jump to instruction 'i' if register 'r' is falsy.
  ScriptOp_JumpIfNonNull     = 8,  // [r,i    ] (r      ) -> ( ) Jump to instruction 'i' if register 'r' is not null.
  ScriptOp_Value             = 9,  // [d,v    ] (       ) -> (d) Load value with index 'v' into register 'd'.
  ScriptOp_ValueNull         = 10, // [d      ] (       ) -> (d) Load null value into register 'd'.
  ScriptOp_ValueBool         = 11, // [d,b    ] (       ) -> (d) Load value boolean 'b' into register 'd'.
  ScriptOp_ValueSmallInt     = 12, // [d,i    ] (       ) -> (d) Load small integer value 'i' into register 'd'.
  ScriptOp_MemLoad           = 13, // [d,k    ] (       ) -> (d) Load from memory at key 'k' into register 'd'.
  ScriptOp_MemStore          = 14, // [s,k    ] (s      ) -> ( ) Store to memory at key 'k' from register 's'.
  ScriptOp_MemLoadDyn        = 15, // [d      ] (d      ) -> (d) Load from memory with a key from register 'd'.
  ScriptOp_MemStoreDyn       = 16, // [s,r    ] (s,r    ) -> ( ) Store a value from register 's' to memory with a key from register 'r'.
  ScriptOp_Extern            = 17, // [d,f,r,c] (r:c    ) -> (d) Invoke extern func 'f' using count 'c' registers starting from 'r' and store result in register 'd'.
  ScriptOp_Truthy            = 18, // [d      ] (d      ) -> (d) Check if register 'd' is truthy.
  ScriptOp_Falsy             = 19, // [d      ] (d      ) -> (d) Check if register 'd' is falsy.
  ScriptOp_NonNull           = 20, // [d      ] (d      ) -> (d) Check if register 'd' is non-null.
  ScriptOp_Type              = 21, // [d      ] (d      ) -> (d) Retrieve the type for register 'd'.
  ScriptOp_Hash              = 22, // [d      ] (d      ) -> (d) Retrieve the hash for register 'd'.
  ScriptOp_Equal             = 23, // [d,s    ] (d,s    ) -> (d) Compare 'd' and 's' and store result in register 'd'.
  ScriptOp_Less              = 24, // [d,s    ] (d,s    ) -> (d) Compare 'd' and 's' and store result in register 'd'.
  ScriptOp_Greater           = 25, // [d,s    ] (d,s    ) -> (d) Compare 'd' and 's' and store result in register 'd'.
  ScriptOp_Add               = 26, // [d,s    ] (d,s    ) -> (d) Add register 's' to 'd'.
  ScriptOp_Sub               = 27, // [d,s    ] (d,s    ) -> (d) Subtract register 's' from 'd'.
  ScriptOp_Mul               = 28, // [d,s    ] (d,s    ) -> (d) Multiply register 'd' by register 's'.
  ScriptOp_Div               = 29, // [d,s    ] (d,s    ) -> (d) Divide register 'd' by register 's'.
  ScriptOp_Mod               = 30, // [d,s    ] (d,s    ) -> (d) Modulo register 'd' by register 's'.
  ScriptOp_Negate            = 31, // [d      ] (d      ) -> (d) Negate register 'd'.
  ScriptOp_Invert            = 32, // [d      ] (d      ) -> (d) Invert register 'd'.
  ScriptOp_Absolute          = 33, // [d      ] (d      ) -> (d) Compute the absolute for register 'd'.
  ScriptOp_RangeMin          = 34, // [d      ] (d      ) -> (d) Retrieve the min component of a range in register 'd'.
  ScriptOp_RangeMax          = 35, // [d      ] (d      ) -> (d) Retrieve the max component of a range in register 'd'.
  ScriptOp_RangeFromTo       = 36, // [x,y    ] (x,y    ) -> (x) Compose a range from min 'x' and max 'y' and store in register 'x'.
  ScriptOp_Clamp             = 37, // [x,y    ] (x,y    ) -> (x) Clamp register 'x' between range 'y' and store in register 'x'.
  ScriptOp_Contains          = 38, // [x,y    ] (x,y    ) -> (x) Check if register 'x' is contained in value 'y' and store in register 'x'.
  ScriptOp_Min               = 39, // [x,y    ] (x,y    ) -> (x) Store the minimum value of 'x' and 'y' in register 'x'.
  ScriptOp_Max               = 40, // [x,y    ] (x,y    ) -> (x) Store the maximum value of 'x' and 'y' in register 'x'.
} ScriptOp;

// clang-format on

typedef struct {
  u16                instruction; // Offset in the code stream.
  ScriptRangeLineCol range;
} ScriptProgramLoc;

typedef struct sScriptProgram {
  struct {
    bool  external;
    void* ptr;
    usize size;
  } code; // Instruction stream (struct layout compatible with DataMem).
  ScriptBinderHash binderHash;
  HeapArray_t(ScriptVal) literals;
  HeapArray_t(ScriptProgramLoc) locations; // Sorted on instruction.
} ScriptProgram;

typedef struct {
  u32         executedOps;
  ScriptPanic panic;
  ScriptVal   val;
} ScriptProgResult;

void script_prog_destroy(ScriptProgram*, Allocator*);
void script_prog_clear(ScriptProgram*, Allocator*);

/**
 * Evaluate the program.
 * Pre-condition: script_prog_validate(program, binder).
 */
ScriptProgResult
script_prog_eval(const ScriptProgram*, ScriptMem*, const ScriptBinder*, void* bindCtx);

/**
 * Validate the given program.
 */
bool script_prog_validate(const ScriptProgram*, const ScriptBinder*);

/**
 * Lookup the source range for the given call-identifier.
 */
ScriptRangeLineCol script_prog_location(const ScriptProgram*, u32 callId);

/**
 * Write the program disassembly for diagnostic purposes.
 */
void   script_prog_write(const ScriptProgram*, DynString* out);
String script_prog_write_scratch(const ScriptProgram*);

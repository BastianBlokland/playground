#include "check/spec.h"
#include "core/array.h"
#include "core/stringtable.h"
#include "script/val.h"

#include "utils.h"

spec(val) {
  it("can type-erase values") {
    check_eq_int(script_type(script_null()), ScriptType_Null);

    check_eq_int(script_type(script_num(42)), ScriptType_Num);
    check_eq_float(script_get_num(script_num(42), 0), 42, 1e-6);

    check_eq_int(script_type(script_bool(true)), ScriptType_Bool);
    check(script_get_bool(script_bool(true), false) == true);

    check_eq_int(script_type(script_num_range_from_to(-2, -1)), ScriptType_NumRange);
    check_eq_int(script_get_num_range_min(script_num_range_from_to(-2, -1), 0), -2);

    const ScriptVal str = script_str(string_hash_lit("Hello World"));
    check_eq_int(script_type(str), ScriptType_Str);
    check(script_get_str(str, 0) == string_hash_lit("Hello World"));

    const ScriptVal str2 = script_str_empty();
    check_eq_int(script_type(str2), ScriptType_Str);
    check(script_get_str(str2, 0) == string_hash_lit(""));
  }

  it("can extract specific types from values") {
    check_eq_float(script_get_num(script_num(42), 1337), 42, 1e-6);
    check_eq_float(script_get_num(script_null(), 1337), 1337, 1e-6);
    check_eq_float(script_get_num(script_bool(false), 1337), 1337, 1e-6);

    check(script_get_bool(script_bool(true), false) == true);
    check(script_get_bool(script_null(), false) == false);

    const ScriptVal str = script_str(string_hash_lit("Hello World"));
    check(script_get_str(str, 42) == string_hash_lit("Hello World"));
    check(script_get_str(script_null(), 42) == 42);
  }

  it("can test if a value is truthy") {
    check(!script_truthy(script_null()));

    check(!script_truthy(script_bool(false)));
    check(script_truthy(script_bool(true)));

    check(script_truthy(script_num(0)));
    check(script_truthy(script_num(-1)));
    check(script_truthy(script_num(42)));

    check(script_truthy(script_str(0)));
    check(script_truthy(script_str(string_hash_lit("Hello World"))));
  }

  it("can test if a value is falsy") {
    check(script_falsy(script_null()));
    check(script_falsy(script_bool(false)));
    check(!script_falsy(script_bool(true)));

    check(!script_falsy(script_num(0)));
    check(!script_falsy(script_num(42)));

    check(!script_falsy(script_str(0)));
    check(!script_falsy(script_str(string_hash_lit("Hello World"))));
  }

  it("can test if a value is not null") {
    check(script_non_null(script_num(42)));
    check(!script_non_null(script_null()));
  }

  it("can return a default if the value is null") {
    check_eq_val(script_val_or(script_num(42), script_num(1337)), script_num(42));
    check_eq_val(script_val_or(script_num(42), script_null()), script_num(42));
    check_eq_val(script_val_or(script_null(), script_num(1337)), script_num(1337));
    check_eq_val(script_val_or(script_null(), script_null()), script_null());
  }

  it("can produce a textual representation for a type") {
    check_eq_string(script_val_type_str(ScriptType_Null), string_lit("null"));
    check_eq_string(script_val_type_str(ScriptType_Num), string_lit("num"));
    check_eq_string(script_val_type_str(ScriptType_NumRange), string_lit("range"));
    check_eq_string(script_val_type_str(ScriptType_Bool), string_lit("bool"));
    check_eq_string(script_val_type_str(ScriptType_Str), string_lit("str"));
  }

  it("can produce a hash for a value type") {
    check_eq_int(script_val_type_hash(ScriptType_Null), string_hash_lit("null"));
    check_eq_int(script_val_type_hash(ScriptType_Num), string_hash_lit("num"));
    check_eq_int(script_val_type_hash(ScriptType_NumRange), string_hash_lit("range"));
    check_eq_int(script_val_type_hash(ScriptType_Bool), string_hash_lit("bool"));
    check_eq_int(script_val_type_hash(ScriptType_Str), string_hash_lit("str"));
  }

  it("can lookup a type from its string-hash") {
    check_eq_int(script_val_type_from_hash(string_hash_lit("null")), ScriptType_Null);
    check_eq_int(script_val_type_from_hash(string_hash_lit("num")), ScriptType_Num);
    check_eq_int(script_val_type_from_hash(string_hash_lit("range")), ScriptType_NumRange);
    check_eq_int(script_val_type_from_hash(string_hash_lit("bool")), ScriptType_Bool);
    check_eq_int(script_val_type_from_hash(string_hash_lit("str")), ScriptType_Str);

    check_eq_int(script_val_type_from_hash(string_hash_lit("")), ScriptType_Null);
    check_eq_int(script_val_type_from_hash(string_hash_lit("hello-world")), ScriptType_Null);
  }

  it("can create a textual representation of a value") {
    const struct {
      ScriptVal value;
      String    expected;
    } testData[] = {
        {script_null(), string_lit("null")},
        {script_num(42), string_lit("42")},
        {script_num(i32_max), string_lit("2147483647")},
        {script_bool(true), string_lit("true")},
        {script_bool(false), string_lit("false")},
        {script_str(string_hash_lit("Hello World")), string_lit("Hello World")},
    };

    // NOTE: Normally we expect the script lexer to register the strings.
    stringtable_add(g_stringtable, string_lit("Hello World"));

    for (u32 i = 0; i != array_elems(testData); ++i) {
      check_eq_string(script_val_scratch(testData[i].value), testData[i].expected);
    }
  }

  it("can create a textual representation of a mask") {
    const struct {
      ScriptMask mask;
      String     expected;
    } testData[] = {
        {script_mask_none, string_lit("none")},
        {script_mask_any, string_lit("any")},
        {script_mask_null, string_lit("null")},
        {script_mask_num, string_lit("num")},
        {script_mask_num_range, string_lit("range")},
        {script_mask_bool, string_lit("bool")},
        {script_mask_str, string_lit("str")},
        {script_mask_null | script_mask_num, string_lit("num?")},
        {
            script_mask_null | script_mask_num | script_mask_str,
            string_lit("null | num | str"),
        },
        {
            script_mask_null | script_mask_num | script_mask_str | script_mask_num_range,
            string_lit("null | num | range | str"),
        },
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      check_eq_string(script_mask_scratch(testData[i].mask), testData[i].expected);
    }
  }

  it("can test if values are equal") {
    const struct {
      ScriptVal a, b;
      bool      expected;
    } testData[] = {
        {script_null(), script_null(), .expected = true},
        {script_null(), script_num(42), .expected = false},
        {script_num(42), script_null(), .expected = false},

        {script_num(42), script_num(42), .expected = true},
        {script_num(-42), script_num(-42), .expected = true},

        {script_num_range_from_to(-42, 1337),
         script_num_range_from_to(-42, 1337),
         .expected = true},
        {script_num_range_from_to(1337, -42),
         script_num_range_from_to(-42, 1337),
         .expected = true},
        {script_num_range_from_to(0, 5), script_num_range_from_to(-42, 1337), .expected = false},

        {script_bool(true), script_bool(true), .expected = true},
        {script_bool(false), script_bool(false), .expected = true},
        {script_bool(false), script_bool(true), .expected = false},

        {script_num(1), script_bool(true), .expected = false},

        {script_str(string_hash_lit("A")), script_null(), .expected = false},
        {
            script_str(string_hash_lit("A")),
            script_str(string_hash_lit("A")),
            .expected = true,
        },
        {
            script_str(string_hash_lit("A")),
            script_str(string_hash_lit("B")),
            .expected = false,
        },
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      if (testData[i].expected) {
        check_eq_val(testData[i].a, testData[i].b);
      } else {
        check_neq_val(testData[i].a, testData[i].b);
      }
    }
  }

  it("can test if values are less") {
    const struct {
      ScriptVal a, b;
      bool      expected;
    } testData[] = {
        {script_null(), script_null(), .expected = false},
        {script_null(), script_num(42), .expected = false},
        {script_num(42), script_null(), .expected = false},

        {script_num(1), script_num(2), .expected = true},
        {script_num(2), script_num(1), .expected = false},
        {script_num(1), script_num(1), .expected = false},

        {script_bool(true), script_bool(true), .expected = false},
        {script_bool(false), script_bool(false), .expected = false},
        {script_bool(true), script_bool(false), .expected = false},
        {script_bool(false), script_bool(true), .expected = true},

        {script_num(1), script_bool(true), .expected = false},

        {
            script_str(string_hash_lit("A")),
            script_str(string_hash_lit("B")),
            .expected = false,
        },
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      if (testData[i].expected) {
        check_less_val(testData[i].a, testData[i].b);
      } else {
        check_msg(
            !script_val_less(testData[i].a, testData[i].b),
            "{} >= {}",
            fmt_text(script_val_scratch(testData[i].a)),
            fmt_text(script_val_scratch(testData[i].b)));
      }
    }
  }

  it("can test if values are greater") {
    const struct {
      ScriptVal a, b;
      bool      expected;
    } testData[] = {
        {script_null(), script_null(), .expected = false},
        {script_null(), script_num(42), .expected = false},
        {script_num(42), script_null(), .expected = false},

        {script_num(2), script_num(1), .expected = true},
        {script_num(1), script_num(2), .expected = false},
        {script_num(1), script_num(1), .expected = false},

        {script_bool(true), script_bool(false), .expected = true},
        {script_bool(true), script_bool(true), .expected = false},
        {script_bool(false), script_bool(false), .expected = false},
        {script_bool(false), script_bool(true), .expected = false},

        {script_num(1), script_bool(true), .expected = false},

        {
            script_str(string_hash_lit("A")),
            script_str(string_hash_lit("B")),
            .expected = false,
        },
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      if (testData[i].expected) {
        check_greater_val(testData[i].a, testData[i].b);
      } else {
        check_msg(
            !script_val_greater(testData[i].a, testData[i].b),
            "{} <= {}",
            fmt_text(script_val_scratch(testData[i].a)),
            fmt_text(script_val_scratch(testData[i].b)));
      }
    }
  }

  it("can negate values") {
    const struct {
      ScriptVal val;
      ScriptVal expected;
    } testData[] = {
        {script_null(), .expected = script_null()},
        {script_num(42), .expected = script_num(-42)},
        {script_bool(true), .expected = script_null()},
        {script_str(string_hash_lit("A")), .expected = script_null()},
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      const ScriptVal actual = script_val_neg(testData[i].val);
      check_eq_val(actual, testData[i].expected);
    }
  }

  it("can invert values") {
    const struct {
      ScriptVal val;
      ScriptVal expected;
    } testData[] = {
        {script_null(), .expected = script_bool(true)},
        {script_num(42), .expected = script_bool(false)},
        {script_bool(true), .expected = script_bool(false)},
        {script_bool(false), .expected = script_bool(true)},
        {script_str(string_hash_lit("A")), .expected = script_bool(false)},
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      const ScriptVal actual = script_val_inv(testData[i].val);
      check_eq_val(actual, testData[i].expected);
    }
  }

  it("can add values") {
    const struct {
      ScriptVal a, b;
      ScriptVal expected;
    } testData[] = {
        {script_null(), script_null(), .expected = script_null()},
        {script_null(), script_num(42), .expected = script_null()},
        {script_num(42), script_null(), .expected = script_null()},
        {script_num(42), script_bool(false), .expected = script_null()},

        {script_num(42), script_num(1), .expected = script_num(43)},
        {script_num(42), script_num(1337), .expected = script_num(1379)},

        {script_bool(true), script_bool(false), .expected = script_null()},

        {
            script_str(string_hash_lit("A")),
            script_str(string_hash_lit("B")),
            .expected = script_null(),
        },
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      const ScriptVal actual = script_val_add(testData[i].a, testData[i].b);
      check_eq_val(actual, testData[i].expected);
    }
  }

  it("can subtract values") {
    const struct {
      ScriptVal a, b;
      ScriptVal expected;
    } testData[] = {
        {script_null(), script_null(), .expected = script_null()},
        {script_null(), script_num(42), .expected = script_null()},
        {script_num(42), script_null(), .expected = script_null()},
        {script_num(42), script_bool(false), .expected = script_null()},

        {script_num(42), script_num(1), .expected = script_num(41)},
        {script_num(42), script_num(1337), .expected = script_num(-1295)},

        {script_bool(true), script_bool(false), .expected = script_null()},

        {
            script_str(string_hash_lit("A")),
            script_str(string_hash_lit("B")),
            .expected = script_null(),
        },
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      const ScriptVal actual = script_val_sub(testData[i].a, testData[i].b);
      check_eq_val(actual, testData[i].expected);
    }
  }

  it("can multiply values") {
    const struct {
      ScriptVal a, b;
      ScriptVal expected;
    } testData[] = {
        {script_null(), script_null(), .expected = script_null()},
        {script_null(), script_num(42), .expected = script_null()},
        {script_num(42), script_null(), .expected = script_null()},
        {script_num(42), script_bool(false), .expected = script_null()},

        {script_num(42), script_num(2), .expected = script_num(84)},
        {script_num(42), script_num(1337), .expected = script_num(56154)},

        {script_bool(true), script_bool(false), .expected = script_null()},

        {
            script_str(string_hash_lit("A")),
            script_str(string_hash_lit("B")),
            .expected = script_null(),
        },
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      const ScriptVal actual = script_val_mul(testData[i].a, testData[i].b);
      check_eq_val(actual, testData[i].expected);
    }
  }

  it("can divide values") {
    const struct {
      ScriptVal a, b;
      ScriptVal expected;
    } testData[] = {
        {script_null(), script_null(), .expected = script_null()},
        {script_null(), script_num(42), .expected = script_null()},
        {script_num(42), script_null(), .expected = script_null()},
        {script_num(42), script_bool(false), .expected = script_null()},

        {script_num(42), script_num(2), .expected = script_num(21)},

        {script_bool(true), script_bool(false), .expected = script_null()},

        {
            script_str(string_hash_lit("A")),
            script_str(string_hash_lit("B")),
            .expected = script_null(),
        },
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      const ScriptVal actual = script_val_div(testData[i].a, testData[i].b);
      check_eq_val(actual, testData[i].expected);
    }
  }

  it("can compute the modulo of values") {
    const struct {
      ScriptVal a, b;
      ScriptVal expected;
    } testData[] = {
        {script_null(), script_null(), .expected = script_null()},
        {script_null(), script_num(42), .expected = script_null()},
        {script_num(42), script_null(), .expected = script_null()},
        {script_num(42), script_bool(false), .expected = script_null()},

        {script_num(42), script_num(1), .expected = script_num(0)},
        {script_num(42), script_num(2), .expected = script_num(0)},
        {script_num(42), script_num(42), .expected = script_num(0)},
        {script_num(42), script_num(4), .expected = script_num(2)},
        {script_num(42), script_num(43), .expected = script_num(42)},
        {script_num(42), script_num(-1), .expected = script_num(0)},
        {script_num(42), script_num(-43), .expected = script_num(42)},

        {script_num(-42), script_num(1), .expected = script_num(0)},
        {script_num(-42), script_num(2), .expected = script_num(0)},
        {script_num(-42), script_num(42), .expected = script_num(0)},
        {script_num(-42), script_num(4), .expected = script_num(-2)},
        {script_num(-42), script_num(43), .expected = script_num(-42)},
        {script_num(-42), script_num(43), .expected = script_num(-42)},
        {script_num(-42), script_num(-1), .expected = script_num(0)},
        {script_num(-42), script_num(-43), .expected = script_num(-42)},

        {
            script_str(string_hash_lit("A")),
            script_str(string_hash_lit("B")),
            .expected = script_null(),
        },
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      const ScriptVal actual = script_val_mod(testData[i].a, testData[i].b);
      check_eq_val(actual, testData[i].expected);
    }
  }

  it("can perform contain checks") {
    const struct {
      ScriptVal val, range;
      ScriptVal expected;
    } testData[] = {
        {script_null(), script_null(), .expected = script_null()},
        {script_null(), script_num_range_from_to(0, 42), .expected = script_null()},
        {script_num(42), script_null(), .expected = script_null()},

        {script_num(0), script_num_range_from_to(0, 0), .expected = script_bool(true)},
        {script_num(-1), script_num_range_from_to(0, 0), .expected = script_bool(false)},
        {script_num(-1), script_num_range_from_to(-2, 0), .expected = script_bool(true)},
        {script_num(0), script_num_range_from_to(-2, 0), .expected = script_bool(true)},
        {script_num(-3), script_num_range_from_to(-2, 0), .expected = script_bool(false)},
        {script_num(1), script_num_range_from_to(-2, 0), .expected = script_bool(false)},
        {script_num(1), script_num_range_from_to(1, 1), .expected = script_bool(true)},
        {script_num(2), script_num_range_from_to(1, 2), .expected = script_bool(true)},

        {script_num_range_from_to(0, 0),
         script_num_range_from_to(0, 0),
         .expected = script_bool(true)},
        {script_num_range_from_to(0, 1),
         script_num_range_from_to(0, 0),
         .expected = script_bool(false)},
        {script_num_range_from_to(1, 2),
         script_num_range_from_to(1, 3),
         .expected = script_bool(true)},
        {script_num_range_from_to(2, 3),
         script_num_range_from_to(1, 4),
         .expected = script_bool(true)},
    };

    for (u32 i = 0; i != array_elems(testData); ++i) {
      const ScriptVal actual = script_val_contains(testData[i].val, testData[i].range);
      check_eq_val(actual, testData[i].expected);
    }
  }
}

#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Checks the bindings against the C API they wrap, since neither the
# C# nor the Godot toolchain runs in this repository's CI:
#
# - every C# DllImport entry point and every m3 call in the Godot
#   sources names a public function with that many parameters;
# - every C# struct has the size and the field offsets of the C struct
#   it mirrors (sequential layout, computed here; the C side printed by
#   the C compiler);
# - the C# version is the headers' version, and the C# AbiVersion
#   expression, which may name only VersionMajor, VersionMinor and
#   VersionPatch, compiled as C, gives M3_ABI_VERSION at that version and
#   at every version in ABI_SAMPLES, so a release that moves the version
#   in both files keeps them matching and a formula that changes on one
#   side only fails.
#
# usage: check_bindings.py [C compiler, default cc]

import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSHARP = os.path.join(ROOT, "bindings", "csharp", "Maul3D.cs")
# Versions both ABI formulas are evaluated at, besides the current one:
# both sides of 1.0 and of the major, minor and patch boundaries.
VERSION_NAMES = {"VersionMajor", "VersionMinor", "VersionPatch"}
ABI_SAMPLES = [(0, 0, 1), (0, 1, 0), (0, 3, 7), (1, 0, 0), (1, 2, 3), (2, 15, 4)]
GODOT = os.path.join(ROOT, "bindings", "godot", "src")

PRIMITIVES = {"byte": 1, "sbyte": 1, "bool": 1, "short": 2, "ushort": 2, "int": 4, "uint": 4,
              "float": 4, "long": 8, "ulong": 8, "double": 8, "IntPtr": 8}


def split_top(text):
    """Top-level comma-separated parts of a parameter list."""
    parts, depth, current = [], 0, ""
    for c in text:
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        if c == "," and depth == 0:
            parts.append(current)
            current = ""
        else:
            current += c
    if current.strip():
        parts.append(current)
    return [p.strip() for p in parts if p.strip()]


def c_api():
    """Public function name -> parameter count."""
    api = {}
    include = os.path.join(ROOT, "include", "maul3d")
    for name in os.listdir(include):
        text = open(os.path.join(include, name)).read()
        for m in re.finditer(r"M3_API\s+[^;(]*?\b(m3\w+)\s*\(([^)]*)\)\s*;", text):
            params = split_top(m.group(2))
            api[m.group(1)] = 0 if params in ([], ["void"]) else len(params)
        for m in re.finditer(r"static inline \w+ (m3\w+)\(([^)]*)\)\n", text):
            params = split_top(m.group(2))
            api[m.group(1)] = 0 if params in ([], ["void"]) else len(params)
    return api


def call_arities(text):
    """(name, argument count) for every m3 call in C++ source text."""
    out = []
    for m in re.finditer(r"\b(m3[A-Z]\w*)\(", text):
        depth, j = 1, m.end()
        while depth and j < len(text):
            depth += {"(": 1, ")": -1}.get(text[j], 0)
            j += 1
        out.append((m.group(1), len(split_top(text[m.end():j - 1]))))
    return out


def csharp_structs(text):
    """C# struct name -> [(field, type)] in declaration order."""
    structs = {}
    for m in re.finditer(r"public struct (\w+)\s*\{", text):
        depth, j = 1, m.end()
        while depth:
            depth += {"{": 1, "}": -1}.get(text[j], 0)
            j += 1
        body = text[m.end():j - 1]
        fields = []
        for line in body.split(";"):
            line = re.sub(r"//[^\n]*", "", line).strip()
            d = re.match(r"public (\w+) ([\w\s,]+)$", line.split("\n")[-1].strip())
            if d and "(" not in line and "=>" not in line:
                fields += [(n.strip(), d.group(1)) for n in d.group(2).split(",")]
        structs[m.group(1)] = fields
    return structs


def layout(name, structs):
    """(size, alignment, [(field, offset)]) under sequential layout."""
    offset, align, offsets = 0, 1, []
    for field, ftype in structs[name]:
        if ftype in PRIMITIVES:
            size = falign = PRIMITIVES[ftype]
        elif ftype in structs:
            size, falign, _ = layout(ftype, structs)
        else:
            size = falign = 4  # an enum : int
        offset = (offset + falign - 1) // falign * falign
        offsets.append((field, offset))
        offset += size
        align = max(align, falign)
    return (offset + align - 1) // align * align, align, offsets


def c_member(field, fields):
    """The C member a C# field mirrors: camelCase, and a C array
    flattened into fields numbered from 0 (Axis0, Axis1) maps back to
    its elements (axis[0], axis[1])."""
    m = re.match(r"(\w*?[A-Za-z])(\d)$", field)
    if m and (m.group(1) + "0") in [f for f, _ in fields]:
        stem = m.group(1)
        return f"{stem[0].lower()}{stem[1:]}[{m.group(2)}]"
    return field[0].lower() + field[1:]


def c_layouts(names, structs, compiler, abi_expression, versions):
    """What the C compiler says about the same structs, and M3_ABI_VERSION
    next to the C# AbiVersion expression at each version."""
    lines = ['#include "maul3d/maul3d.h"', "#include <stddef.h>", "#include <stdio.h>",
             "static uint32_t CsAbi(uint32_t VersionMajor, uint32_t VersionMinor,",
             "                      uint32_t VersionPatch)", "{",
             f"    return {abi_expression};", "}",
             "int main(void)", "{",
             '    printf("major %u\\n", (unsigned)M3_VERSION_MAJOR);',
             '    printf("minor %u\\n", (unsigned)M3_VERSION_MINOR);',
             '    printf("patch %u\\n", (unsigned)M3_VERSION_PATCH);']
    for name in names:
        c = "m3" + name
        lines.append(f'    printf("{name} size %zu\\n", sizeof({c}));')
        for field, _ in structs[name]:
            member = c_member(field, structs[name])
            lines.append(f'    printf("{name}.{field} %zu\\n", offsetof({c}, {member}));')
    for major, minor, patch in versions:
        # M3_ABI_VERSION expands where it is used, so it follows the
        # version macros redefined here.
        lines += ["#undef M3_VERSION_MAJOR", "#undef M3_VERSION_MINOR", "#undef M3_VERSION_PATCH",
                  f"#define M3_VERSION_MAJOR {major}", f"#define M3_VERSION_MINOR {minor}",
                  f"#define M3_VERSION_PATCH {patch}",
                  f'    printf("abi {major}.{minor}.{patch} %u,%u\\n", (unsigned)M3_ABI_VERSION, '
                  f"(unsigned)CsAbi({major}, {minor}, {patch}));"]
    lines += ["    return 0;", "}"]
    with tempfile.TemporaryDirectory() as tmp:
        source = os.path.join(tmp, "layout.c")
        binary = os.path.join(tmp, "layout")
        with open(source, "w") as f:
            f.write("\n".join(lines) + "\n")
        subprocess.run([compiler, "-I", os.path.join(ROOT, "include"), source, "-o", binary],
                       check=True)
        result = subprocess.run([binary], check=True, capture_output=True, text=True).stdout
    return dict(line.rsplit(" ", 1) for line in result.strip().split("\n"))


def main():
    compiler = sys.argv[1] if len(sys.argv) > 1 else "cc"
    api = c_api()
    failures = []
    text = open(CSHARP).read()
    for m in re.finditer(r'EntryPoint = "(m3\w+)"\)\]\s*(?:\[[^\]]*\]\s*)*public static extern '
                         r"[\w.]+ \w+\(([^)]*)\)", text):
        name, params = m.group(1), split_top(m.group(2))
        if name not in api:
            failures.append(f"C#: {name} is not a public function")
        elif len(params) != api[name]:
            failures.append(f"C#: {name} takes {api[name]} arguments, the import {len(params)}")
    for source in sorted(os.listdir(GODOT)):
        for name, count in call_arities(open(os.path.join(GODOT, source)).read()):
            if name not in api:
                failures.append(f"Godot {source}: {name} is not a public function")
            elif count != api[name]:
                failures.append(f"Godot {source}: {name} takes {api[name]} arguments, "
                                f"called with {count}")
    structs = csharp_structs(text)
    checked = [n for n in re.findall(r"Check<(\w+)>\(\d+\)", text)]
    missing = [n for n in structs if n not in checked]
    if missing:
        failures.append("C#: structs without a layout check: " + ", ".join(missing))
    numbers = []
    for part in ("Major", "Minor", "Patch"):
        version = re.search(rf"public const uint Version{part} = (\d+);", text)
        numbers.append(int(version.group(1)) if version else 0)
        if version is None:
            failures.append(f"C#: no Version{part}")
    abi = re.search(r"public const uint AbiVersion =\s*([^;]+);", text)
    expression = " ".join(abi.group(1).split()) if abi else "0"
    # Only the three version names may appear: any other name would
    # resolve to the header in the C probe (M3_ABI_VERSION would compare
    # the header with itself).
    others = sorted(set(re.findall(r"\b[A-Za-z_]\w*", expression)) - VERSION_NAMES)
    compiled = abi is not None and not others and re.fullmatch(r"[\w\s|&^<>=!?:()+*-]+",
                                                              expression)
    if abi is None or not compiled:
        failures.append("C#: no AbiVersion expression this check can compile"
                        + (": it names " + ", ".join(others) if others else ""))
        expression = "0"
    versions = [tuple(numbers)] + [v for v in ABI_SAMPLES if v != tuple(numbers)]
    c = c_layouts(sorted(structs), structs, compiler, expression, versions)
    for name in sorted(structs):
        size, _, offsets = layout(name, structs)
        if int(c[f"{name} size"]) != size:
            failures.append(f"C#: {name} is {size} bytes, m3{name} {c[name + ' size']}")
        for field, offset in offsets:
            if int(c[f"{name}.{field}"]) != offset:
                failures.append(f"C#: {name}.{field} at {offset}, C at {c[name + '.' + field]}")
    for m in re.finditer(r"Check<(\w+)>\((\d+)\)", text):
        if m.group(1) in structs and layout(m.group(1), structs)[0] != int(m.group(2)):
            failures.append(f"C#: LayoutCheck expects {m.group(1)} at {m.group(2)} bytes")
    for part, number in zip(("major", "minor", "patch"), numbers):
        if str(number) != c[part]:
            failures.append(f"C#: Version{part.title()} is not M3_VERSION_{part.upper()} "
                            f"({c[part]})")
    for major, minor, patch in versions if compiled else []:
        expected, actual = (int(v) for v in c[f"abi {major}.{minor}.{patch}"].split(","))
        if actual != expected:
            failures.append(f"C#: AbiVersion at {major}.{minor}.{patch} is {actual:#x}, "
                            f"M3_ABI_VERSION {expected:#x}")
    for failure in failures:
        print(failure)
    if not failures:
        print("bindings match the C API")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

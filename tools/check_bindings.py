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
#   the C compiler).
#
# usage: check_bindings.py [C compiler, default cc]

import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSHARP = os.path.join(ROOT, "bindings", "csharp", "Maul3D.cs")
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


def c_layouts(names, structs, compiler):
    """What the C compiler says about the same structs."""
    lines = ['#include "maul3d/maul3d.h"', "#include <stddef.h>", "#include <stdio.h>",
             "int main(void)", "{"]
    for name in names:
        c = "m3" + name
        lines.append(f'    printf("{name} size %zu\\n", sizeof({c}));')
        for field, _ in structs[name]:
            member = c_member(field, structs[name])
            lines.append(f'    printf("{name}.{field} %zu\\n", offsetof({c}, {member}));')
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
    c = c_layouts(sorted(structs), structs, compiler)
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
    for failure in failures:
        print(failure)
    if not failures:
        print("bindings match the C API")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

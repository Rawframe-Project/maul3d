#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The checks, in three tiers. Each tier includes the one before it.
#
#   tools/check.sh changed   the files changed against origin/main: the
#                            source rules, formatting, the bindings when the
#                            API or a binding changed, clang-tidy on the
#                            changed sources, and the linux-clang-20-asan
#                            preset (assertions on) for the tests the
#                            changes reach. Under two minutes on a warm
#                            build; run it before every push.
#   tools/check.sh pr        what the pull request check runs on this
#                            machine: every rule, formatting of every file,
#                            the bindings, the documentation site,
#                            clang-tidy on every source, and the
#                            linux-gcc-14 and linux-clang-20-asan presets.
#   tools/check.sh full      also every other Linux preset (Clang Release,
#                            ThreadSanitizer, shared with the exports test,
#                            32-bit x86 with GCC and Clang) and the bench
#                            scenes.
#
# The builds are the CI cells' CMake presets (CMakePresets.json), in
# out/<preset>; the other trees go to out/check-*. Needs gcc-14, clang-20,
# clang-tidy-20, ninja and the clang-format version CI pins; the
# documentation site also needs python3-markdown, and full needs 32-bit
# libraries for the i686 presets. The organization's CI image has them
# all. ThreadSanitizer does not start inside docker unless the container
# runs with --security-opt seccomp=unconfined. The org documentation check
# runs when the organization's .github checkout is at ORG_DEFAULTS
# (default: ../../.github from the repository). On a shared machine, run
# it through the workspace's tools/heavy.sh.
# JOBS sets the parallel jobs, at most CMAKE_BUILD_PARALLEL_LEVEL when that
# is set; the default is 8.
set -euo pipefail

tier=${1:-changed}
case "$tier" in
changed | pr | full) ;;
*)
    echo "usage: tools/check.sh [changed|pr|full]" >&2
    exit 2
    ;;
esac

cd "$(dirname "$0")/.."
root=$PWD
out=$root/out
mkdir -p "$out"
org=${ORG_DEFAULTS:-$root/../../.github}
# Parallel jobs: JOBS, else CMAKE_BUILD_PARALLEL_LEVEL, else 8, and never
# more than CMAKE_BUILD_PARALLEL_LEVEL when it is set, since a shared
# machine sets it as the cap.
jobs=${JOBS:-${CMAKE_BUILD_PARALLEL_LEVEL:-8}}
cap=${CMAKE_BUILD_PARALLEL_LEVEL:-$jobs}
for n in "$jobs" "$cap"; do
    [[ "$n" =~ ^[1-9][0-9]*$ ]] || {
        echo "JOBS and CMAKE_BUILD_PARALLEL_LEVEL must be positive integers" >&2
        exit 2
    }
done
[ "$jobs" -le "$cap" ] || jobs=$cap

step() { printf '\n== %s\n' "$*"; }

# A CI cell: its configure, build and test presets. More ctest arguments
# (a label filter) follow the preset's name.
preset() {
    local name=$1
    shift
    step "preset $name, $jobs jobs"
    cmake --preset "$name" >"$out/$name.log"
    cmake --build --preset "$name" -j "$jobs" >>"$out/$name.log"
    ctest --preset "$name" -j "$jobs" "$@"
}

# The ctest labels a changed path reaches; "all" when it reaches every
# suite. Nearly every library source does: the golden scenes step the
# whole engine.
labels_for() {
    case "$1" in
    *.md | LICENSE | .github/* | docs/* | bindings/* | tools/*.py | tools/*.txt | tools/check.sh) ;;
    bench/* | test/test_bench.c) ;; # the bench scenes run in Release only: tools/check.sh full
    test/test_golden.c) echo "golden golden_hostile" ;;
    test/test_*.c) [ ! -f "$1" ] || basename "$1" .c | sed 's/^test_//' ;;
    *) echo all ;;
    esac
}

source_checks() {
    step "source, module, length, float environment and snippet rules"
    python3 tools/check_source.py
    python3 tools/check_modules.py
    python3 tools/check_lengths.py
    python3 tools/check_float_env.py
    python3 tools/check_guide.py
    if [ -f "$org/scripts/docs_check.py" ]; then
        python3 "$org/scripts/docs_check.py"
    else
        echo "skipped the organization's documentation check: no checkout at $org"
    fi
}

bindings() {
    step "the bindings match the C API"
    python3 tools/check_bindings.py clang-20
}

tidy() { # files
    step "clang-tidy on $# files"
    cmake -S "$root" -B "$out/check-tidy" -G Ninja -DCMAKE_C_COMPILER=clang-20 \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DMAUL3D_BUILD_TESTS=OFF >"$out/check-tidy.log"
    printf '%s\n' "$@" | xargs -P "$jobs" -n 1 clang-tidy-20 -p "$out/check-tidy" --quiet \
        --warnings-as-errors='*' --checks=-readability-function-cognitive-complexity 2>/dev/null
}

# The files CI formats, untracked ones included, since they are about to
# be committed: the bindings keep their own languages' styles.
all_c_files() {
    git ls-files --cached --others --exclude-standard 'include/*.h' 'src/*.[ch]' 'test/*.[ch]' \
        'tools/*.[ch]' 'bench/*.[ch]' 'samples/*.[ch]' 'testbed/*.[ch]'
}

# Whether CI formats this path: the same globs, where * crosses folders
# as it does in git's pathspecs.
formatted() {
    case "$1" in
    include/*.h | src/*.[ch] | test/*.[ch] | tools/*.[ch] | bench/*.[ch] | samples/*.[ch] | \
        testbed/*.[ch]) return 0 ;;
    *) return 1 ;;
    esac
}

if [ "$tier" = changed ]; then
    # Against HEAD the diff would hold only uncommitted work and check
    # nothing that is committed.
    if ! git rev-parse --verify --quiet origin/main >/dev/null; then
        echo "origin/main is missing: run git fetch origin main first" >&2
        exit 2
    fi
    base=$(git merge-base HEAD origin/main)
    mapfile -t changed < <({
        git diff --name-only "$base"
        git ls-files --others --exclude-standard
    } | sort -u)
    if [ ${#changed[@]} -eq 0 ]; then
        echo "nothing changed against origin/main"
        exit 0
    fi
    source_checks
    present=()
    for f in "${changed[@]}"; do
        if [ -f "$f" ] && formatted "$f"; then
            present+=("$f")
        fi
    done
    if [ ${#present[@]} -gt 0 ]; then
        step "clang-format on ${#present[@]} changed files"
        clang-format --dry-run -Werror "${present[@]}"
    fi
    for f in "${changed[@]}"; do
        case "$f" in
        include/maul3d/* | bindings/*)
            bindings
            break
            ;;
        esac
    done
    sources=()
    for f in "${present[@]+"${present[@]}"}"; do
        case "$f" in src/*.c) sources+=("$f") ;; esac
    done
    if [ ${#sources[@]} -gt 0 ]; then
        tidy "${sources[@]}"
    fi
    labels=""
    for f in "${changed[@]}"; do labels="$labels $(labels_for "$f")"; done
    if [ -z "${labels// /}" ]; then
        echo "no test reached by the changes"
        exit 0
    fi
    if [[ " $labels " == *" all "* ]]; then
        preset linux-clang-20-asan
    else
        regex=$(tr ' ' '\n' <<<"$labels" | grep -v '^$' | sort -u | paste -sd '|')
        echo "tests labeled $regex"
        preset linux-clang-20-asan -L "^($regex)\$"
    fi
    exit 0
fi

source_checks

step "clang-format on every file"
mapfile -t files < <(all_c_files)
# An empty list would make clang-format read stdin and pass.
[ ${#files[@]} -gt 0 ] || {
    echo "no files to format" >&2
    exit 1
}
clang-format --dry-run -Werror "${files[@]}"

bindings

step "the documentation site builds"
if python3 -c 'import markdown' 2>/dev/null; then
    python3 tools/build_docs.py
else
    echo "skipped: python3-markdown is not installed (the CI image has it)"
fi

mapfile -t sources < <(git ls-files 'src/*.c')
tidy "${sources[@]}"

preset linux-gcc-14
preset linux-clang-20-asan

[ "$tier" = full ] || exit 0

for name in linux-clang-20 linux-clang-20-tsan linux-shared linux-i686-gcc-14 linux-i686-clang-20 \
    linux-gcc-14-bench; do
    preset "$name"
done

step "all checks passed"

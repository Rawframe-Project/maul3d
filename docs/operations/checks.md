# Checks

What verifies a change, locally and in CI.

## Locally

`tools/check.sh` runs the checks in three tiers; each includes the one
before it. Its builds are the CI cells' presets from `CMakePresets.json`,
in `out/<preset>`, and its other trees go to `out/check-*`.

| Tier | Runs |
|---|---|
| `changed` | On the files changed against `origin/main`: the source and test hygiene rules, formatting, the bindings when the API or a binding changed, clang-tidy on the changed sources (on every source when a header under `src/` or `include/` changed), the `linux-clang-20-asan` preset for the tests the changes reach, and five seconds of each fuzz target when the library code, a build file or a fuzz target changed. Under two minutes on a warm build. |
| `pr` | What the pull request check runs on one machine: every rule, formatting of every file, the bindings, the documentation site, clang-tidy on every source, the `linux-gcc-14` and `linux-clang-20-asan` presets, and ten seconds of each fuzz target. |
| `full` | Also every other Linux preset (Clang Release, ThreadSanitizer, the shared library with the exports test, 32-bit x86 with GCC and Clang), the bench scenes and a minute of each fuzz target. |

Run `changed` before every push and `pr` before opening a pull request.
On the shared development machine, run `pr` and `full` through the
workspace's heavy-job wrapper. clang-tidy runs on the Release build,
where the assertions compile out, and on the Debug build, where they are
live, since each hides findings the other shows; its header filter comes
from `tools/tidy_filter.sh`.

## In CI

`.github/workflows/ci.yml` runs on every pull request, every push to
`main`, every night and on dispatch. A new push to a pull request cancels
its running CI; runs on `main` are never cancelled. The Linux jobs run in
the organization's CI image, pinned by digest. The ruleset requires only
the `required` job, which passes when every other job passed or was
skipped, and the title check.

| Job | What it guards |
|---|---|
| `changes` | Whether anything beyond documentation changed; when nothing did, the build jobs are skipped. |
| `docs` | The organization's documentation check, from the pinned commit of `.github`. |
| `zizmor` | The workflows: injection, persisted credentials, permissions, unpinned actions. |
| `checks` | `clang-format`; the family's source, documentation, module, length, snippet and floating-point environment checks; test hygiene over the sources and over the registered tests of the default and the shared configurations; the bindings against the C API; that the documentation site builds; and clang-tidy on the Release and the Debug build. It runs on documentation changes too, since the snippet check and the site read `docs/`. |
| `build` | One call of the organization's `cpp.yml` per preset: GCC 14 on x64 and arm64 Linux, Clang 20, Clang with AddressSanitizer and UBSan on the scalar backend (each test repeated up to three times on a pull request), Clang with ThreadSanitizer, the shared library with the package test, 32-bit x86 with GCC 14 and with Clang 20, Clang on arm64 macOS, and `clang-cl` on x64 and arm64 Windows. Each builds with warnings as errors and runs the whole suite, the header tests and the golden scenes included. |
| `bench` | On `main` and every night, never on a pull request: the benchmark scenes in Release on the six native 64-bit cells, with their hashes pinned like the golden scenes. The release gate reads its last run on `main`. |
| `msvc-headers` | The public headers alone under MSVC's `cl`, as C17 and C++17 with `/W4 /WX`. |
| `web` | The library and its tests under Emscripten with threads, run by Node, the golden scenes included. On a pull request and a push to `main`, the plain unit tests link at `-O1` (`MAUL3D_WEB_UNIT_LINK`), while the suites that hold results to committed values or two runs to the same bits link at the build's level: `golden`, `bench`, `warmhold`, `math`, `base`, `concurrency`, `draw`, `introspection`, `journal` and `replayfile`. wasm-opt rewrites the library's code at link time too, so a unit test linked lower runs a module a Release link does not give. The release pull request, every night and on dispatch, every test links at the build's level. |
| `fuzz` | On a pull request and a push to `main`: each fuzz target, seeded from the shared scene and from the newest nightly corpus, restored read-only, for 20 seconds on a pull request and 90 on `main`. |
| `fuzz-targets`, `fuzz-nightly` | Every night and on dispatch: each target for ten minutes from its corpus of the nights before, in a job of its own, saving the corpus for the next night. |
| `testbed` | On `main` and every night: the interactive testbed builds against raylib. |
| `prune-caches` | On `main` only, after every job that saves a cache: the organization's `cache-prune.yml` deletes each cache entry a newer save on `main` replaced, so the per-commit ccache entries never crowd out the fuzz corpus. |

The `build`, `bench`, `fuzz`, `web` and `testbed` jobs restore ccache in
every run and save it only from `main`. Besides `ci.yml`:

- `pr-title.yml` checks that the pull request title is a conventional
  commit with one of the scopes it lists, and refuses a breaking marker.
- `pr-gate.yml` closes a pull request from outside the organization
  unless it closes an issue labeled `accepted`.
- `codeql.yml` runs CodeQL on pull requests, on `main` and on a schedule,
  in the CI image.
- `scorecard.yml` grades the repository's security practices for the
  README badge, and Dependabot proposes weekly updates of the pinned
  actions.
- `pages.yml` publishes the documentation site from `main`, and
  `release.yml` is [the release](release.md).

## Test hygiene

`tools/check_test_hygiene.py`, shared by the family, rejects sleeps,
wall-clock reads, retry loops and skips in the test sources, and skip
properties in the CMake files. A use that must stay carries
`// hygiene: <reason>` on its line or the one before. With `--ctest` it
also checks the registered tests, and `--self-test` runs it over the
fixtures in `tools/test_hygiene/`. A test never skips at run time: a
suite that cannot run on a platform or in a configuration is left out
when CMake configures, as `maul_add_test`'s `POSIX` flag does on Windows
and its `WHITEBOX` flag in the shared build.

## Fuzzing

Each fuzz target checks promises that must hold for any input, and
crashes when one breaks:

- `fuzz_snapshot` restores any bytes: they are refused with
  `m3_errorFormat` or `m3_errorVersion` and leave the world as it was,
  or a world restored from them writes a snapshot that restores into a
  fresh world and writes the same bytes again.
- `fuzz_journal` replays any bytes as a tape into a fresh world: they are
  refused and leave the world as it was, or they replay into a second
  fresh world on the same hash, and that world steps on.
- `fuzz_replay` decodes any bytes as a replay container: they are
  refused, or decode to a view that encodes back to the same bytes, whose
  snapshot and journal are then restored and replayed or refused.
- `fuzz_restore_step` restores any bytes over the shared scene: a world
  that restore accepts must step, answer queries and snapshot like one
  the API built, and its snapshot must restore in turn.

`fuzz_seeds` writes their seeds from the shared scene, so they always
match the formats the targets read.

## Determinism in CI

Every cell runs the golden scenes against the values compiled in from
`test/hashes.txt`, so a result that differs on one platform, compiler,
build type or backend fails that cell. The sanitizer cell runs the
scalar backend, the x64 cells AVX2 and the arm64 cells NEON, and all of
them must give the same bits. [Development](development.md#golden-values)
says how the values are regenerated.

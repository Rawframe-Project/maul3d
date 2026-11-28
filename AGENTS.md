# Maul3D

Maul3D is a deterministic 3D physics engine for games in C: rigid bodies, joints, characters, vehicles, soft bodies, voxel destruction and water, in a world that snapshots, restores and replays to the bit. It is part of the Maul family; the family rules are in [docs/internals/maul-conventions.md](docs/internals/maul-conventions.md), and [docs/internals/overview.md](docs/internals/overview.md) explains how the parts fit.

It never compromises on:

1. **Same bits everywhere.** The same calls reach the same bits on every platform, compiler, build type, SIMD backend and worker count. Every CI cell checks the golden values in `test/hashes.txt`.
2. **Exact rollback and replay.** A restore continues exactly as the original run; a journal replays onto the same bits, or is refused and changes nothing.
3. **Hostile input.** Every argument, snapshot, tape and container is checked and refused with a status in every build, before anything changes. A restored or replayed world holds only values the API could have built. Untrusted bytes never count as misuse.
4. **Fixed memory.** A world takes its pools and step scratch from its def's allocator when it is created; the step never allocates.

## Commands

```sh
tools/check.sh changed          # before every push: rules, format, the asan preset for the tests the change reaches (< 2 min)
tools/check.sh pr               # what the pull request check runs (< 10 min)
tools/check.sh full             # adds the other Linux presets, the bench scenes and a minute of fuzzing
cmake --preset linux-clang-20-asan && cmake --build --preset linux-clang-20-asan   # one CI cell
ctest --preset linux-clang-20-asan -L joints   # one suite, by label
```

`git config core.hooksPath tools/hooks` makes every push run the `changed` tier first. Run the `pr` and `full` tiers through the workspace's heavy-job wrapper on the shared machine. Every CI cell is a preset in `CMakePresets.json`; add a cell there, never as commands in a workflow. Each suite is `test/test_<label>.c`, registered with `maul_add_test` in `test/CMakeLists.txt` and labeled with its name. [docs/operations/development.md](docs/operations/development.md) has the toolchain, the build options and how to regenerate the golden values.

## Ways to hurt yourself

- **Killing by pattern.** Never `pkill -f`, `pgrep | kill` or `kill` a PID found by matching a name or path: other sessions build and test on the same machine with the same binary names. Kill only a PID you started.
- **The machine's GCC.** A system's default GCC 13 cannot build C23, and `cmake/CompilerFlags.cmake` refuses it. Use `gcc-14` or `clang-20`, as the presets do.
- **Another clang-format.** Only the version the CI image pins formats as CI checks; format with that one.
- **ThreadSanitizer in the CI image.** It does not start inside the container unless it runs with `--security-opt seccomp=unconfined`; run the TSan preset on the host.
- **Regenerating golden values to make a test pass.** A moved line in `test/hashes.txt` is a determinism bug until a pull request says which change moved it and why, and the scalar and AVX2 builds agree on the new value. The bench lines are checked only on `main` and nightly, so a change to the step runs the `linux-gcc-14-bench` preset before its pull request. From 1.0.0 such a change is titled `feat`, or `fix` when it corrects a documented result, and [docs/operations/release.md](docs/operations/release.md) says when it bumps `M3_SOLVER_REV`.
- **Editing the version.** `M3_VERSION_*` in `include/maul3d/base.h` and the version in `bindings/csharp/Maul3D.cs` sit between release-please markers; only the release pull request changes them.
- **Editing shared family files.** These are synced from the canonical copies in the organization's `.github` repository (`maul/family_sync.py`) and fail CI when they differ: `docs/internals/maul-conventions.md`; `.clang-format`, `.clang-tidy`, `.editorconfig`, `.gitattributes`, `.gitignore`; `cmake/CompilerFlags.cmake`, `cmake/Config.cmake.in`, `cmake/Install.cmake`, `cmake/Library.cmake`, `cmake/pkg-config.pc.in`, `cmake/ReadVersion.cmake`, `cmake/Testing.cmake`, `cmake/HeaderCheck/CMakeLists.txt`; `bench/CMakeLists.txt`; `src/cpu.c`, `src/cpu.h`, `src/fpenv.c`, `src/fpenv.h`, `src/hash.c`, `src/simd.h`; `test/test_fenv.h`, `test/test_fpenv.c`, `test/test_harness.h`; `tools/hooks/pre-push`, `tools/test_hygiene/`; and `tools/build_docs.py`, `check_docs.py`, `check_exports.py`, `check_float_env.py`, `check_guide.py`, `check_lengths.py`, `check_modules.py`, `check_nodiscard.py`, `check_source.py`, `check_test_hygiene.py`, `fuzz_target.sh`, `gen_api.py`, `run_quarantine.sh`, `tidy_filter.sh`. Change the canonical copy first.

## Invariants

- **Floating point:** no fast math and no contraction (`cmake/CompilerFlags.cmake`); fused multiply-add only through `src/simd.h`. `src/` calls only the `<math.h>` functions in `tools/libm-allowed.txt`, all correctly rounded; the angle functions are the engine's own. Min, max and abs go through `m3MinF`, `m3MaxF`, `m3AbsF`.
- **Floating-point environment:** every public function enters the library's environment (`src/fpenv.h`), except those listed in `tools/float-env-allowed.txt` under the reason they compute no floats; a getter that only returns stored floats goes there, in order. `tools/check_float_env.py` checks both ways.
- **Order:** every pass walks slots in ascending order, pairs stay sorted by key, ties break to the lower index, freed slots are reused in FIFO order. Parallel work writes only per-item slots, so the split never changes results.
- **State:** every array a world owns is a row of the state table in `src/world_state.c`, which allocates, snapshots and restores it, with the check restore runs on it. `m3World_Hash` is the curated walk in `src/world_hash.c`. A new array goes into the table with its check, and into the hash when the step reads it, or rollback or the hash silently breaks.
- **Restore and replay:** a snapshot or tape value is held to the same rule as the setter or create that makes it, the slots a type does not use included (`src/snapshot_values.c`, `src/snapshot_bounds.c`, `src/joint.c`). A new setter rule goes into restore and replay in the same pull request.
- **Formats:** a change to the snapshot's bytes bumps `M3_SNAPSHOT_VERSION`, to a tape's `M3_JOURNAL_VERSION`, to a replay container's `M3_REPLAY_VERSION` (`src/state_prefix.h`). `src/format_layout.c` pins the layout of every recorded struct; changing one is a format change. The golden `ROLLBACK_SNAPSHOT` and `JOURNAL_TAPE` lines show both. Every public writer records itself in the journal (`m3JournalRecord`) after its checks pass, and every record replays through `src/journal_replay*.c`.
- **Ids and misuse:** a stale id counts one misuse; untrusted bytes and refusals the world's state decides count none. Refuse through `m3Refuse` and the list helpers in `src/misuse.h`. Every writer first checks that no host code is running, so a callback cannot change the world. `docs/internals/overview.md` has the full contract.
- **Getters:** take `const m3World*`; a null world reads the null values and counts nothing; a stale id or the wrong kind reads them and counts one misuse. A list getter takes a buffer and its capacity, returns the total, measures with a null buffer and capacity 0, and clears the buffer when it refuses.
- **Version:** `M3_VERSION_*` in `include/maul3d/base.h`; CMake and `M3_ABI_VERSION` read it, and `tools/check_bindings.py` holds the C# binding's copy to it.
- **Public headers** are the common subset of C17 and C++17 and declare only the API; every public function has a `///` block with `@param`, `@return` and a thread safety paragraph that `tools/check_docs.py` accepts. Writers say "a world is used by one thread at a time", readers add "or by several readers at once".
- **Bindings:** `tools/check_bindings.py` checks every C# entry point and struct in `bindings/csharp/Maul3D.cs` and every call in `bindings/godot/` against the C API; a changed signature or struct changes the binding in the same pull request.
- **Modules:** `tools/modules.txt` lists which module may include which. A new include needs a line there.
- **Lengths and complexity:** a function past 80 lines or a file past 1000 lines is split, or listed in `tools/length-exceptions.txt` with a ceiling and a reason; the list only shrinks. A function kept above clang-tidy's complexity threshold carries a `NOLINT` with its reason.
- **Return values:** every internal function returning `bool` carries `M3_NODISCARD` (`tools/check_nodiscard.py`); a call that means to drop one says so with `(void)`.
- **Assertions:** `M3_ASSERT` (`src/core.h`) checks internal invariants only, never caller input; it traps without `NDEBUG` and compiles out of Release.
- **Memory:** `src/allocator.c` is the only file that reaches `malloc`; everything else takes memory from the world's allocator.
- **Workflows:** every action is pinned by commit SHA with its version in a comment, the CI image by digest, and each job declares the least `permissions` it needs.

## Testing

Follow the [repository standard's Tests section](https://github.com/Rawframe-Project/.github/blob/main/docs/repository-standard.md#tests); in this repository:

- Test through the public headers. `test/helpers.h` wraps status calls for scene setup; a test about why a call refuses calls the public function and checks its status and the misuse count. A suite that must reach `src/` is marked `WHITEBOX`, and the shared build leaves it out.
- A promised output is pinned by a golden scene in `test/test_golden.c`, not by a new hash in a suite. Prefer extending a scene to adding one.
- Determinism is tested with twins: the same calls in two worlds (serial and on the task hooks, recorded and replayed, rolled back and not) must reach the same `m3World_Hash`.
- A new check in restore or replay gets a forged snapshot or tape one value past the bound, refused with `m3_errorFormat` and the world left as it was, next to the value at the bound, accepted.
- A new getter with a setter joins the value oracle in `test/fuzz_oracle.h`, so the fuzzers check that every restored value reads back into its setter.
- A bug fix comes with a test that fails without the fix.
- Every C snippet of `docs/user/` appears unchanged in `test/test_guide.c`; `tools/check_guide.py` enforces it.

## Documentation

Follow the [repository standard's Documentation section](https://github.com/Rawframe-Project/.github/blob/main/docs/repository-standard.md#documentation). A change in what a caller must do or can rely on updates the header's `///` block and the page of `docs/user/` for that part, and the matching code in `test/test_guide.c` when it is in a snippet. The API reference is generated from the headers into the documentation site; never commit it.

## Pull requests

Titles are conventional commits. Scopes: `world`, `body`, `shape`, `collision`, `solver`, `joint`, `vehicle`, `character`, `softbody`, `voxel`, `water`, `query`, `event`, `snapshot`, `journal`, `replay`, `simd`, `draw`, `bindings`, `tools`, `testbed`, `samples`, `bench`, `build`, `family`. Example: `fix(joint): restore refuses a spherical target the setter refuses`.

A change of results names the golden lines it moves and why. A change to the snapshot, tape or container bytes names the format version it bumps.

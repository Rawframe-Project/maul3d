# Maul3D

A deterministic 3D physics engine for games in C17, with no
dependencies and an MIT license, built for destruction and lockstep
multiplayer: the whole world snapshots, restores and replays to the
bit on every platform CI covers: x64 and arm64 Linux and Windows,
arm64 macOS, and WebAssembly.

- [The guide](guide.html): the engine-host contract, chapter by
  chapter: rollback, the journal, destruction, vehicles, the
  character controller, soft bodies, water, heightfields, the
  replay studio, and lockstep networking.
- [API reference](api.html): every public function, generated from
  the headers.
- [Samples](samples.html): small complete programs, from a falling
  stack to a snapshot round-trip to a skid-steer tank.
- [The changelog](changelog.html): every release and what changed.
- [The repository](https://github.com/Rawframe-Project/maul3d):
  source, releases, and the test suites that keep the promises.

## The promises

1. Same inputs, same bits, every platform, every thread count.
2. Snapshot, restore, journal replay: bit-exact, always.
3. Bad input is refused with a reason; it never corrupts quietly.

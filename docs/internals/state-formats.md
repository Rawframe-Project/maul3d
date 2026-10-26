# State formats

maul3d writes three state formats: the snapshot (`m3World_Snapshot`), the journal tape (`m3World_StartJournal` to `m3World_StopJournal`) and the replay container (`m3EncodeReplay`). None of them is read across releases: only the release that wrote the bytes reads them, and any other refuses them with `m3_errorVersion`. A host keeps assets, not state, across an upgrade. maul3d writes no asset format.

## The state prefix

Every state format starts with the same 16 bytes, shared with maul2d, each a little-endian `uint32_t`:

| Offset | Field | Meaning |
|---|---|---|
| 0 | `magic` | Four ASCII bytes naming the library and the format. |
| 4 | `abiVersion` | The writer's `M3_ABI_VERSION`. A reader refuses any other value with `m3_errorVersion`. |
| 8 | `formatVersion` | The format's own number, bumped whenever its bytes change. A reader refuses any other value with `m3_errorVersion`. |
| 12 | `headerBytes` | The size of the whole header, this prefix and the format's own fields, a multiple of 8. The content starts there. A reader refuses any other value with `m3_errorInvalid`. |

A reader checks in this order:
- Bytes shorter than the prefix, or with another magic, are not this format: `m3_errorInvalid`.
- Another ABI version or format number means another release wrote them: `m3_errorVersion`.
- Another header size, or bytes too short to hold the header, are corrupt: `m3_errorInvalid`.
- The format's own fields follow. Another build hash is `m3_errorVersion`; another world shape is `m3_errorInvalid`.

None of these refusals counts as misuse. A header embedded in another, such as a replay container's snapshot, is checked the same way, and the outer reader returns its refusal.

The golden lines that pin a format's bytes hash them from offset 8, with each embedded header's `abiVersion` zeroed. A separate check confirms that each of those words is this release's, so a release bump moves no line.

## The headers

`src/state_prefix.h` holds each format's magic, number and header size, and `m3MakeStatePrefix` and `m3CheckStatePrefix` write and check the prefix for all three. The magic is written as a `uint32_t` whose hexadecimal spelling is the name, `0x4D33534E` for `M3SN`, as maul2d writes its own.

| Format | Magic | Number | Header | After the prefix |
|---|---|---|---|---|
| Snapshot | `M3SN` | 5 | 56 bytes | `configHash` (`uint64_t`) at 16, then the eight capacities (`int32_t`) at 24: bodies, shapes, meshes, joints, voxels, characters, vehicles, soft bodies |
| Journal tape | `M3JN` | 1 | 16 bytes | nothing; the records start at 16 |
| Replay container | `M3RP` | 3 | 40 bytes | `finalHash` (`uint64_t`) at 16, then `snapshotBytes`, `journalBytes`, `stepCount` and `opCount` (`int32_t`) at 24, then the snapshot and the tape, each with its own header |

The snapshot's `configHash` covers what the bytes mean within one release: the format number, the solver revision, the sizes of `m3real` and of a position, and the floating-point policy. A snapshot restores only into a world with the same eight capacities.

A tape's records follow its prefix, each an `int32_t` op, an `int32_t` payload size and the payload. `m3World_StartJournal` writes the prefix when it starts, so a session of no op is 16 bytes, and a run of records cut at record boundaries behind that prefix is a tape too.

A container's reader checks its own prefix, then its sizes, then the embedded snapshot's prefix and the tape's prefix and records. The rest of the snapshot is checked by `m3World_Restore` when it is restored.

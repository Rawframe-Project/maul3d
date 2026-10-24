# Release

Releases are made by release-please from the conventional commit titles on `main`. Versions follow Semantic Versioning; before 1.0.0 any release may change the API, the ABI and the snapshot and journal formats, and a breaking change bumps the minor version. Tags are `vX.Y.Z`.

## Steps

1. release-please keeps a draft pull request titled `chore: release X.Y.Z` open. It moves the version between the markers in `include/maul3d/base.h`, where CMake reads it and from which `M3_ABI_VERSION` follows, and in `bindings/csharp/Maul3D.cs`, whose `AbiVersion` follows the same way. It updates `.release-please-manifest.json` and adds an entry to `CHANGELOG.md` from the `feat`, `fix` and `perf` titles.
2. Rewrite that entry in user language before merging: what changed for someone using the library, grouped as Added, Changed, Fixed, Removed and Security. Leave out refactors, tests, CI and documentation. Say what a caller must change for a breaking change. Put the same entry in the pull request's body between release-please's `---` lines, since the GitHub release notes come from the body. release-please rewrites both whenever anything else lands on `main`, so this is the last change before the merge.
3. Check that the `required` CI job passed on the release pull request on every cell, and that the last `bench` run on `main` passed, since the benchmark scenes run on `main` and every night, not on pull requests.
4. Mark the pull request ready and merge it. release-please tags the merge commit and publishes the GitHub release with the changelog entry as its notes. The Pages workflow publishes the documentation from `main`.
5. The `assets` job of `.github/workflows/release.yml` attaches the source archive `maul3d-X.Y.Z.tar.gz`, its `SHA256SUMS`, an SPDX SBOM and a build provenance attestation. Check that they are there. A user verifies a download with `sha256sum -c --ignore-missing SHA256SUMS` (the file also lists the SBOM) and `gh attestation verify maul3d-X.Y.Z.tar.gz -R Rawframe-Project/maul3d`.

## Notes

- A version that is not the next one by the commit titles is set with a `Release-As: X.Y.Z` footer in the commit that should produce it.
- Before 1.0 every release has its own ABI: the shared library's `SOVERSION` is the whole version, the CMake package accepts only the exact version, and `m3CreateWorld` refuses a def stamped by another release's headers with `m3_errorVersion`. `tools/check_bindings.py` checks that the C# binding carries the headers' version.
- The shared family files must match their canonical copies in the organization's `.github` repository before a release.
- Tags are never deleted or moved. A broken release is fixed by the next one.

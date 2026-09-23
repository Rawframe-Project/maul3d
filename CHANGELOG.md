# Changelog

All notable changes to this project are recorded here. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/). Before
1.0.0, any minor release may change the API, the ABI and the
snapshot and journal formats.

## [Unreleased]

Work toward 0.0.1, the first release of the reworked library.

### Changed

- The version history restarts at 0.0.1. Earlier numbered releases
  were withdrawn.

### Fixed

- A step that stalled on a full pair table or starved scratch leaked
  the previous step's pair and manifold stash. The stash now lives in
  the world, sized once at creation, so a step never allocates.

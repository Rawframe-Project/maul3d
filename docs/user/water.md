# Water volumes

## Water volumes

m3CreateWaterVolume fills a world-anchored axis-aligned box with
still or flowing water (up to 8 per world; the surface is the box
top; there is NO fluid simulation by law). Rigid bodies get
buoyancy proportional to their submerged bounds applied at the
submerged centroid (off-center bites right a listing crate), drag
that pulls their velocity toward the flow, and spin damping; the
model reads shape bounds, deliberately approximate and
deterministically so. Soft particles trade gravity for buoyancy
under the density-1000 particle convention (density 1000 suspends a
particle, denser fluids lift it, thinner ones let it sink) and
drag toward the flow like wind. The tide law: creating or
destroying a volume wakes every body its box touches, so sleepers
float when the flood arrives and fall when it leaves. Volumes
journal (ops 74/75), snapshot (v48), and hash only while one is
alive; a dry world keeps its exact pre-water bits.

Snapshot economics: hull and mesh content travel
count-derived, so an empty slot costs bytes, not kilobytes, and a
box hull costs its used prefix instead of a 5808-byte slab.
Snapshot sizes GROW with world content: measure with
m3World_GetSnapshotSize per keyframe, never once per session.

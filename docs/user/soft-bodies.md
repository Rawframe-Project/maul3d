# Soft bodies

## Soft bodies

A soft body (`m3CreateSoftBody`) is an XPBD particle lattice: a
box of particles (rope n x 1 x 1, cloth n x m x 1, jelly n x m x
k) bound by structural and face-diagonal distance constraints
with compliance (zero is rigid rods), solved one Gauss-Seidel
sweep per substep in fixed index order: deterministic by
construction, under the same four gates as everything else.

- Particles collide with the whole rigid shape set (planes,
  spheres, capsules, boxes and hulls, voxel chunks, meshes,
  heightfields) and take friction from the touched shape. Contact
  with dynamic bodies is two-way: jelly has weight.
- `m3SoftBody_PinParticle` freezes a particle where it is;
  `m3SoftBody_AnchorParticle` captures it in a BODY's frame: it
  rides the body, the lattice's pull lands on the body at the
  anchor, and a dead body releases its anchors silently. Cloth
  hangs from beams and follows them when they move. Both return an
  `m3Result`: an anchor past a full table of
  `M3_MAX_SOFTBODY_ANCHORS` is `m3_errorLimit`, which counts no
  misuse, and `m3SoftBody_AnchorToSoftBody` shares that table.
- Soft-versus-soft and self-collision are NOT in this version
  (documented out, the bullet-versus-bullet precedent); particle
  counts cap at 512 and anchors at 32 per body.
- Destruction interplay holds: carve the floor from under a
  resting rope and it falls the same step.
- All soft state (positions, previous positions, inverse masses,
  edges, anchors) snapshots, hashes, replays, and rolls back
  bit-exact like every other pool.
- A deterministic curiosity worth knowing: a perfectly vertical
  rope BALANCES on its end forever, because bit-exact floats
  carry no noise to seed the buckle. Give real scenes a nudge.

## Soft body depth

Four knobs deepen the particle lattices (all default-off,
bit-exact with earlier versions when unused). Bend
(`bendCompliance`): second-neighbor tethers along each lattice
axis stiffen ropes and sheets against folding; positions carry no
frames, so twist stiffness is deliberately out. Pressure
(`pressure`): closed lattices (every axis count at least 2) hold
a target volume multiple through one global constraint over their
surface; 2 inflates a cube toward double volume. Tet bodies
(`m3CreateTetSoftBody`): explicit points and tetrahedra, each tet
holding its create volume rigidly (the incompressible jelly);
edges come from tet edges; flat or inverted tets refuse. The bind
tether (`maxDeviation`): every particle clamps to a radius around
its create position each substep, the cheap skinned-vertex limit
that keeps cloth from exploding; the clamp runs before the volume
rows, so a crushed tet still restores its volume. Double-sided
cloth needs no switch: particles are spheres and collide from
both sides by construction.

## Soft interactions and fields

Lattices collide with EACH OTHER (canonical particle pairs behind
lattice-bound gates; self-collision stays out, the structure rods
hold a lattice apart), and they hold hands:
m3SoftBody_AnchorToSoftBody pins particle to particle across lattices
with silent release when either side dies. Wind
(m3World_SetWind) drives particles with proportional drag toward
a gusted velocity whose phase ACCUMULATES in the snapshot: a
rollback resumes the exact same wave. Conveyors
(m3Shape_SetSurfaceVelocity) feed the contact solver's tangential
target: crates ride belts at belt speed, and a belt on a dynamic
deck recoils by Newton. All of it journals, snapshots, and twins.

Particle cloth is POROUS to rigid bodies of comparable radius
under load: a lattice sheet is a grid of contact spheres, and a
sphere the size of the grid gaps can press between them when the
span stretches. Densify the particles, thicken their radius, or
back the sheet with a thin box when you need a true barrier (a v1
bound of the box-lattice substrate, recorded here so nobody
relearns it).

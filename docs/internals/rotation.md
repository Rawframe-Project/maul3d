# Free rotation

A rigid body with no torque on it keeps its world angular momentum L and its rotational energy, while its spin w precesses through the body. maul3d turns every free dynamic body by a splitting of Euler's equations that keeps L to rounding, keeps the energy within a bounded error that does not drift, and turns symmetric tops and spheres exactly. This page specifies the method step by step, so that another library can implement the same steps in the same order. maul-vfx follows it.

It replaced an implicit Newton step on the spin. That step damped precession: a 2 m rod lost most of its energy in seconds (#132), and an ordinary 0.5 x 1 x 1.5 m box lost up to 14% of its energy in one second at 20 rad/s (#133). The orientation step that followed it moved the body by its end-of-substep spin held constant, which made world L wobble by up to 4% (#134). The splitting moves the spin and the pose together, so both problems go.

## Where it runs

Each substep integrates velocities, solves the biased rows, integrates positions and relaxes. The flow runs in the position integration, on the spin the solve left, for every awake dynamic body:

- whose rotation is not locked about any axis; and
- whose principal moments pass the applies check below.

Any other mover keeps the first-order turn `q = normalize(q + 0.5 (h w, 0) q)` with its spin unchanged: a kinematic body, a body with a locked rotation axis, or a body whose moments fail the check.

The spin is never re-derived from the pose. Contact and joint impulses act on w in the solve before the flow and in the relax pass after it. Angular damping stays in the velocity integration, as the implicit factor `w = w / (1 + h c)`. That factor scales L uniformly, and the flow then runs on the result.

The solver's accumulated rotation takes the turn the pose actually made: `deltaRot = normalize((q' conj(q)) deltaRot)`.

## Conventions

- Quaternions are `(x, y, z, w)`. The product is Hamilton's, as `m3MulQuat` computes it, and `conj(q) = (-x, -y, -z, w)`.
- `Rotate(q, v)` is `v + w t + u x t`, with `u = (x, y, z)` and `t = 2 u x v`. `InvRotate(q, v)` is `Rotate(conj(q), v)`.
- `normalize(q)` divides by `sqrt(x^2 + y^2 + z^2 + w^2)`, and gives the identity when that is below 2^-23.
- `CosSin(a)` is the library's deterministic cosine and sine, `m3ComputeCosSin`.
- `r(e, a)` is the turn about a unit axis e by angle a: with `(c, s) = CosSin(a / 2)`, `r = (s e, c)`.
- All arithmetic is float, with no fused multiply-add, except the principal frame, which is double.

## The principal frame

The frame is computed once per mass update, whenever a body's shapes, their density or its type change. A restore runs the same update for every body, so the frame is derived state and never stored.

The local inertia, a symmetric tensor, is converted to double and diagonalized by cyclic Jacobi:

1. Start from the eigenvector matrix V = identity.
2. A sweep visits the pairs (0, 1), (0, 2), (1, 2), in that order, and rotates every pair whose entry `a[p][q]` is not zero:
   - `theta = (a[q][q] - a[p][p]) / (2 a[p][q])`;
   - `t = 1 / (|theta| + sqrt(theta^2 + 1))`, negated when theta is negative;
   - `c = 1 / sqrt(t^2 + 1)` and `s = t c`;
   - with r the third index: `a[p][p] -= t a[p][q]`, `a[q][q] += t a[p][q]`, `a[p][q] = a[q][p] = 0`;
   - `a[p][r]` and `a[q][r]` become `c a[p][r] - s a[q][r]` and `s a[p][r] + c a[q][r]`, both from their old values, mirrored into `a[r][p]` and `a[r][q]`;
   - each row k of V: `V[k][p]` and `V[k][q]` become `c V[k][p] - s V[k][q]` and `s V[k][p] + c V[k][q]`, both from their old values.
3. Stop when all three off-diagonal entries are exactly zero, or after 16 sweeps.
4. Sort the diagonal largest first by insertion sort with strict comparisons, so a tie keeps the lower column first, and permute V's columns with it.
5. For each column, make its largest-magnitude component positive, the first on a tie. Then, if `det(V) < 0`, negate the third column.
6. The moments are the sorted diagonal, rounded to float: `I_max >= I_mid >= I_min`.
7. The frame is V as a quaternion, by Shepperd's method:
   - pick the largest of the trace and the three diagonal entries, comparing with `>=` in that order;
   - compute the four components from it;
   - give w a sign of `+`, normalize in double, and round each component to float.

The frame maps principal coordinates to body coordinates: `I_local = R diag(I_max, I_mid, I_min) R^T`.

**Applies check.** `I_max` is finite, `I_min >= FLT_MIN`, and `I_min / I_max >= FLT_MIN`.

## The flow

For a substep of length h, a body at rotation q with world spin w:

1. **Scale the moments.** Compute `s = (1, I_mid / I_max, I_min / I_max)`. Only the ratios matter, and scaling keeps the momentum near the spin's size for a body of any mass.
2. **Equal moments.** If `s.y == 1` and `s.z == 1`, the spin is constant.
   - With `n = |w|`, if `n > 0`, set `q' = normalize(r(w / n, h n) q)`.
   - w is left bit for bit. Done.
3. **Momentum.** Set `invMid = 1 / s.y` and `invMin = 1 / s.z`, with `invMax = 1`.
   - `P = normalize(q frame)`.
   - `wp = InvRotate(P, w)`.
   - `L = (wp.x, s.y wp.y, s.z wp.z)`, the principal momentum.
4. **Pieces.** With `|L|` the length of L:
   - `a = h |L| (invMin - invMid)` and `b = h |L| (invMid - invMax)`;
   - `n = ceil(cbrt(max(a^2 b, a b^2)) / 0.25)`, clamped to the range 1 to 64;
   - `t = h / n`.
5. **Residual turns.** Start from `T = identity`, and repeat n times:
   - the min axis for `t / 2`;
   - the max axis for `t`;
   - the min axis for `t / 2`.
6. **The Casimir turn.** With `l = |L|`, if `l > 0`, set `T = T r(L / l, h l invMid)`. This turn leaves L as it is, and it is exact at any angle.
7. **The pose.** `q' = normalize(q ((frame T) conj(frame)))`.
8. **The spin.**
   - `P' = normalize(q' frame)`.
   - `w' = Rotate(P', (L.x, L.y / s.y, L.z / s.z))`. These are divisions: multiplying by a rounded inverse would scale a body's spin by the same factor every substep.
   - The read-back correction: `B = InvRotate(P', w')`, then `R = (B.x, s.y B.y, s.z B.z)`, then `d = (|L|^2 - |R|^2) / (2 |R|^2)`.
   - If d is finite, `w' = w' + d w'`.

**A residual turn** of axis i, the max axis 0 or the min axis 2, for time tau, with `inv_i` being invMax or invMin:

1. The angle: `theta = tau L_i (inv_i - invMid)`, and `(ch, sh) = CosSin(theta / 2)`.
2. Append the turn: `T = T (sh e_i, ch)`, with `e_i` the unit vector of axis i.
3. The double angle: `c = ch^2 - sh^2` and `s = 2 sh ch`.
4. Turn the pair across the axis. With `j = (i + 1) mod 3` and `k = (i + 2) mod 3`:
   - `nj = c L_j + s L_k`;
   - `nk = c L_k - s L_j`.
5. Correct its length: `d = (L_j^2 + L_k^2 - nj^2 - nk^2) / (2 (nj^2 + nk^2))`. If d is finite, `nj += d nj` and `nk += d nk`.
6. Store `L_j = nj` and `L_k = nk`. `L_i` is untouched.

### Why each step is there

- **The Casimir split.** The kinetic energy splits as `|L|^2 / (2 I_mid)` plus two residual terms, `L_max^2 (1/I_max - 1/I_mid) / 2` and `L_min^2 (1/I_min - 1/I_mid) / 2`. Each term has an exact flow: a turn of L, and of the body with it, about one axis. The first term commutes with the others, so only the two residuals are split, in Strang order. For a symmetric top one residual is zero, so the method is exact.
- **The piece count.** The splitting's error per substep grows as `a^2 b` and `a b^2`. Pieces cut it by the square of their count, and a symmetric top never takes more than one.
- **Composing T.** The turns are composed in the principal frame and given to the pose once. Re-basing the pose into the principal frame and back would round it every substep, and turn the body while its spin does not turn.
- **The two length corrections.** A body's turn angle repeats every substep, and so would the rounding of its cosine and sine; the residual turns' correction removes that. The spin and pose the next substep reads back round alike from one substep to the next, so their errors add up like a bias; the read-back correction removes that.

## Accuracy

Splitting choices, compared in double. Each entry is the worst relative energy change over 10 s at 20 rad/s, at four substeps of 60 Hz, over 40 spin directions:

| Box, full extents | Five turns, min/2 mid/2 max mid/2 min/2 | Casimir split, min/2 max min/2 |
|---|---|---|
| 0.5 x 1 x 1.5 | 1.1e-4 | 4.5e-5 |
| 0.2 x 1 x 2 | 3.4e-5 | 8.3e-5 |
| 0.9 x 1 x 1.1 | 3.3e-4 | 6.1e-7 |
| 1 x 1 x 1.5, a symmetric top | 2.0e-4 | 5e-13 |
| 0.1 x 0.1 x 2, a rod | 1.4e-4 | 5e-13 |

The best five-turn order was min/2 mid/2 max mid/2 min/2. Index order, x/2 y/2 z, was among the worst.

The world, in float, measured by `test_rigid_flow` and `test_solver`:

- **A triaxial box.** The 0.5 x 1 x 1.5 m box at 20 rad/s for 10 s, spun near each principal axis, the unstable middle one included:
  - world L holds within 1.9e-4 and energy within 4.7e-5;
  - the long axis's share of L ends within 0.0012 of an RK4 reference of Euler's equations.
- **Rods.** A 2 m rod of radius 0.05 m at 10 rad/s, 45 degrees off its axis, keeps its energy within 4e-5 over 10 s along each body axis, and within 7e-5 laid along each diagonal of two axes, where only its principal frame shows it is a symmetric top.
- **Fast tumblers.** Fifty flat boxes at 30 to 800 rad/s, past a radian per substep, on one substep and on four, hold their energy within 1.9e-3 and their world angular momentum within 4.9e-5 over a second.

## Sources

- R. I. McLachlan, "Explicit Lie-Poisson integration and the Euler equations", Physical Review Letters 71, 1993. The splitting of the free rigid body into exactly solvable turns about its principal axes.
- F. Fassò, "Comparison of splitting algorithms for the rigid body", Journal of Computational Physics 189, 2003. The splittings compared, among them the separation of the symmetric part.

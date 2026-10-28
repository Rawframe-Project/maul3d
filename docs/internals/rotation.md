# Free rotation

A rigid body with no torque on it keeps its world angular momentum L and its rotational energy, while its spin w precesses through the body. maul3d turns every free dynamic body by a splitting of Euler's equations that keeps L to rounding, keeps the energy within a bounded error that does not drift, and turns symmetric tops and spheres exactly.

This page specifies the method operation by operation, in the order and grouping the code computes it. Another library that performs the same float operations gets the same bits; maul-vfx follows it.

It replaced an implicit Newton step on the spin. That step damped precession: a 2 m rod lost most of its energy in seconds (#132), and an ordinary 0.5 x 1 x 1.5 m box lost up to 14% of its energy in one second at 20 rad/s (#133). The orientation step that followed it moved the body by its end-of-substep spin held constant, which made world L wobble by up to 4% (#134). The splitting moves the spin and the pose together, so both problems go.

## Where it runs

Each substep integrates velocities, solves the biased rows, integrates positions and relaxes. The flow runs in the position integration, on the spin the solve left, for every awake dynamic body:

- whose rotation is not locked about any axis; and
- whose principal moments pass the applies check below.

Any other mover keeps the first-order turn, with its spin unchanged: a kinematic body, a body with a locked rotation axis, or a body whose moments fail the check. With `d = (h w.x, h w.y, h w.z)` and `a = 0.5 d`:

```
q' = normalize(q + (a.x q.w + a.y q.z - a.z q.y,
                    -a.x q.z + a.y q.w + a.z q.x,
                    a.x q.y - a.y q.x + a.z q.w,
                    -(a.x q.x + a.y q.y + a.z q.z)))
```

The spin is never re-derived from the pose. Contact and joint impulses act on w in the solve before the flow and in the relax pass after it. Angular damping stays in the velocity integration, as the implicit factor `w = (1 / (1 + h c)) w`, so the flow runs on the damped spin.

The solver's accumulated rotation takes the turn the pose made: `deltaRot = normalize((q' * conj(q)) * deltaRot)`.

## Notation

Everything below is float unless it says double. There is no fused multiply-add anywhere. Expressions are written as C evaluates them, left to right, so `a * b * c` is `(a * b) * c` and `a + b + c` is `(a + b) + c`. Parentheses group nothing else.

- **Vectors.**
  - `dot(a, b) = a.x * b.x + a.y * b.y + a.z * b.z`.
  - `cross(a, b) = (a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)`.
  - `s v = (s * v.x, s * v.y, s * v.z)`, and `a + b` and `a - b` componentwise.
  - `length(v) = sqrtf(dot(v, v))`.
- **Quaternions** are `(x, y, z, w)`.
  - `a * b` is the Hamilton product:
    - `w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z`;
    - `x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y`;
    - `y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x`;
    - `z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w`.
  - `conj(q) = (-q.x, -q.y, -q.z, q.w)`.
  - `normalize(q)`:
    - `n = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)`;
    - if `n < 1.19209290e-7f`, the identity `(0, 0, 0, 1)`;
    - else, with `i = 1 / n`, `(i * q.x, i * q.y, i * q.z, i * q.w)`.
  - `rotate(q, v)`: with `u = (q.x, q.y, q.z)` and `t = 2 cross(u, v)`, it is `(v + q.w t) + cross(u, t)`.
  - `unrotate(q, v) = rotate(conj(q), v)`.
- **Max.** `max(a, b) = a > b ? a : b`.
- **Finite.** `finite(x)` is `x - x == 0`.
- **Cosine and sine.** `cossin(a)` returns `(c, s)`:
  1. `x = remainderf(a, 6.2831855f)`, which is exact.
  2. `k = floorf(x * 0.63661975f + 0.5f)`.
  3. `r = (x - k * 1.5707964f) - k * -4.371139e-08f`, and `r2 = r * r`.
  4. The series:
     - `s0 = r * (1 + r2 * (-0.16666667f + r2 * (0.008333334f + r2 * (-0.0001984127f + r2 * 2.7557319e-06f))))`;
     - `c0 = 1 + r2 * (-0.5f + r2 * (0.041666668f + r2 * (-0.0013888889f + r2 * 2.4801588e-05f)))`.
  5. The quadrant is `(k == k ? (int)k : 0) & 3`. It picks `(c, s)`:
     - 0: `(c0, s0)`;
     - 1: `(-s0, c0)`;
     - 2: `(-c0, -s0)`;
     - 3: `(s0, -c0)`.
  6. Normalize: `m = sqrtf(s * s + c * c)`, `i = m > 0 ? 1 / m : 0`, and the result is `(c * i, s * i)`.
- **A turn.** `turn(e, angle)` about a unit axis e: with `(c, s) = cossin(0.5f * angle)`, it is `(s * e.x, s * e.y, s * e.z, c)`.

## The principal frame

The frame is computed once per mass update, whenever a body's shapes, their density or its type change. A restore runs the same update for every body, so the frame is derived state and never stored.

The local inertia, a symmetric float tensor with columns `cx`, `cy` and `cz`, is converted to double as `a[0][0] = cx.x`, `a[1][1] = cy.y`, `a[2][2] = cz.z`, `a[0][1] = a[1][0] = cy.x`, `a[0][2] = a[2][0] = cz.x` and `a[1][2] = a[2][1] = cz.y`. It is then diagonalized by cyclic Jacobi, all in double:

1. Start from `V = identity`.
2. A sweep visits the pairs (0, 1), (0, 2), (1, 2), in that order. It rotates every pair whose `a[p][q] != 0`, with `r = 3 - p - q`:
   - `theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])`;
   - `t = 1.0 / (fabs(theta) + sqrt(theta * theta + 1.0))`, then `t = -t` if `theta < 0`;
   - `c = 1.0 / sqrt(t * t + 1.0)` and `s = t * c`;
   - with `apr = a[p][r]` and `aqr = a[q][r]`:
     - `a[p][p] -= t * a[p][q]`, then `a[q][q] += t * a[p][q]`, then `a[p][q] = a[q][p] = 0`;
     - `a[p][r] = c * apr - s * aqr`, then `a[r][p] = a[p][r]`;
     - `a[q][r] = s * apr + c * aqr`, then `a[r][q] = a[q][r]`;
   - for each row k, with `vkp = V[k][p]` and `vkq = V[k][q]`: `V[k][p] = c * vkp - s * vkq` and `V[k][q] = s * vkp + c * vkq`.
3. Before each sweep, stop if `a[0][1]`, `a[0][2]` and `a[1][2]` are all exactly zero. Run at most 16 sweeps.
4. Sort: `d = (a[0][0], a[1][1], a[2][2])`, by insertion sort.
   - For `i = 1, 2` and `j = i` down to 1, while `d[j] > d[j - 1]`, swap them and swap V's columns j and j - 1.
   - A tie keeps the lower column first.
5. Signs. For each column j:
   - find `big`, the first row with the largest `fabs(V[row][j])`, compared with `>`;
   - if `V[big][j] < 0`, negate the column.
6. Determinant. Then, if `det < 0`, negate column 2, where `det = V[0][0] * (V[1][1] * V[2][2] - V[2][1] * V[1][2]) - V[0][1] * (V[1][0] * V[2][2] - V[2][0] * V[1][2]) + V[0][2] * (V[1][0] * V[2][1] - V[2][0] * V[1][1])`.
7. The moments are `((float)d[0], (float)d[1], (float)d[2])`, that is `I_max >= I_mid >= I_min`.
8. The frame is V as a quaternion `(q0, q1, q2, q3)`, by the first branch that holds, with `tr = V[0][0] + V[1][1] + V[2][2]`:
   - **`tr >= V[0][0] && tr >= V[1][1] && tr >= V[2][2]`.** `s = 2.0 * sqrt(1.0 + tr)`; then
     `q3 = 0.25 * s`, `q0 = (V[2][1] - V[1][2]) / s`, `q1 = (V[0][2] - V[2][0]) / s`, `q2 = (V[1][0] - V[0][1]) / s`.
   - **`V[0][0] >= V[1][1] && V[0][0] >= V[2][2]`.** `s = 2.0 * sqrt(1.0 + V[0][0] - V[1][1] - V[2][2])`; then
     `q3 = (V[2][1] - V[1][2]) / s`, `q0 = 0.25 * s`, `q1 = (V[0][1] + V[1][0]) / s`, `q2 = (V[0][2] + V[2][0]) / s`.
   - **`V[1][1] >= V[2][2]`.** `s = 2.0 * sqrt(1.0 + V[1][1] - V[0][0] - V[2][2])`; then
     `q3 = (V[0][2] - V[2][0]) / s`, `q0 = (V[0][1] + V[1][0]) / s`, `q1 = 0.25 * s`, `q2 = (V[1][2] + V[2][1]) / s`.
   - **Otherwise.** `s = 2.0 * sqrt(1.0 + V[2][2] - V[0][0] - V[1][1])`; then
     `q3 = (V[1][0] - V[0][1]) / s`, `q0 = (V[0][2] + V[2][0]) / s`, `q1 = (V[1][2] + V[2][1]) / s`, `q2 = 0.25 * s`.
   - Then `i = (q3 < 0 ? -1.0 : 1.0) / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)`, and the frame is `((float)(q0 * i), (float)(q1 * i), (float)(q2 * i), (float)(q3 * i))`.

The frame maps principal coordinates to body coordinates: `I_local = R diag(I_max, I_mid, I_min) R^T`.

**Applies check.** `finite(I_max) && I_min >= FLT_MIN && I_min / I_max >= FLT_MIN`.

## The flow

The constants are `M3_FLOW_PIECE_TURN = 0.25f` and `M3_FLOW_MAX_PIECES = 64`. For a substep of length h, a body at rotation q with world spin w, moments `(Ix, Iy, Iz) = (I_max, I_mid, I_min)` and frame F:

1. **Scale.** `S = (1, Iy / Ix, Iz / Ix)`.
2. **Equal moments.** If `S.y == 1 && S.z == 1`:
   - with `n = length(w)`, if `n > 0`, `q' = normalize(turn((1 / n) w, h * n) * q)`, else `q' = q`;
   - w is left bit for bit, and the flow ends.
3. **Inverses.** `invMid = 1 / S.y`, `invMin = 1 / S.z` and `invMax = 1`.
4. **Momentum.** `p = unrotate(normalize(q * F), w)`, then `L = (p.x, S.y * p.y, S.z * p.z)`.
5. **Pieces.** `len = length(L)`, `a = h * len * (invMin - invMid)`, `b = h * len * (invMid - invMax)`, and `m = max(a * a * b, a * b * b)`.
   - n is the first of `1, 2, ..., 63` with `!(m > r * r * r)`, where `r = (float)n * 0.25f`. If none holds, `n = 64`.
   - `t = h / (float)n`.
6. **Residual turns.** Start from `T = (0, 0, 0, 1)`. Repeat n times:
   - `residual(2, invMin, 0.5f * t)`;
   - `residual(0, invMax, t)`;
   - `residual(2, invMin, 0.5f * t)`.
7. **The Casimir turn.** `len = length(L)`. If `len > 0`, `T = T * turn((1 / len) L, h * len * invMid)`. This turn leaves L as it is.
8. **The pose.** `q' = normalize(q * ((F * T) * conj(F)))`.
9. **The spin.** With `P = normalize(q' * F)`:
   - `spin = (L.x, L.y / S.y, L.z / S.z)` and `o = rotate(P, spin)`;
   - read back: `miss = spin - unrotate(P, o)`;
   - `w' = o + rotate(P, miss)`.

**`residual(i, inv, tau)`.** Here i is the max axis 0 or the min axis 2, `l = (L.x, L.y, L.z)`, `j = (i + 1) % 3` and `k = (i + 2) % 3`.

1. **The angle.** `angle = tau * l[i] * (inv - invMid)`, and `(hc, hs) = cossin(0.5f * angle)`.
2. **The turn.** `T = T * (i == 0 ? hs : 0, 0, i == 2 ? hs : 0, hc)`.
3. **The double angle.** `c = hc * hc - hs * hs` and `s = 2.0f * hs * hc`.
4. **The pair.** With `lj = l[j]` and `lk = l[k]`: `nj = c * lj + s * lk` and `nk = c * lk - s * lj`.
5. **Its length.**
   - `before = lj * lj + lk * lk`, `after = nj * nj + nk * nk`, and `d = 0.5f * (before - after) / after`;
   - if `finite(d)`, `nj = nj + d * nj` and `nk = nk + d * nk`.
6. **Store.** `l[j] = nj` and `l[k] = nk`. `l[i]` is untouched.

### Why each step is there

- **The Casimir split.** The kinetic energy splits as `|L|^2 / (2 I_mid)` plus two residual terms, `L_max^2 (1/I_max - 1/I_mid) / 2` and `L_min^2 (1/I_min - 1/I_mid) / 2`. Each term has an exact flow: a turn of L, and of the body with it, about one axis. The first term commutes with the others, so only the two residuals are split, in Strang order. For a symmetric top one residual is zero, so the method is exact.
- **Scaling.** Only the moments' ratios matter, and scaling keeps L near the spin's size for a body of any mass.
- **The piece count.** The splitting's error per substep grows as `a^2 b` and `a b^2`. Pieces cut it by the square of their count, and a symmetric top never takes more than one. The count compares cubes in float rather than taking a cube root, since a C library's cube root is not correctly rounded.
- **Composing T.** The turns are composed in the principal frame and given to the pose once. Re-basing the pose into the principal frame and back would round it every substep, and turn the body while its spin does not turn.
- **The two corrections.** A body's turn angle repeats every substep, and so would the rounding of its cosine and sine; the residual turns' length correction removes that. The spin and pose the next substep reads back round alike from one substep to the next, so their errors add up like a bias; the read-back correction removes that. It corrects each component of the read-back. Correcting only its length would pay the rounding across a thin rod, about the spin times 6e-8 at the rod's large moments, from the rod's far smaller momentum about its own axis, and drain that spin: a rod with moments (1, 1, 1e-4) kept 27% of it after 10 s at 480 substeps a second.
- **Divisions.** The spin divides by the scaled moments. Multiplying by a rounded inverse would scale a body's spin by the same factor every substep.

Only correctly rounded or exact library functions appear above. `sqrt`, `sqrtf`, `fabs` and `floorf` are correctly rounded, and `tools/libm-allowed.txt` lists them for maul3d's sources. `remainderf` is exact, and only the public header `maul3d/core_math.h` calls it, outside the sources that list covers.

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
  - world L holds within 1.5e-5 and energy within 3.9e-5;
  - the long axis's share of L ends within 0.0014 of an RK4 reference of Euler's equations.
- **Rods.** A 2 m rod of radius 0.05 m at 10 rad/s, 45 degrees off its axis, keeps its energy within 2.8e-5 over 10 s along each body axis. Laid along each diagonal of two axes, where only its principal frame shows it is a symmetric top, it keeps it within 7e-5. Spun at 3 rad/s along its axis or off it, a rod of moments (1, 1, r) keeps its spin about the axis within 2.5e-5 over 10 s for every r from 1e-1 down to 1e-12, and a 2 m rod of radius 1.2 cm in a world within 2e-6.
- **Fast tumblers.** Fifty flat boxes at 30 to 800 rad/s, past a radian per substep, on one substep and on four, hold their energy within 1.9e-3 and their world angular momentum within 3.6e-5 over a second.

## Sources

- R. I. McLachlan, "Explicit Lie-Poisson integration and the Euler equations", Physical Review Letters 71, 1993. The splitting of the free rigid body into exactly solvable turns about its principal axes.
- F. Fassò, "Comparison of splitting algorithms for the rigid body", Journal of Computational Physics 189, 2003. The splittings compared, among them the separation of the symmetric part.

# Sequential impulse with warm starting

**Status:** accepted

Contacts and constraints are solved with a warm-started sequential impulse
solver, one Island at a time.

## Considered options

- **XPBD.** Uniform handling of joints and soft bodies, no inverted mass matrix,
  excellent sub-step stability — but less direct semantics for rigid-body friction
  and restitution, and static friction is harder to get exactly right.
- **TGS Soft / sub-stepped soft constraints.** The most stable stacking, at
  several times the computation and the highest implementation complexity.
  Rejected on budget: physics shares CPU with a software rasterizer, so extra
  sub-steps per frame are the one thing this engine cannot afford.
- **No stacking stability.** Incompatible with the hundreds-to-two-thousand Body
  target just agreed.

## Consequences

Warm starting and constraint ordering must be done properly, or stacks go soft,
jittery, and drift — most of the solver's difficulty lives here rather than in the
impulse math. Later joints (currently out of scope) plug into the same constraint
framework without re-architecting.

Because warm starting requires per-Contact persistent state keyed across steps, the
engine already knows whether a Contact existed last step — which is what makes
`Begin`/`Persist`/`End` Contact phases nearly free.

That key is the geometry a point came from, not its place in the step's list. A corner
of the source polygon is its own identity, and a vertex a clip inserted is named by the
vertex it was cut from and the plane that cut it, tagged with the shape that supplied
the polygon. A rank would renumber the survivors whenever a manifold shrinks — a cube
rolling onto its edge drops from four points to two — which hands a point the impulse a
different corner had accumulated. The tag is what keeps two shapes' corner indices, or
the two sides of a box pair, from being read as each other; a reference-face flip then
misses a warm start rather than applying it to the wrong point.

Two-point manifolds are solved as one 2×2 block, because two points solved in sequence
settle on unequal impulses and their difference is a couple about the axis they share.
Three- and four-point manifolds keep the per-point path: their block is exactly singular.
The diagonal of that block is free — `prepare` already computed each point's effective
mass and angular response, and the block reuses them; only the off-diagonal coupling is
rebuilt per iteration.

Each constraint point stores `inv_inertia(anchor × axis)` for all three solve axes,
so a velocity iteration applies an impulse with a scalar multiply instead of two
quaternion rotations. That costs 72 bytes per point — roughly 1.7× the constraint
array — and bought about 46% off a packed 2000-Body lattice (32 ms → 17 ms at the
developed peak), because every iteration of every contact reads it. The trade goes
the other way only if the solve is bandwidth-bound rather than arithmetic-bound,
which this one is not: the constraint array still fits a 2000-Body scene in cache.

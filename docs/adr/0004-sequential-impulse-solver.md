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

Each constraint point stores `inv_inertia(anchor × axis)` for all three solve axes,
so a velocity iteration applies an impulse with a scalar multiply instead of two
quaternion rotations. That costs 72 bytes per point — roughly 1.7× the constraint
array — and bought about 46% off a packed 2000-Body lattice (32 ms → 17 ms at the
developed peak), because every iteration of every contact reads it. The trade goes
the other way only if the solve is bandwidth-bound rather than arithmetic-bound,
which this one is not: the constraint array still fits a 2000-Body scene in cache.

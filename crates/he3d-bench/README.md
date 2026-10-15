# he3d-bench

Criterion benchmarks and heap accounting for HE3D. Internal tooling:
`publish = false`, depends on no backend crate, builds and runs headless.

The point of this crate is that performance claims in the engine are *measured*
rather than asserted.

## Running

```sh
# Full criterion run: physics, renderer, assets, core/math.
cargo bench -p he3d-bench

# Smoke run: every case exactly once. This is what CI runs.
cargo bench -p he3d-bench -- --test

# Heap and size_of tables (installs a counting global allocator).
cargo run -p he3d-bench --bin memreport --release

# Cost versus step index, and whether a pile ever sleeps.
cargo run -p he3d-bench --bin settle --release

# One pile, every hundredth step.
cargo run -p he3d-bench --bin settle --release -- 500 1500 12

# One target only.
cargo bench -p he3d-bench --bench physics
```

## What it measures

| Target | Covers |
|---|---|
| `benches/physics.rs` | `World::step` across freefall, a packed lattice, a settled sleeping scene, static terrain, and broadphase cell sizes; plus `add_body` and construction |
| `benches/renderer.rs` | `Renderer::new`, resolution / triangle-count / object-count sweeps, and each anti-aliasing mode |
| `benches/assets.rs` | `parse_obj` and `decode_image` on the shipped `biplane` fixtures, plus a scaling synthetic OBJ |
| `benches/core_math.rs` | The `glam`/`Transform` operations the engine leans on |

`src/bin/memreport.rs` installs `alloc::CountingAlloc` and prints, per scenario,
the allocations made, bytes requested, and peak added, plus a `size_of` table.

`src/bin/settle.rs` prints cost as a function of step index, alongside the Contact
count and how many Bodies have slept. A criterion case answers "what does one step
cost now"; a scene dropped into contact ramps up and then decays, so that single
number is the *peak*. `settle` is how "does this scene ever settle and sleep" is
answered rather than assumed.

## What the suite found

Measured on an AMD Ryzen 7 5800X with rustc 1.99.0. Reproduce with the commands
above; the scenes are defined in `src/workloads.rs`.

### Memory (`memreport`)

- **Steady-state work allocates nothing.** A settled `World` stepped a hundred times
  and a `Renderer` drawing a hundred frames both report zero allocations — both reuse
  their buffers. That is the baseline: any per-call allocation is a finding, not
  overhead.
- **A Body costs ~196 requested bytes**, and `World::new()` allocates nothing at all;
  storage grows in as Bodies are added.
- **A renderer's memory is its framebuffers.** `Renderer::new(1280x720)` requests
  ~7.4 MB in three allocations (colour + inverse depth, four bytes each per pixel).
  Nothing else in the engine holds memory like that.
- **The renderer allocates per visible triangle.** Drawing the 8-cube scene at
  640x360 makes ~49 short-lived allocations per frame (~3.9 KB), all freed inside the
  frame, so the peak never moves. One is `self.missing_texture.clone()` per frame
  (`renderer.rs:232`); the other ~48 are one `Vec` per front-facing triangle from
  `clip_triangle_to_near` (`renderer.rs:1005`). At 60 Hz that is ~3000 allocator
  calls per second that a reusable scratch buffer would remove.
- **Decoding is one shot.** `decode_image` of the 256x256 BMP is a single 256 KB
  allocation; a 10 000-triangle static mesh costs ~134 bytes per triangle.

### Speed

- **Physics cost tracks contacts, not Bodies.** 2000 Bodies out of contact step in
  0.23 ms; the same 2000 packed in contact peak at 17 ms and settle to 5.5 ms.
  Recorded in `docs/physics-spec.md` §12.
- **A pile the solver cannot converge never sleeps.** `settle` pokes at a lattice of
  unit cubes layered on a slab. Six layers settle around step 800, sleep, and then
  cost 0.05 ms for the whole pile. Eight layers (residual speed at least twice the
  sleep threshold) and twelve layers never sleep in 1500 steps and pay for their
  contacts forever — 2.3 ms and 5.5 ms per step. Raising the iteration count does not
  help: at 48 velocity iterations the eight-layer pile is still awake at close to
  four times the cost. This is the tall-stack limitation of §14 risk 7, measured.
- **The velocity solve was arithmetic-bound, and the inertia rotation was most of
  it.** Each impulse recomputed two quaternion rotations to turn `anchor × axis` into
  angular velocity. Storing `inv_inertia(anchor × axis)` per point instead (ADR 0004)
  cut the per-iteration cost by about 60%: the packed 2000-Body lattice went from
  32 ms to 17 ms at the same contact count, and `static_terrain/100` halved. `freefall`
  and `settled_asleep` did not move, which is the point — the win is where the contacts
  are, and nowhere else.
- **A renderer frame is pixels, and the hard floor is the clear.** An empty 720p
  frame — clearing colour + inverse depth, ~7.4 MB — costs 83.6 µs, which is
  0.092 ns per pixel at 64x36, 320x180, and 1280x720 alike. Drawing the 8-cube scene
  raises the 720p frame to 772 µs; the 688 µs difference is pixel fill (16x the
  pixels from 320x180 buys 12x the fill). Geometry *count* is not the driver: 1000
  objects still cost only 501 µs at 64x36, because their triangles are small.
- **`Renderer::new` pays page faults, once.** Allocating the 720p buffers costs
  797 µs — nearly ten times the 83.6 µs it takes to clear those same buffers
  afterwards — because fresh pages are faulted in on first touch. The per-frame cost
  is the 83.6 µs, not the 797 µs.
- **Per-triangle setup shows up when triangles are small.** 10 000 sub-pixel
  triangles at 64x36 cost 617 µs — 62 ns each, which includes the one heap allocation
  per front-facing triangle noted above.
- **Anti-aliasing multiplies pixel or sample work.** At 64x36: none 8.4 µs, MSAA
  34 µs (x4 samples), FXAA 22 µs (a full-screen pass), TAA 46 µs, SSAA x4 60 µs. The
  full-screen passes scale with pixel count, so they grow with resolution.
- **Assets are well under a frame.** `biplane.obj` parses in ~122 µs; `biplane.bmp`
  decodes in ~93 µs.

## Reading the numbers

- **Physics timing.** An active scene is measured with `iter_batched`: the scene is
  built and warmed up in setup (untimed) so the timed step is taken in the
  developed, contact-loaded state. A freshly built scene is much cheaper to step
  than a developed one, and a long `iter` drifts toward sleep — both would lie.
  See the module comment in `benches/physics.rs`.
- **The peak is not the steady state.** A scene dropped into contact is most
  expensive a dozen or so steps in, before positional correction pushes the pile
  apart, and cheapest once it sleeps. The physics cases report the peak; `settle`
  reports the curve between them. Quote whichever one the question is about.
- **The zeros in `memreport` are the interesting ones.** A settled `World` stepped
  a hundred times and a `Renderer` drawing a hundred frames both report **zero
  allocations**: both reuse their buffers. Any nonzero here is a real per-call
  allocation, not a one-time setup cost.
- **Absolute numbers are machine-, toolchain-, and scene-dependent.** The numbers
  cited in `docs/physics-spec.md` §12 were measured on one machine; reproduce them
  with `cargo bench -p he3d-bench --bench physics` before comparing.

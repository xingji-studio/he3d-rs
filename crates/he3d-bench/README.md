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
| `benches/renderer.rs` | `Renderer::new`, resolution / triangle-count / object-count sweeps, per-fragment fill (a screen-filling quad, textured and constant-colour), and each anti-aliasing mode |
| `benches/frame.rs` | One frame of two thousand Bodies stepped and drawn together, plus the draw alone and the pose copy alone: `docs/physics-spec.md` §14 risk 4 |
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
- **The renderer used to allocate per visible triangle; now it allocates once per
  frame.** Drawing the 8-cube scene at 640x360 a hundred times makes 100 allocations
  — one per frame, `self.missing_texture.clone()` (`renderer.rs:232`). Each
  front-facing triangle used to cost one `Vec` from `clip_triangle_to_near`
  (~48 per frame); clipping now writes into a fixed four-vertex stack buffer, so at
  60 Hz that is ~60 allocator calls per second instead of ~3000.
- **Decoding is one shot.** `decode_image` of the 256x256 BMP is a single 256 KB
  allocation; a 10 000-triangle static mesh costs ~134 bytes per triangle.

### Speed

- **Physics cost tracks contacts, not Bodies.** 2000 Bodies out of contact step in
  0.23 ms; the same 2000 packed in contact peak at 19 ms and settle to 6.3 ms.
  Recorded in `docs/physics-spec.md` §12.
- **Simulating and drawing share a CPU additively.** One frame of two thousand Bodies
  stepped and drawn at 1280x720 is 27.3 ms — a 19.0 ms step, a 9.0 ms draw, and a
  0.01 ms pose copy — against a 28.0 ms sum of the parts. Geometry and physics were
  profiled separately until `benches/frame.rs` put them in one frame; doing so costs no
  more than the two apart, and about 37 fps is the ceiling at the packed peak. This is
  `docs/physics-spec.md` §14 risk 4, measured.
- **Sleep is decided by convergence, and convergence fails with height.** `settle`
  pokes at a lattice of unit cubes layered on a slab. Six layers settle around step
  1100, sleep, and then cost 0.05 ms for the whole pile. Eight layers sit on the edge:
  at the default 12 iterations the pile never converges (a few Bodies hold 0.05–0.07
  m/s against the 0.05 m/s threshold), whereas 16, 20, 24, 28, 32, 40 and 48
  iterations all reach sleep. Ten layers needs 64; twelve does not converge even at 96
  and sheds edge Bodies. This is the tall-stack limitation of §14 risk 7, measured.
- **Correct warm-start identity costs about 10% of a dense lattice's peak, and buys
  back the convergence that was non-monotone.** Matching a contact point to its own
  previous step by the geometry that produced it — rather than by its rank in the
  step's list, which renumbers the survivors whenever a manifold shrinks — leaves the
  pile holding ~20% more Contacts at the peak step. The twelve-layer peak went 17 → 19
  ms and its settled cost 5.5 → 6.3 ms; the eight-layer pile, which used to converge
  only at 16, 24, 32 and 48 iterations, now converges at every count from 16 up.
- **The velocity solve was arithmetic-bound, and the inertia rotation was most of
  it.** Each impulse recomputed two quaternion rotations to turn `anchor × axis` into
  angular velocity. Storing `inv_inertia(anchor × axis)` per point instead (ADR 0004)
  cut the per-iteration cost by about 60%: the packed 2000-Body lattice went from
  32 ms to 17 ms at the same contact count, and `static_terrain/100` halved. `freefall`
  and `settled_asleep` did not move, which is the point — the win is where the contacts
  are, and nowhere else.
- **A two-point block solve costs ~3% and cannot fix a stack.** Two points solved in
  sequence settle on unequal impulses, and their difference is a torque about the axis
  they share — a cube rolled onto its edge picks up a spin nothing applied. Inverting
  the two normal constraints as one 2×2 block removes it, for about 3% on the packed
  lattice. It does not move the tall-stack boundary: that limit is the load between
  layers, not coupling inside one contact.
- **A renderer frame is pixels, and the hard floor is the clear.** An empty 720p
  frame — clearing colour + inverse depth, ~7.4 MB — costs 82.5 µs, which is
  0.089 ns per pixel at 64x36, 320x180, and 1280x720 alike. Drawing the 8-cube scene
  raises the 720p frame to 709 µs; the 626 µs difference is pixel fill (16x the
  pixels from 320x180 buys 12x the fill). Geometry *count* is not the driver: 1000
  objects still cost only 391 µs at 64x36, because their triangles are small.
- **Textured fill is texture sampling, not the depth test.** A single quad covering
  the 1280x720 framebuffer costs 7.17 ms when textured and 1.90 ms when its texture is
  a 1x1 constant — two triangles either way, so the 5.3 ms difference is
  `Texture::sample` plus shading, about 5.7 ns per pixel (measured split: ~4.0 ms
  sampling, ~1.8 ms shading). Both are throughput-bound scalar code, so most of a
  full-screen textured frame is spent there; the depth test, the perspective divide,
  and the colour and depth writes are the rest.
- **`Renderer::new` pays page faults, once.** Allocating the 720p buffers costs
  797 µs — nearly ten times the 83.6 µs it takes to clear those same buffers
  afterwards — because fresh pages are faulted in on first touch. The per-frame cost
  is the 83.6 µs, not the 797 µs.
- **Per-triangle setup shows up when triangles are small.** 10 000 sub-pixel
  triangles at 64x36 cost 530 µs — 53 ns each, now that a triangle costs no heap
  allocation and one composed rotation per vertex instead of two.
- **Anti-aliasing multiplies pixel or sample work.** At 64x36: none 7.3 µs, MSAA
  31.5 µs (x4 samples), FXAA 21 µs (a full-screen pass), TAA 44 µs, SSAA x4 57 µs. The
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

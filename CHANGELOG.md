# Changelog

All notable changes to the HE3D workspace crates are documented here.

## [Unreleased]

### Added
- `crates/he3d-bench`: a `frame` criterion target that steps and draws one scene of
  two thousand Bodies in the same frame, closing `docs/physics-spec.md` §14 risk 4 —
  physics and the software rasterizer had only ever been profiled separately. The
  combined frame is 27.3 ms at 1280x720: a 19.0 ms step, a 9.0 ms draw, and a 0.01 ms
  application-owned pose copy, against a 28.0 ms sum of the parts, so the two share one
  CPU additively rather than for worse. `workloads.rs` gains `sim_render_scene`, which
  pairs each Body with the object that draws it and exposes the per-frame `sync`. §12's
  frame-budget paragraph and §14 risk 4 now carry the measured numbers.

### Changed
- `he3d-renderer`: `draw_mesh` no longer recomputes the camera's inverse rotation, the
  view-space light direction, the pixel scale (with its `tan`), the viewport
  half-extents, and the near plane for every RenderObject; a private `Frame` computes
  them once a frame. Measured at roughly 1-3% on the object- and triangle-heavy
  renderer arms and within run-to-run noise on the fill-bound ones — small, and not the
  10-20% the earlier estimate guessed — so the change is kept as structural rather than
  for the number. Behaviour is unchanged.
- `CONTEXT.md`: scanline rasterization is recorded as ruled out, with the ceiling it
  was measured against. No code changed. The sign test it would skip costs about 19%
  of the fill-bound arm and 1.7% of the many-small-triangle arm, and removing it
  without changing a pixel is not possible, so the win is not worth a new coverage
  rule.

## [0.5.0] - 2026-10-18

### Breaking
- `he3d-core`: `Camera` now carries a `Projection` instead of a bare `fov_y_degrees`,
  and `Camera::new` is replaced by `Camera::perspective` and `Camera::orthographic`.
  The projection is an enum so that a perspective field of view cannot exist on an
  orthographic camera; a call `Camera::new(position, orientation, fov)` becomes
  `Camera::perspective(position, orientation, fov)`.

### Added
- `he3d-core` / `he3d-renderer`: an orthographic projection. `Camera::orthographic`
  takes a world-unit view height — a length, not an angle — and the framebuffer's
  aspect ratio supplies the width, mirroring how a perspective camera's field of view
  and aspect supply the whole view box. Rasterizing an orthographic triangle
  interpolates texture coordinates linearly instead of perspective-correctly, because
  that projection is affine and `1/z` is not; a test on a triangle spanning a 3× depth
  range pins the two apart. Depth keeps the reciprocal view depth, and back-face
  culling uses the constant `+Z` view axis because the eye sits at infinity.
  `he3d-bench` gains a `render/ortho/tex/{w}x{h}` fill-rate arm beside the perspective
  one.

### Changed
- The four `examples/` demos are `publish = false`, like `he3d-bench`. They exist to be
  run from this repository, and each names an SDL3 backend, so publishing one would make
  SDL3 a build requirement for anyone who merely resolved it. `cargo publish --workspace`
  now skips them instead of sweeping them in.
- `CONTEXT.md` gains an **Open work** section, the one part of that file which is not
  vocabulary, and its charter sentence now says so. It carries the renderer's landed
  optimizations and their measurements, the `floor`-to-truncation attempt that was ruled
  out, the three candidates left unmeasured (scanline rasterization, an interleaved
  colour/depth buffer, hoisting the per-frame constants out of `draw_mesh`), and
  where the §5 comment migration stands.
- `he3d-core` / `he3d-renderer`: the `# Errors` and `# Panics` sections whose Chinese
  line preceded its English translation are swapped, which is the last way those two
  files disagreed with `AGENTS.md` §5's English-first rule.
- `he3d-core`: the four primitive constructors' docs now say what they build rather
  than only which plane they lie in. `Mesh::triangle` is an isosceles triangle whose
  base spans `width` at `y = -height / 2` and whose apex sits at `(0, height / 2)`,
  so the bounding box is centred on the origin while the centroid sits a sixth of the
  height below it; its flat normal is `-Z` and its UVs are that bounding box's
  corners. `Mesh::plane` is centred on the origin with a `+Y` normal, and its two
  triangles' UVs tile the unit square once. `Mesh::cube` and `Mesh::sphere` are
  centred on the origin with outward normals, the cube giving each face one unit
  square of UVs and the sphere using longitude and latitude. The rest of `mesh.rs`'s
  comments are unwrapped to one sentence per line, which `AGENTS.md` §5 asks of a
  file being touched. No code changed.
- `he3d-renderer`: `rasterize_textured` is split into a `ScreenTriangle` (the
  triangle's raster-space edge and depth/texture coefficients), a `Shading` (the
  per-triangle fixed-point light and a 1x1 texture's pre-shaded colour) and a
  `resolve_msaa_pixel` that owns the four samples of one pixel. The loop that remains is
  66 lines instead of the 102 that had forced an `#[expect(clippy::too_many_lines)]`,
  which is gone. Behaviour is unchanged.
- Every file this cycle has reached now follows `AGENTS.md` §5: the facade crate's docs,
  both flight-simulator sources, the biplane example, `he3d-core`'s and `he3d-physics`'s
  crate docs, `he3d-assets`'s `obj.rs` and `image/mod.rs`, and the two-sentence Chinese
  line the orthographic commit left behind in `he3d-core`'s `camera.rs`. Wrapped
  sentences are joined, sentences sharing a line are split, and a paragraph's Chinese
  sentences now follow its English ones. The English text and the Chinese text are each
  preserved character for character.
- Standard rustdoc section headings — `# Examples`, `# Errors`, `# Panics` — stay
  English-only, and a Chinese line beneath one is that heading's translation rather
  than a second heading. Only the facade crate's custom headings (`# Where the code
  is`, `# The binary chooses the backend`) are bilingual, because they are prose
  headings with no rustdoc keyword. This reverses the previous cycle's `# 示例`, which
  promoted the Chinese translation to a heading of its own in six files.

### Fixed
- `examples/physics-test`: a held arrow no longer flings the cube off the platform. The
  control applied its 6 N·m torque on every step and relied on 0.05 angular damping to
  bound the spin, whose balance sits in the hundreds of radians per second; it now drives
  the roll toward a 6 rad/s target and stops adding torque once there. A test spins a free
  cube for 20 s, holds the roll near the target, and fails at 455 rad/s without the guard.

## [0.4.0] - 2026-10-17

### Breaking
- **Physics results differ from 0.3.0, so a scene does not replay identically across
  the two.** Two solver defects were fixed and both change the trajectory of anything
  with two-point contacts; the engine stays deterministic, but only within a version.
  A Body pivoting on an edge no longer picks up a spin about that edge — a 2-point
  manifold's two normal constraints are solved as one 2×2 block instead of in sequence
  (2.04e-4 rad/s → 4.2e-7 at the default 12 iterations) — and warm starting now matches
  a contact point by the geometry that produced it rather than by its rank in the
  manifold, so a point inherits the impulse it earned instead of one a different corner
  had accumulated. The second fix reaches `Contact::phase` as well, because phases are
  derived from that same match: which contacts report `Persist` rather than
  `Begin`/`End` differs. Measured on the benchmark workloads, the packed 2000-Body
  lattice peaks at 19 ms instead of 17 ms and settles to 6.3 ms instead of 5.5 ms, and
  the eight-layer pile's convergence is no longer non-monotone in the iteration count.
  The two `he3d-physics` entries under **Changed** carry the full measurements.

### Added
- `crates/he3d-bench`: a criterion benchmark suite (physics, renderer, assets,
  core/math) and a `memreport` binary that counts heap allocations through a
  counting global allocator and prints `size_of` for the core types. The crate is
  `publish = false` and depends on no backend crate, so it builds and runs headless.
  CI runs it with `cargo bench -p he3d-bench -- --test` (smoke only).
- `he3d-bench`'s `settle` binary: cost as a function of step index beside the Contact
  count and the number of sleeping Bodies, which is how "does this pile ever settle
  and sleep" is measured rather than assumed.
- `.github/workflows/ci.yml`: a `bench` job that compiles and smoke-runs the suite.
- `he3d-bench`'s renderer target: a fill-rate sweep
  (`render/fill/{solid,tex}/{320x180,1280x720}`) of one screen-filling quad, so a
  frame's cost is the fixed clear plus one per-fragment cost per pixel. The `solid`
  and `tex` arms separate the constant-texture fast path from sampling a real
  texture. This is the pair that shows where a flight-simulator-style textured
  terrain spends its time.

### Changed
- `he3d-renderer`: a triangle is clipped into a fixed four-vertex stack buffer
  instead of a fresh `Vec`, which removes the only per-triangle allocation in a frame
  (`memreport` on the 8-cube scene: ~49 allocations per frame → 1, the remaining one
  being the per-frame `missing_texture` clone). 10 000 small triangles at 64x36:
  611 µs → 530 µs, and 1000 objects: 496 µs → 391 µs.
- `he3d-renderer`: `draw_mesh` composes the object rotation and the world→view
  rotation into a single quaternion and folds the two translations into one vector, so
  each vertex costs one rotation instead of two. Shading moves to view space, where
  the light direction is rotated once per mesh, so the dot product is unchanged. This
  is what carries most of the per-object win above.
- `he3d-renderer`: a 1x1 texture is constant, so its shaded colour is hoisted out of
  the per-pixel loop and the branch is never paid by an ordinary texture. A
  full-screen solid-colour quad at 1280x720 fell from 7.81 ms to 1.93 ms; the
  textured quad is unchanged by this.
- `he3d-core`: `Texture::sample` drops a branch that `x - floor(x)` can never take
  (the fraction is in `[0, 1)` for every finite coordinate) and indexes with 32-bit
  arithmetic. A full-screen 256x256-textured quad at 1280x720 went from 7.81 ms to
  7.45 ms.
- `he3d-core`/`he3d-renderer`: the per-fragment path drops two clamp lower bounds
  that cannot be reached — a texel index is never negative because the wrapped
  coordinate is in `[0, 1)`, and a shaded channel is never negative because the light
  is clamped to `0..=512`. The same textured quad fell from 7.45 ms to 7.17 ms.
  Replacing `floor` with a truncation-based wrap was tried and rejected: it was ~25%
  slower end-to-end (9.70 ms), because the `f32 → i32 → f32` round trip costs more
  than the `floor` the compiler inlines. With that, `sample` plus `shade` is
  throughput-bound — measured, sampling is ~4.0 ms of the 7.17 ms and shading ~1.8 ms
  — so the sampler is no longer the lever on a textured frame.
- `docs/physics-spec.md` §12 now carries numbers measured by that suite, replacing
  the earlier unmeasured figures, and distinguishes a packed scene's **peak** step
  from its **settled** one. §14 risk 7's tall-stack limit is measured rather than
  assumed: a six-layer pile settles and sleeps at the default iterations, an
  eight-layer pile needs 16 or more, and ten layers and taller do not converge at any
  practical count. This corrects an earlier claim that no iteration count could settle
  an eight-layer pile, which was an artifact of reading a 1500-step `settle` window as
  if it were the whole run.
- `docs/physics-spec.md` §14 risk 7 also records what was tried and ruled out for that
  limit, so the dead ends are not walked again: sweep reordering (no effect, in any of
  the three orders tried), the positional-correction pass (not the cause), and a block
  solver over a manifold's four points (its linear system is exactly
  singular, the checkerboard impulse pattern being in its null space). No code changed.
- `he3d-physics`: each constraint point now stores `inv_inertia(anchor × axis)` for
  all three solve axes, so a velocity iteration applies an impulse with a scalar
  multiply instead of two quaternion rotations. It costs 72 bytes per point (about
  1.7× the constraint array) and bought roughly 46% off a packed 2000-Body lattice —
  32 ms → 17 ms at the developed peak — while `freefall` and `settled_asleep` were
  unchanged. The solve is mathematically the same, but reassociating the arithmetic
  moves a chaotic pile's trajectory: the eight-layer pile's residual speed went from
  0.16 to 0.11 m/s, still well above the sleep threshold, and the six-layer pile still
  settles and sleeps. See `docs/adr/0004`.
- `he3d-physics`: the velocity and position solvers now resolve each constraint's
  pair of Bodies once per constraint instead of once per point and per axis, behind a
  `ContactPair` type. Behaviour and timing are both unchanged — an A/B of the packed
  lattice puts the two within run-to-run noise, because the loop is arithmetic-bound
  — so this is a structural change: the win is that the resolve-once invariant is now
  impossible to violate rather than merely observed.
- `he3d-physics`: a two-point manifold's two normal constraints are now inverted
  together as one 2×2 block (`ContactBlock`), with the friction solve placed *before*
  it so the block's answer is the iteration's final word. Solving the two points in
  sequence settles them on unequal impulses, and their difference is a torque about the
  axis the two points share — which is exactly a cube rolled onto its edge, and was
  measured as a phantom spin of 2.04e-4 rad/s. Three- and four-point manifolds keep the
  per-point path, byte for byte: their block is singular and their points are coplanar,
  so reordering them buys nothing and costs ~4% on a contact-dense lattice. A singular
  or NaN block falls back to the per-point path. Cost: ~3% on the packed lattice
  (`step/lattice_on_slab/2000`), and no extra iteration count.
- `he3d-physics`: warm-start identity is now the geometry a contact point came from,
  not its rank in the step's list. A corner of the source polygon is its own identity;
  a vertex inserted by a clip is named by the vertex it was cut from and the plane that
  cut it, tagged with the shape that supplied the polygon, so a reference-face flip
  misses a warm start instead of applying it to the wrong point. Ranking renumbers the
  survivors whenever a manifold shrinks — a cube rolling onto its edge drops from four
  points to two — which hands a point the impulse a different corner had accumulated.
  Measured: the packed lattice holds ~20% more Contacts at the peak step, so the
  twelve-layer peak went 17 ms → 19 ms and its settled cost 5.5 ms → 6.3 ms; and the
  eight-layer pile's convergence is no longer non-monotone (16, 20, 24, 28, 32, 40 and
  48 iterations all reach sleep, where 20, 28 and 40 used to fail). The positional
  order is kept, as the solver's sweep order only.
- `he3d-bench`: `settle`'s summary now runs each reference pile for 2000 steps instead
  of 1500. The shorter window reported "does not sleep" for a six-layer pile that does
  sleep, just after step 1100; the window has to outlast the shortest pile in the table
  for the sleep column to mean anything.
- `docs/physics-spec.md` §12 and §14 risk 7, `docs/adr/0004`, and
  `crates/he3d-bench/README.md` are re-measured against the above: the packed
  2000-Body lattice peaks at 19 ms and settles to 6.3 ms (was 17 ms / 5.5 ms), the
  cell-size sweep is 25.8 / 18.7 / 19.2 ms, and the six-layer pile sleeps around step
  1100. §14 risk 7 now lists the two changes that landed without moving the tall-stack
  boundary, and its positional-correction entry is re-measured: with the pass off the
  eight-layer pile holds 0.38 m/s at step 1500 instead of 0.07, so the pass is doing its
  job more clearly than before, not less.
- `examples/physics-test`: the controls separate translation from rotation, which the
  `A`/`D` keys used to couple. `A`/`D` now apply only force, and the left and right
  arrows apply only torque (6.0 about the horizontal Z axis, up from 4.0); space still
  jumps and `R` still resets. A cube at rest resists a roll until the torque exceeds
  `m·g·half_side ≈ 4.9`, which is why 4.0 could not turn it on its own. The arrows are
  SDL keycodes, so they are tracked apart from the ASCII `keys` array.

### Removed
- The tutorial documentation and the `he3d-docs` crate.

## [0.3.0] - 2026-09-22

### Breaking
- **The physics crate was rewritten and no longer depends on `gp3d`.**
  `he3d-physics` depends only on `he3d-math`; a Body is identified by its own
  `BodyId`, and the application owns both the Body-to-render-object association
  and the per-frame copy of the pose. It never sees a `Scene`, an `ObjectId`,
  or a `Transform`. Contact phases, warm starting, and per-Island sleep are part
  of the rewrite (see `docs/adr/0001`-`docs/adr/0004`).
- **The binary chooses the backend; no Cargo feature does.** `he3d` depends on no
  backend crate, optional or otherwise, and its `std` feature now means only
  "compile the library crates with `std`". Depend on `he3d-platform-sdl3` or
  `he3d-platform-std` and construct `SdlBackend::new` / `StdBackend::new` in
  `main`. See `docs/adr/0005`.
- `he3d::create_backend`, `he3d::BackendInitError`, and
  `he3d_platform::PlatformFactory` are gone: each backend has an inherent `new`
  with its own error type, so a construction failure stays specific to the backend
  that produced it.
- `he3d_platform_sdl3::BackendError` is now `SdlBackendError`.

### Changed
- The `he3d` crate documentation no longer contains code blocks; the complete
  programs live in `examples/`.
- The facade's headless smoke test moved to `he3d-platform-std`, where the backend
  it drives lives.

## [0.2.0] - 2026-08-24

> Note: `he3d-physics` 0.2.0 transitively depends on the yanked `gp3d` 0.2.0, so fresh resolves of that version fail. Existing `Cargo.lock`s are unaffected; point new projects at 0.3.0.

### Breaking
- `#[non_exhaustive]` added to all public enums and config structs:
  - `he3d-core`: `Error`
  - `he3d-platform`: `Event`, `PlatformError`, `BackendConfig`
  - `he3d-renderer`: `AntiAliasing`, `RendererConfig`
  - `he3d-platform-std`: `StdBackendError`
  - `he3d-platform-sdl3`: `BackendError`
  - `he3d`: `BackendInitError`
- `BackendConfig::new(title, width, height)` is now the canonical constructor
  (literal construction is no longer possible).

### Added
- `create_backend` collects each failing backend's reason into
  `BackendInitError` instead of swallowing it.
- `From<StdBackendError>` / `From<BackendError>` conversions and a
  `std::error::Error` impl for `BackendInitError` (std feature).
- `Debug` impls for `Renderer`, `StdBackend`, `SdlBackend`.

### Quality
- Workspace lints: `clippy::pedantic` and `missing_docs` now enforced with
  `-D warnings` in CI.
- `#![forbid(unsafe_code)]` on all unsafe-free crates; `// SAFETY:` comments on
  every remaining unsafe block.
- Bilingual documentation (`# Errors`, `# Panics`, examples) across the public API.

## [0.1.0] - 2026-08-20

- Initial release: `he3d`, `he3d-math`, `he3d-core`, `he3d-renderer`,
  `he3d-assets`, `he3d-physics`, `he3d-platform`, `he3d-platform-std`,
  `he3d-platform-sdl3`.

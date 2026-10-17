# Changelog

All notable changes to the HE3D workspace crates are documented here.

## [Unreleased]

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
  eight-layer pile needs 16 or more and the outcome is not monotone in the count, and
  ten layers and taller do not converge at any practical count. This corrects an
  earlier claim that no iteration count could settle an eight-layer pile, which was an
  artifact of reading the 1500-step `settle` window as if it were the whole run.
- `docs/physics-spec.md` §14 risk 7 also records what was tried and ruled out for that
  limit, so the dead ends are not walked again: sweep reordering (no effect, in any of
  the three orders tried), the positional-correction pass (not the cause — disabling it
  only jams the pile at 0.15 m of penetration, and fewer position iterations is worse),
  and a block solver over a manifold's four points (its linear system is exactly
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

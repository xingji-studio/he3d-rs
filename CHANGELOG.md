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

### Changed
- `docs/physics-spec.md` §12 now carries numbers measured by that suite, replacing
  the earlier unmeasured figures, and distinguishes a packed scene's **peak** step
  from its **settled** one. §14 risk 7's tall-stack limit is measured rather than
  assumed: a six-layer pile settles and sleeps at the default iterations, an
  eight-layer pile needs 16 or more and the outcome is not monotone in the count, and
  ten layers and taller do not converge at any practical count. This corrects an
  earlier claim that no iteration count could settle an eight-layer pile, which was an
  artifact of reading the 1500-step `settle` window as if it were the whole run.
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

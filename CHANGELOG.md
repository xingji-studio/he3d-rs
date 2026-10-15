# Changelog

All notable changes to the HE3D workspace crates are documented here.

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

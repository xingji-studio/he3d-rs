# Changelog

All notable changes to the HE3D workspace crates are documented here.

## [0.2.0] - 2026-08-23

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

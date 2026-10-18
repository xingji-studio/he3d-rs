[中文](README.zh.md)

# HE3D (Rust)

[![crates.io](https://img.shields.io/crates/v/he3d.svg)](https://crates.io/crates/he3d)
[![docs.rs](https://docs.rs/he3d/badge.svg)](https://docs.rs/he3d)
[![CI](https://github.com/xingji-studio/he3d-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/xingji-studio/he3d-rs/actions/workflows/ci.yml)

A `no_std + alloc` software 3D engine written in Rust: rasterizer, scene graph, assets,
physics, and platform backends (right-handed, +Z forward, inverse-depth buffer).

## Crates

| Crate | Purpose |
|---|---|
| `he3d` | **Single entry point** — re-exports the crates below except the backends; `cargo add he3d` |
| `he3d-math` | Coordinate conventions and linear algebra via `glam` |
| `he3d-core` | Scene graph, resources, materials, stable generational IDs |
| `he3d-renderer` | Software rasterizer and post-processing |
| `he3d-assets` | OBJ / glTF / BMP / PNG / JPEG byte-based decoders |
| `he3d-physics` | Rigid-body World: uniform-grid broadphase, analytic and triangle narrowphase, sequential-impulse solver, Island sleep |
| `he3d-platform` | The `Platform` trait: backend-neutral input / presentation / timing |
| `he3d-platform-std` | `StdBackend`, presenting to standard output (stdout) |
| `he3d-platform-sdl3` | `SdlBackend`, presenting to an SDL3 window |
| `he3d-bench` | **Internal** criterion benchmarks and a heap report; `publish = false` |

Every library crate is `no_std + alloc` when built without `std`, and the optional
`std` feature adds std-backed impls (such as `std::error::Error`) while enabling
`std` in the crates it depends on.

## Quick start

```sh
cargo run -p he3d-triangle                    # SDL3 window
cargo run -p he3d-physics-test                # SDL3 window, A/D push / arrows spin / space / R
cargo run -p he3d-biplane                     # SDL3 window, loads OBJ + BMP
cargo run -p he3d-flight-simulator            # SDL3 window, streamed terrain
```

Each demo names its backend type in `main` and depends on that backend's crate
directly. Nothing in the engine picks a backend for the application:

```rust
use he3d_platform_sdl3::SdlBackend;

let mut backend = SdlBackend::new(BackendConfig::new("demo", 640, 360))?;
```

A backend is whatever implements `Platform` from `he3d-platform`; `he3d` depends
on no backend crate. To run without a display, depend on `he3d-platform-std` and
construct `StdBackend` instead; the same rendered pixels become half-block
characters on standard output, and `--frames N` limits the run (useful headless).
See [ADR 0005](docs/adr/0005-backend-chosen-by-the-binary.md).

## Build & test

```sh
cargo test --workspace
cargo clippy --workspace --all-targets
cargo fmt --all --check
# no_std (bare-metal) checks
cargo check -p he3d-math -p he3d-core -p he3d-renderer \
            -p he3d-assets -p he3d-physics -p he3d-platform --no-default-features
```

`he3d-physics` depends only on `he3d-math`: it never sees a `Scene`, an `ObjectId`,
or a `Transform`. A `Body` carries its own `BodyId`, and the application owns both
the link between a Body and whatever draws it and the per-frame copy of the pose.
The feature set it plans to grow into, and the limits it currently has, are in
[`docs/physics-spec.md`](docs/physics-spec.md).

`scripts/check-deps.sh` verifies that every crate cargo unpacked into the
registry still matches the checksummed archive it came from; run it when a build
error points at dependency source rather than at this workspace.

## Benchmarks

Performance claims are measured, not asserted. `crates/he3d-bench` holds a
criterion suite and a heap report; it is `publish = false`, depends on no backend
crate, and so builds and runs headless.

```sh
cargo bench -p he3d-bench                  # full run: physics, renderer, frame, assets, core/math
cargo bench -p he3d-bench -- --test        # smoke: every case once (what CI runs)
cargo run  -p he3d-bench --bin memreport   # heap allocation + size_of tables
cargo run  -p he3d-bench --bin settle      # cost vs step index, and sleep state
```

`memreport` installs a counting global allocator and prints, per scenario, the
number of allocations, the bytes requested, and the peak added. It is how "resting
Bodies cost nothing" is checked: a settled `World` stepped a hundred times, and a
`Renderer` drawing a hundred frames, both report **zero allocations** — both reuse
their buffers. The measured numbers behind [`docs/physics-spec.md`](docs/physics-spec.md)
§12 come from these benches.

## Publishing

Crates are published in dependency order (`he3d-math` → `he3d-core` →
`he3d-renderer` → `he3d-assets` → `he3d-platform` → `he3d-platform-std` →
`he3d-platform-sdl3` → `he3d-physics` → `he3d`). See `cargo package --list` before
each release.

`he3d-bench` and the four `examples/` demos are `publish = false`, so
`cargo publish --workspace` skips them.

## Attributions

The biplane model (`biplane.obj`) used by the examples is taken from
[Sebastian Lague's Software-Rasterizer](https://github.com/SebLague/Software-Rasterizer)
(MIT License, Copyright (c) 2025 Sebastian Lague). Its texture (`biplane.bmp`)
is an original asset of the HE3D project, redistributed via the
[he3d-cpp](https://github.com/xingji-studio/he3d-cpp) reference renderer. See
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) for the full license text.

## License

MIT

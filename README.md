# HE3D (Rust)

[![crates.io](https://img.shields.io/crates/v/he3d.svg)](https://crates.io/crates/he3d)
[![docs.rs](https://docs.rs/he3d/badge.svg)](https://docs.rs/he3d)
[![CI](https://github.com/xingji-studio/he3d-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/xingji-studio/he3d-rs/actions/workflows/ci.yml)

A `no_std + alloc` software 3D engine: rasterizer, scene graph, assets, physics, and
platform backends — a Rust port of the [HE3D](https://github.com/xingji-studio/HE3D)
reference renderer (right-handed, +Z forward, inverse-depth buffer).

## Crates

| Crate | Purpose |
|---|---|
| `he3d` | **Single entry point** — depends on and re-exports every crate below; `cargo add he3d` |
| `he3d-math` | Coordinate conventions and linear algebra via `glam` |
| `he3d-core` | Scene graph, resources, materials, stable generational IDs |
| `he3d-renderer` | Software rasterizer and post-processing |
| `he3d-assets` | OBJ / glTF / BMP / PNG / JPEG byte-based decoders |
| `he3d-physics` | `ObjectId` adapter over the [gp3d](https://github.com/xingji-studio/gp3d-rs) engine |
| `he3d-platform` | Backend-neutral input / presentation / timing traits |
| `he3d-platform-std` | Rust standard-library backend, presenting to standard output (stdout) |
| `he3d-platform-sdl3` | SDL3 window backend |

All crates are `no_std + alloc` by default; the optional `std` feature only adds
convenience impls (`std::error::Error`) and never enables `std` for dependencies.

## Quick start

Presentation backends are opt-in features on `he3d` and interchangeable:

```sh
cargo run -p he3d-triangle --no-default-features --features std   # stdout backend
cargo run -p he3d-physics-test                                    # SDL3 window (default)
cargo run -p he3d-physics-test --no-default-features --features std -- --frames 60
cargo run -p he3d-flight-simulator --no-default-features --features std -- --frames 60
```

`he3d::create_backend` picks among the enabled backends at runtime (sdl3 first,
then std), so application code never names a backend type. `--frames N` limits
the run (useful headless).

## Build & test

```sh
cargo test --workspace
cargo clippy --workspace --all-targets
cargo fmt --all --check
# no_std (bare-metal) checks
cargo check -p he3d-math -p he3d-core -p he3d-renderer \
            -p he3d-assets -p he3d-physics -p he3d-platform --no-default-features
```

`he3d-physics` depends on the standalone [gp3d](https://github.com/xingji-studio/gp3d-rs)
engine; CI checks it out to `../gp3d-rs`.

## Publishing

Crates are published in dependency order (gp3d first, then `he3d-math` → `he3d-core`
→ `he3d-renderer` → `he3d-assets` → `he3d-platform` → `he3d-platform-std` →
`he3d-platform-sdl3` → `he3d-physics`). See `cargo package --list` before each release.

## Attributions

The biplane model and texture used by the examples are taken from
[Sebastian Lague's Software-Rasterizer](https://github.com/SebLague/Software-Rasterizer)
(MIT License, Copyright (c) 2025 Sebastian Lague). See
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) for the full license text.

## License

MIT

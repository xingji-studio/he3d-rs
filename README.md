# HE3D (Rust)

A `no_std + alloc` software 3D engine: rasterizer, scene graph, assets, physics, and
platform backends — a Rust port of the [HE3D](https://github.com/xingji-studio/HE3D)
reference renderer (right-handed, +Z forward, inverse-depth buffer).

## Crates

| Crate | Purpose |
|---|---|
| `he3d-math` | Coordinate conventions and linear algebra via `glam` |
| `he3d-core` | Scene graph, resources, materials, stable generational IDs |
| `he3d-renderer` | Software rasterizer and post-processing |
| `he3d-assets` | OBJ / glTF / BMP / PNG / JPEG byte-based decoders |
| `he3d-physics` | `ObjectId` adapter over the [gp3d](https://github.com/xingji-studio/gp3d-rs) engine |
| `he3d-platform` | Backend-neutral input / presentation / timing traits |
| `he3d-platform-std` | Terminal framebuffer backend (Unix) |
| `he3d-platform-sdl3` | SDL3 window backend |

All crates are `no_std + alloc` by default; the optional `std` feature only adds
convenience impls (`std::error::Error`) and never enables `std` for dependencies.

## Quick start

```sh
cargo run -p he3d-triangle --features terminal        # terminal demo
cargo run -p he3d-physics-test                        # SDL3 window (default)
cargo run -p he3d-physics-test --features terminal -- --frames 60
cargo run -p he3d-flight-simulator --features terminal -- --frames 60
```

`--frames N` limits the run (useful headless); backends are chosen with the
`terminal` / `sdl3` features.

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

## License

MIT

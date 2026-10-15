[English](README.md)

# HE3D (Rust)

[![crates.io](https://img.shields.io/crates/v/he3d.svg)](https://crates.io/crates/he3d)
[![docs.rs](https://docs.rs/he3d/badge.svg)](https://docs.rs/he3d)
[![CI](https://github.com/xingji-studio/he3d-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/xingji-studio/he3d-rs/actions/workflows/ci.yml)

一个用 Rust 编写的 `no_std + alloc` 软件 3D 引擎：光栅化器、场景图、资源、物理与平台后端（右手坐标系，+Z 向前，反向深度缓冲）。

## Crates

| Crate | 用途 |
|---|---|
| `he3d` | **单一入口** — 重导出下面除后端外的所有 crate；`cargo add he3d` |
| `he3d-math` | 坐标约定与线性代数（基于 `glam`） |
| `he3d-core` | 场景图、资源、材质、稳定世代 ID |
| `he3d-renderer` | 软件光栅化器与后处理 |
| `he3d-assets` | OBJ / glTF / BMP / PNG / JPEG 字节解码器 |
| `he3d-physics` | 刚体 World：均匀网格 broadphase、解析与三角形 narrowphase、sequential-impulse 求解器、Island 休眠 |
| `he3d-platform` | `Platform` trait：后端无关的输入/呈现/计时 |
| `he3d-platform-std` | `StdBackend`，输出到标准输出（stdout） |
| `he3d-platform-sdl3` | `SdlBackend`，输出到 SDL3 窗口 |
| `he3d-bench` | **内部** criterion 基准与堆报告；`publish = false` |

每个库 crate 在不启用 `std` 时是 `no_std + alloc`，可选的 `std` 特性添加基于 std 的实现（如 `std::error::Error`），同时为其依赖的 crate 启用 `std`。

## 快速开始

```sh
cargo run -p he3d-triangle                    # SDL3 窗口
cargo run -p he3d-physics-test                # SDL3 窗口，WASD / 空格 / R
cargo run -p he3d-biplane                     # SDL3 窗口，加载 OBJ + BMP
cargo run -p he3d-flight-simulator            # SDL3 窗口，流式地形
```

每个 demo 在 `main` 中命名其后端类型，并直接依赖该后端的 crate。引擎本身不为应用选择后端：

```rust
use he3d_platform_sdl3::SdlBackend;

let mut backend = SdlBackend::new(BackendConfig::new("demo", 640, 360))?;
```

后端就是实现了 `he3d-platform` 中 `Platform` 的类型；`he3d` 不依赖任何后端 crate。要在无显示器环境下运行，依赖 `he3d-platform-std` 并构造 `StdBackend`；同样的渲染像素会变成半角字符输出到标准输出，`--frames N` 限制运行帧数（适合无头环境）。见 [ADR 0005](docs/adr/0005-backend-chosen-by-the-binary.md)。

## 构建与测试

```sh
cargo test --workspace
cargo clippy --workspace --all-targets
cargo fmt --all --check
# no_std（裸机）检查
cargo check -p he3d-math -p he3d-core -p he3d-renderer \
            -p he3d-assets -p he3d-physics -p he3d-platform --no-default-features
```

`he3d-physics` 只依赖 `he3d-math`：它永远看不到 `Scene`、`ObjectId` 或 `Transform`。`Body` 自带 `BodyId`，应用同时拥有 Body 与绘制它的对象之间的关联，以及每帧位姿的副本。它的能力规划与当前限制写在 [`docs/physics-spec.md`](docs/physics-spec.md)。

`scripts/check-deps.sh` 验证 cargo 解包到 registry 的每个 crate 是否仍与其校验和归档一致；当构建错误指向依赖源码而非本仓库时运行它。

## 基准测试

性能数字是量出来的，不是断言的。`crates/he3d-bench` 存放 criterion 基准套件与堆报告；它 `publish = false`、不依赖任何后端 crate，可无头构建与运行。

```sh
cargo bench -p he3d-bench                  # 完整运行：physics、renderer、assets、core/math
cargo bench -p he3d-bench -- --test        # 冒烟：每个用例跑一次（CI 运行的就是这个）
cargo run  -p he3d-bench --bin memreport   # 堆分配 + size_of 表
cargo run  -p he3d-bench --bin settle      # 成本随步数的变化，以及睡眠状态
```

`memreport` 安装一个计数型全局分配器，逐场景打印分配次数、请求字节数与新增峰值。"静止 Body 几乎不花钱"的说法就是这样检验的：一个已稳定的 `World` 连步一百次、一个 `Renderer` 连画一百帧，两者都报告**零分配**——因为两者都复用缓冲。[`docs/physics-spec.md`](docs/physics-spec.md) §12 的实测数字正来自这套基准。

## 发布

Crates 按依赖顺序发布（`he3d-math` → `he3d-core` → `he3d-renderer` → `he3d-assets` → `he3d-platform` → `he3d-platform-std` → `he3d-platform-sdl3` → `he3d-physics` → `he3d`）。每次发布前查看 `cargo package --list`。

## 署名

示例中使用的双翼机模型（`biplane.obj`）来自 [Sebastian Lague 的 Software-Rasterizer](https://github.com/SebLague/Software-Rasterizer)（MIT 许可证，Copyright (c) 2025 Sebastian Lague）。其纹理（`biplane.bmp`）是 HE3D 项目的原创资源，通过 [he3d-cpp](https://github.com/xingji-studio/he3d-cpp) 参考渲染器重新分发。完整许可证文本见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。

## 许可证

MIT

//! HE3D — a `no_std + alloc` software 3D engine.
//! HE3D —— 一个 `no_std + alloc` 软件 3D 引擎。
//!
//! This crate is the single entry point for the whole engine: it re-exports the
//! math, core scene graph, software renderer, assets, physics, and platform crates
//! under one name so `cargo add he3d` pulls everything.
//! 本 crate 是整套引擎的唯一入口：它在同一个名字下重导出数学、核心场景图、
//! 软件渲染器、资源、物理与平台 crate，因此 `cargo add he3d` 即可获得全部。
//!
//! # The binary chooses the backend
//! # 后端由二进制选择
//!
//! `he3d` depends on no backend crate, not even optionally. A backend is a type
//! that implements the [`Platform`] trait from [`he3d_platform`] — `SdlBackend` in
//! `he3d-platform-sdl3`, `StdBackend` in `he3d-platform-std`, or one written
//! outside this repository — and the program that owns `main` names that type,
//! depends on its crate, and constructs it. Nothing here chooses on the
//! application's behalf, and no Cargo feature selects a backend: features are
//! additive across a dependency graph, so such a feature would be decided by
//! whichever crate happened to enable it. See `docs/adr/0005`.
//! `he3d` 不依赖任何后端 crate，连可选依赖也不是。后端就是实现了 [`he3d_platform`] 中
//! [`Platform`] 的类型——`he3d-platform-sdl3` 的 `SdlBackend`、`he3d-platform-std` 的
//! `StdBackend`，或本仓库之外写的实现——由拥有 `main` 的程序点名该类型、依赖它的 crate
//! 并构造它。这里没有任何东西替应用选择，也没有哪个 Cargo feature 负责选后端：feature 在
//! 依赖图中是叠加的，那样的 feature 会由碰巧启用它的 crate 决定。见 `docs/adr/0005`。
//!
//! Depending on the SDL3 backend makes SDL3 a build requirement, because `sdl3-sys`
//! declares `links = "SDL3"`; the stdout backend needs nothing from the system.
//! 依赖 SDL3 后端会让 SDL3 成为构建前提，因为 `sdl3-sys` 声明了 `links = "SDL3"`；
//! stdout 后端不需要系统中的任何东西。
//!
//! The `std` feature of this crate means one thing only: compile the library crates
//! with `std`. It never selects a backend. Asset-format features forward to
//! [`he3d_assets`]: `png`, `jpeg`, and `gltf` opt into those decoders (BMP is always
//! enabled).
//! 本 crate 的 `std` 特性只有一个含义：给各库 crate 开 `std`。它从不选后端。资源格式
//! 特性转发到 [`he3d_assets`]：`png`、`jpeg`、`gltf` 可启用对应解码器（BMP 始终启用）。
//!
//! # Where the code is
//! # 代码在哪里
//!
//! The complete programs live in `examples/`, and CI compiles them:
//! `examples/triangle` is the minimal one, and the rest add physics, asset loading,
//! and streamed terrain, in that order.
//! 完整程序在 `examples/`，CI 会编译它们：`examples/triangle` 是最小的一个，其余依次加上
//! 物理、资源加载与流式地形。
//!
//! Between a Body and the thing that draws it, the engine provides no binding layer
//! and will not: the application keeps the association and copies the pose each
//! frame. That is what lets a Body exist without being rendered, and one render
//! object carry several Bodies. See `docs/adr/0001`.
//! 引擎不为 Body 与"画它的东西"之间提供绑定层，也不打算提供：关联由应用保存，位姿每帧由
//! 应用复制。正因如此，Body 可以存在而不被渲染，一个渲染对象也可以承载多个 Body。见
//! `docs/adr/0001`。

#![no_std]
#![forbid(unsafe_code)]

pub use he3d_assets;
pub use he3d_core;
pub use he3d_math;
pub use he3d_physics;
pub use he3d_platform;
pub use he3d_renderer;

// Commonly used types, flattened for `use he3d::*;`.
// 常用类型扁平化，方便 `use he3d::*;`。
// `#[doc(inline)]` keeps each type's own documentation at this path.
// `#[doc(inline)]` 让每个类型自身的文档显示在此路径下。
#[doc(inline)]
pub use he3d_assets::{decode_image, parse_obj};
#[cfg(feature = "gltf")]
#[doc(inline)]
pub use he3d_assets::{parse_glb, parse_gltf};
#[doc(inline)]
pub use he3d_core::{
   Camera, ColorRgba8, Error, MAX_VERTEX_COUNT, Material, Mesh, MeshId, ObjectId, Projection,
   RenderObject, Rgb, Scene, Texture, TextureId, Transform,
};
#[doc(inline)]
pub use he3d_math::{EulerRot, Mat4, Quat, Vec2, Vec3, Vec4};
#[doc(inline)]
pub use he3d_physics::{
   Aabb, BodyDesc, BodyId, Contact, ContactPartner, ContactPhase, MassProperties, MotionType,
   Shape, StaticMeshId, World,
};
#[doc(inline)]
pub use he3d_platform::{BackendConfig, Event, KEY_ESCAPE, KeyCode, Platform, PlatformError};
#[doc(inline)]
pub use he3d_renderer::{AntiAliasing, MAX_SSAA_SCALE, Renderer, RendererConfig};

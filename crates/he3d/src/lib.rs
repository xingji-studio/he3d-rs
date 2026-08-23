//! HE3D — a `no_std + alloc` software 3D engine.
//! HE3D —— 一个 `no_std + alloc` 软件 3D 引擎。
//!
//! This crate is the single entry point for the whole engine: it re-exports the
//! math, core scene graph, software renderer, assets, physics, and platform crates
//! under one name so `cargo add he3d` pulls everything.
//! 本 crate 是整套引擎的唯一入口：它在同一个名字下重导出数学、核心场景图、
//! 软件渲染器、资源、物理与平台 crate，因此 `cargo add he3d` 即可获得全部。
//!
//! Presentation backends are opt-in and interchangeable:
//! 呈现后端按需选择且可互换：
//!
//! * `std` — the Rust standard-library backend, presenting to the standard
//!   library's standard output (stdout). The name reflects that it targets the
//!   standard library, not any particular display.
//!   `std` —— Rust 标准库后端，呈现到标准库标准输出（stdout）。命名反映它面向
//!   标准库，而非任何特定显示设备。
//! * `sdl3` — the SDL3 window backend (requires a desktop OS).
//!   `sdl3` —— SDL3 窗口后端（需要桌面操作系统）。
//!
//! [`create_backend`] picks among the enabled backends at runtime (sdl3 first,
//! then the std backend), so application code never names a backend type.
//! [`create_backend`] 在运行时从已启用的后端中选择（先 sdl3，后 std 后端），
//! 因此应用代码从不提及任何后端类型。

#![no_std]
#![forbid(unsafe_code)]

extern crate alloc;

use alloc::boxed::Box;
use alloc::string::String;
use core::fmt;

pub use he3d_assets;
pub use he3d_core;
pub use he3d_math;
pub use he3d_physics;
pub use he3d_platform;
pub use he3d_renderer;

#[cfg(feature = "sdl3")]
pub use he3d_platform_sdl3;
#[cfg(feature = "std")]
pub use he3d_platform_std;

// Commonly used types, flattened for `use he3d::*;`.
// 常用类型扁平化，方便 `use he3d::*;`。
pub use he3d_assets::{decode_image, parse_obj};
pub use he3d_core::{
   Camera, ColorRgba8, Error, MAX_VERTEX_COUNT, Material, Mesh, MeshId, ObjectId, RenderObject,
   Rgb, Scene, Texture, TextureId, Transform,
};
pub use he3d_math::{EulerRot, Mat4, Quat, Vec2, Vec3, Vec4};
pub use he3d_physics::PhysicsWorld;
pub use he3d_platform::{
   BackendConfig, Event, KEY_ESCAPE, KeyCode, Platform, PlatformError, PlatformFactory,
};
pub use he3d_renderer::{AntiAliasing, Renderer, RendererConfig};

/// Error produced when no enabled backend could be constructed.
/// 没有启用的后端可被构造时产生的错误。
#[derive(Clone, Debug)]
pub struct BackendInitError {
   message: String,
}

impl BackendInitError {
   fn new(message: String) -> Self {
      Self { message }
   }
}

impl fmt::Display for BackendInitError {
   fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
      f.write_str(&self.message)
   }
}

/// Construct the platform backend selected by the enabled features.
/// 构造由已启用特性选中的平台后端。
///
/// `sdl3` is tried first, then the standard-library (stdout) backend, so a window
/// is preferred when available and the program still works headless.
/// 优先尝试 `sdl3`，再尝试标准库（stdout）后端：有窗口时用窗口，无头环境仍可用。
/// Returns [`BackendInitError`] when none of the enabled backends could be created
/// (or when no backend feature is enabled).
/// 当所有已启用的后端都无法创建（或未启用任何后端特性）时返回 [`BackendInitError`]。
///
/// # Errors
/// 未启用任何后端特性，或所有已启用后端都无法构造时返回 [`BackendInitError`]。
/// Returns [`BackendInitError`] when no backend feature is enabled or every
/// enabled backend fails to construct.
pub fn create_backend(config: BackendConfig<'_>) -> Result<Box<dyn Platform>, BackendInitError> {
   // `config` is unused in builds where no backend feature is enabled.
   // 在未启用任何后端特性的构建中，`config` 不会被使用。
   let _ = config;
   #[cfg(feature = "sdl3")]
   if let Ok(backend) = he3d_platform_sdl3::SdlBackend::create(config) {
      return Ok(Box::new(backend));
   }
   #[cfg(feature = "std")]
   if let Ok(backend) = he3d_platform_std::StdBackend::create(config) {
      return Ok(Box::new(backend));
   }
   Err(BackendInitError::new(
      "no backend available: enable the `std` or `sdl3` feature of `he3d`".into(),
   ))
}

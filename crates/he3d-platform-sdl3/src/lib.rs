//! HE3D SDL3 backend.
//! HE3D SDL3 后端。
//!
//! Window creation, event pumping, RGBA streaming-texture presentation, and frame pacing on top of SDL3.
//! 基于 SDL3 的窗口创建、事件泵送、RGBA 流式纹理呈现和帧节奏控制。
//!
//! Depending on this crate makes SDL3 a build requirement: `sdl3-sys` declares
//! `links = "SDL3"`, so cargo looks for the library even before anything runs.
//! 依赖本 crate 会让 SDL3 成为构建前提：`sdl3-sys` 声明了 `links = "SDL3"`，因此
//! 在运行任何东西之前，cargo 就要找到该库。

pub use he3d_assets;
pub use he3d_platform;
pub use he3d_renderer;

mod sdl;
pub use sdl::{SdlBackend, SdlBackendError};

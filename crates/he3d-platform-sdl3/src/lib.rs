//! HE3D SDL3 backend.
//! HE3D SDL3 后端。
//!
//! Window creation, event pumping, RGBA streaming-texture presentation, and frame pacing on top of SDL3.
//! 基于 SDL3 的窗口创建、事件泵送、RGBA 流式纹理呈现和帧节奏控制。

pub use he3d_assets;
pub use he3d_platform;
pub use he3d_renderer;

mod sdl;
pub use sdl::{BackendError, SdlBackend};

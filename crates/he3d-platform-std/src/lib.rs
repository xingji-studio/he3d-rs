//! HE3D std backend.
//! HE3D 标准库后端。
//!
//! The Rust standard-library backend: terminal framebuffer presentation, filesystem access, a monotonic clock, and the default integration-test host. Terminal ANSI output is one presentation capability, not the crate's identity.
//! Rust 标准库后端：终端帧缓冲呈现、文件系统访问、单调时钟，以及默认的集成测试宿主。终端 ANSI 输出只是其中一种呈现能力，并非本 crate 的全部身份。

pub use he3d_assets;
pub use he3d_platform;
pub use he3d_renderer;

#[cfg(unix)]
mod terminal;
#[cfg(unix)]
pub use terminal::{TerminalBackend, TerminalError};

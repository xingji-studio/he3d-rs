//! HE3D std backend.
//! HE3D 标准库后端。
//!
//! The Rust standard-library backend: presents the framebuffer to the standard
//! library's standard output (stdout), plus filesystem access, a monotonic clock,
//! and the default integration-test host. Standard-output ANSI output is one
//! presentation capability, not the crate's identity.
//! Rust 标准库后端：将帧缓冲呈现到标准库标准输出（stdout），并提供文件系统访问、
//! 单调时钟，以及默认的集成测试宿主。标准输出的 ANSI 呈现只是其中一种呈现能力，
//! 并非本 crate 的全部身份。

pub use he3d_assets;
pub use he3d_platform;
pub use he3d_renderer;

#[cfg(unix)]
mod stdout;
#[cfg(unix)]
pub use stdout::{StdBackend, StdBackendError};

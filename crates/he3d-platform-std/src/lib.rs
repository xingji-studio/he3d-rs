//! HE3D std backend.
//! HE3D 标准库后端。
//!
//! The Rust standard-library backend: presents the framebuffer to the standard
//! library's standard output (stdout) using ANSI 24-bit color, and reads keys
//! from standard input in raw mode (skipped when stdin is not a TTY).
//! Standard-output ANSI output is one presentation capability, not the crate's
//! identity.
//! Rust 标准库后端：使用 ANSI 24 位真彩色将帧缓冲呈现到标准库标准输出（stdout），
//! 并以原始模式从标准输入读取按键（标准输入不是 TTY 时跳过）。
//! 标准输出的 ANSI 呈现只是其中一种呈现能力，并非本 crate 的全部身份。
//!
//! Nothing in the engine chooses a backend for an application, and this crate
//! needs nothing from the system, so it is the one a headless build can always
//! use.
//! 引擎里没有任何东西替应用挑后端；本 crate 不需要系统中的任何东西，因此无头构建总能
//! 用它。

pub use he3d_assets;
pub use he3d_platform;
pub use he3d_renderer;

#[cfg(unix)]
mod stdout;
#[cfg(unix)]
pub use stdout::{StdBackend, StdBackendError};

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

//! HE3D platform.
//! HE3D 平台。
//!
//! Backend-neutral traits for input, presentation, and timing. Backends (`he3d-platform-std`, `he3d-platform-sdl3`) implement [`Platform`]; engines and examples only ever talk to this trait.
//! 与后端无关的输入、呈现和计时特质。后端（`he3d-platform-std`、`he3d-platform-sdl3`）实现 [`Platform`]；引擎和示例只与本特质打交道。

extern crate alloc;

use alloc::vec::Vec;
use core::fmt;

pub use he3d_core::ColorRgba8;

/// A normalized key code. ASCII keys are lowercased; special keys use small constants such as [`KEY_ESCAPE`].
/// 规范化后的键码。ASCII 键被转成小写；特殊键使用诸如 [`KEY_ESCAPE`] 的小常量。
pub type KeyCode = i32;

/// Escape key code (also produced for Ctrl+C).
/// 退出键码（Ctrl+C 也会产生）。
pub const KEY_ESCAPE: KeyCode = 27;

/// Input events delivered by [`Platform::poll_events`].
/// 由 [`Platform::poll_events`] 投递的输入事件。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
   /// A key was pressed.
   /// 某个键被按下。
   KeyDown(KeyCode),
   /// A key was released (may be synthesized by backends without release events, such as the standard-library (stdout) backend).
   /// 某个键被释放（可能由没有释放事件的后端合成，例如终端）。
   KeyUp(KeyCode),
}

/// Errors returned by presentation and backend operations.
/// 呈现和后端操作返回的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlatformError {
   /// The pixel buffer does not match the declared dimensions.
   /// 像素缓冲区与声明的尺寸不匹配。
   InvalidFramebuffer,
   /// The backend could not complete the operation.
   /// 后端无法完成该操作。
   BackendUnavailable,
}

impl fmt::Display for PlatformError {
   fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
      match self {
         PlatformError::InvalidFramebuffer => {
            f.write_str("framebuffer does not match declared dimensions")
         }
         PlatformError::BackendUnavailable => f.write_str("backend operation unavailable"),
      }
   }
}

/// Normalize an ASCII key to lowercase, matching the reference renderer.
/// 将 ASCII 键规范化为小写，与参考渲染器保持一致。
#[inline]
#[must_use]
pub fn normalize_key(key: KeyCode) -> KeyCode {
   if (i32::from(b'A')..=i32::from(b'Z')).contains(&key) {
      key + (i32::from(b'a') - i32::from(b'A'))
   } else {
      key
   }
}

/// Backend-neutral construction parameters for a platform backend.
/// 与后端无关的平台后端构造参数。
///
/// Every backend accepts the same shape; backends without a native title (such as the standard-library (stdout) backend) simply ignore [`BackendConfig::title`].
/// 每个后端都接受相同的结构；没有原生标题的后端（如终端）会直接忽略 [`BackendConfig::title`]。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BackendConfig<'a> {
   /// Window title; ignored by backends that have no title.
   /// 窗口标题；无标题的后端会忽略它。
   pub title: &'a str,
   /// Render width in pixels.
   /// 以像素计的渲染宽度。
   pub width: u32,
   /// Render height in pixels.
   /// 以像素计的渲染高度。
   pub height: u32,
}

/// A platform backend that can construct itself from a [`BackendConfig`].
/// 一个可以根据 [`BackendConfig`] 自行构造的平台后端。
///
/// This is the portable construction entry point: application code picks a concrete backend at compile time (via Cargo features) and calls [`PlatformFactory::create`], then drives it purely through [`Platform`].
/// 这是可移植的构造入口：应用程序代码在编译期（通过 Cargo feature）挑选具体后端并调用 [`PlatformFactory::create`]，之后完全通过 [`Platform`] 驱动它。
pub trait PlatformFactory: Platform {
   /// Backend-specific construction error (e.g. an SDL init failure).
   /// 后端特有的构造错误（例如 SDL 初始化失败）。
   type CreateError: core::fmt::Display;

   /// Construct the backend, or return a backend-specific error.
   /// 构造后端，或返回后端特有的错误。
   ///
   /// # Errors
   /// 返回后端特有的 [`PlatformFactory::CreateError`]，例如初始化失败或尺寸无效。
   /// Returns a backend-specific [`PlatformFactory::CreateError`], such as an
   /// initialization failure or an invalid size.
   fn create(config: BackendConfig<'_>) -> Result<Self, Self::CreateError>
   where
      Self: Sized;
}

/// Backend-neutral platform services.
/// 与后端无关的平台服务。
pub trait Platform {
   /// Present an RGBA framebuffer (top-to-bottom, left-to-right).
   /// 呈现一个 RGBA 帧缓冲（自上而下、从左到右）。
   ///
   /// # Errors
   /// 像素缓冲长度与声明尺寸不符时返回 [`PlatformError::InvalidFramebuffer`]；
   /// 后端无法完成呈现时返回 [`PlatformError::BackendUnavailable`]。
   /// Returns [`PlatformError::InvalidFramebuffer`] when the pixel buffer length
   /// does not match the declared dimensions, or [`PlatformError::BackendUnavailable`]
   /// when the backend cannot complete the presentation.
   fn present(
      &mut self,
      width: u32,
      height: u32,
      pixels: &[ColorRgba8],
   ) -> Result<(), PlatformError>;

   /// Poll pending events without blocking.
   /// 非阻塞地轮询待处理事件。
   fn poll_events(&mut self) -> Vec<Event>;

   /// Whether the presentation surface wants to close.
   /// 呈现表面是否希望关闭。
   fn should_close(&self) -> bool;

   /// Request that the surface close.
   /// 请求关闭该表面。
   fn request_close(&mut self);

   /// Monotonic time in seconds.
   /// 以秒计的单调时钟时间。
   fn time_seconds(&self) -> f64;

   /// Sleep for the given number of milliseconds.
   /// 休眠给定的毫秒数。
   fn sleep_ms(&mut self, milliseconds: u64);

   /// Update the surface title. Backends without a title (e.g. the standard-library stdout backend) use the default no-op implementation.
   /// 更新表面标题。没有标题的后端（如终端）使用默认的空实现。
   fn set_title(&mut self, _title: &str) {}
}

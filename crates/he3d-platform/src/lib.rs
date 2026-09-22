#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

//! HE3D platform.
//! HE3D 平台。
//!
//! The [`Platform`] trait and the types that go with it: input events, presentation
//! parameters, and the errors a backend may return. A backend is a type that
//! implements [`Platform`] — `SdlBackend`, `StdBackend`, or one written elsewhere —
//! and the binary chooses which one to construct. This crate states the contract;
//! it holds no registry and no constructor of its own.
//! [`Platform`] 特质以及随之而来的类型：输入事件、呈现参数，以及后端可能返回的错误。
//! 后端就是实现了 [`Platform`] 的类型——`SdlBackend`、`StdBackend`，或别处写的实现——
//! 由二进制决定构造哪一个。本 crate 只声明契约：它不含登记表，也没有自己的构造函数。

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
#[non_exhaustive]
pub enum Event {
   /// A key was pressed.
   /// 某个键被按下。
   KeyDown(KeyCode),
   /// A key was released (may be synthesized by backends without release events, such as the standard-library (stdout) backend).
   /// 某个键被释放（可能由没有释放事件的后端合成，例如标准输出 stdout 后端）。
   KeyUp(KeyCode),
}

/// Errors returned by presentation and backend operations.
/// 呈现和后端操作返回的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
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

/// Normalize an ASCII key to lowercase, so key codes compare case-insensitively.
/// 将 ASCII 键规范化为小写，使键码比较不区分大小写。
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
/// 每个后端都接受相同的结构；没有原生标题的后端（如标准输出 stdout 后端）会直接忽略 [`BackendConfig::title`]。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
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

impl<'a> BackendConfig<'a> {
   /// Construct a backend configuration.
   /// 构造一个后端配置。
   #[must_use]
   pub const fn new(title: &'a str, width: u32, height: u32) -> Self {
      Self {
         title,
         width,
         height,
      }
   }
}

/// Backend-neutral platform services.
/// 与后端无关的平台服务。
///
/// The trait is deliberately object-safe and carries no thread bound: `SdlBackend`
/// wraps a thread-affine SDL renderer, so a `Box<dyn Platform>` would not be `Send`
/// either. An application that commits to one backend can hold the concrete type
/// and skip the allocation; one that wants to switch at runtime can box it.
/// 本特质刻意保持 object-safe，且不携带线程约束：`SdlBackend` 包装了线程亲和的 SDL
/// 渲染器，因此 `Box<dyn Platform>` 同样不是 `Send`。只用一个后端的应用可以直接持有具体
/// 类型，省掉这次分配；想在运行期切换的应用可以自己装箱。
///
/// # Examples
/// 示例：实现一个最小 mock 后端。
/// ```
/// use he3d_platform::{ColorRgba8, Event, Platform, PlatformError};
///
/// struct Mock;
///
/// impl Platform for Mock {
///    fn present(
///       &mut self,
///       _width: u32,
///       _height: u32,
///       _pixels: &[ColorRgba8],
///    ) -> Result<(), PlatformError> {
///       Ok(())
///    }
///    fn poll_events(&mut self) -> std::vec::Vec<Event> {
///       std::vec::Vec::new()
///    }
///    fn should_close(&self) -> bool { false }
///    fn request_close(&mut self) {}
///    fn time_seconds(&self) -> f64 { 0.0 }
///    fn sleep_ms(&mut self, _milliseconds: u64) {}
/// }
/// ```
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
   /// 更新表面标题。没有标题的后端（如标准输出 stdout 后端）使用默认的空实现。
   fn set_title(&mut self, _title: &str) {}
}

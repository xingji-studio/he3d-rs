use std::time::{Duration, Instant};

use sdl3::event::Event as SdlEvent;
use sdl3::pixels::PixelFormat;
use sdl3::render::{Texture, WindowCanvas};
use sdl3::{EventPump, Sdl, VideoSubsystem};

use he3d_platform::{
   BackendConfig, ColorRgba8, Event, KEY_ESCAPE, Platform, PlatformError, PlatformFactory,
   normalize_key,
};

/// Errors from SDL3 backend construction.
/// SDL3 后端构造错误。
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum BackendError {
   /// SDL initialization failed.
   /// SDL 初始化失败。
   Init(String),
   /// An SDL subsystem failed to start.
   /// SDL 子系统启动失败。
   Sdl(String),
   /// Window creation failed.
   /// 窗口创建失败。
   Window(String),
   /// Texture creation failed.
   /// 纹理创建失败。
   Texture(String),
}

impl core::fmt::Display for BackendError {
   fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
      match self {
         BackendError::Init(message) => write!(f, "SDL init failed: {message}"),
         BackendError::Sdl(message) => write!(f, "SDL subsystem failed: {message}"),
         BackendError::Window(message) => write!(f, "window creation failed: {message}"),
         BackendError::Texture(message) => write!(f, "texture creation failed: {message}"),
      }
   }
}

impl std::error::Error for BackendError {}

/// The SDL3 window backend.
/// SDL3 窗口后端。
///
/// Owns one window and its software canvas.
/// 拥有一个窗口及其软件画布。
/// Presents an RGBA framebuffer through a streaming texture that is recreated when the framebuffer resolution changes (the old texture is released first).
/// 通过流式纹理呈现 RGBA 帧缓冲，当帧缓冲分辨率变化时会重建纹理（先释放旧纹理）。
/// Single-window by design: the `Sdl` context is owned by this struct and only quits when the struct is dropped, so destroying the window never quits a shared SDL session behind other windows.
/// 设计上为单窗口：`Sdl` 上下文由本结构体持有，仅在该结构体被释放时退出，因此销毁窗口绝不会退出其他窗口背后的共享 SDL 会话。
pub struct SdlBackend {
   canvas: WindowCanvas,
   event_pump: EventPump,
   texture: Option<Texture>,
   texture_size: (u32, u32),
   close_requested: bool,
   start: Instant,
   // Field order matters for drop order: `_sdl` drops last so SDL_Quit happens only after the window, canvas, and event pump are destroyed.
   // 字段顺序影响释放顺序：`_sdl` 最后释放，因此 SDL_Quit 只在窗口、画布和事件泵被销毁之后发生。
   _video: VideoSubsystem,
   _sdl: Sdl,
}

impl core::fmt::Debug for SdlBackend {
   fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
      // SDL handle types are not `Debug`; print the tracking state only.
      // SDL 句柄类型未实现 `Debug`；只打印跟踪状态。
      f.debug_struct("SdlBackend")
         .field("texture_size", &self.texture_size)
         .field("close_requested", &self.close_requested)
         .finish_non_exhaustive()
   }
}

impl PlatformFactory for SdlBackend {
   type CreateError = BackendError;

   /// Create a windowed backend at the given logical size.
   /// 在给定的逻辑尺寸下创建窗口后端。
   fn create(config: BackendConfig<'_>) -> Result<Self, BackendError> {
      let sdl = sdl3::init().map_err(|e| BackendError::Init(e.to_string()))?;
      let video = sdl.video().map_err(|e| BackendError::Sdl(e.to_string()))?;
      let window = video
         .window(config.title, config.width, config.height)
         .position_centered()
         .build()
         .map_err(|e| BackendError::Window(e.to_string()))?;
      let canvas = window.into_canvas();
      let event_pump = sdl
         .event_pump()
         .map_err(|e| BackendError::Sdl(e.to_string()))?;
      Ok(Self {
         canvas,
         event_pump,
         texture: None,
         texture_size: (0, 0),
         close_requested: false,
         start: Instant::now(),
         _video: video,
         _sdl: sdl,
      })
   }
}

impl SdlBackend {
   fn ensure_texture(&mut self, width: u32, height: u32) -> Result<(), BackendError> {
      if self.texture.is_some() && self.texture_size == (width, height) {
         return Ok(());
      }
      // Dropping the old texture releases it before creating the new one.
      // 丢弃旧纹理会在创建新纹理之前将其释放。
      self.texture = None;
      let creator = self.canvas.texture_creator();
      let texture = creator
         .create_texture_streaming(PixelFormat::RGBA32, width, height)
         .map_err(|e| BackendError::Texture(e.to_string()))?;
      self.texture = Some(texture);
      self.texture_size = (width, height);
      Ok(())
   }
}

/// `ColorRgba8` is `#[repr(C)]` with four `u8` fields, so its slice is layout-compatible with `[u8; 4]` and safe to view as bytes. This assertion keeps the `present` reinterprecast honest if the struct layout ever changes.
/// `ColorRgba8` 是 `#[repr(C)]`，包含四个 `u8` 字段，因此其切片与 `[u8; 4]` 布局兼容，可以安全地按字节查看。此断言确保若该结构体布局将来改变，`present` 的内存重解释不会静默失真。
const _: () = assert!(core::mem::size_of::<ColorRgba8>() == 4);

impl Platform for SdlBackend {
   fn present(
      &mut self,
      width: u32,
      height: u32,
      pixels: &[ColorRgba8],
   ) -> Result<(), PlatformError> {
      if width == 0 || height == 0 {
         return Err(PlatformError::InvalidFramebuffer);
      }
      let expected = (width as usize)
         .checked_mul(height as usize)
         .ok_or(PlatformError::InvalidFramebuffer)?;
      if pixels.len() != expected {
         return Err(PlatformError::InvalidFramebuffer);
      }

      self
         .ensure_texture(width, height)
         .map_err(|_| PlatformError::BackendUnavailable)?;

      // SAFETY: `ColorRgba8` is `#[repr(C)]` with four `u8` fields, so a
      // `&[ColorRgba8]` is layout-compatible with `&[u8; 4]`; the `size_of == 4`
      // assertion above guards this, and the length stays in-bounds because
      // `pixels.len() * 4` mirrors the exact byte count of the slice.
      // 安全性：`ColorRgba8` 是 `#[repr(C)]` 且含四个 `u8` 字段，因此 `&[ColorRgba8]`
      // 与 `&[u8; 4]` 布局兼容；上面的 `size_of == 4` 断言为此兜底，且 `pixels.len() * 4`
      // 与该切片的实际字节数严格一致，不会越界。
      let bytes =
         unsafe { core::slice::from_raw_parts(pixels.as_ptr().cast::<u8>(), pixels.len() * 4) };

      let Self {
         canvas, texture, ..
      } = self;
      let texture = texture.as_mut().ok_or(PlatformError::BackendUnavailable)?;
      texture
         .update(None, bytes, width as usize * 4)
         .map_err(|_| PlatformError::BackendUnavailable)?;
      canvas.clear();
      canvas
         .copy(texture, None, None)
         .map_err(|_| PlatformError::BackendUnavailable)?;
      canvas.present();
      Ok(())
   }

   // SDL keycodes are enum-backed `u32` values that always fit in the `i32`
   // `KeyCode` space (the SDL extended-key mask lives below `i32::MAX`), so the
   // wrapping cast cannot occur in practice.
   // SDL 键码是由枚举承载的 `u32` 值，且始终能容纳在 `i32` 的 `KeyCode` 空间中
   // （SDL 扩展键掩码低于 `i32::MAX`），因此实际不会发生回绕。
   #[expect(clippy::cast_possible_wrap)]
   fn poll_events(&mut self) -> Vec<Event> {
      let mut events = Vec::new();
      for event in self.event_pump.poll_iter() {
         match event {
            SdlEvent::Quit { .. } => self.close_requested = true,
            SdlEvent::KeyDown {
               keycode: Some(keycode),
               ..
            } => {
               let code = normalize_key(keycode as u32 as i32);
               if code == KEY_ESCAPE {
                  self.close_requested = true;
               }
               events.push(Event::KeyDown(code));
            }
            SdlEvent::KeyUp {
               keycode: Some(keycode),
               ..
            } => {
               let code = normalize_key(keycode as u32 as i32);
               events.push(Event::KeyUp(code));
            }
            _ => {}
         }
      }
      events
   }

   fn should_close(&self) -> bool {
      self.close_requested
   }

   fn request_close(&mut self) {
      self.close_requested = true;
   }

   fn time_seconds(&self) -> f64 {
      self.start.elapsed().as_secs_f64()
   }

   fn sleep_ms(&mut self, milliseconds: u64) {
      std::thread::sleep(Duration::from_millis(milliseconds));
   }

   fn set_title(&mut self, title: &str) {
      let _ = self.canvas.window_mut().set_title(title);
   }
}

/// Anti-aliasing mode. Only `None` is implemented today; other modes are added in later stages and are rejected by [`crate::Renderer::new`].
/// 抗锯齿模式。目前只实现了 `None`；其余模式将在后续阶段加入，且会被 [`crate::Renderer::new`] 拒绝。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AntiAliasing {
   #[default]
   None,
}

/// Construction parameters for the software renderer.
/// 软件渲染器的构造参数。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RendererConfig {
   /// Framebuffer width in pixels.
   /// 帧缓冲宽度（像素）。
   pub width: u32,
   /// Framebuffer height in pixels.
   /// 帧缓冲高度（像素）。
   pub height: u32,
   /// Requested anti-aliasing mode.
   /// 请求的抗锯齿模式。
   pub anti_aliasing: AntiAliasing,
}

impl RendererConfig {
   /// A full-resolution config with no anti-aliasing.
   /// 无抗锯齿的全分辨率配置。
   pub const fn new(width: u32, height: u32) -> Self {
      Self {
         width,
         height,
         anti_aliasing: AntiAliasing::None,
      }
   }
}

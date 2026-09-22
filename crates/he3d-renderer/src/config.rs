/// Maximum supersampling scale.
///
/// SSAA multiplies the framebuffer by `scale²`, so this is what keeps a request
/// from asking for sixteen times the pixels of an already capped output.
/// 超采样最大倍率。SSAA 把帧缓冲放大 `scale²` 倍，因此这个上限防止请求在已经受限的输出
/// 之上再索取十六倍的像素。
pub const MAX_SSAA_SCALE: u32 = 4;

/// Anti-aliasing mode applied at the end of each frame.
/// 每帧结束时应用的抗锯齿模式。
///
/// Exactly one mode is selected at construction time; modes are mutually exclusive,
/// mirroring the single-selection nature of a rasterizer configuration.
/// 构造时恰好选择一种模式；各模式互斥，反映光栅化配置的单选性质。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum AntiAliasing {
   /// No anti-aliasing: render at full output resolution.
   /// 无抗锯齿：按输出分辨率直接渲染。
   #[default]
   None,
   /// 4× multi-sample anti-aliasing: four samples per pixel are rasterized,
   /// depth-tested, and averaged when the frame is presented.
   /// 4× 多重采样抗锯齿：每像素光栅化 4 个样本，逐样本深度测试，呈现时平均。
   Msaa,
   /// Luma-based edge anti-aliasing applied as a post-processing pass.
   /// 基于亮度的边缘抗锯齿，作为后处理阶段应用。
   Fxaa,
   /// Temporal anti-aliasing: Halton-jittered projection blended against a
   /// clamped history buffer from the previous frame.
   /// 时间抗锯齿：Halton 抖动投影，并与上一帧的钳制历史缓冲混合。
   Taa,
   /// Supersampling: render internally at `scale`× the output resolution, then
   /// box-downsample when presenting. `scale` is clamped to `1..=MAX_SSAA_SCALE`.
   /// 超采样：内部按输出分辨率的 `scale` 倍渲染，呈现时盒式降采样。
   /// `scale` 被钳制在 `1..=MAX_SSAA_SCALE`。
   Ssaa(u32),
}

/// Construction parameters for the software renderer.
/// 软件渲染器的构造参数。
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
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
   #[must_use]
   pub const fn new(width: u32, height: u32) -> Self {
      Self {
         width,
         height,
         anti_aliasing: AntiAliasing::None,
      }
   }
}

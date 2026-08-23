use alloc::vec::Vec;
use he3d_math::scalar::floor;

use crate::{ColorRgba8, Error, Rgb};

/// Maximum width or height of a texture, matching the reference renderer.
/// 纹理的最大宽度或高度，与参考渲染器一致。
pub const MAX_TEXTURE_DIMENSION: u32 = 8192;

/// A 2D texture owning an RGBA8 pixel buffer, with nearest-neighbor wrap sampling.
/// 一个拥有 RGBA8 像素缓冲的 2D 纹理，支持最近邻环绕采样。
#[derive(Clone, Debug, PartialEq)]
pub struct Texture {
   width: u32,
   height: u32,
   pixels: Vec<ColorRgba8>,
}

impl Texture {
   /// Construct a texture, validating dimensions and buffer length.
   /// 构造一个纹理，校验尺寸和缓冲长度。
   ///
   /// # Errors
   /// 尺寸为 0 或超过 `MAX_TEXTURE_DIMENSION`、像素数量与尺寸不符，或尺寸乘积溢出时返回相应错误。
   /// Returns an error when a dimension is zero or exceeds `MAX_TEXTURE_DIMENSION`,
   /// when the pixel count does not match the dimensions, or when the product overflows.
   pub fn new(width: u32, height: u32, pixels: Vec<ColorRgba8>) -> Result<Self, Error> {
      if width == 0
         || height == 0
         || width > MAX_TEXTURE_DIMENSION
         || height > MAX_TEXTURE_DIMENSION
      {
         return Err(Error::InvalidSize);
      }
      let expected = (width as usize)
         .checked_mul(height as usize)
         .ok_or(Error::Overflow)?;
      if pixels.len() != expected {
         return Err(Error::BufferMismatch);
      }
      Ok(Self {
         width,
         height,
         pixels,
      })
   }

   /// Construct a texture from a borrowed pixel slice.
   /// 从借用的像素切片构造一个纹理。
   ///
   /// # Errors
   /// 与 [`Texture::new`] 相同。
   /// Same errors as [`Texture::new`].
   pub fn from_pixels(width: u32, height: u32, pixels: &[ColorRgba8]) -> Result<Self, Error> {
      Self::new(width, height, pixels.to_vec())
   }

   /// Build a 1×1 solid-color texture: any UV samples the same color.
   /// 构建一个 1×1 纯色纹理：任何 UV 都采样到相同颜色。
   #[must_use]
   pub fn solid(color: ColorRgba8) -> Self {
      Self {
         width: 1,
         height: 1,
         pixels: alloc::vec![color],
      }
   }

   /// Texture width in pixels.
   /// 纹理宽度（像素）。
   #[must_use]
   pub fn width(&self) -> u32 {
      self.width
   }

   /// Texture height in pixels.
   /// 纹理高度（像素）。
   #[must_use]
   pub fn height(&self) -> u32 {
      self.height
   }

   /// The underlying pixel buffer, row-major.
   /// 底层像素缓冲，按行优先排列。
   #[must_use]
   pub fn pixels(&self) -> &[ColorRgba8] {
      &self.pixels
   }

   /// Sample a texel with `[0, 1)`-wrapped nearest-neighbor addressing.
   /// 使用 `[0, 1)` 环绕的最近邻寻址采样一个纹素。
   ///
   /// Out-of-range or non-finite coordinates are wrapped/clamped and never index out of bounds.
   /// 越界或非有限坐标会被环绕/截断，绝不会产生越界索引。
   #[must_use]
   pub fn sample(&self, u: f32, v: f32) -> ColorRgba8 {
      let x = sample_axis(u, self.width);
      let y = sample_axis(v, self.height);
      self.pixels[y * self.width as usize + x]
   }

   /// Sample and convert to linear light.
   /// 采样并转换为线性光。
   #[must_use]
   pub fn sample_linear(&self, u: f32, v: f32) -> Rgb {
      self.sample(u, v).to_linear()
   }
}

/// Map a wrapped `[0, 1)` coordinate to a texel index in `[0, size)`.
/// 将环绕后的 `[0, 1)` 坐标映射到 `[0, size)` 范围内的纹素索引。
//
// `size` is bounded by MAX_TEXTURE_DIMENSION and `wrapped` is in [0, 1), so the
// float→i64→usize path below cannot overflow or meaningfully lose precision.
// `size` 受 MAX_TEXTURE_DIMENSION 约束且 `wrapped` 位于 [0, 1) 内，因此下面的
// float→i64→usize 路径不会溢出或产生有意义的精度损失。
#[expect(
   clippy::cast_possible_truncation,
   clippy::cast_precision_loss,
   clippy::cast_sign_loss
)]
fn sample_axis(coord: f32, size: u32) -> usize {
   let wrapped = wrap_uv(coord);
   let scaled = wrapped * size as f32;
   let index = scaled as i64;
   index.clamp(0, i64::from(size) - 1) as usize
}

/// Return the fractional part of `x` in `[0, 1)`, matching the reference renderer's `fracf` wrap semantics.
/// 返回 `x` 在 `[0, 1)` 范围内的小数部分，与参考渲染器的 `fracf` 环绕语义一致。
fn wrap_uv(x: f32) -> f32 {
   let frac = x - floor(x);
   if frac < 0.0 { frac + 1.0 } else { frac }
}

#[cfg(test)]
mod tests {
   use alloc::vec;

   use super::*;

   fn checker() -> Texture {
      // 2x2 texture: RGBA of known distinct colors.
      // 2x2 纹理：已知不同颜色的 RGBA。
      let pixels = vec![
         ColorRgba8::from_rgb(255, 0, 0),
         ColorRgba8::from_rgb(0, 255, 0),
         ColorRgba8::from_rgb(0, 0, 255),
         ColorRgba8::from_rgb(255, 255, 255),
      ];
      Texture::new(2, 2, pixels).unwrap()
   }

   #[test]
   fn samples_corners() {
      let tex = checker();
      assert_eq!(tex.sample(0.0, 0.0), ColorRgba8::from_rgb(255, 0, 0));
      assert_eq!(tex.sample(0.99, 0.99), ColorRgba8::from_rgb(255, 255, 255));
   }

   #[test]
   fn wraps_out_of_range_coordinates() {
      let tex = checker();
      // u=1.5 wraps to 0.5 -> x=1.
      // u=1.5 环绕为 0.5 -> x=1。
      assert_eq!(tex.sample(1.5, 0.0), ColorRgba8::from_rgb(0, 255, 0));
      // Negative v wraps to the positive fractional part.
      // 负的 v 环绕到正的分数部分。
      assert_eq!(tex.sample(0.0, -0.5), tex.sample(0.0, 0.5));
   }

   #[test]
   fn rejects_zero_dimension() {
      assert_eq!(Texture::new(0, 2, Vec::new()), Err(Error::InvalidSize));
   }

   #[test]
   fn rejects_oversized_dimension() {
      assert_eq!(
         Texture::new(MAX_TEXTURE_DIMENSION + 1, 2, Vec::new()),
         Err(Error::InvalidSize)
      );
   }

   #[test]
   fn rejects_buffer_mismatch() {
      let pixels = vec![ColorRgba8::BLACK; 3];
      assert_eq!(Texture::new(2, 2, pixels), Err(Error::BufferMismatch));
   }

   #[test]
   fn solid_samples_constant_color() {
      let tex = Texture::solid(ColorRgba8::from_rgb(10, 20, 30));
      assert_eq!(tex.width(), 1);
      assert_eq!(tex.height(), 1);
      assert_eq!(tex.sample(0.0, 0.0), ColorRgba8::from_rgb(10, 20, 30));
      assert_eq!(tex.sample(0.5, 0.5), ColorRgba8::from_rgb(10, 20, 30));
      assert_eq!(tex.sample(1.5, -0.5), ColorRgba8::from_rgb(10, 20, 30));
   }
}

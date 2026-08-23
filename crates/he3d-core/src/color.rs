use core::ops::{Add, Mul, Sub};

/// A packed 8-bit RGBA color, laid out in RGBA byte order.
/// 一个打包的 8 位 RGBA 颜色，按 RGBA 字节顺序排列。
///
/// This is the storage format for textures and framebuffers.
/// 这是纹理和帧缓冲的存储格式。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Hash)]
#[repr(C)]
pub struct ColorRgba8 {
   pub r: u8,
   pub g: u8,
   pub b: u8,
   pub a: u8,
}

// Compile-time layout guarantee: `ColorRgba8` must stay a plain 4-byte RGBA tuple.
// 编译期布局保证：`ColorRgba8` 必须保持为一个普通的 4 字节 RGBA 元组。
// Backends reinterpret its slice as `&[u8]` for framebuffer upload, so adding fields or padding here would silently corrupt memory.
// 后端会将其切片重新解释为 `&[u8]` 以用于帧缓冲上传，因此在此添加字段或填充会静默地破坏内存。
const _: () = assert!(core::mem::size_of::<ColorRgba8>() == 4);

impl ColorRgba8 {
   pub const BLACK: Self = Self::from_rgba(0, 0, 0, 255);
   pub const WHITE: Self = Self::from_rgba(255, 255, 255, 255);
   pub const TRANSPARENT: Self = Self::from_rgba(0, 0, 0, 0);
   /// The conventional "missing texture" color.
   /// 约定的“缺失纹理”颜色。
   pub const MAGENTA: Self = Self::from_rgb(255, 0, 255);

   #[inline]
   pub const fn from_rgba(r: u8, g: u8, b: u8, a: u8) -> Self {
      Self { r, g, b, a }
   }

   #[inline]
   pub const fn from_rgb(r: u8, g: u8, b: u8) -> Self {
      Self { r, g, b, a: 255 }
   }

   /// Convert to linear-light components in `[0, 1]`.
   /// 转换为 `[0, 1]` 范围内的线性光分量。
   #[inline]
   pub fn to_linear(self) -> Rgb {
      Rgb {
         r: self.r as f32 / 255.0,
         g: self.g as f32 / 255.0,
         b: self.b as f32 / 255.0,
      }
   }
}

/// A linear-light RGB color with `f32` components.
/// 一个使用 `f32` 分量的线性光 RGB 颜色。
///
/// Components are conventionally in `[0, 1]` but are not clamped by default; lighting math may produce values outside the range, and conversion to [`ColorRgba8`] clamps them.
/// 分量通常位于 `[0, 1]` 内，但默认不会截断；光照运算可能产生超出该范围的值，转换为 [`ColorRgba8`] 时会进行截断。
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Rgb {
   pub r: f32,
   pub g: f32,
   pub b: f32,
}

impl Rgb {
   pub const BLACK: Self = Self::new(0.0, 0.0, 0.0);
   pub const WHITE: Self = Self::new(1.0, 1.0, 1.0);

   #[inline]
   pub const fn new(r: f32, g: f32, b: f32) -> Self {
      Self { r, g, b }
   }

   /// Quantize to 8-bit RGBA, truncating toward zero.
   /// 量化到 8 位 RGBA，向零截断。
   ///
   /// The `as u8` cast saturates: values below 0 clamp to 0, values above 1 clamp to 255, and NaN clamps to 0.
   /// `as u8` 转换是饱和的：小于 0 的值截断为 0，大于 1 的值截断为 255，NaN 截断为 0。
   /// This matches the reference renderer's `(uint8_t)(color * 255.0f)` behavior.
   /// 这与参考渲染器的 `(uint8_t)(color * 255.0f)` 行为一致。
   #[inline]
   pub fn to_rgba8(self) -> ColorRgba8 {
      ColorRgba8 {
         r: (self.r * 255.0) as u8,
         g: (self.g * 255.0) as u8,
         b: (self.b * 255.0) as u8,
         a: 255,
      }
   }
}

impl Add for Rgb {
   type Output = Self;
   #[inline]
   fn add(self, rhs: Self) -> Self {
      Self::new(self.r + rhs.r, self.g + rhs.g, self.b + rhs.b)
   }
}

impl Sub for Rgb {
   type Output = Self;
   #[inline]
   fn sub(self, rhs: Self) -> Self {
      Self::new(self.r - rhs.r, self.g - rhs.g, self.b - rhs.b)
   }
}

impl Mul<f32> for Rgb {
   type Output = Self;
   #[inline]
   fn mul(self, rhs: f32) -> Self {
      Self::new(self.r * rhs, self.g * rhs, self.b * rhs)
   }
}

/// Component-wise multiplication.
/// 逐分量相乘。
impl Mul for Rgb {
   type Output = Self;
   #[inline]
   fn mul(self, rhs: Self) -> Self {
      Self::new(self.r * rhs.r, self.g * rhs.g, self.b * rhs.b)
   }
}

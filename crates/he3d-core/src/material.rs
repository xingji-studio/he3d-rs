use crate::Rgb;

/// Per-object shading parameters.
/// 每个对象的着色参数。
///
/// First version only carries a tint; texture tint and additional surface parameters are added without breaking the constructor shape.
/// 首个版本只携带一个色调；纹理色调和额外的表面参数会在不破坏构造函数形态的前提下添加。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Material {
   /// Multiplicative tint applied to the sampled texture or solid color.
   /// 应用于采样纹理或纯色的倍增色调。
   pub tint: Rgb,
}

impl Material {
   pub const fn new(tint: Rgb) -> Self {
      Self { tint }
   }
}

impl Default for Material {
   fn default() -> Self {
      Self { tint: Rgb::WHITE }
   }
}

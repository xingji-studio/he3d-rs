use he3d_math::Vec3;

use crate::Rgb;

/// A directional light: one direction plus color and strength.
/// 方向光：一个光照方向加上颜色与强度。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct DirectionalLight {
   /// Direction from a surface toward the light; not a position. The shading term
   /// is `normal · direction`, so the renderer normalizes this and ignores its
   /// length.
   /// 从表面指向光源的方向，不是位置。着色项为 `normal · direction`，因此渲染器会将其
   /// 归一化，长度被忽略。
   pub direction: Vec3,
   /// Light color and strength.
   /// 光的颜色和强度。
   pub color: Rgb,
   /// Base brightness added to every lit surface.
   /// 添加到每个被照亮表面的基础亮度。
   pub ambient: f32,
}

impl Default for DirectionalLight {
   fn default() -> Self {
      Self {
         direction: Vec3::new(0.0, -1.0, 1.0),
         color: Rgb::WHITE,
         ambient: 0.15,
      }
   }
}

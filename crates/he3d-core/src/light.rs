use he3d_math::Vec3;

use crate::Rgb;

/// A directional light: a parallel light ray direction plus color/strength.
/// 方向光：一条平行光线方向加上颜色/强度。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct DirectionalLight {
   /// Ray direction (not a position); will be normalized by the renderer.
   /// 光线方向（不是位置）；渲染器会将其归一化。
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

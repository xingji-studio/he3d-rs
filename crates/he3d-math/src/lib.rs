#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

//! HE3D math layer.
//! HE3D 数学层。
//!
//! The single place in the workspace that touches [`glam`]. Every other crate depends on `he3d-math` instead of `glam` directly, so replacing the underlying linear-algebra library only requires editing this crate.
//! workspace 中唯一直接接触 [`glam`] 的地方。其他所有 crate 都依赖 `he3d-math` 而非直接依赖 `glam`，因此替换底层线性代数库只需改动本 crate。
//!
//! Project-wide coordinate conventions:
//! 全局坐标约定：
//!
//! * right-handed coordinate system,
//! * 右手坐标系，
//! * +Z is the forward axis (`axis::FORWARD`),
//! * +Z 为前向轴（`axis::FORWARD`），
//! * field-of-view angles are in degrees (convert via [`conv`]),
//! * 视场角以度为单位（通过 [`conv`] 转换），
//! * inverse depth buffer (`1/z`, larger means closer).
//! * 逆深度缓冲（`1/z`，值越大越近）。

pub use glam;
pub use glam::{EulerRot, Mat4, Quat, Vec2, Vec3, Vec4};

/// Semantic aliases so call sites read as domain types rather than raw `glam` names. These are the portable surface of this crate.
/// 语义别名，使调用处读起来像领域类型而非原始的 `glam` 名称。这些别名是本 crate 的可移植表面。
pub type Vector2 = Vec2;
/// Alias of [`Vector2`] with a domain-friendly name.
/// [`Vector2`] 的领域友好别名。
pub type Vector3 = Vec3;
/// Alias of [`Vec3`] with a domain-friendly name.
/// [`Vec3`] 的领域友好别名。
pub type Vector4 = Vec4;
/// Alias of [`Vec4`] with a domain-friendly name.
/// [`Vec4`] 的领域友好别名。
pub type Matrix4 = Mat4;
/// Alias of [`Mat4`] with a domain-friendly name.
/// [`Mat4`] 的领域友好别名。
pub type Quaternion = Quat;

/// Angle-unit conversion helpers.
/// 角度单位转换辅助工具。
pub mod conv {
   use core::f32::consts::PI;

   /// Multiply degrees by this to get radians.
   /// 乘以它可将度转换为弧度。
   pub const DEG_TO_RAD: f32 = PI / 180.0;
   /// Multiply radians by this to get degrees.
   /// 乘以它可将弧度转换为度。
   pub const RAD_TO_DEG: f32 = 180.0 / PI;

   /// Convert an angle in degrees to radians.
   /// 将角度从度转换为弧度。
   #[inline]
   #[must_use]
   pub fn deg_to_rad(degrees: f32) -> f32 {
      degrees * DEG_TO_RAD
   }

   /// Convert an angle in radians to degrees.
   /// 将角度从弧度转换为度。
   #[inline]
   #[must_use]
   pub fn rad_to_deg(radians: f32) -> f32 {
      radians * RAD_TO_DEG
   }
}

/// Scalar transcendental functions for `f32`, provided by `libm` so that `no_std` targets have `sin`/`cos`/`floor`/etc. without the host `std`.
/// `f32` 的标量超越函数，由 `libm` 提供，使 `no_std` 目标无需宿主 `std` 即可使用 `sin`/`cos`/`floor` 等函数。
pub mod scalar {
   pub use libm::{cosf as cos, floorf as floor, sinf as sin, sqrtf as sqrt, tanf as tan};
}

/// Basis vectors encoding the project's handedness and forward axis.
/// 编码项目坐标系手性与前向轴的基向量。
pub mod axis {
   use crate::Vec3;

   /// The forward axis (+Z).
   /// 前向轴（+Z）。
   pub const FORWARD: Vec3 = Vec3::Z;
   /// The up axis (+Y).
   /// 上方向轴（+Y）。
   pub const UP: Vec3 = Vec3::Y;
   /// The right axis (+X).
   /// 右方向轴（+X）。
   pub const RIGHT: Vec3 = Vec3::X;
}

#[cfg(test)]
mod tests {
   use super::conv;
   use crate::glam;

   #[test]
   fn identity_quat_is_noop() {
      let v = glam::Vec3::X;
      assert_eq!(glam::Quat::IDENTITY * v, v);
   }

   #[test]
   fn degree_radian_roundtrip() {
      let rad = conv::deg_to_rad(90.0);
      assert!((rad - core::f32::consts::FRAC_PI_2).abs() < 1e-6);
      assert!((conv::rad_to_deg(rad) - 90.0).abs() < 1e-5);
   }

   #[test]
   fn forward_is_positive_z() {
      assert_eq!(crate::axis::FORWARD, glam::Vec3::Z);
   }
}

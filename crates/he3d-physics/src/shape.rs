use core::f32::consts::PI;

use he3d_math::{Quat, Vec3};

use crate::aabb::Aabb;

/// How a Body's mass is distributed: inverse mass and inverse diagonal inertia.
/// Body 的质量分布：逆质量与逆惯量对角阵。
///
/// The inertia is stored as three inverse values along the Body's **local** axes.
/// Every supported Shape is symmetric about its local axes, so a diagonal tensor
/// is exact and a full 3x3 matrix is never needed.
/// 惯量以沿 Body **局部**轴的三组逆值存储。所有受支持的 Shape 都关于局部轴对称，因此
/// 对角阵是精确的，永远不需要完整的 3x3 矩阵。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
   /// Inverse mass. Zero means nothing can accelerate the Body.
   /// 逆质量。为零表示没有任何东西能加速该 Body。
   pub inv_mass: f32,
   /// Inverse inertia per local axis. A zero component means the Body does not
   /// resist rotation about that axis.
   /// 局部各轴的逆惯量。某分量为零表示 Body 不抵抗绕该轴的旋转。
   pub inv_inertia: Vec3,
}

impl MassProperties {
   /// The properties of a Body that nothing can accelerate.
   /// 没有任何东西能加速的 Body 的质量属性。
   pub const IMMOVABLE: Self = Self {
      inv_mass: 0.0,
      inv_inertia: Vec3::ZERO,
   };

   /// Whether this Body can be accelerated at all.
   /// 该 Body 是否根本不能被加速。
   #[must_use]
   pub fn is_immovable(&self) -> bool {
      self.inv_mass == 0.0 && self.inv_inertia == Vec3::ZERO
   }
}

/// A convex collision shape.
/// 凸碰撞形状。
///
/// Cylinder, cone, and convex hull are planned but not implemented yet; see
/// `docs/physics-spec.md` §4.
/// 圆柱、圆锥与凸包已规划但尚未实现；见 `docs/physics-spec.md` §4。
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum Shape {
   /// An axis-aligned box described by its half extents.
   /// 由半长描述的轴对齐盒。
   Box {
      /// Half the box's size on each local axis.
      /// 盒在局部各轴上尺寸的一半。
      half_extents: Vec3,
   },
   /// A sphere described by its radius.
   /// 由半径描述的球。
   Sphere {
      /// The sphere's radius.
      /// 球的半径。
      radius: f32,
   },
}

impl Shape {
   /// An axis-aligned box with the given half extents.
   /// 给定半长的轴对齐盒。
   ///
   /// Half extents are clamped to be non-negative, so a negative input yields a
   /// degenerate (zero-volume) box rather than an inverted one.
   /// 半长被钳制为非负，因此负输入得到退化的（零体积）盒，而不是内外翻转的盒。
   #[must_use]
   pub fn cuboid(half_extents: Vec3) -> Self {
      Self::Box {
         half_extents: half_extents.max(Vec3::ZERO),
      }
   }

   /// A sphere with the given radius, clamped to be non-negative.
   /// 给定半径的球；半径被钳制为非负。
   #[must_use]
   pub fn sphere(radius: f32) -> Self {
      Self::Sphere {
         radius: radius.max(0.0),
      }
   }

   /// The shape's volume, used to sanity-check mass input.
   /// 形状的体积，用于校验质量输入。
   #[must_use]
   pub fn volume(&self) -> f32 {
      match *self {
         Self::Box { half_extents } => 8.0 * half_extents.x * half_extents.y * half_extents.z,
         Self::Sphere { radius } => 4.0 / 3.0 * PI * radius * radius * radius,
      }
   }

   /// The local inertia tensor's diagonal for a given total mass.
   /// 给定总质量时，局部惯量张量的对角元。
   #[must_use]
   pub fn local_inertia(&self, mass: f32) -> Vec3 {
      match *self {
         Self::Box { half_extents } => {
            let h = half_extents;
            // A solid box: I_xx = m/3 * (hy^2 + hz^2), and cyclically.
            // 实心长方体：I_xx = m/3 * (hy^2 + hz^2)，其余按轮换。
            Vec3::new(
               mass / 3.0 * (h.y * h.y + h.z * h.z),
               mass / 3.0 * (h.x * h.x + h.z * h.z),
               mass / 3.0 * (h.x * h.x + h.y * h.y),
            )
         }
         Self::Sphere { radius } => {
            // A solid sphere is isotropic: I = 2/5 * m * r^2.
            // 实心球是各向同性的：I = 2/5 * m * r^2。
            Vec3::splat(0.4 * mass * radius * radius)
         }
      }
   }

   /// Inverse mass properties for a given total mass.
   /// 给定总质量时的逆质量属性。
   ///
   /// A non-finite or non-positive `mass` yields [`MassProperties::IMMOVABLE`]. A
   /// component of the inertia tensor that is zero (a degenerate shape) yields a
   /// zero inverse on that axis, so the Body simply never resists that rotation
   /// instead of producing `inf`.
   /// 非有限或非正的 `mass` 返回 [`MassProperties::IMMOVABLE`]。惯量张量的某个分量为零
   /// （退化形状）时，该轴的逆值为零，于是 Body 只是不抵抗该轴旋转，而不会产生 `inf`。
   #[must_use]
   pub fn mass_properties(&self, mass: f32) -> MassProperties {
      if !mass.is_finite() || mass <= 0.0 {
         return MassProperties::IMMOVABLE;
      }
      let inertia = self.local_inertia(mass);
      MassProperties {
         inv_mass: 1.0 / mass,
         inv_inertia: Vec3::new(
            inverse_or_zero(inertia.x),
            inverse_or_zero(inertia.y),
            inverse_or_zero(inertia.z),
         ),
      }
   }

   /// The world-space bounding box of this shape placed at `position` with
   /// `orientation`.
   /// 该形状以 `position` 与 `orientation` 摆放时的世界空间包围盒。
   #[must_use]
   pub fn aabb(&self, position: Vec3, orientation: Quat) -> Aabb {
      match *self {
         Self::Box { half_extents } => {
            // Rotating the local basis and taking absolute values gives the
            // world-axis extents directly: extent_i = sum_j |R[i][j]| * h_j.
            // 旋转局部基向量并取绝对值即可直接得到世界轴向上的半长：
            // extent_i = sum_j |R[i][j]| * h_j。
            let x = orientation * Vec3::X;
            let y = orientation * Vec3::Y;
            let z = orientation * Vec3::Z;
            let abs = |v: Vec3| Vec3::new(v.x.abs(), v.y.abs(), v.z.abs());
            let (ax, ay, az) = (abs(x), abs(y), abs(z));
            let extent = Vec3::new(
               ax.x * half_extents.x + ay.x * half_extents.y + az.x * half_extents.z,
               ax.y * half_extents.x + ay.y * half_extents.y + az.y * half_extents.z,
               ax.z * half_extents.x + ay.z * half_extents.y + az.z * half_extents.z,
            );
            Aabb::from_center_half_extents(position, extent)
         }
         Self::Sphere { radius } => Aabb::from_center_half_extents(position, Vec3::splat(radius)),
      }
   }
}

/// `1.0 / value`, or zero when the value is not positive and finite.
/// `1.0 / value`，当值非正或非有限时返回零。
fn inverse_or_zero(value: f32) -> f32 {
   if value.is_finite() && value > 0.0 {
      1.0 / value
   } else {
      0.0
   }
}

#[cfg(test)]
mod tests {
   use super::{MassProperties, Shape};
   use he3d_math::{Quat, Vec3};

   #[test]
   fn cuboid_inertia_matches_the_solid_box_formula() {
      let shape = Shape::cuboid(Vec3::new(1.0, 2.0, 3.0));
      let inertia = shape.local_inertia(12.0);
      // m/3 * (hy^2 + hz^2) = 4 * (4 + 9) = 52.
      assert!((inertia.x - 52.0).abs() < 1e-4);
      // m/3 * (hx^2 + hz^2) = 4 * (1 + 9) = 40.
      assert!((inertia.y - 40.0).abs() < 1e-4);
      // m/3 * (hx^2 + hy^2) = 4 * (1 + 4) = 20.
      assert!((inertia.z - 20.0).abs() < 1e-4);
   }

   #[test]
   fn sphere_inertia_is_isotropic() {
      let shape = Shape::sphere(2.0);
      let inertia = shape.local_inertia(5.0);
      let expected = 0.4 * 5.0 * 4.0;
      assert!((inertia.x - expected).abs() < 1e-4);
      assert!((inertia.y - expected).abs() < 1e-4);
      assert!((inertia.z - expected).abs() < 1e-4);
   }

   #[test]
   fn non_positive_mass_is_immovable() {
      assert_eq!(
         Shape::sphere(1.0).mass_properties(0.0),
         MassProperties::IMMOVABLE
      );
      assert_eq!(
         Shape::sphere(1.0).mass_properties(f32::NAN),
         MassProperties::IMMOVABLE
      );
      assert!(Shape::sphere(1.0).mass_properties(0.0).is_immovable());
   }

   #[test]
   fn degenerate_shape_yields_zero_inverse_inertia() {
      let flat = Shape::cuboid(Vec3::new(1.0, 0.0, 1.0));
      let props = flat.mass_properties(2.0);
      // Zero extent on Y means the box has no resistance about Y; no infinities.
      assert!(props.inv_inertia.y.is_finite());
      assert!(props.inv_inertia.x.is_finite());
      assert!(props.inv_mass.is_finite());
   }

   #[test]
   fn axis_aligned_aabb_is_exact() {
      let shape = Shape::cuboid(Vec3::new(1.0, 2.0, 3.0));
      let aabb = shape.aabb(Vec3::new(10.0, 0.0, 0.0), Quat::IDENTITY);
      assert!((aabb.min.x - 9.0).abs() < 1e-5);
      assert!((aabb.max.x - 11.0).abs() < 1e-5);
      assert!((aabb.max.y - 2.0).abs() < 1e-5);
      assert!((aabb.max.z - 3.0).abs() < 1e-5);
   }

   #[test]
   fn rotated_aabb_grows_to_cover_the_corners() {
      let shape = Shape::cuboid(Vec3::splat(1.0));
      // A 45 degree turn about Z makes the diagonal the X/Y extent.
      let aabb = shape.aabb(
         Vec3::ZERO,
         Quat::from_rotation_z(core::f32::consts::FRAC_PI_4),
      );
      let expected = 2.0_f32.sqrt();
      assert!((aabb.max.x - expected).abs() < 1e-4);
      assert!((aabb.max.y - expected).abs() < 1e-4);
      assert!((aabb.max.z - 1.0).abs() < 1e-5);
   }

   #[test]
   fn negative_extents_are_clamped_not_inverted() {
      assert_eq!(
         Shape::cuboid(Vec3::splat(-1.0)),
         Shape::Box {
            half_extents: Vec3::ZERO
         }
      );
      assert_eq!(Shape::sphere(-2.0), Shape::Sphere { radius: 0.0 });
   }
}

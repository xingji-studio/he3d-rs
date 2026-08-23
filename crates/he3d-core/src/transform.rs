use he3d_math::{Mat4, Quat, Vec3};

/// A world-space placement: position, orientation, and (optionally) scale.
/// 一个世界空间中的放置：位置、朝向和（可选的）缩放。
///
/// Scale defaults to one and is intentionally distinct from position so a renderer or physics step can ignore it without ambiguity.
/// 缩放默认为 1，并刻意与位置区分开，以便渲染器或物理步进能够毫无歧义地忽略它。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Transform {
   /// Translation in world space.
   /// 世界空间中的平移。
   pub position: Vec3,
   /// Rotation in world space.
   /// 世界空间中的旋转。
   pub orientation: Quat,
   /// Uniform or per-axis scale.
   /// 均匀或按轴缩放。
   pub scale: Vec3,
}

impl Transform {
   /// Place an object at `position` with identity rotation and unit scale.
   /// 在 `position` 处放置对象，旋转为单位旋转、缩放为单位缩放。
   #[must_use]
   pub const fn from_position(position: Vec3) -> Self {
      Self {
         position,
         orientation: Quat::IDENTITY,
         scale: Vec3::ONE,
      }
   }

   /// Place an object at `position` with the given `orientation` and unit scale.
   /// 在 `position` 处以给定的 `orientation` 放置对象，缩放为单位缩放。
   #[must_use]
   pub const fn from_position_orientation(position: Vec3, orientation: Quat) -> Self {
      Self {
         position,
         orientation,
         scale: Vec3::ONE,
      }
   }

   /// Compose into a single model matrix (scale → rotate → translate).
   /// 组合成单个模型矩阵（缩放 → 旋转 → 平移）。
   #[inline]
   #[must_use]
   pub fn to_matrix(self) -> Mat4 {
      Mat4::from_scale_rotation_translation(self.scale, self.orientation, self.position)
   }

   /// Local +Z transformed into world space.
   /// 局部 +Z 轴变换到世界空间。
   #[inline]
   #[must_use]
   pub fn forward(self) -> Vec3 {
      self.orientation * Vec3::Z
   }

   /// Local +Y transformed into world space.
   /// 局部 +Y 轴变换到世界空间。
   #[inline]
   #[must_use]
   pub fn up(self) -> Vec3 {
      self.orientation * Vec3::Y
   }

   /// Local +X transformed into world space.
   /// 局部 +X 轴变换到世界空间。
   #[inline]
   #[must_use]
   pub fn right(self) -> Vec3 {
      self.orientation * Vec3::X
   }
}

impl Default for Transform {
   fn default() -> Self {
      Self {
         position: Vec3::ZERO,
         orientation: Quat::IDENTITY,
         scale: Vec3::ONE,
      }
   }
}

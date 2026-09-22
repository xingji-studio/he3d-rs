use he3d_math::Vec3;

/// An axis-aligned bounding box in world space.
/// 世界空间中的轴对齐包围盒。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
   /// The corner with the smallest coordinate on every axis.
   /// 每个轴上坐标最小的角点。
   pub min: Vec3,
   /// The corner with the largest coordinate on every axis.
   /// 每个轴上坐标最大的角点。
   pub max: Vec3,
}

impl Aabb {
   /// Build a box from two opposite corners, ordering them if necessary.
   /// 由两个对角点构造包围盒；如有必要会重新排序。
   #[must_use]
   pub fn new(a: Vec3, b: Vec3) -> Self {
      Self {
         min: a.min(b),
         max: a.max(b),
      }
   }

   /// Build a box from a center and half extents.
   /// 由中心与半长构造包围盒。
   #[must_use]
   pub fn from_center_half_extents(center: Vec3, half_extents: Vec3) -> Self {
      Self {
         min: center - half_extents,
         max: center + half_extents,
      }
   }

   /// The center of the box.
   /// 包围盒的中心。
   #[must_use]
   pub fn center(&self) -> Vec3 {
      (self.min + self.max) * 0.5
   }

   /// The half extents of the box.
   /// 包围盒的半长。
   #[must_use]
   pub fn half_extents(&self) -> Vec3 {
      (self.max - self.min) * 0.5
   }

   /// Whether two boxes share any volume. Touching faces count as overlapping.
   /// 两个包围盒是否有体积重叠。面接触算作重叠。
   #[must_use]
   pub fn overlaps(&self, other: &Self) -> bool {
      self.min.cmple(other.max).all() && self.max.cmpge(other.min).all()
   }

   /// The smallest box containing both inputs.
   /// 包含两者的最小包围盒。
   #[must_use]
   pub fn merged(&self, other: &Self) -> Self {
      Self {
         min: self.min.min(other.min),
         max: self.max.max(other.max),
      }
   }

   /// Whether a point lies inside the box, faces included.
   /// 点是否位于包围盒内（含面）。
   #[must_use]
   pub fn contains_point(&self, point: Vec3) -> bool {
      point.cmpge(self.min).all() && point.cmple(self.max).all()
   }

   /// Grow the box by `margin` on every axis.
   /// 每个轴向外扩展 `margin`。
   #[must_use]
   pub fn expanded(&self, margin: f32) -> Self {
      let margin = Vec3::splat(margin);
      Self {
         min: self.min - margin,
         max: self.max + margin,
      }
   }
}

#[cfg(test)]
mod tests {
   use super::Aabb;
   use he3d_math::Vec3;

   #[test]
   fn orders_corners() {
      let aabb = Aabb::new(Vec3::new(1.0, 2.0, 3.0), Vec3::new(-1.0, -2.0, -3.0));
      assert_eq!(aabb.min, Vec3::new(-1.0, -2.0, -3.0));
      assert_eq!(aabb.max, Vec3::new(1.0, 2.0, 3.0));
   }

   #[test]
   fn touching_faces_overlap() {
      let a = Aabb::from_center_half_extents(Vec3::ZERO, Vec3::ONE);
      let b = Aabb::from_center_half_extents(Vec3::new(2.0, 0.0, 0.0), Vec3::ONE);
      assert!(a.overlaps(&b));
   }

   #[test]
   fn separated_boxes_do_not_overlap() {
      let a = Aabb::from_center_half_extents(Vec3::ZERO, Vec3::ONE);
      let b = Aabb::from_center_half_extents(Vec3::new(2.5, 0.0, 0.0), Vec3::ONE);
      assert!(!a.overlaps(&b));
   }
}

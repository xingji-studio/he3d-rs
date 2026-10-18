use he3d_math::scalar::sqrt;
use he3d_math::{Quat, Vec3};

use crate::aabb::Aabb;
use crate::body::BodyId;
use crate::shape::Shape;
use crate::static_geometry::StaticMeshId;
use crate::storage::Slot;
use crate::world::StaticSlot;

/// A half-line: a starting point and a direction.
/// 一条射线：起点与方向。
///
/// `direction` need not be normalized; [`crate::World::cast_ray`] normalizes it, so a hit's distance is always a world length.
/// `direction` 无需归一化；[`crate::World::cast_ray`] 会归一化它，因此命中的距离始终是世界长度。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
   /// Where the ray starts.
   /// 射线的起点。
   pub origin: Vec3,
   /// Which way it travels; the length of this vector is ignored.
   /// 行进方向；该向量的长度会被忽略。
   pub direction: Vec3,
}

impl Ray {
   /// A ray from `origin` toward `direction`.
   /// 从 `origin` 朝 `direction` 的射线。
   #[must_use]
   pub const fn new(origin: Vec3, direction: Vec3) -> Self {
      Self { origin, direction }
   }
}

/// What a raycast hit.
/// 射线检测命中的对象。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RayTarget {
   /// A Body, named by its handle.
   /// 一个 Body，由其句柄命名。
   Body(BodyId),
   /// A registered Static mesh.
   /// 一块已注册的 Static mesh。
   Static(StaticMeshId),
}

/// Where and what a raycast hit.
/// 射线检测在何处命中了何物。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
   /// What the ray met.
   /// 射线遇到的对象。
   pub target: RayTarget,
   /// The world-space point where the ray met the surface.
   /// 射线与表面相遇处的世界空间点。
   pub point: Vec3,
   /// The world-space surface normal at [`RayHit::point`].
   /// [`RayHit::point`] 处的世界空间表面法线。
   ///
   /// A Body's is its outward surface normal; a Static triangle's is the geometric normal of its winding, `(b - a) × (c - a)`.
   /// Body 的是其朝外的表面法线；Static 三角形的是其绕序的几何法线，`(b - a) × (c - a)`。
   pub normal: Vec3,
   /// The distance from the ray's origin to [`RayHit::point`], in world units.
   /// 从射线起点到 [`RayHit::point`] 的距离，世界单位。
   pub distance: f32,
}

/// The nearest thing `ray` meets within `max_distance`.
/// `ray` 在 `max_distance` 内遇到的最近对象。
///
/// Static geometry is scanned linearly, one triangle at a time behind its own bound, the same way the narrow phase reaches it.
/// Body shapes are tested directly, since there are few of them and each test is cheap.
/// Static geometry 以线性方式扫描，逐三角形、各自先过自身包围盒，与窄相位访问它的方式相同。
/// Body 形状直接测试，因为它们数量少且每次测试都很便宜。
pub(crate) fn cast_ray(
   slots: &[Slot],
   static_meshes: &[StaticSlot],
   ray: Ray,
   max_distance: f32,
) -> Option<RayHit> {
   if !ray.origin.is_finite() || !ray.direction.is_finite() {
      return None;
   }
   if max_distance.is_nan() || max_distance < 0.0 {
      return None;
   }
   let direction_length_squared = ray.direction.length_squared();
   if direction_length_squared <= 0.0 {
      return None;
   }
   let direction = ray.direction / sqrt(direction_length_squared);
   let mut nearest = max_distance;
   let mut best: Option<RayHit> = None;

   for (index, slot) in slots.iter().enumerate() {
      let Some(body) = slot.get() else {
         continue;
      };
      let Ok(index) = u32::try_from(index) else {
         break;
      };
      let hit = match body.shape {
         Shape::Sphere { radius } => {
            ray_sphere(ray.origin, direction, body.position, radius, nearest)
         }
         Shape::Box { half_extents } => ray_box(
            ray.origin,
            direction,
            body.position,
            body.orientation,
            half_extents,
            nearest,
         ),
         Shape::Capsule {
            half_height,
            radius,
         } => ray_capsule(
            ray.origin,
            direction,
            body.position,
            body.orientation,
            half_height,
            radius,
            nearest,
         ),
      };
      let Some((distance, normal)) = hit else {
         continue;
      };
      if distance < nearest {
         nearest = distance;
         best = Some(RayHit {
            target: RayTarget::Body(BodyId::new(index, slot.generation)),
            point: ray.origin + direction * distance,
            normal,
            distance,
         });
      }
   }

   for (index, slot) in static_meshes.iter().enumerate() {
      let Some(mesh) = slot.mesh.as_ref() else {
         continue;
      };
      if !ray_reaches_aabb(ray.origin, direction, &mesh.bounds, nearest) {
         continue;
      }
      let Ok(index) = u32::try_from(index) else {
         break;
      };
      let mesh_id = StaticMeshId::new(index, slot.generation);
      for triangle_index in 0..mesh.triangle_count() {
         if !ray_reaches_aabb(
            ray.origin,
            direction,
            &mesh.triangle_bounds[triangle_index],
            nearest,
         ) {
            continue;
         }
         let hit = ray_triangle(
            ray.origin,
            direction,
            mesh.triangle(triangle_index),
            nearest,
         );
         let Some((distance, normal)) = hit else {
            continue;
         };
         if distance < nearest {
            nearest = distance;
            best = Some(RayHit {
               target: RayTarget::Static(mesh_id),
               point: ray.origin + direction * distance,
               normal,
               distance,
            });
         }
      }
   }

   best
}

/// Where a ray meets a sphere, if it does within `max_distance`.
/// 射线在 `max_distance` 内与球相交的位置（若有）。
fn ray_sphere(
   origin: Vec3,
   direction: Vec3,
   center: Vec3,
   radius: f32,
   max_distance: f32,
) -> Option<(f32, Vec3)> {
   let to_origin = origin - center;
   let projected = to_origin.dot(direction);
   let discriminant = projected * projected - (to_origin.length_squared() - radius * radius);
   if discriminant < 0.0 {
      return None;
   }
   let root = sqrt(discriminant);
   // The nearer root is behind the origin when it starts inside; then the exit point is the one that is met.
   // 当起点在球内时，较近的根位于起点之后；此时遇到的是出射点。
   let mut distance = -projected - root;
   if distance < 0.0 {
      distance = -projected + root;
   }
   if distance < 0.0 || distance > max_distance {
      return None;
   }
   let point = origin + direction * distance;
   let offset = point - center;
   let normal = if offset.length_squared() > 0.0 {
      offset / sqrt(offset.length_squared())
   } else {
      -direction
   };
   Some((distance, normal))
}

/// Where a ray meets a box, if it does within `max_distance`.
/// 射线在 `max_distance` 内与盒相交的位置（若有）。
fn ray_box(
   origin: Vec3,
   direction: Vec3,
   center: Vec3,
   orientation: Quat,
   half_extents: Vec3,
   max_distance: f32,
) -> Option<(f32, Vec3)> {
   let inverse = orientation.inverse();
   let local_origin = inverse * (origin - center);
   let local_direction = inverse * direction;
   let mut near = f32::NEG_INFINITY;
   let mut far = f32::INFINITY;
   for axis in 0..3 {
      let origin_axis = component(local_origin, axis);
      let direction_axis = component(local_direction, axis);
      let extent = component(half_extents, axis);
      if direction_axis.abs() < f32::EPSILON {
         if origin_axis.abs() > extent {
            return None;
         }
         continue;
      }
      let first = (-extent - origin_axis) / direction_axis;
      let second = (extent - origin_axis) / direction_axis;
      near = near.max(first.min(second));
      far = far.min(first.max(second));
      if near > far {
         return None;
      }
   }
   // A negative `near` means the origin is inside; the box is then left through the far face.
   // `near` 为负表示起点在盒内；此时从远端面离开盒。
   let distance = if near >= 0.0 { near } else { far };
   if distance < 0.0 || distance > max_distance {
      return None;
   }
   let local_hit = local_origin + local_direction * distance;
   let normal = box_face_normal(local_hit, half_extents, orientation);
   Some((distance, normal))
}

/// The outward normal of the box face a point on the box lies on.
/// 盒上的某个点所在面的朝外法线。
///
/// The point is already on the surface, so the face it belongs to is the one it sits closest to.
/// 该点已在表面上，因此它所属的面就是它离得最近的那个面。
fn box_face_normal(local_hit: Vec3, half_extents: Vec3, orientation: Quat) -> Vec3 {
   let mut axis = 0;
   let mut smallest_gap = f32::MAX;
   for candidate in 0..3 {
      let gap = component(half_extents, candidate) - component(local_hit, candidate).abs();
      if gap < smallest_gap {
         smallest_gap = gap;
         axis = candidate;
      }
   }
   let sign = if component(local_hit, axis) >= 0.0 {
      1.0
   } else {
      -1.0
   };
   let mut local_normal = Vec3::ZERO;
   set_component(&mut local_normal, axis, sign);
   let world = orientation * local_normal;
   if world.length_squared() > 0.0 {
      world / sqrt(world.length_squared())
   } else {
      world
   }
}

/// Where a ray meets a capsule, if it does within `max_distance`.
/// 射线在 `max_distance` 内与胶囊相交的位置（若有）。
///
/// The capsule is a segment swept by a sphere, so its surface is the finite cylinder around the segment plus the two hemispherical caps.
/// The nearest accepted hit across those three features is the entry point, or the exit point when the origin starts inside.
/// 胶囊是一条线段被球扫掠而成，因此其表面是线段周围的有限圆柱加上两个半球端帽。
/// 这三个特征中最小的被接受命中即为入射点；当起点在内部时则是出射点。
fn ray_capsule(
   origin: Vec3,
   direction: Vec3,
   center: Vec3,
   orientation: Quat,
   half_height: f32,
   radius: f32,
   max_distance: f32,
) -> Option<(f32, Vec3)> {
   let inverse = orientation.inverse();
   let local_origin = inverse * (origin - center);
   let local_direction = inverse * direction;
   let mut distance = f32::INFINITY;

   // The cylindrical middle: a circle in the XZ plane, kept only between the two caps.
   // 圆柱中部：XZ 平面上的一个圆，只在两个端帽之间保留。
   let axial = local_direction.x * local_direction.x + local_direction.z * local_direction.z;
   if axial > f32::EPSILON {
      let b = 2.0 * (local_origin.x * local_direction.x + local_origin.z * local_direction.z);
      let c = local_origin.x * local_origin.x + local_origin.z * local_origin.z - radius * radius;
      let discriminant = b * b - 4.0 * axial * c;
      if discriminant >= 0.0 {
         let root = sqrt(discriminant);
         for candidate in [(-b - root) / (2.0 * axial), (-b + root) / (2.0 * axial)] {
            if candidate >= 0.0 && candidate <= max_distance {
               let along_axis = local_origin.y + candidate * local_direction.y;
               if along_axis.abs() <= half_height + f32::EPSILON {
                  distance = distance.min(candidate);
               }
            }
         }
      }
   }

   // The two caps: a sphere at each end, kept only on its own side of the middle.
   // 两个端帽：每端一颗球，只保留在各自一侧的部分。
   for cap in [half_height, -half_height] {
      let cap_centre = Vec3::new(0.0, cap, 0.0);
      let to_origin = local_origin - cap_centre;
      let b = to_origin.dot(local_direction);
      let c = to_origin.length_squared() - radius * radius;
      let discriminant = b * b - c;
      if discriminant < 0.0 {
         continue;
      }
      let root = sqrt(discriminant);
      for candidate in [-b - root, -b + root] {
         if candidate < 0.0 || candidate > max_distance {
            continue;
         }
         let along_axis = local_origin.y + candidate * local_direction.y;
         let on_cap = if cap >= 0.0 {
            along_axis >= half_height - f32::EPSILON
         } else {
            along_axis <= -half_height + f32::EPSILON
         };
         if on_cap {
            distance = distance.min(candidate);
         }
      }
   }

   if !distance.is_finite() {
      return None;
   }
   let local_hit = local_origin + local_direction * distance;
   let local_foot = Vec3::new(0.0, local_hit.y.clamp(-half_height, half_height), 0.0);
   let offset = local_hit - local_foot;
   let local_normal = if offset.length_squared() > 0.0 {
      offset / sqrt(offset.length_squared())
   } else {
      -local_direction
   };
   let world = orientation * local_normal;
   let normal = if world.length_squared() > 0.0 {
      world / sqrt(world.length_squared())
   } else {
      -direction
   };
   Some((distance, normal))
}

/// Where a ray meets a triangle, if it does within `max_distance`.
/// 射线在 `max_distance` 内与三角形相交的位置（若有）。
///
/// This is the Möller–Trumbore test: two barycentric coordinates decide the hit, and the third names the distance.
/// 这是 Möller–Trumbore 测试：两个重心坐标决定是否命中，第三个给出距离。
fn ray_triangle(
   origin: Vec3,
   direction: Vec3,
   triangle: [Vec3; 3],
   max_distance: f32,
) -> Option<(f32, Vec3)> {
   let edge_first = triangle[1] - triangle[0];
   let edge_second = triangle[2] - triangle[0];
   let cross = direction.cross(edge_second);
   let determinant = edge_first.dot(cross);
   if determinant.abs() < 1e-12 {
      return None;
   }
   let inverse = 1.0 / determinant;
   let to_origin = origin - triangle[0];
   let first_barycentric = to_origin.dot(cross) * inverse;
   if !(0.0..=1.0).contains(&first_barycentric) {
      return None;
   }
   let second_cross = to_origin.cross(edge_first);
   let second_barycentric = direction.dot(second_cross) * inverse;
   if second_barycentric < 0.0 || first_barycentric + second_barycentric > 1.0 {
      return None;
   }
   let distance = edge_second.dot(second_cross) * inverse;
   if distance < 0.0 || distance > max_distance {
      return None;
   }
   let normal = edge_first.cross(edge_second).normalize_or_zero();
   Some((distance, normal))
}

/// Whether a ray reaches an axis-aligned box within `max_distance`.
/// 射线在 `max_distance` 内是否够得着一个轴对齐包围盒。
fn ray_reaches_aabb(origin: Vec3, direction: Vec3, bounds: &Aabb, max_distance: f32) -> bool {
   let mut near = 0.0_f32;
   let mut far = max_distance;
   for axis in 0..3 {
      let origin_axis = component(origin, axis);
      let direction_axis = component(direction, axis);
      let lower = component(bounds.min, axis);
      let upper = component(bounds.max, axis);
      if direction_axis.abs() < f32::EPSILON {
         if origin_axis < lower || origin_axis > upper {
            return false;
         }
         continue;
      }
      let first = (lower - origin_axis) / direction_axis;
      let second = (upper - origin_axis) / direction_axis;
      near = near.max(first.min(second));
      far = far.min(first.max(second));
      if near > far {
         return false;
      }
   }
   true
}

/// One component of a three-vector, indexed 0..=2.
/// 三维向量的一个分量，下标 0..=2。
fn component(vector: Vec3, index: usize) -> f32 {
   match index % 3 {
      0 => vector.x,
      1 => vector.y,
      _ => vector.z,
   }
}

/// Overwrite one component of a three-vector, indexed 0..=2.
/// 覆盖三维向量的一个分量，下标 0..=2。
fn set_component(vector: &mut Vec3, index: usize, value: f32) {
   match index % 3 {
      0 => vector.x = value,
      1 => vector.y = value,
      _ => vector.z = value,
   }
}

#[cfg(test)]
mod tests {
   use super::{Ray, RayTarget};
   use crate::body::BodyDesc;
   use crate::shape::Shape;
   use crate::world::World;
   use he3d_math::Vec3;

   /// A flat triangle on the XZ plane, wound so its normal is `+Y`.
   /// XZ 平面上的一个平坦三角形，绕序使其法线为 `+Y`。
   fn ground() -> [Vec3; 3] {
      [
         Vec3::new(0.0, 0.0, 0.0),
         Vec3::new(0.0, 0.0, 1.0),
         Vec3::new(1.0, 0.0, 0.0),
      ]
   }

   #[test]
   fn a_ray_meets_a_sphere_at_the_nearer_surface() {
      let mut world = World::new();
      let ball =
         world.add_body(BodyDesc::dynamic(Shape::sphere(1.0), 1.0).at(Vec3::new(0.0, 0.0, 5.0)));
      let hit = world
         .cast_ray(Ray::new(Vec3::ZERO, Vec3::Z), 100.0)
         .expect("the ray meets the sphere");
      assert_eq!(hit.target, RayTarget::Body(ball));
      assert!((hit.distance - 4.0).abs() < 1e-4, "got {}", hit.distance);
      assert!((hit.point - Vec3::new(0.0, 0.0, 4.0)).length() < 1e-4);
      assert!((hit.normal - Vec3::new(0.0, 0.0, -1.0)).length() < 1e-4);
   }

   #[test]
   fn a_direction_length_does_not_change_the_distance() {
      let mut world = World::new();
      world.add_body(BodyDesc::dynamic(Shape::sphere(1.0), 1.0).at(Vec3::new(0.0, 0.0, 5.0)));
      let hit = world
         .cast_ray(Ray::new(Vec3::ZERO, Vec3::Z * 10.0), 1000.0)
         .expect("the ray meets the sphere");
      assert!((hit.distance - 4.0).abs() < 1e-4, "got {}", hit.distance);
   }

   #[test]
   fn a_ray_that_misses_returns_nothing() {
      let mut world = World::new();
      world.add_body(BodyDesc::dynamic(Shape::sphere(1.0), 1.0).at(Vec3::new(0.0, 0.0, 5.0)));
      assert!(
         world
            .cast_ray(Ray::new(Vec3::ZERO, Vec3::X), 100.0)
            .is_none()
      );
      assert!(
         world.cast_ray(Ray::new(Vec3::ZERO, Vec3::Z), 3.0).is_none(),
         "the sphere is beyond max_distance"
      );
   }

   #[test]
   fn a_zero_direction_reaches_nothing() {
      let mut world = World::new();
      world.add_body(BodyDesc::dynamic(Shape::sphere(1.0), 1.0));
      assert!(
         world
            .cast_ray(Ray::new(Vec3::ZERO, Vec3::ZERO), 100.0)
            .is_none()
      );
   }

   #[test]
   fn a_ray_from_inside_a_sphere_leaves_through_the_far_side() {
      let mut world = World::new();
      world.add_body(BodyDesc::dynamic(Shape::sphere(1.0), 1.0));
      let hit = world
         .cast_ray(Ray::new(Vec3::ZERO, Vec3::Z), 100.0)
         .expect("the exit point is hit");
      assert!((hit.distance - 1.0).abs() < 1e-4, "got {}", hit.distance);
      assert!(
         (hit.normal - Vec3::Z).length() < 1e-4,
         "the outward normal at the exit"
      );
   }

   #[test]
   fn a_ray_meets_a_box_face() {
      let mut world = World::new();
      let block =
         world.add_body(BodyDesc::stationary(Shape::cuboid(Vec3::splat(1.0))).at(Vec3::ZERO));
      let hit = world
         .cast_ray(Ray::new(Vec3::new(0.0, 0.0, -5.0), Vec3::Z), 100.0)
         .expect("the ray meets the box");
      assert_eq!(hit.target, RayTarget::Body(block));
      assert!((hit.distance - 4.0).abs() < 1e-4, "got {}", hit.distance);
      assert!((hit.normal - Vec3::new(0.0, 0.0, -1.0)).length() < 1e-4);
   }

   #[test]
   fn a_rotated_box_turns_its_normal_with_it() {
      let mut world = World::new();
      world.add_body(
         BodyDesc::stationary(Shape::cuboid(Vec3::splat(1.0)))
            .at(Vec3::ZERO)
            .oriented(he3d_math::Quat::from_rotation_z(
               core::f32::consts::FRAC_PI_6,
            )),
      );
      let hit = world
         .cast_ray(
            Ray::new(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0)),
            100.0,
         )
         .expect("the ray meets the rotated box");
      // A 30 degree box about Z tilts its top normal to (-sin 30, cos 30, 0).
      // 绕 Z 旋转 30 度的盒把其顶面法线倾斜到 (-sin 30, cos 30, 0)。
      assert!((hit.normal.x + 0.5).abs() < 1e-3, "got {:?}", hit.normal);
      assert!(
         (hit.normal.y - 0.866_025).abs() < 1e-3,
         "got {:?}",
         hit.normal
      );
      assert!(
         (hit.distance - 3.845_3).abs() < 1e-3,
         "got {}",
         hit.distance
      );
   }

   #[test]
   fn a_ray_meets_static_geometry() {
      let mut world = World::new();
      let mesh = world.add_static_mesh(&ground()).expect("one triangle");
      let hit = world
         .cast_ray(
            Ray::new(Vec3::new(0.25, 5.0, 0.25), Vec3::new(0.0, -1.0, 0.0)),
            100.0,
         )
         .expect("the ray meets the triangle");
      assert_eq!(hit.target, RayTarget::Static(mesh));
      assert!((hit.distance - 5.0).abs() < 1e-4, "got {}", hit.distance);
      assert!((hit.normal - Vec3::Y).length() < 1e-4);
   }

   #[test]
   fn the_nearest_hit_wins_across_bodies_and_static_geometry() {
      let mut world = World::new();
      world.add_static_mesh(&ground()).expect("one triangle");
      let ball =
         world.add_body(BodyDesc::dynamic(Shape::sphere(1.0), 1.0).at(Vec3::new(0.25, 3.0, 0.25)));
      // The ray first meets the sphere at y = 4, then the ground at y = 0.
      // 射线先遇到 y = 4 处的球，再遇到 y = 0 处的地面。
      let hit = world
         .cast_ray(
            Ray::new(Vec3::new(0.25, 10.0, 0.25), Vec3::new(0.0, -1.0, 0.0)),
            100.0,
         )
         .expect("something is hit");
      assert_eq!(hit.target, RayTarget::Body(ball));
      assert!((hit.distance - 6.0).abs() < 1e-4, "got {}", hit.distance);
   }

   #[test]
   fn a_ray_meets_a_capsule_along_its_side() {
      let mut world = World::new();
      let pill = world.add_body(BodyDesc::stationary(Shape::capsule(1.0, 0.5)));
      let hit = world
         .cast_ray(
            Ray::new(Vec3::new(5.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)),
            100.0,
         )
         .expect("the ray meets the capsule side");
      assert_eq!(hit.target, RayTarget::Body(pill));
      assert!((hit.distance - 4.5).abs() < 1e-4, "got {}", hit.distance);
      assert!(
         (hit.normal - Vec3::X).length() < 1e-4,
         "got {:?}",
         hit.normal
      );
   }

   #[test]
   fn a_ray_meets_a_capsule_at_its_cap() {
      let mut world = World::new();
      world.add_body(BodyDesc::stationary(Shape::capsule(1.0, 0.5)));
      let hit = world
         .cast_ray(
            Ray::new(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0)),
            100.0,
         )
         .expect("the ray meets the top cap");
      assert!((hit.distance - 3.5).abs() < 1e-4, "got {}", hit.distance);
      assert!(
         (hit.normal - Vec3::Y).length() < 1e-4,
         "got {:?}",
         hit.normal
      );
   }

   #[test]
   fn a_ray_past_a_capsule_cap_returns_nothing() {
      let mut world = World::new();
      world.add_body(BodyDesc::stationary(Shape::capsule(1.0, 0.5)));
      assert!(
         world
            .cast_ray(
               Ray::new(Vec3::new(5.0, 3.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)),
               100.0
            )
            .is_none(),
         "the ray passes above the capsule"
      );
   }

   #[test]
   fn a_ray_meets_a_rotated_capsule() {
      let mut world = World::new();
      world.add_body(BodyDesc::stationary(Shape::capsule(1.0, 0.5)).oriented(
         he3d_math::Quat::from_rotation_z(core::f32::consts::FRAC_PI_2),
      ));
      // Rotated 90 degrees about Z, the capsule lies along X, so a ray down the Y axis meets its side at y = 0.5.
      let hit = world
         .cast_ray(
            Ray::new(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1.0, 0.0)),
            100.0,
         )
         .expect("the ray meets the rotated capsule");
      assert!((hit.distance - 4.5).abs() < 1e-4, "got {}", hit.distance);
      assert!(
         (hit.normal - Vec3::Y).length() < 1e-4,
         "got {:?}",
         hit.normal
      );
   }
}

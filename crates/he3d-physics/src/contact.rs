use he3d_math::Vec3;

use crate::body::BodyId;
use crate::static_geometry::StaticMeshId;

/// The maximum number of points a single pair of Bodies can produce.
/// 一对 Body 最多能产生的接触点数。
///
/// Clipping one convex face against another yields at most one vertex per edge of
/// the reference face, so a box-on-box manifold never exceeds four; and four
/// coplanar points are the minimum needed to pin a resting face against rocking
/// about either axis. The constraint arrays are sized to this, so it is also the
/// compile-time capacity.
/// 把一个凸面裁剪到另一个上，最多为参考面的每条棱产生一个顶点，因此盒对盒的流形不会超过
/// 四个；而四个共面点正是把一个静止面在两个轴上都钉住、不让它摇摆所需的最小数量。约束数组
/// 按此分配，因此它同时也是编译期容量。
pub(crate) const MAX_MANIFOLD_POINTS: usize = 4;

/// Whether a Contact is appearing, persisting, or disappearing this step.
/// 该 Contact 本步是正在出现、持续存在，还是正在消失。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ContactPhase {
   /// The pair touched this step and did not touch last step.
   /// 本步相碰，上一步未相碰。
   Begin,
   /// The pair touched last step and still touches.
   /// 上一步相碰，本步仍相碰。
   Persist,
   /// The pair touched last step and does not touch now.
   /// 上一步相碰，本步不再相碰。
   End,
}

/// What lies on the far side of a Contact's normal.
/// Contact 法线另一侧的东西。
///
/// A Contact is always anchored on a Body; the other side is either a second Body
/// or a piece of Static geometry, which has no Body of its own.
/// 一个 Contact 总以某个 Body 为一端；另一端要么是另一个 Body，要么是一块本身没有 Body
/// 的 Static geometry。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ContactPartner {
   /// Another Body.
   /// 另一个 Body。
   Body(BodyId),
   /// A registered piece of Static geometry.
   /// 一块已注册的 Static geometry。
   Static(StaticMeshId),
}

/// A single point at which two things touch.
/// 两样东西相互接触的单个点。
///
/// The normal points from `a` toward `b`; `depth` is how far the two shapes
/// overlap along it. A pair that separates still reports an `End` contact built
/// from the last state in which it overlapped, so an application can always
/// observe both ends of a contact's lifetime.
/// 法线由 `a` 指向 `b`；`depth` 是两形状沿该方向的重叠量。已经分离的一对仍会报出一个
/// 由最后一次重叠状态构造的 `End` 接触，因此应用总能观察到一段接触生命周期的两端。
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct Contact {
   /// The Body the normal points away from.
   /// 法线背离的那个 Body。
   pub a: BodyId,
   /// What the normal points toward.
   /// 法线指向的东西。
   pub b: ContactPartner,
   /// The contact point, in world space.
   /// 接触点（世界空间）。
   pub point: Vec3,
   /// The unit contact normal, pointing from `a` toward `b`.
   /// 单位接触法线，由 `a` 指向 `b`。
   pub normal: Vec3,
   /// Penetration depth along the normal. Always positive for a live Contact.
   /// 沿法线的穿透深度。存活接触恒为正。
   pub depth: f32,
   /// Where this Contact is in its lifetime.
   /// 该接触在其生命周期中的位置。
   pub phase: ContactPhase,
}

impl Contact {
   /// Whether the Contact has the given Body on either side.
   /// 该 Contact 的任意一侧是否为给定的 Body。
   #[must_use]
   pub fn involves(&self, id: BodyId) -> bool {
      self.a == id || self.b == ContactPartner::Body(id)
   }
}

/// One point of a manifold, before it becomes a solved constraint.
/// 流形中的一个点，尚未成为已求解的约束。
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ManifoldPoint {
   /// World-space contact position.
   pub position: Vec3,
   /// Penetration depth along the manifold normal.
   pub depth: f32,
   /// A stable identifier for warm-start matching between steps.
   pub feature: u32,
}

/// The points at which two shapes overlap, with the normal between them.
/// 两个形状重叠处的接触点，以及它们之间的法线。
#[derive(Clone, Copy, Debug)]
pub(crate) struct Manifold {
   /// Unit normal pointing from the first shape toward the second.
   pub normal: Vec3,
   /// The contact points, in a deterministic order.
   pub points: [ManifoldPoint; MAX_MANIFOLD_POINTS],
   /// How many entries of `points` are live.
   pub count: usize,
}

impl Default for Manifold {
   fn default() -> Self {
      Self {
         normal: Vec3::Z,
         points: [ManifoldPoint::default(); MAX_MANIFOLD_POINTS],
         count: 0,
      }
   }
}

impl Manifold {
   /// An empty manifold.
   pub(crate) fn empty() -> Self {
      Self::default()
   }

   /// Append a point, ignoring anything past the manifold's capacity.
   pub(crate) fn push(&mut self, point: ManifoldPoint) {
      if self.count < MAX_MANIFOLD_POINTS {
         self.points[self.count] = point;
         self.count += 1;
      }
   }
}

/// One point of a contact after the solver has claimed it.
/// 求解器接管后的单个接触点。
///
/// `anchor_a` and `anchor_b` are the contact position relative to each Body's
/// center of mass, captured once so that every solver iteration applies its
/// impulse at exactly the same point.
/// `anchor_a` 与 `anchor_b` 是接触点相对于各 Body 质心的位置，只捕获一次，使每次求解
/// 迭代都在完全相同的点上施加冲量。
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ConstraintPoint {
   /// World-space contact position, captured when the constraint was built.
   pub position: Vec3,
   /// Contact position relative to Body `a`'s center.
   pub anchor_a: Vec3,
   /// Contact position relative to Body `b`'s center.
   pub anchor_b: Vec3,
   /// Penetration depth at the start of the step.
   pub depth: f32,
   /// Feature id, used to carry impulses across steps.
   pub feature: u32,
   /// Accumulated normal impulse.
   pub normal_impulse: f32,
   /// Accumulated impulse along each friction tangent.
   pub tangent_impulse: [f32; 2],
   /// Effective mass along the normal.
   pub normal_mass: f32,
   /// Effective mass along each friction tangent.
   pub tangent_mass: [f32; 2],
   /// Velocity bias from restitution, applied to real velocities.
   pub bias: f32,
   /// Penetration bias, applied only to pseudo-velocities so that removing
   /// overlap never leaves kinetic energy behind.
   pub position_bias: f32,
   /// Accumulated pseudo-impulse that removes penetration.
   pub position_impulse: f32,
}

/// A solved constraint: one normal, two friction tangents, and up to four points.
/// 一条已求解的约束：一条法线、两条摩擦切线，以及至多四个接触点。
#[derive(Clone, Copy, Debug)]
pub(crate) struct ContactConstraint {
   /// The handle of the first Body, kept so an `End` contact can still name it
   /// after the Body itself has been removed.
   /// 第一个 Body 的句柄。保留它是为了在 Body 已被移除之后，`End` 接触仍能指代它。
   pub id_a: BodyId,
   /// What lies on the other side.
   /// 另一侧是什么。
   pub id_b: ContactPartner,
   /// Slot index of the first Body.
   /// 第一个 Body 的槽位索引。
   pub a: u32,
   /// Slot index of the other Body, or [`crate::storage::STATIC_PARTNER`] when the
   /// other side is Static geometry.
   /// 另一侧 Body 的槽位索引；当另一侧是 Static geometry 时为
   /// [`crate::storage::STATIC_PARTNER`]。
   pub b: u32,
   /// Distinguishes constraints that share a Body pair.
   ///
   /// All of a Body's contacts against Static geometry share the same `(a, b)`, so
   /// this carries the mesh and triangle identity instead. It is zero for a
   /// Body-versus-Body constraint.
   /// 区分共享同一对 `(a, b)` 的约束。一个 Body 与 Static geometry 的所有接触共享同一个
   /// `(a, b)`，因此这里改为携带网格与三角形的身份。Body 对 Body 的约束此值为零。
   pub partner_key: u64,
   /// Unit normal from `a` toward `b`.
   /// 由 `a` 指向 `b` 的单位法线。
   pub normal: Vec3,
   /// The two friction tangents, orthogonal to the normal and to each other.
   /// 两条摩擦切线，与法线正交且彼此正交。
   pub tangents: [Vec3; 2],
   /// Combined Coulomb friction coefficient.
   /// 合并后的库仑摩擦系数。
   pub friction: f32,
   /// How many entries of `points` are live.
   /// `points` 中存活的条目数。
   pub point_count: u8,
   /// The contact points.
   /// 接触点。
   pub points: [ConstraintPoint; MAX_MANIFOLD_POINTS],
}

impl Default for ContactConstraint {
   fn default() -> Self {
      Self {
         id_a: BodyId::new(0, 0),
         id_b: ContactPartner::Body(BodyId::new(0, 0)),
         a: 0,
         b: 0,
         partner_key: 0,
         normal: Vec3::Z,
         tangents: [Vec3::X, Vec3::Y],
         friction: 0.0,
         point_count: 0,
         points: [ConstraintPoint::default(); MAX_MANIFOLD_POINTS],
      }
   }
}

impl ContactConstraint {
   /// The live points.
   /// 存活的接触点。
   pub(crate) fn points(&self) -> &[ConstraintPoint] {
      &self.points[..usize::from(self.point_count)]
   }

   /// The live points, mutably.
   /// 存活的接触点（可变）。
   pub(crate) fn points_mut(&mut self) -> &mut [ConstraintPoint] {
      let count = usize::from(self.point_count);
      &mut self.points[..count]
   }

   /// The key that identifies this constraint across steps.
   ///
   /// Both constraint lists are sorted by it, so warm starting can match a pair
   /// with a binary search.
   /// 跨步识别这条约束的键。两份约束列表都按它排序，因此 warm starting 可以用二分查找
   /// 匹配同一对。
   pub(crate) fn sort_key(&self) -> (u32, u32, u64) {
      (self.a, self.b, self.partner_key)
   }
}

/// Build an orthonormal basis around `normal`, returned as the two tangents.
/// 围绕 `normal` 构造一组正交基，返回两条切线。
pub(crate) fn tangent_basis(normal: Vec3) -> [Vec3; 2] {
   // Pick the axis least aligned with the normal so the cross product is stable.
   // 选取与法线最不共线的轴，使叉积保持数值稳定。
   let helper = if normal.x.abs() > normal.y.abs() {
      Vec3::new(0.0, 1.0, 0.0)
   } else {
      Vec3::new(1.0, 0.0, 0.0)
   };
   let first = normal.cross(helper).normalize_or_zero();
   let second = normal.cross(first);
   [first, second]
}

#[cfg(test)]
// Test fixtures turn small loop counters into depths; the values sit far inside
// the range f32 represents exactly.
// 测试夹具把小整数循环计数转换为深度；这些值远在 f32 能精确表示的范围内。
#[allow(clippy::cast_precision_loss)]
mod tests {
   use super::{ContactConstraint, Manifold, ManifoldPoint, tangent_basis};
   use he3d_math::Vec3;

   #[test]
   fn a_manifold_stops_at_its_capacity() {
      let mut manifold = Manifold::empty();
      for index in 0..10 {
         manifold.push(ManifoldPoint {
            depth: index as f32,
            ..ManifoldPoint::default()
         });
      }
      assert_eq!(manifold.count, 4);
   }

   #[test]
   fn tangents_are_orthonormal_to_the_normal() {
      for normal in [
         Vec3::X,
         Vec3::Y,
         Vec3::Z,
         Vec3::new(1.0, 2.0, 3.0).normalize(),
      ] {
         let [t0, t1] = tangent_basis(normal);
         assert!(normal.dot(t0).abs() < 1e-5, "t0 not orthogonal");
         assert!(normal.dot(t1).abs() < 1e-5, "t1 not orthogonal");
         assert!(t0.dot(t1).abs() < 1e-5, "tangents not orthogonal");
         assert!((t0.length() - 1.0).abs() < 1e-5);
         assert!((t1.length() - 1.0).abs() < 1e-5);
      }
   }

   #[test]
   fn an_empty_constraint_has_no_points() {
      let constraint = ContactConstraint::default();
      assert!(constraint.points().is_empty());
   }
}

use he3d_math::Vec3;
use he3d_math::scalar::sqrt;

use crate::body::Body;
use crate::contact::ContactConstraint;
use crate::storage::{Slot, constraint_bodies, constraint_bodies_ref};

/// Velocity iterations per step.
///
/// Tall stacks need more than short ones: with plain sequential impulses the
/// constraint graph converges by Gauss-Seidel, so the count must scale with the
/// height of what is stacked. See the tall-stack limitation in
/// `docs/physics-spec.md`.
/// 每步的速度迭代次数。高塔比矮堆需要更多：纯序列冲量下约束图靠 Gauss-Seidel 收敛，
/// 因此迭代次数必须随堆叠高度增长。见 `docs/physics-spec.md` 中的高塔限制。
pub(crate) const DEFAULT_VELOCITY_ITERATIONS: u32 = 12;

/// Fraction of the remaining penetration removed per step.
/// 每步消除的剩余穿透比例。
///
/// A relaxation factor, not an exact correction. Removing all of the error in one
/// step would overshoot and oscillate, because the contact's own impulse in the same
/// step has already moved the geometry. A fifth per step clears 96% of a slop-sized
/// overlap in about fifteen steps while staying far below the overshoot bound of 1.
/// 松弛因子而非精确修正。一步消除全部误差会过冲并振荡，因为同一步中接触自身的冲量已经改变
/// 了几何。每步五分之一约十五步清除 slop 量级重叠的 96%，同时远低于 1 这个过冲边界。
const BAUMGARTE: f32 = 0.2;

/// Penetration allowed to persist, so resting contacts stop pushing.
/// 允许保留的穿透量，使静止接触不再持续推挤。
///
/// A resting Body sinks by `g * dt^2` before its contact answers — about 0.7 mm at
/// 1/120 s and one gravity — so the slop has to be comfortably larger than that or
/// the contact would alternate between pushing and not pushing. It is also small
/// enough to be invisible on a Body of ordinary size.
/// 静止的 Body 在其接触作出回应前会下沉 `g * dt^2`——在 1/120 秒、一个重力下约 0.7
/// 毫米——因此 slop 必须明显大于它，否则接触会在"推"与"不推"之间反复切换。同时它也小到
/// 在常规尺寸的 Body 上不可见。
const PENETRATION_SLOP: f32 = 0.005;

/// Approach speed below which restitution is suppressed.
///
/// Without this a resting Body bounces forever on the tiny velocity gravity adds
/// each step, which is the classic source of stack jitter.
///
/// Gravity adds `g * dt` — about 0.08 m/s at 1/120 s — to a resting Body's approach
/// speed every step, so the threshold only has to sit well above that. A whole metre
/// per second is a deliberate order-of-magnitude margin: anything slower is a settle
/// rather than an impact, and the cost is that a 0.9 m/s ball does not bounce.
/// 低于此接近速度时抑制弹性。若没有它，静止的 Body 会在重力每步加入的微小速度上永远
/// 弹跳，这正是堆叠抖动的经典来源。
/// 重力每步给静止 Body 的接近速度加上 `g * dt`——在 1/120 秒下约 0.08 m/s——因此阈值
/// 只需明显高于它。整整 1 米/秒是刻意留出的一个数量级余量：更慢的都算沉降而非撞击，代价是
/// 0.9 m/s 的球不会回弹。
const RESTITUTION_THRESHOLD: f32 = 1.0;

/// Iterations of the positional-correction pass.
/// 位置修正阶段的迭代次数。
///
/// Position correction relaxes over the same contact graph as the velocity solve,
/// but it only has to push overlap out of the manifold rather than converge a force
/// network, so it needs far fewer passes. Three is enough for a resting stack's
/// contacts to agree with each other without paying for a second full solve.
/// 位置修正是在与速度求解相同的接触图上做松弛，但它只需把重叠推出流形，而不必收敛一个力
/// 网络，因此需要的遍数少得多。三遍足以让静止堆叠的各接触互相协调，又不必再付一次完整求解
/// 的代价。
const POSITION_ITERATIONS: u32 = 3;

/// An effective mass below this is treated as infinite and receives no impulse.
/// 有效质量低于此值时视为无穷大，不接收任何冲量。
///
/// It is compared against a sum of inverse masses and inverse inertia terms, so its
/// unit is `1/kg`: this rejects anything heavier than about 10^8 kg, which no impulse
/// one frame can produce will move.
/// 它与逆质量、逆惯量项之和比较，因此单位是 `1/kg`：它拒绝一切重于约 10^8 千克的东西，
/// 而一帧所能产生的任何冲量都推不动那种东西。
const INFINITE_EFFECTIVE_MASS: f32 = 1e-8;

/// A carried-over impulse whose squared magnitude is below this is not re-applied.
/// 沿用冲量的模平方低于此值时不再重新施加。
///
/// The comparison is against the squared magnitude, so the real cutoff is `1e-4 N*s`.
/// That changes a 1 kg Body's velocity by 1e-4 m/s — five hundred times below the
/// sleep threshold — so dropping it cannot affect anything a caller can observe.
/// 比较对象是模的平方，因此真正的阈值是 `1e-4 N*s`。它只能改变 1 kg Body 的速度
/// 1e-4 m/s——比睡眠阈值低五百倍——因此丢弃它不会影响调用者能观察到的任何东西。
const MINIMUM_WARM_START_IMPULSE_SQUARED: f32 = 1e-8;

/// Tuning for the sequential-impulse solver.
/// 序列冲量求解器的调参。
#[derive(Clone, Copy, Debug)]
pub(crate) struct SolverSettings {
   /// Velocity iterations per step.
   pub(crate) velocity_iterations: u32,
}

impl Default for SolverSettings {
   fn default() -> Self {
      Self {
         velocity_iterations: DEFAULT_VELOCITY_ITERATIONS,
      }
   }
}

/// Whether a pair can move at all, and so is worth solving.
///
/// Two immovable Bodies constrain nothing; skipping them keeps static scenery from
/// consuming iterations.
/// 两个不可移动的 Body 之间没有任何约束；跳过它们可避免静态景物消耗迭代。
pub(crate) fn is_solvable(first: &Body, second: &Body) -> bool {
   first.motion.is_pushable() || second.motion.is_pushable()
}

/// Compute effective masses and velocity biases for one step.
///
/// The restitution bias is taken from the approach velocity *before* any impulse
/// is applied, which is why this runs before warm starting.
/// 为一步计算有效质量与速度偏置。弹性偏置取自施加任何冲量**之前**的接近速度，因此这一
/// 步必须在 warm starting 之前运行。
pub(crate) fn prepare(slots: &[Slot], constraints: &mut [ContactConstraint], delta_time: f32) {
   let inverse_delta = 1.0 / delta_time;
   for constraint in constraints.iter_mut() {
      let Some((first, second)) =
         constraint_bodies_ref(slots, constraint.a as usize, constraint.b as usize)
      else {
         continue;
      };
      let normal = constraint.normal;
      let tangents = constraint.tangents;
      let inverse_mass_sum = first.mass.inv_mass + second.map_or(0.0, |body| body.mass.inv_mass);

      for point in constraint.points_mut() {
         let arm_first = point.anchor_a;
         let arm_second = point.anchor_b;

         // Effective mass is the relative velocity's response to a unit impulse.
         // 有效质量是相对速度对单位冲量的响应。
         let normal_term = normal_effective_mass(
            first,
            second,
            arm_first,
            arm_second,
            normal,
            inverse_mass_sum,
         );
         point.normal_mass = reciprocal_or_zero(normal_term);
         for (index, tangent) in tangents.into_iter().enumerate() {
            let term = normal_effective_mass(
               first,
               second,
               arm_first,
               arm_second,
               tangent,
               inverse_mass_sum,
            );
            point.tangent_mass[index] = reciprocal_or_zero(term);
         }

         let approach = relative_velocity(first, second, arm_first, arm_second, normal);
         // The velocity solve only ever removes energy, so a resting stack ends a
         // step with exactly zero separating velocity and can fall asleep.
         // Penetration is removed separately, through pseudo-velocities.
         // 速度求解只会移除能量，因此静止堆叠在一步结束时分离速度恰好为零，能够入睡。
         // 穿透另行处理，通过伪速度消除。
         let mut bias = 0.0;
         if approach < -RESTITUTION_THRESHOLD {
            let restitution = second.map_or(0.0, |body| body.restitution);
            let restitution = first.restitution.max(restitution);
            if restitution > 0.0 {
               bias = -restitution * approach;
            }
         }
         point.bias = bias;
         point.position_bias =
            BAUMGARTE * inverse_delta * (point.depth - PENETRATION_SLOP).max(0.0);
      }
   }
}

/// Apply the impulses carried over from the previous step.
///
/// This is what lets a resting stack converge in a handful of iterations instead
/// of rebuilding its contact forces from zero every step.
/// 这使静止的堆叠能在少数几次迭代内收敛，而不必每步从零重建接触力。
pub(crate) fn warm_start(slots: &mut [Slot], constraints: &[ContactConstraint]) {
   for constraint in constraints {
      let normal = constraint.normal;
      let tangents = constraint.tangents;
      let Some(mut pair) =
         ContactPair::resolve(slots, constraint.a as usize, constraint.b as usize)
      else {
         continue;
      };
      for point in constraint.points() {
         let mut impulse = normal * point.normal_impulse;
         for (index, tangent) in tangents.into_iter().enumerate() {
            impulse += tangent * point.tangent_impulse[index];
         }
         if impulse.length_squared() <= MINIMUM_WARM_START_IMPULSE_SQUARED {
            continue;
         }
         pair.apply_impulse(point.anchor_a, point.anchor_b, impulse);
      }
   }
}

/// Drive every contact toward a non-negative separating velocity.
///
/// Friction is solved along two tangents inside the Coulomb cone
/// `|jt| <= mu * jn`.
/// 将每个接触推向非负的分离速度。摩擦沿两条切线在库仑锥 `|jt| <= mu * jn` 内求解。
pub(crate) fn solve_velocities(
   slots: &mut [Slot],
   constraints: &mut [ContactConstraint],
   settings: SolverSettings,
) {
   for _ in 0..settings.velocity_iterations {
      for constraint in constraints.iter_mut() {
         solve_constraint_velocity(slots, constraint);
      }
   }
}

/// Drive one constraint's points toward a non-negative separating velocity.
/// 将一条约束的各个点推向非负的分离速度。
fn solve_constraint_velocity(slots: &mut [Slot], constraint: &mut ContactConstraint) {
   let normal = constraint.normal;
   let tangents = constraint.tangents;
   let friction = constraint.friction;
   // Resolve the pair once for the whole constraint, not once per point and
   // per tangent: the Bodies a constraint names cannot change while it is
   // being solved, and re-deriving them is the inner loop's largest fixed
   // cost.
   // 整条约束只解析一次 Body 对，而不是每个点、每条切线各解析一次：求解期间约束所指
   // 的 Body 不会改变，而重复解析正是内循环最大的一笔固定开销。
   let Some(mut pair) = ContactPair::resolve(slots, constraint.a as usize, constraint.b as usize)
   else {
      return;
   };

   for point in constraint.points_mut() {
      let relative = pair.relative_velocity(point.anchor_a, point.anchor_b, normal);
      let lambda = -point.normal_mass * (relative - point.bias);
      let previous = point.normal_impulse;
      // A contact can push but never pull.
      // 接触只能推，永远不能拉。
      point.normal_impulse = (previous + lambda).max(0.0);
      let delta = point.normal_impulse - previous;
      pair.apply_impulse(point.anchor_a, point.anchor_b, normal * delta);

      for (index, tangent) in tangents.into_iter().enumerate() {
         let relative = pair.relative_velocity(point.anchor_a, point.anchor_b, tangent);
         let lambda = -point.tangent_mass[index] * relative;
         let limit = friction * point.normal_impulse;
         let previous = point.tangent_impulse[index];
         point.tangent_impulse[index] = (previous + lambda).clamp(-limit, limit);
         let delta = point.tangent_impulse[index] - previous;
         pair.apply_impulse(point.anchor_a, point.anchor_b, tangent * delta);
      }
   }
}

/// Push remaining overlap apart using pseudo-velocities only.
///
/// Running positional correction against a separate velocity channel is what keeps
/// it from turning into kinetic energy. That is the difference between a stack
/// that settles and sleeps and one that buzzes against its own contacts forever.
/// 仅使用伪速度推开剩余重叠。让位置修正作用于独立的通道，是它不会转化为动能的原因，
/// 也是"能稳定并入睡的堆叠"与"永远在自己接触上嗡嗡响的堆叠"之间的差别。
pub(crate) fn solve_positions(slots: &mut [Slot], constraints: &mut [ContactConstraint]) {
   for _ in 0..POSITION_ITERATIONS {
      for constraint in constraints.iter_mut() {
         let normal = constraint.normal;
         let Some(mut pair) =
            ContactPair::resolve(slots, constraint.a as usize, constraint.b as usize)
         else {
            continue;
         };
         for point in constraint.points_mut() {
            let relative = pair.pseudo_relative_velocity(point.anchor_a, point.anchor_b, normal);
            let lambda = -point.normal_mass * (relative - point.position_bias);
            let previous = point.position_impulse;
            point.position_impulse = (previous + lambda).max(0.0);
            let delta = point.position_impulse - previous;
            pair.apply_pseudo_impulse(point.anchor_a, point.anchor_b, normal * delta);
         }
      }
   }
}

/// The two Bodies a constraint acts on, resolved once and held for the whole solve.
/// 一条约束所作用的两个 Body，解析一次并在整个求解过程中持有。
///
/// Resolving is not a detail the loops should repeat: the pair behind a constraint
/// is fixed for as long as the constraint is solved, and every point and every
/// iteration must push on exactly that pair. Folding the lookup, the two
/// relative-velocity readings, and the two impulse writers behind one type is what
/// makes "resolve once" something the code cannot get wrong — the hot loop has no
/// way to reach a Body except through a resolved pair.
/// 解析不该由各个循环反复进行：一条约束背后的 Body 对在其求解期间是固定的，每个点、每次
/// 迭代都必须正好作用于该对。把查找、两个相对速度读数与两个冲量写入收进一个类型，使"只解析
/// 一次"成为代码无法做错的事——热循环除了经由已解析的对，没有别的途径触及 Body。
///
/// A missing second Body is Static geometry: it never moves, so it contributes
/// nothing to a relative velocity and receives nothing from an impulse.
/// 缺失的第二个 Body 表示 Static geometry：它永不移动，因此对相对速度没有贡献，也不从冲量
/// 中接收任何东西。
struct ContactPair<'a> {
   /// The Body the constraint's first side names.
   /// 约束第一侧所指的 Body。
   first: &'a mut Body,
   /// The other side, or `None` for Static geometry.
   /// 另一侧；对于 Static geometry 为 `None`。
   second: Option<&'a mut Body>,
}

impl<'a> ContactPair<'a> {
   /// Resolve the pair a constraint names, or `None` when a slot is empty.
   /// 解析一条约束所指的对；当某个槽位为空时返回 `None`。
   fn resolve(slots: &'a mut [Slot], first: usize, second: usize) -> Option<Self> {
      let (first, second) = constraint_bodies(slots, first, second)?;
      Some(Self { first, second })
   }

   /// The relative velocity along `axis`, measured at the two contact anchors.
   /// 沿 `axis` 在两个接触锚点处测量的相对速度。
   fn relative_velocity(&self, arm_first: Vec3, arm_second: Vec3, axis: Vec3) -> f32 {
      relative_velocity(
         self.first,
         self.second.as_deref(),
         arm_first,
         arm_second,
         axis,
      )
   }

   /// The relative pseudo-velocity along `axis`, measured at the contact anchors.
   /// 沿 `axis` 在接触锚点处测量的相对伪速度。
   fn pseudo_relative_velocity(&self, arm_first: Vec3, arm_second: Vec3, axis: Vec3) -> f32 {
      let first_share = self.first.pseudo_point_velocity(arm_first).dot(axis);
      let second_share = self
         .second
         .as_deref()
         .map_or(0.0, |body| body.pseudo_point_velocity(arm_second).dot(axis));
      second_share - first_share
   }

   /// Apply one impulse, at the given anchors.
   ///
   /// The impulse pushes `second` along its own direction and `first` against it. A
   /// missing `second` is Static geometry and simply receives nothing.
   /// 在给定锚点施加一个冲量。冲量沿自身方向推 `second`，并反向推 `first`。缺失的
   /// `second` 表示 Static geometry，它不接收任何东西。
   fn apply_impulse(&mut self, arm_first: Vec3, arm_second: Vec3, impulse: Vec3) {
      self.first.linear_velocity -= impulse * self.first.mass.inv_mass;
      self.first.angular_velocity -= self.first.apply_inv_inertia(arm_first.cross(impulse));
      if let Some(second) = self.second.as_deref_mut() {
         second.linear_velocity += impulse * second.mass.inv_mass;
         second.angular_velocity += second.apply_inv_inertia(arm_second.cross(impulse));
      }
   }

   /// Apply one impulse to the pair's pseudo-velocities, at the given anchors.
   /// 向该对在给定锚点施加一个作用于伪速度的冲量。
   fn apply_pseudo_impulse(&mut self, arm_first: Vec3, arm_second: Vec3, impulse: Vec3) {
      self.first.pseudo_linear_velocity -= impulse * self.first.mass.inv_mass;
      self.first.pseudo_angular_velocity -= self.first.apply_inv_inertia(arm_first.cross(impulse));
      if let Some(second) = self.second.as_deref_mut() {
         second.pseudo_linear_velocity += impulse * second.mass.inv_mass;
         second.pseudo_angular_velocity += second.apply_inv_inertia(arm_second.cross(impulse));
      }
   }
}

/// The relative velocity along `axis`, for a borrow that cannot write.
///
/// A missing second Body is Static geometry: it never moves, so its share of the
/// relative velocity is zero.
/// 沿 `axis` 测量的相对速度，用于无法写入的借用。缺失的第二个 Body 表示 Static
/// geometry：它永不移动，因此其对相对速度的贡献为零。
fn relative_velocity(
   first: &Body,
   second: Option<&Body>,
   arm_first: Vec3,
   arm_second: Vec3,
   axis: Vec3,
) -> f32 {
   let first_share = first.point_velocity(arm_first).dot(axis);
   let second_share = second.map_or(0.0, |body| body.point_velocity(arm_second).dot(axis));
   second_share - first_share
}

/// The effective mass of one contact along `axis`, before taking its reciprocal.
/// 单个接触沿 `axis` 的有效质量（取倒数之前）。
fn normal_effective_mass(
   first: &Body,
   second: Option<&Body>,
   arm_first: Vec3,
   arm_second: Vec3,
   axis: Vec3,
   inverse_mass_sum: f32,
) -> f32 {
   let first_arm_cross = arm_first.cross(axis);
   let second_term = second.map_or(0.0, |body| {
      let second_arm_cross = arm_second.cross(axis);
      second_arm_cross.dot(body.apply_inv_inertia(second_arm_cross))
   });
   inverse_mass_sum + first_arm_cross.dot(first.apply_inv_inertia(first_arm_cross)) + second_term
}

/// `1.0 / value`, or zero when the value cannot produce a usable impulse.
/// `1.0 / value`；当该值无法产生可用冲量时返回零。
fn reciprocal_or_zero(value: f32) -> f32 {
   if value.is_finite() && value > INFINITE_EFFECTIVE_MASS {
      1.0 / value
   } else {
      0.0
   }
}

/// The geometric mean of two friction coefficients.
///
/// A geometric mean keeps a frictionless surface frictionless no matter what it
/// touches, which an arithmetic mean does not.
/// 两个摩擦系数的几何平均。几何平均使无摩擦表面无论接触什么都保持无摩擦，而算术平均
/// 做不到这一点。
pub(crate) fn combine_friction(first: f32, second: f32) -> f32 {
   sqrt(first.max(0.0) * second.max(0.0))
}

#[cfg(test)]
mod tests {
   use super::{combine_friction, is_solvable, reciprocal_or_zero};
   use crate::body::{Body, BodyDesc};
   use crate::shape::Shape;

   fn body(motion: fn(Shape) -> BodyDesc) -> Body {
      Body::from_desc(&motion(Shape::sphere(0.5)))
   }

   #[test]
   fn two_immovable_bodies_are_not_solvable() {
      let stationary = body(BodyDesc::stationary);
      let kinematic = body(BodyDesc::kinematic);
      let dynamic = Body::from_desc(&BodyDesc::dynamic(Shape::sphere(0.5), 1.0));
      assert!(!is_solvable(&stationary, &kinematic));
      assert!(is_solvable(&stationary, &dynamic));
      assert!(is_solvable(&dynamic, &kinematic));
   }

   #[test]
   fn friction_combines_through_a_geometric_mean() {
      assert!((combine_friction(0.0, 0.9) - 0.0).abs() < 1e-6);
      assert!((combine_friction(0.25, 1.0) - 0.5).abs() < 1e-6);
      assert!((combine_friction(-1.0, 1.0) - 0.0).abs() < 1e-6);
   }

   #[test]
   fn a_degenerate_effective_mass_yields_no_impulse() {
      assert!(reciprocal_or_zero(0.0).abs() < 1e-9);
      assert!(reciprocal_or_zero(-1.0).abs() < 1e-9);
      assert!(reciprocal_or_zero(f32::NAN).abs() < 1e-9);
      assert!((reciprocal_or_zero(2.0) - 0.5).abs() < 1e-6);
   }
}

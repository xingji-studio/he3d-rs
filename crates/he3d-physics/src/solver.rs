use he3d_math::Vec3;
use he3d_math::scalar::sqrt;

use crate::body::Body;
use crate::contact::{ConstraintPoint, ContactConstraint};
use crate::storage::{Slot, constraint_bodies, constraint_bodies_ref};

/// Index of the contact normal among a point's three solve axes.
/// 接触法线在某个点的三条求解轴中的下标。
const NORMAL_AXIS: usize = 0;
/// Index of the first friction tangent among a point's three solve axes.
/// 第一条摩擦切线在某个点的三条求解轴中的下标。
const TANGENT_AXIS_0: usize = 1;
/// Index of the second friction tangent among a point's three solve axes.
/// 第二条摩擦切线在某个点的三条求解轴中的下标。
const TANGENT_AXIS_1: usize = 2;

/// Velocity iterations per step.
///
/// Tall stacks need more than short ones: with plain sequential impulses the constraint graph converges by Gauss-Seidel, so the count must scale with the height of what is stacked.
/// See the tall-stack limitation in `docs/physics-spec.md`.
/// 每步的速度迭代次数。
/// 高塔比矮堆需要更多：纯序列冲量下约束图靠 Gauss-Seidel 收敛，因此迭代次数必须随堆叠高度增长。
/// 见 `docs/physics-spec.md` 中的高塔限制。
pub(crate) const DEFAULT_VELOCITY_ITERATIONS: u32 = 12;

/// Fraction of the remaining penetration removed per step.
/// 每步消除的剩余穿透比例。
///
/// A relaxation factor, not an exact correction.
/// Removing all of the error in one step would overshoot and oscillate, because the contact's own impulse in the same step has already moved the geometry.
/// A fifth per step clears 96% of a slop-sized overlap in about fifteen steps while staying far below the overshoot bound of 1.
/// 松弛因子而非精确修正。
/// 一步消除全部误差会过冲并振荡，因为同一步中接触自身的冲量已经改变了几何。
/// 每步五分之一约十五步清除 slop 量级重叠的 96%，同时远低于 1 这个过冲边界。
const BAUMGARTE: f32 = 0.2;

/// Penetration allowed to persist, so resting contacts stop pushing.
/// 允许保留的穿透量，使静止接触不再持续推挤。
///
/// A resting Body sinks by `g * dt^2` before its contact answers — about 0.7 mm at 1/120 s and one gravity — so the slop has to be comfortably larger than that or the contact would alternate between pushing and not pushing.
/// It is also small enough to be invisible on a Body of ordinary size.
/// 静止的 Body 在其接触作出回应前会下沉 `g * dt^2`——在 1/120 秒、一个重力下约 0.7 毫米——因此 slop 必须明显大于它，否则接触会在"推"与"不推"之间反复切换。
/// 同时它也小到在常规尺寸的 Body 上不可见。
const PENETRATION_SLOP: f32 = 0.005;

/// Approach speed below which restitution is suppressed.
///
/// Without this a resting Body bounces forever on the tiny velocity gravity adds each step, which is the classic source of stack jitter.
///
/// Gravity adds `g * dt` — about 0.08 m/s at 1/120 s — to a resting Body's approach speed every step, so the threshold only has to sit well above that.
/// A whole metre per second is a deliberate order-of-magnitude margin: anything slower is a settle rather than an impact, and the cost is that a 0.9 m/s ball does not bounce.
/// 低于此接近速度时抑制弹性。
/// 若没有它，静止的 Body 会在重力每步加入的微小速度上永远弹跳，这正是堆叠抖动的经典来源。
/// 重力每步给静止 Body 的接近速度加上 `g * dt`——在 1/120 秒下约 0.08 m/s——因此阈值只需明显高于它。
/// 整整 1 米/秒是刻意留出的一个数量级余量：更慢的都算沉降而非撞击，代价是 0.9 m/s 的球不会回弹。
const RESTITUTION_THRESHOLD: f32 = 1.0;

/// Iterations of the positional-correction pass.
/// 位置修正阶段的迭代次数。
///
/// Position correction relaxes over the same contact graph as the velocity solve, but it only has to push overlap out of the manifold rather than converge a force network, so it needs far fewer passes.
/// Three is enough for a resting stack's contacts to agree with each other without paying for a second full solve.
/// 位置修正是在与速度求解相同的接触图上做松弛，但它只需把重叠推出流形，而不必收敛一个力网络，因此需要的遍数少得多。
/// 三遍足以让静止堆叠的各接触互相协调，又不必再付一次完整求解的代价。
const POSITION_ITERATIONS: u32 = 3;

/// An effective mass below this is treated as infinite and receives no impulse.
/// 有效质量低于此值时视为无穷大，不接收任何冲量。
///
/// It is compared against a sum of inverse masses and inverse inertia terms, so its unit is `1/kg`: this rejects anything heavier than about 10^8 kg, which no impulse one frame can produce will move.
/// 它与逆质量、逆惯量项之和比较，因此单位是 `1/kg`：它拒绝一切重于约 10^8 千克的东西，而一帧所能产生的任何冲量都推不动那种东西。
const INFINITE_EFFECTIVE_MASS: f32 = 1e-8;

/// A carried-over impulse whose squared magnitude is below this is not re-applied.
/// 沿用冲量的模平方低于此值时不再重新施加。
///
/// The comparison is against the squared magnitude, so the real cutoff is `1e-4 N*s`.
/// That changes a 1 kg Body's velocity by 1e-4 m/s — five hundred times below the sleep threshold — so dropping it cannot affect anything a caller can observe.
/// 比较对象是模的平方，因此真正的阈值是 `1e-4 N*s`。
/// 它只能改变 1 kg Body 的速度 1e-4 m/s——比睡眠阈值低五百倍——因此丢弃它不会影响调用者能观察到的任何东西。
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
/// Two immovable Bodies constrain nothing; skipping them keeps static scenery from consuming iterations.
/// 两个不可移动的 Body 之间没有任何约束；跳过它们可避免静态景物消耗迭代。
pub(crate) fn is_solvable(first: &Body, second: &Body) -> bool {
   first.motion.is_pushable() || second.motion.is_pushable()
}

/// Compute effective masses and velocity biases for one step.
///
/// The restitution bias is taken from the approach velocity *before* any impulse is applied, which is why this runs before warm starting.
/// 为一步计算有效质量与速度偏置。
/// 弹性偏置取自施加任何冲量**之前**的接近速度，因此这一步必须在 warm starting 之前运行。
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

         // Effective mass is the relative velocity's response to a unit impulse, and it is needed along all three axes: the normal and then the two tangents.
         // The angular half of that response is what the solver later applies its impulses with, so it is kept here instead of being recomputed in every velocity iteration.
         // 有效质量是相对速度对单位冲量的响应，三条轴上都要求它：法线与两条切线。
         // 该响应的角速度部分正是求解器稍后施加冲量所用的，因此在此保留，而不是每次速度迭代重算。
         let mut reciprocal_masses = [0.0; 3];
         for (index, axis) in [normal, tangents[0], tangents[1]].into_iter().enumerate() {
            let (term, response) = effective_mass_and_response(
               first,
               second,
               arm_first,
               arm_second,
               axis,
               inverse_mass_sum,
            );
            point.angular_response[0][index] = response[0];
            point.angular_response[1][index] = response[1];
            reciprocal_masses[index] = reciprocal_or_zero(term);
         }
         point.normal_mass = reciprocal_masses[0];
         point.tangent_mass = [reciprocal_masses[1], reciprocal_masses[2]];

         let approach = relative_velocity(first, second, arm_first, arm_second, normal);
         // The velocity solve only ever removes energy, so a resting stack ends a step with exactly zero separating velocity and can fall asleep.
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
/// This is what lets a resting stack converge in a handful of iterations instead of rebuilding its contact forces from zero every step.
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
         // Applying the three axes one at a time adds up to that same impulse, and the point already holds each axis's angular response.
         // 三条轴逐条施加，合起来就是同一个冲量；每个轴的角速度响应已存在点上。
         pair.apply_impulse(point, NORMAL_AXIS, normal, point.normal_impulse);
         pair.apply_impulse(point, TANGENT_AXIS_0, tangents[0], point.tangent_impulse[0]);
         pair.apply_impulse(point, TANGENT_AXIS_1, tangents[1], point.tangent_impulse[1]);
      }
   }
}

/// Drive every contact toward a non-negative separating velocity.
///
/// Friction is solved along two tangents inside the Coulomb cone `|jt| <= mu * jn`.
/// 将每个接触推向非负的分离速度。
/// 摩擦沿两条切线在库仑锥 `|jt| <= mu * jn` 内求解。
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
   // Resolve the pair once for the whole constraint, not once per point and per tangent: the Bodies a constraint names cannot change while it is being solved, and re-deriving them is the inner loop's largest fixed cost.
   // 整条约束只解析一次 Body 对，而不是每个点、每条切线各解析一次：求解期间约束所指的 Body 不会改变，而重复解析正是内循环最大的一笔固定开销。
   let Some(mut pair) = ContactPair::resolve(slots, constraint.a as usize, constraint.b as usize)
   else {
      return;
   };

   // Two points are solved together, because solving them in sequence is what puts a couple on the Body.
   // Three and four points cannot be: their block is singular.
   // 两个点一起求解，因为逐个求解正是把力偶加到 Body 上的原因。
   // 三个与四个点做不到这一点：它们的块是奇异的。
   if constraint.point_count == TWO_POINT_MANIFOLD {
      // The block goes last so that its answer is the last word of the iteration.
      // A friction impulse changes the normal velocities, and only the block can put those back; leaving friction for afterwards drags the two normals apart again, which is the very couple the block exists to remove.
      // 块放在最后，使它的答案就是本次迭代的最终答案。
      // 摩擦冲量会改变法向速度，而只有块能把它们放回去；把摩擦留到之后又会把两个法向冲量拉开，而那正是块存在的理由。
      solve_friction(&mut pair, constraint.points_mut(), tangents, friction);
      solve_normal_block(&mut pair, constraint, normal);
   } else {
      // A face manifold keeps the original order — each point's normal and then its friction, point by point — because a block cannot help it and the order a solver sweeps in is what its tall-stack behaviour was measured with.
      // 面流形保持原来的次序——逐点先法线后摩擦——因为块对它无能为力，而求解器的扫描次序正是当初测量高塔行为时所用的那个。
      for point in constraint.points_mut() {
         solve_normal(&mut pair, point, normal);
         solve_friction_point(&mut pair, point, tangents, friction);
      }
   }
}

/// Drive every point's friction impulse to the edge of the Coulomb cone.
/// 将每个点的摩擦冲量推到库仑锥的边缘。
///
/// Friction is solved one point and one tangent at a time rather than as a block: the cone limit couples each tangent to the normal impulse at its own point, so the two points do not share an answer the way the normals of a two-point manifold do.
/// 摩擦逐点、逐切线求解，而不作为一个块：锥限把每条切线与它自己那个点的法向冲量耦合起来，因此两个点并不像两点流形的法向那样共享一个答案。
fn solve_friction(
   pair: &mut ContactPair,
   points: &mut [ConstraintPoint],
   tangents: [Vec3; 2],
   friction: f32,
) {
   for point in points {
      solve_friction_point(pair, point, tangents, friction);
   }
}

/// Drive one point's friction impulses to the edge of the Coulomb cone.
/// 将单个点的摩擦冲量推到库仑锥的边缘。
fn solve_friction_point(
   pair: &mut ContactPair,
   point: &mut ConstraintPoint,
   tangents: [Vec3; 2],
   friction: f32,
) {
   for (index, tangent) in tangents.into_iter().enumerate() {
      let relative = pair.relative_velocity(point.anchor_a, point.anchor_b, tangent);
      let lambda = -point.tangent_mass[index] * relative;
      let limit = friction * point.normal_impulse;
      let previous = point.tangent_impulse[index];
      point.tangent_impulse[index] = (previous + lambda).clamp(-limit, limit);
      let delta = point.tangent_impulse[index] - previous;
      pair.apply_impulse(point, TANGENT_AXIS_0 + index, tangent, delta);
   }
}

/// The point count whose normal constraints are solved as one block.
/// 其法向约束作为一个块求解的点数。
const TWO_POINT_MANIFOLD: u8 = 2;

/// How much of the block's diagonal product the determinant has to keep to be used.
/// 块的行列式必须保住的、占对角线乘积的比例。
///
/// The ratio is `1 - correlation²`, so this turns down blocks whose two points nearly cannot be told apart, where an impulse anywhere moves both the same way.
/// There is no unique answer to hand back there; one point at a time is what such a pair would have got anyway.
/// 该比例等于 `1 - 相关系数²`，因此它会拒绝两点几乎无法区分的块——那种情形下任何一处的冲量都会把两点同样地推动，没有唯一答案可回填；而逐个求解本来也是这种对子唯一能得到的东西。
const BLOCK_SINGULAR_RATIO: f32 = 1e-6;

/// Solve one point's normal constraint on its own.
/// 单独求解一个点的法向约束。
fn solve_normal(pair: &mut ContactPair, point: &mut ConstraintPoint, normal: Vec3) {
   let relative = pair.relative_velocity(point.anchor_a, point.anchor_b, normal);
   let lambda = -point.normal_mass * (relative - point.bias);
   let previous = point.normal_impulse;
   // A contact can push but never pull.
   // 接触只能推，永远不能拉。
   point.normal_impulse = (previous + lambda).max(0.0);
   let delta = point.normal_impulse - previous;
   pair.apply_impulse(point, NORMAL_AXIS, normal, delta);
}

/// Solve each point's normal constraint on its own.
/// 逐个求解各点的法向约束。
fn solve_normals_individually(
   pair: &mut ContactPair,
   points: &mut [ConstraintPoint],
   normal: Vec3,
) {
   for point in points {
      solve_normal(pair, point, normal);
   }
}

/// Solve a two-point manifold's normal constraints as one block.
/// 把一个两点流形的法向约束作为一个块求解。
///
/// Solving the two points in sequence leaves a couple on the Body.
/// The first point's impulse changes the velocity the second one measures, so the pair settles on unequal impulses, and their difference is a torque about the axis the two points share.
/// A cube rolling over an edge is exactly two points, and that couple is rotation about the roll axis that nothing applied.
/// 逐个求解两个点会在 Body 上留下一个力偶。
/// 第一个点的冲量改变了第二个点测到的速度，于是两点最终分到的冲量不等，而它们的差就是绕两点共享那条轴的力矩。
/// 方块滚过一条棱时恰好是两点，而该力偶就是无人施加的绕滚动轴的旋转。
///
/// The block needs both points to be movable: an impulse can only be shared out between points that can receive one.
/// 块要求两个点都能移动：冲量只能在能接收它的点之间分配。
fn solve_normal_block(pair: &mut ContactPair, constraint: &mut ContactConstraint, normal: Vec3) {
   let first = constraint.points[0];
   let second = constraint.points[1];
   let inverse_mass_sum = pair.inverse_mass_sum();
   // The cross-talk term: what an impulse at one point does to the other's relative velocity along the shared normal.
   // It is the same in both directions, which is why the block is symmetric.
   // 串扰项：一个点上的冲量对另一个点沿公共法线的相对速度做了什么。
   // 两个方向相同，因此块是对称的。
   let block = ContactBlock {
      first: reciprocal_or_zero(first.normal_mass),
      coupling: inverse_mass_sum
         + second.angular_response[0][NORMAL_AXIS].dot(first.anchor_a.cross(normal))
         + second.angular_response[1][NORMAL_AXIS].dot(first.anchor_b.cross(normal)),
      second: reciprocal_or_zero(second.normal_mass),
   };
   let velocity = (
      pair.relative_velocity(first.anchor_a, first.anchor_b, normal) - first.bias,
      pair.relative_velocity(second.anchor_a, second.anchor_b, normal) - second.bias,
   );
   let previous = (first.normal_impulse, second.normal_impulse);
   let Some((next_first, next_second)) = block.solve(previous, velocity) else {
      solve_normals_individually(pair, &mut constraint.points[..2], normal);
      return;
   };
   constraint.points[0].normal_impulse = next_first;
   constraint.points[1].normal_impulse = next_second;
   pair.apply_impulse(&first, NORMAL_AXIS, normal, next_first - previous.0);
   pair.apply_impulse(&second, NORMAL_AXIS, normal, next_second - previous.1);
}

/// The 2×2 effective mass of a two-point manifold's normal constraints.
/// 两点流形法向约束的 2×2 有效质量。
///
/// The two points of a manifold are not independent: an impulse at one moves the other's relative velocity, and `coupling` is that cross-talk.
/// Solving the diagonal alone is what leaves the pair with unequal impulses — and unequal impulses at two points apart along an axis are a torque about it.
/// 一个流形的两点并不独立：一个点上的冲量会改变另一个点的相对速度，而 `coupling` 正是这种串扰。
/// 只解对角项，正是使两点冲量不等的原因——而沿某条轴分开的两点冲量不等，就是绕那条轴的力矩。
#[derive(Clone, Copy, Debug)]
struct ContactBlock {
   /// `K[0][0]`.
   first: f32,
   /// `K[0][1]`, equal to `K[1][0]`.
   coupling: f32,
   /// `K[1][1]`.
   second: f32,
}

impl ContactBlock {
   /// The impulse pair the block settles on, or `None` when it cannot be inverted.
   ///
   /// `previous` is the pair already applied and `velocity` the relative velocities it left behind; the result is the pair the two points should have accumulated.
   /// It is clamped the way a contact is: a point may push but never pull, so a point the block would have to pull leaves the block, and the other is re-solved against it at zero.
   /// 返回该块最终落定的冲量对；矩阵不可逆时返回 `None`。
   /// `previous` 是已施加的冲量对，`velocity` 是它留下的相对速度；结果是两点应当累计到的冲量。
   /// 结果按接触的规则截断：点只能推、永不拉，因此块需要某个点去拉时，该点离开块，另一个点对着零重新求解。
   fn solve(self, previous: (f32, f32), velocity: (f32, f32)) -> Option<(f32, f32)> {
      let determinant = self.first * self.second - self.coupling * self.coupling;
      let threshold = self.first * self.second * BLOCK_SINGULAR_RATIO;
      // `<=` is false for a NaN, so an unusable determinant is turned down by name rather than by the comparison.
      // `<=` 对 NaN 为假，因此无法使用的行列式要显式拒绝，而不能靠比较。
      if determinant <= threshold || determinant.is_nan() {
         return None;
      }
      let (previous_first, previous_second) = previous;
      let (velocity_first, velocity_second) = velocity;
      // The impulse that would hold both points exactly at their bias, before the non-negativity clamp: `x = lambda - K⁻¹ v`.
      // 在施加非负截断之前，能把两点都恰好保持在各自偏置上的冲量：`x = lambda - K⁻¹ v`。
      let inverse = 1.0 / determinant;
      let mut next_first = previous_first
         - inverse * (self.second * velocity_first - self.coupling * velocity_second);
      let mut next_second = previous_second
         - inverse * (self.first * velocity_second - self.coupling * velocity_first);
      if next_first < 0.0 && next_second < 0.0 {
         next_first = 0.0;
         next_second = 0.0;
      } else if next_first < 0.0 {
         // Held at zero, the first point stops contributing, so the second one is solved against the first's remaining impulse.
         // 第一个点被压在零上、不再出力，因此第二个点对着第一个点残留的冲量求解。
         next_first = 0.0;
         next_second = (previous_second
            + (self.coupling * previous_first - velocity_second) / self.second)
            .max(0.0);
      } else if next_second < 0.0 {
         next_second = 0.0;
         next_first = (previous_first
            + (self.coupling * previous_second - velocity_first) / self.first)
            .max(0.0);
      }
      Some((next_first, next_second))
   }
}

/// Push remaining overlap apart using pseudo-velocities only.
///
/// Running positional correction against a separate velocity channel is what keeps it from turning into kinetic energy.
/// That is the difference between a stack that settles and sleeps and one that buzzes against its own contacts forever.
/// 仅使用伪速度推开剩余重叠。
/// 让位置修正作用于独立的通道，是它不会转化为动能的原因，也是"能稳定并入睡的堆叠"与"永远在自己接触上嗡嗡响的堆叠"之间的差别。
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
            pair.apply_pseudo_impulse(point, normal, delta);
         }
      }
   }
}

/// The two Bodies a constraint acts on, resolved once and held for the whole solve.
/// 一条约束所作用的两个 Body，解析一次并在整个求解过程中持有。
///
/// Resolving is not a detail the loops should repeat: the pair behind a constraint is fixed for as long as the constraint is solved, and every point and every iteration must push on exactly that pair.
/// Folding the lookup, the two relative-velocity readings, and the two impulse writers behind one type is what makes "resolve once" something the code cannot get wrong — the hot loop has no way to reach a Body except through a resolved pair.
/// 解析不该由各个循环反复进行：一条约束背后的 Body 对在其求解期间是固定的，每个点、每次迭代都必须正好作用于该对。
/// 把查找、两个相对速度读数与两个冲量写入收进一个类型，使"只解析一次"成为代码无法做错的事——热循环除了经由已解析的对，没有别的途径触及 Body。
///
/// A missing second Body is Static geometry: it never moves, so it contributes nothing to a relative velocity and receives nothing from an impulse.
/// 缺失的第二个 Body 表示 Static geometry：它永不移动，因此对相对速度没有贡献，也不从冲量中接收任何东西。
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

   /// The two Bodies' inverse masses added up, the pair's share of a contact block.
   ///
   /// The term is the same at every point of a constraint, because it belongs to the pair rather than to a point, so the block solver reads it here instead of carrying a copy on each point.
   /// 两个 Body 的逆质量之和，即该对在一个接触块中的那一份。
   /// 该项对同一条约束的每个点都相同，因为它属于这对 Body 而不属于某个点，因此块求解器在这里读取它，而不必在每个点上存一份。
   fn inverse_mass_sum(&self) -> f32 {
      self.first.mass.inv_mass
         + self
            .second
            .as_deref()
            .map_or(0.0, |body| body.mass.inv_mass)
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

   /// Apply an impulse of `delta` along `axis` at a contact point.
   ///
   /// The impulse pushes `second` along its own direction and `first` against it.
   /// A missing `second` is Static geometry and simply receives nothing.
   /// `axis` is an index into the point's solve axes, and `direction` is the axis itself; the point already holds the angular response for it.
   /// 在接触点沿 `axis` 施加大小为 `delta` 的冲量。
   /// 冲量沿自身方向推 `second`，并反向推 `first`。
   /// 缺失的 `second` 表示 Static geometry，它不接收任何东西。
   /// `axis` 是该点求解轴的下标，`direction` 是对应的轴向量；该轴的角速度响应已存在点上。
   fn apply_impulse(&mut self, point: &ConstraintPoint, axis: usize, direction: Vec3, delta: f32) {
      let impulse = direction * delta;
      self.first.linear_velocity -= impulse * self.first.mass.inv_mass;
      self.first.angular_velocity -= point.angular_response[0][axis] * delta;
      if let Some(second) = self.second.as_deref_mut() {
         second.linear_velocity += impulse * second.mass.inv_mass;
         second.angular_velocity += point.angular_response[1][axis] * delta;
      }
   }

   /// Apply an impulse of `delta` along the contact normal to the pair's pseudo-velocities, at a contact point.
   /// 沿接触法线向该对在接触点施加大小为 `delta` 的冲量，作用于伪速度。
   fn apply_pseudo_impulse(&mut self, point: &ConstraintPoint, normal: Vec3, delta: f32) {
      let impulse = normal * delta;
      self.first.pseudo_linear_velocity -= impulse * self.first.mass.inv_mass;
      self.first.pseudo_angular_velocity -= point.angular_response[0][NORMAL_AXIS] * delta;
      if let Some(second) = self.second.as_deref_mut() {
         second.pseudo_linear_velocity += impulse * second.mass.inv_mass;
         second.pseudo_angular_velocity += point.angular_response[1][NORMAL_AXIS] * delta;
      }
   }
}

/// The relative velocity along `axis`, for a borrow that cannot write.
///
/// A missing second Body is Static geometry: it never moves, so its share of the relative velocity is zero.
/// 沿 `axis` 测量的相对速度，用于无法写入的借用。
/// 缺失的第二个 Body 表示 Static geometry：它永不移动，因此其对相对速度的贡献为零。
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

/// The effective mass of one contact along `axis`, and the angular response the solver needs in order to apply an impulse along that same axis.
///
/// The two are returned together because both need `inv_inertia(anchor × axis)`: the mass is that vector's dot product with `anchor × axis`, and the response *is* it.
/// 返回接触沿 `axis` 的有效质量，以及求解器沿同一轴施加冲量所需的角速度响应。
/// 两者一起返回是因为都需要 `inv_inertia(anchor × axis)`：质量是该向量与 `anchor × axis` 的点积，而响应本身就是它。
fn effective_mass_and_response(
   first: &Body,
   second: Option<&Body>,
   arm_first: Vec3,
   arm_second: Vec3,
   axis: Vec3,
   inverse_mass_sum: f32,
) -> (f32, [Vec3; 2]) {
   let cross_first = arm_first.cross(axis);
   let response_first = first.apply_inv_inertia(cross_first);
   let mut response = [response_first, Vec3::ZERO];
   let mut second_term = 0.0;
   if let Some(body) = second {
      let cross_second = arm_second.cross(axis);
      response[1] = body.apply_inv_inertia(cross_second);
      second_term = cross_second.dot(response[1]);
   }
   (
      inverse_mass_sum + cross_first.dot(response_first) + second_term,
      response,
   )
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
/// A geometric mean keeps a frictionless surface frictionless no matter what it touches, which an arithmetic mean does not.
/// 两个摩擦系数的几何平均。
/// 几何平均使无摩擦表面无论接触什么都保持无摩擦，而算术平均做不到这一点。
pub(crate) fn combine_friction(first: f32, second: f32) -> f32 {
   sqrt(first.max(0.0) * second.max(0.0))
}

#[cfg(test)]
mod tests {
   use super::{ContactBlock, combine_friction, is_solvable, reciprocal_or_zero};
   use crate::body::{Body, BodyDesc};
   use crate::shape::Shape;

   /// The two relative velocities a block leaves behind after an impulse pair.
   ///
   /// This is `v + K * impulse`, the same arithmetic the solver performs on the Bodies, so a test can ask whether the block actually settled the pair.
   /// 施加一对冲量后该块留下的两个相对速度。
   /// 这就是 `v + K * impulse`，与求解器作用在 Body 上的算法相同，因此测试可以问该块是否真的把这一对安顿好了。
   fn after(block: ContactBlock, impulse: (f32, f32), velocity: (f32, f32)) -> (f32, f32) {
      (
         velocity.0 + block.first * impulse.0 + block.coupling * impulse.1,
         velocity.1 + block.coupling * impulse.0 + block.second * impulse.1,
      )
   }

   /// The block of a box resting on the edge it pivots over: unit inverse mass and `1/12` of inertia, two points apart along the edge.
   /// 压在它所绕的那条棱上的方块的块：逆质量为 1、惯量为 `1/12`，两点沿棱分开。
   fn edge_block() -> ContactBlock {
      ContactBlock {
         first: 2.5,
         coupling: -0.5,
         second: 2.5,
      }
   }

   #[test]
   fn a_block_leaves_both_points_either_touching_or_separating() {
      let block = edge_block();
      for velocity in [(-1.0, -0.3), (-0.2, -1.0), (-1.0, -1.0), (-5.0, 0.0)] {
         let impulse = block
            .solve((0.0, 0.0), velocity)
            .expect("a two-point edge block is invertible");
         assert!(impulse.0 >= 0.0 && impulse.1 >= 0.0, "impulse {impulse:?}");
         let settled = after(block, impulse, velocity);
         assert!(
            settled.0.abs() < 1e-5 && settled.1.abs() < 1e-5,
            "velocity {velocity:?} settled to {settled:?}"
         );
      }
   }

   #[test]
   fn points_that_are_already_separating_receive_nothing() {
      let block = edge_block();
      let impulse = block
         .solve((0.0, 0.0), (1.0, 0.5))
         .expect("a two-point edge block is invertible");
      assert!(
         impulse.0.abs() < 1e-9 && impulse.1.abs() < 1e-9,
         "got {impulse:?}"
      );
   }

   #[test]
   fn a_point_the_block_would_pull_leaves_it_and_the_other_settles_alone() {
      // The block wants a negative impulse at the first point — it would have to pull — so only the second one pushes, and it settles on its own.
      // 块在第一个点上想要一个负冲量——那意味着它得去拉——因此只有第二个点出推的力，它独自安顿下来。
      let block = edge_block();
      let velocity = (1.0, -1.0);
      let impulse = block
         .solve((0.0, 0.0), velocity)
         .expect("a two-point edge block is invertible");
      assert!(impulse.0.abs() < 1e-9, "the pulled point must take none");
      assert!((impulse.1 - 0.4).abs() < 1e-5, "got {impulse:?}");
      let settled = after(block, impulse, velocity);
      assert!(
         settled.0 >= 0.0,
         "a point taking no impulse must not be pulled"
      );
      assert!(settled.1.abs() < 1e-5, "got {settled:?}");
   }

   #[test]
   fn a_settled_block_asks_for_no_more_impulse() {
      // Warm starting hands the block the impulse it settled on last step, and the velocities that impulse left behind are zero — so it must ask for nothing.
      // A block that re-requested its old impulse would pump energy into a resting contact every iteration.
      // Warm starting 会把上一步落定的冲量交给块，而该冲量留下的速度是零——因此它必须什么也不再要。
      // 若块反复索要旧冲量，就会每次迭代都往静止接触里灌能量。
      let block = edge_block();
      let impulse = block
         .solve((0.0, 0.0), (-1.0, -0.3))
         .expect("a two-point edge block is invertible");
      let settled = after(block, impulse, (-1.0, -0.3));
      let again = block
         .solve(impulse, settled)
         .expect("a two-point edge block is invertible");
      assert!(
         (again.0 - impulse.0).abs() < 1e-5 && (again.1 - impulse.1).abs() < 1e-5,
         "{impulse:?} then {again:?}"
      );
   }

   #[test]
   fn a_block_whose_points_cannot_be_told_apart_is_turned_down() {
      // Four coplanar points are famously singular; two points that an impulse moves the same way are the same problem, and falling back to one at a time is what such a pair can have.
      // 四个共面点是出了名的奇异；两个会被同一个冲量以同样方式推动的点是同一个问题，而回退为逐个求解正是这样的对子能得到的东西。
      let degenerate = ContactBlock {
         first: 2.5,
         coupling: 2.5,
         second: 2.5,
      };
      assert!(degenerate.solve((0.0, 0.0), (-1.0, 0.0)).is_none());
      let immovable = ContactBlock {
         first: 0.0,
         coupling: 0.0,
         second: 0.0,
      };
      assert!(immovable.solve((0.0, 0.0), (-1.0, 0.0)).is_none());
   }

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

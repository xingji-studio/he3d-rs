use he3d_math::{Quat, Vec3};

use crate::aabb::Aabb;
use crate::shape::{MassProperties, Shape};

/// How a Body's motion is produced.
/// Body 的运动是如何产生的。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum MotionType {
   /// Driven by gravity, forces, impulses, and Contacts. It has finite mass, and
   /// anything may move it.
   /// 由重力、力、冲量与 Contact 驱动。具有有限质量，任何东西都可以移动它。
   #[default]
   Dynamic,
   /// Never moves, and no Contact can move it.
   /// 永不移动，且没有任何 Contact 能移动它。
   Static,
   /// Driven by a velocity the app sets, and no Contact can change it. It pushes
   /// Dynamic Bodies as though infinitely massive.
   /// 由应用设定的速度驱动，没有任何 Contact 能改变它。它以无穷大的质量推动
   /// Dynamic。
   Kinematic,
}

impl MotionType {
   /// Whether gravity, forces, and impulses accelerate a Body in this mode.
   /// 该模式下重力、力与冲量是否会加速 Body。
   #[must_use]
   pub fn is_simulated(self) -> bool {
      matches!(self, Self::Dynamic)
   }

   /// Whether the Body is carried by its own velocity each step.
   /// 该 Body 是否每步被自身速度带动。
   #[must_use]
   pub fn moves(self) -> bool {
      !matches!(self, Self::Static)
   }

   /// Whether a Contact can change this Body's velocity.
   /// Contact 是否能改变该 Body 的速度。
   #[must_use]
   pub fn is_pushable(self) -> bool {
      matches!(self, Self::Dynamic)
   }
}

/// A stable generational handle to a Body within a [`crate::World`].
/// 指向 [`crate::World`] 中某个 Body 的稳定代际句柄。
///
/// It becomes permanently invalid once that Body is removed: the slot's
/// generation advances, so a stale handle is rejected rather than silently
/// addressing whatever Body reuses the slot.
/// 该 Body 被移除后此句柄永久失效：槽位的代际会递增，因此过期句柄会被拒绝，而不会静默
/// 地指向复用该槽位的另一个 Body。
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct BodyId {
   index: u32,
   generation: u32,
}

impl BodyId {
   /// Mint a handle. Only a World may do this.
   /// 铸造句柄。只有 World 可以这样做。
   pub(crate) const fn new(index: u32, generation: u32) -> Self {
      Self { index, generation }
   }

   /// The slot this handle addresses within its World.
   /// 此句柄在其 World 中寻址的槽位。
   #[must_use]
   pub const fn index(self) -> u32 {
      self.index
   }

   /// The slot generation this handle was minted against.
   /// 铸造此句柄时所对应的槽位代际。
   #[must_use]
   pub const fn generation(self) -> u32 {
      self.generation
   }
}

/// Default friction coefficient for a new Body.
/// 新建 Body 的默认摩擦系数。
///
/// The middle of the usable dry-sliding range, so a Body built without
/// `with_friction` neither slides like ice nor refuses to be pushed. A scene that
/// cares about the value should set it.
/// 干滑动可用区间的中值，使未调用 `with_friction` 的 Body 既不会像冰面一样滑动，也不会
/// 推不动。真正在意该值的场景应当显式设置。
pub const DEFAULT_FRICTION: f32 = 0.5;

/// The sleep-group tag a Body carries when it belongs to no sleeping Island.
/// Body 不属于任何睡眠 Island 时所携带的睡眠组标记。
pub(crate) const NO_SLEEP_GROUP: u32 = u32::MAX;

/// Construction parameters for a Body.
/// Body 的构造参数。
///
/// Mass is the input and the inertia tensor is derived from the Shape, so the two
/// can never disagree. See `docs/physics-spec.md` §3.
/// 输入的是质量，惯量张量由 Shape 推导，因此二者永不可能不一致。见
/// `docs/physics-spec.md` §3。
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct BodyDesc {
   /// The Body's collision shape.
   /// Body 的碰撞形状。
   pub shape: Shape,
   /// How the Body's motion is produced.
   /// Body 的运动如何产生。
   pub motion: MotionType,
   /// Mass in kilograms. Ignored when `motion` is not [`MotionType::Dynamic`].
   /// 质量（千克）。当 `motion` 不是 [`MotionType::Dynamic`] 时忽略。
   pub mass: f32,
   /// Initial position.
   /// 初始位置。
   pub position: Vec3,
   /// Initial orientation. Normalized on insertion.
   /// 初始朝向。插入时会被归一化。
   pub orientation: Quat,
   /// Initial linear velocity.
   /// 初始线速度。
   pub linear_velocity: Vec3,
   /// Initial angular velocity.
   /// 初始角速度。
   pub angular_velocity: Vec3,
   /// Coulomb friction coefficient.
   /// 库仑摩擦系数。
   pub friction: f32,
   /// Bounciness, from 0 (no bounce) upward.
   /// 弹性，0 表示不反弹。
   pub restitution: f32,
   /// Velocity decay per second applied to linear motion.
   /// 每秒施加于线运动的衰减。
   pub linear_damping: f32,
   /// How much of the World's gravity reaches this Body. See
   /// [`BodyDesc::with_gravity_scale`].
   /// World 的重力有多少作用于此 Body。见 [`BodyDesc::with_gravity_scale`]。
   pub gravity_scale: f32,
   /// Velocity decay per second applied to angular motion.
   /// 每秒施加于角运动的衰减。
   pub angular_damping: f32,
}

impl BodyDesc {
   /// A Body in the given motion mode, with the given shape.
   /// 给定运动模式与形状的 Body。
   #[must_use]
   pub fn new(shape: Shape, motion: MotionType) -> Self {
      Self {
         shape,
         motion,
         mass: 1.0,
         position: Vec3::ZERO,
         orientation: Quat::IDENTITY,
         linear_velocity: Vec3::ZERO,
         angular_velocity: Vec3::ZERO,
         friction: DEFAULT_FRICTION,
         restitution: 0.0,
         linear_damping: 0.0,
         gravity_scale: 1.0,
         angular_damping: 0.0,
      }
   }

   /// A Dynamic Body of the given shape and mass. A non-positive or non-finite
   /// mass yields a Body nothing can accelerate.
   /// 给定形状与质量的 Dynamic Body。非正或非有限的质量得到无法被加速的 Body。
   #[must_use]
   pub fn dynamic(shape: Shape, mass: f32) -> Self {
      Self::new(shape, MotionType::Dynamic).with_mass(mass)
   }

   /// A Static Body: it never moves.
   /// Static Body：永不移动。
   #[must_use]
   pub fn stationary(shape: Shape) -> Self {
      Self::new(shape, MotionType::Static)
   }

   /// A Kinematic Body, moved by whatever velocity the app sets on it.
   /// Kinematic Body，由应用设定的速度驱动。
   #[must_use]
   pub fn kinematic(shape: Shape) -> Self {
      Self::new(shape, MotionType::Kinematic)
   }

   /// Set the mass.
   /// 设置质量。
   #[must_use]
   pub fn with_mass(mut self, mass: f32) -> Self {
      self.mass = mass;
      self
   }

   /// Set the initial position.
   /// 设置初始位置。
   #[must_use]
   pub fn at(mut self, position: Vec3) -> Self {
      self.position = position;
      self
   }

   /// Set the initial orientation.
   /// 设置初始朝向。
   #[must_use]
   pub fn oriented(mut self, orientation: Quat) -> Self {
      self.orientation = orientation;
      self
   }

   /// Set the initial linear velocity.
   /// 设置初始线速度。
   #[must_use]
   pub fn with_velocity(mut self, velocity: Vec3) -> Self {
      self.linear_velocity = velocity;
      self
   }

   /// Set the initial angular velocity.
   /// 设置初始角速度。
   #[must_use]
   pub fn with_angular_velocity(mut self, angular_velocity: Vec3) -> Self {
      self.angular_velocity = angular_velocity;
      self
   }

   /// Set the friction coefficient.
   /// 设置摩擦系数。
   #[must_use]
   pub fn with_friction(mut self, friction: f32) -> Self {
      self.friction = friction;
      self
   }

   /// Set the restitution.
   /// 设置弹性。
   #[must_use]
   pub fn with_restitution(mut self, restitution: f32) -> Self {
      self.restitution = restitution;
      self
   }

   /// Scale the World's gravity for this Body.
   /// 按比例缩放作用于此 Body 的 World 重力。
   ///
   /// `0.0` means gravity never touches it, which is what a caller wants when
   /// something else already owns the Body's motion — a flight model, or anything
   /// kinematic-in-spirit that still needs contacts and friction. `1.0` is the
   /// default and `2.0` falls twice as fast.
   /// `0.0` 表示重力永不作用于它，这正是当别的东西已经拥有该 Body 的运动时想要的——例如
   /// 飞行模型，或任何在精神上近似运动学、却仍需要接触与摩擦的东西。默认为 `1.0`，`2.0`
   /// 则以两倍速度下坠。
   ///
   /// Negative or non-finite values are clamped to zero.
   /// 负值或非有限值被钳制为零。
   #[must_use]
   pub fn with_gravity_scale(mut self, scale: f32) -> Self {
      self.gravity_scale = if scale.is_finite() && scale > 0.0 {
         scale
      } else {
         0.0
      };
      self
   }

   /// Damping applied to linear and angular velocity.
   /// 施加于线速度与角速度的阻尼。
   #[must_use]
   pub fn with_damping(mut self, linear: f32, angular: f32) -> Self {
      self.linear_damping = linear;
      self.angular_damping = angular;
      self
   }
}

/// A Body's simulation state. Owned by a [`crate::World`].
/// Body 的模拟状态。由 [`crate::World`] 持有。
#[derive(Clone, Copy, Debug)]
pub(crate) struct Body {
   pub(crate) shape: Shape,
   pub(crate) motion: MotionType,
   pub(crate) mass: MassProperties,
   pub(crate) position: Vec3,
   pub(crate) orientation: Quat,
   pub(crate) linear_velocity: Vec3,
   pub(crate) angular_velocity: Vec3,
   pub(crate) force: Vec3,
   pub(crate) torque: Vec3,
   /// Velocity that exists only to push overlap apart.
   ///
   /// Positional correction runs against these instead of the real velocities, so
   /// resolving penetration never leaves kinetic energy behind to jitter a
   /// resting stack.
   /// 仅用于推开重叠的速度。位置修正作用于它们而非真实速度，因此消除穿透不会留下让静止
   /// 堆叠抖动的动能。
   pub(crate) pseudo_linear_velocity: Vec3,
   /// Angular counterpart of `pseudo_linear_velocity`.
   /// `pseudo_linear_velocity` 的角对应量。
   pub(crate) pseudo_angular_velocity: Vec3,
   pub(crate) friction: f32,
   pub(crate) restitution: f32,
   pub(crate) linear_damping: f32,
   /// How much of the World's gravity reaches this Body.
   /// World 的重力有多少作用于此 Body。
   pub(crate) gravity_scale: f32,
   pub(crate) angular_damping: f32,
   /// Whether the Body is currently excluded from simulation.
   pub(crate) sleeping: bool,
   /// Seconds spent below the sleep velocity thresholds.
   pub(crate) sleep_timer: f32,
   /// Whether the Body was still moving after the previous step's solve.
   ///
   /// Waking a neighbour must ask this rather than look at the current velocity:
   /// every awake Dynamic Body carries a fresh gravity increment early in a step,
   /// so a raw velocity test calls it "moving" every single step and no stack ever
   /// stays asleep.
   /// Body 在上一步求解之后是否仍在运动。唤醒邻居必须问这个，而不是看当前速度：每个清醒的
   /// Dynamic Body 在一步的早期都带着刚加上的重力增量，因此直接看速度会每步都判定它"在动"，
   /// 任何堆叠都无法保持睡眠。
   pub(crate) was_moving: bool,
   /// The Island this Body went to sleep with, or [`NO_SLEEP_GROUP`].
   ///
   /// It survives the Body waking up: that is exactly how a disturbance to one
   /// member is turned into waking the whole Island on the next step.
   /// Body 与之一起入睡的 Island，或 [`NO_SLEEP_GROUP`]。
   /// 它在 Body 醒来之后依然保留：这正是把"某个成员被扰动"转化为"下一步唤醒整个 Island"
   /// 的机制。
   pub(crate) sleep_group: u32,
}

impl Body {
   /// Build the simulation state for a description.
   pub(crate) fn from_desc(desc: &BodyDesc) -> Self {
      let mass = if desc.motion.is_simulated() {
         desc.shape.mass_properties(desc.mass)
      } else {
         MassProperties::IMMOVABLE
      };
      let orientation = if desc.orientation.length_squared() > 0.0 {
         desc.orientation.normalize()
      } else {
         Quat::IDENTITY
      };
      Self {
         shape: desc.shape,
         motion: desc.motion,
         mass,
         position: desc.position,
         orientation,
         linear_velocity: desc.linear_velocity,
         angular_velocity: desc.angular_velocity,
         force: Vec3::ZERO,
         torque: Vec3::ZERO,
         pseudo_linear_velocity: Vec3::ZERO,
         pseudo_angular_velocity: Vec3::ZERO,
         friction: desc.friction.max(0.0),
         restitution: desc.restitution.max(0.0),
         linear_damping: desc.linear_damping.max(0.0),
         gravity_scale: desc.gravity_scale,
         angular_damping: desc.angular_damping.max(0.0),
         sleeping: false,
         sleep_timer: 0.0,
         was_moving: false,
         sleep_group: NO_SLEEP_GROUP,
      }
   }

   /// The Body's world-space bounding box.
   pub(crate) fn aabb(&self) -> Aabb {
      self.shape.aabb(self.position, self.orientation)
   }

   /// Apply the Body's inverse world inertia tensor to a world-space vector.
   ///
   /// The tensor is diagonal in local space, so this rotates into local space,
   /// scales per axis, and rotates back: `R * (I⁻¹ ⊙ (R⁻¹ v))`.
   /// 该张量在局部空间是对角的，因此这里先旋入局部空间、各轴缩放、再旋回：
   /// `R * (I⁻¹ ⊙ (R⁻¹ v))`。
   pub(crate) fn apply_inv_inertia(&self, value: Vec3) -> Vec3 {
      let local = self.orientation.inverse() * value;
      self.orientation * (local * self.mass.inv_inertia)
   }

   /// Velocity of the Body at a point given relative to its center of mass,
   /// including the contribution of its spin.
   /// Body 在相对于质心给定点处的速度，含自转的贡献。
   pub(crate) fn point_velocity(&self, anchor: Vec3) -> Vec3 {
      self.linear_velocity + self.angular_velocity.cross(anchor)
   }

   /// The pseudo-velocity at a point, used only by positional correction.
   /// 该点处的伪速度，仅用于位置修正。
   pub(crate) fn pseudo_point_velocity(&self, anchor: Vec3) -> Vec3 {
      self.pseudo_linear_velocity + self.pseudo_angular_velocity.cross(anchor)
   }

   /// Drop the positional-correction velocities once they have been applied.
   /// 在伪速度被应用之后将其清零。
   pub(crate) fn clear_pseudo_velocities(&mut self) {
      self.pseudo_linear_velocity = Vec3::ZERO;
      self.pseudo_angular_velocity = Vec3::ZERO;
   }

   /// Whether the Body is moving slowly enough to be a sleep candidate.
   pub(crate) fn is_below_sleep_threshold(&self, linear: f32, angular: f32) -> bool {
      self.linear_velocity.length_squared() <= linear * linear
         && self.angular_velocity.length_squared() <= angular * angular
   }

   /// Stop the Body and reset its sleep timer.
   pub(crate) fn put_to_sleep(&mut self) {
      self.linear_velocity = Vec3::ZERO;
      self.angular_velocity = Vec3::ZERO;
      self.force = Vec3::ZERO;
      self.torque = Vec3::ZERO;
      self.sleeping = true;
      self.sleep_timer = 0.0;
   }

   /// Return the Body to simulation.
   pub(crate) fn wake(&mut self) {
      self.sleeping = false;
      self.sleep_timer = 0.0;
   }

   /// Record whether the Body is moving, for the next step's wake decisions.
   pub(crate) fn refresh_movement(&mut self, linear: f32, angular: f32) {
      self.was_moving = !self.is_below_sleep_threshold(linear, angular);
   }
}

#[cfg(test)]
mod tests {
   use super::{Body, BodyDesc, BodyId, MotionType};
   use crate::shape::Shape;
   use he3d_math::{Quat, Vec3};

   #[test]
   fn motion_mode_reports_its_own_semantics() {
      assert!(MotionType::Dynamic.is_simulated());
      assert!(MotionType::Dynamic.moves());
      assert!(MotionType::Dynamic.is_pushable());

      assert!(!MotionType::Static.is_simulated());
      assert!(!MotionType::Static.moves());
      assert!(!MotionType::Static.is_pushable());

      assert!(!MotionType::Kinematic.is_simulated());
      assert!(MotionType::Kinematic.moves());
      assert!(!MotionType::Kinematic.is_pushable());
   }

   #[test]
   fn generated_bodies_carry_derived_mass() {
      let dynamic = Body::from_desc(&BodyDesc::dynamic(Shape::sphere(1.0), 2.0));
      assert!(dynamic.mass.inv_mass > 0.0);

      let stationary = Body::from_desc(&BodyDesc::stationary(Shape::sphere(1.0)));
      assert!(stationary.mass.is_immovable());

      let kinematic = Body::from_desc(&BodyDesc::kinematic(Shape::sphere(1.0)));
      assert!(kinematic.mass.is_immovable());
   }

   #[test]
   fn a_non_unit_orientation_is_normalized_on_construction() {
      let desc =
         BodyDesc::dynamic(Shape::sphere(1.0), 1.0).oriented(Quat::from_xyzw(0.0, 0.0, 0.0, 3.0));
      let body = Body::from_desc(&desc);
      assert!((body.orientation.length() - 1.0).abs() < 1e-5);
   }

   #[test]
   fn inverse_inertia_acts_along_the_bodys_own_axes() {
      let desc = BodyDesc::dynamic(Shape::cuboid(Vec3::new(1.0, 2.0, 4.0)), 3.0)
         .oriented(Quat::from_rotation_y(0.7));
      let body = Body::from_desc(&desc);
      // The world direction of the Body's local X axis must come back scaled by
      // inv_inertia.x and still lie on that same axis.
      let world_x = body.orientation * Vec3::X;
      let applied = body.apply_inv_inertia(world_x);
      assert!((applied.length() - body.mass.inv_inertia.x).abs() < 1e-4);
      assert!(
         applied.cross(world_x).length() < 1e-4,
         "the result left the axis: {applied:?}"
      );
   }

   #[test]
   fn velocity_at_combines_linear_and_angular_motion() {
      let mut desc = BodyDesc::dynamic(Shape::sphere(0.5), 1.0);
      desc.linear_velocity = Vec3::new(0.0, 0.0, 1.0);
      desc.angular_velocity = Vec3::new(0.0, 1.0, 0.0);
      let body = Body::from_desc(&desc);
      // omega x r at a point on +Z is +X, so the two contributions add.
      let at = body.point_velocity(Vec3::new(0.0, 0.0, 1.0));
      assert!((at.x - 1.0).abs() < 1e-5, "got {at:?}");
      assert!(at.y.abs() < 1e-5, "got {at:?}");
      assert!((at.z - 1.0).abs() < 1e-5, "got {at:?}");
   }

   #[test]
   fn ids_compare_by_slot_and_generation() {
      assert_eq!(BodyId::new(1, 0), BodyId::new(1, 0));
      assert_ne!(BodyId::new(1, 0), BodyId::new(1, 1));
      assert_ne!(BodyId::new(1, 0), BodyId::new(2, 0));
      assert_eq!(BodyId::new(7, 3).index(), 7);
      assert_eq!(BodyId::new(7, 3).generation(), 3);
   }
}

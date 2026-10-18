use alloc::vec::Vec;

use he3d_math::{Quat, Vec3};

use crate::aabb::Aabb;
use crate::body::{Body, BodyDesc, BodyId, MotionType, NO_SLEEP_GROUP};
use crate::broadphase::{DEFAULT_CELL_SIZE, SpatialGrid};
use crate::contact::{
   ConstraintPoint, Contact, ContactConstraint, ContactPartner, ContactPhase, MAX_MANIFOLD_POINTS,
   tangent_basis,
};
use crate::narrowphase;
use crate::query::{self, Ray, RayHit};
use crate::shape::{MassProperties, Shape};
use crate::solver::{self, SolverSettings};
use crate::static_geometry::{StaticMesh, StaticMeshId, StaticTopology};
use crate::storage::{STATIC_PARTNER, Slot};

/// Linear speed, in units per second, below which a Body counts as at rest.
/// 线速度（单位/秒）低于此值时，Body 视为静止。
pub const SLEEP_LINEAR_THRESHOLD: f32 = 0.05;

/// Angular speed, in radians per second, below which a Body counts as at rest.
/// 角速度（弧度/秒）低于此值时，Body 视为静止。
pub const SLEEP_ANGULAR_THRESHOLD: f32 = 0.05;

/// Seconds a Body must stay below both sleep thresholds before it sleeps.
/// Body 必须持续低于两个睡眠阈值多少秒后才会入睡。
pub const SLEEP_TIME: f32 = 0.5;

/// The default gravity, in units per second squared.
/// 默认重力，单位/秒²。
pub const DEFAULT_GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

/// One slot in the World's Static geometry storage.
/// World 静态几何存储中的一个槽位。
#[derive(Clone, Debug)]
pub(crate) struct StaticSlot {
   pub(crate) generation: u32,
   pub(crate) mesh: Option<StaticMesh>,
}

/// Owns a set of Bodies and advances them through time.
/// 持有一组 Body 并推进它们的时间演化。
///
/// A World knows nothing about rendering.
/// It never sees an `ObjectId`, a `Scene`, or a `Transform`; a Body's placement is a position and an orientation, and the application owns the link between a Body and whatever draws it.
/// See `CONTEXT.md` and `docs/adr/0001`.
/// World 对渲染一无所知。
/// 它永远看不到 `ObjectId`、`Scene` 或 `Transform`；Body 的摆放是位置与朝向，而 Body 与"画它的东西"之间的联系由应用拥有。
/// 见 `CONTEXT.md` 与 `docs/adr/0001`。
///
/// # Examples
/// 示例
/// ```
/// use he3d_math::Vec3;
/// use he3d_physics::{BodyDesc, Shape, World};
///
/// let mut world = World::new();
/// let ball = world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(0.0, 5.0, 0.0)));
/// for _ in 0..60 {
///     world.step(1.0 / 60.0);
/// }
/// let y = world.position(ball).expect("the ball exists").y;
/// assert!(y < 5.0, "the ball should have fallen, got y={y}");
/// ```
#[derive(Clone, Debug)]
pub struct World {
   slots: Vec<Slot>,
   free: Vec<u32>,
   gravity: Vec3,
   settings: SolverSettings,
   grid: SpatialGrid,
   /// Bodies taking part in this step, as handles.
   active: Vec<BodyId>,
   /// The bounding box of each entry in `active`, in the same order.
   aabbs: Vec<Aabb>,
   /// Constraints built by this step.
   constraints: Vec<ContactConstraint>,
   /// The previous step's constraints, kept for warm starting and contact phases.
   previous: Vec<ContactConstraint>,
   /// Bodies to wake after contact detection.
   wakers: Vec<usize>,
   /// The contact list published for the step just taken.
   contacts: Vec<Contact>,
   /// Registered Static geometry, by slot.
   static_meshes: Vec<StaticSlot>,
   /// Free Static geometry slots.
   static_free: Vec<u32>,
   /// The shared topology every Static mesh welds into, so an edge between two separate meshes is still recognised as an Internal edge.
   /// 所有 Static mesh 焊接进的共享拓扑，因此两个独立 mesh 之间的边仍被识别为 Internal edge。
   topology: StaticTopology,
   /// Friction of Static geometry, combined with each Body's own.
   /// Static geometry 的摩擦系数，与各 Body 自身的系数合并。
   static_friction: f32,
   /// The members of each sleeping Island, by group id.
   /// 各睡眠 Island 的成员，按组 id 索引。
   sleep_groups: Vec<Vec<u32>>,
   /// Group ids whose members have all woken and which may be reused.
   /// 成员已全部醒来、可以复用的组 id。
   free_sleep_groups: Vec<u32>,
   /// Groups found with a woken member this step.
   /// 本步发现有成员已醒来的组。
   woken_groups: Vec<u32>,
   /// Union-find parent per slot, rebuilt every step.
   /// 每个槽位的并查集父指针，每步重建。
   island_parent: Vec<u32>,
   /// The Island root of each awake dynamic Body, or [`NO_SLEEP_GROUP`].
   /// 每个清醒 Dynamic Body 所属 Island 的根，或 [`NO_SLEEP_GROUP`]。
   island_root: Vec<u32>,
   /// Whether every dynamic Body under each root is settled.
   /// 每个根下的所有 Dynamic Body 是否都已稳定。
   island_settled: Vec<bool>,
   /// Whether each root has any dynamic Body at all.
   /// 每个根下是否至少有一个 Dynamic Body。
   island_has_dynamic: Vec<bool>,
   /// The sleep group being built for each root this step.
   /// 本步为每个根构建中的睡眠组。
   island_group: Vec<u32>,
}

impl Default for World {
   fn default() -> Self {
      Self::new()
   }
}

impl World {
   /// An empty World under [`DEFAULT_GRAVITY`].
   /// 处于 [`DEFAULT_GRAVITY`] 下的空 World。
   #[must_use]
   pub fn new() -> Self {
      Self {
         slots: Vec::new(),
         free: Vec::new(),
         gravity: DEFAULT_GRAVITY,
         settings: SolverSettings::default(),
         grid: SpatialGrid::new(DEFAULT_CELL_SIZE),
         active: Vec::new(),
         aabbs: Vec::new(),
         constraints: Vec::new(),
         previous: Vec::new(),
         wakers: Vec::new(),
         contacts: Vec::new(),
         static_meshes: Vec::new(),
         static_free: Vec::new(),
         topology: StaticTopology::default(),
         static_friction: crate::body::DEFAULT_FRICTION,
         sleep_groups: Vec::new(),
         free_sleep_groups: Vec::new(),
         woken_groups: Vec::new(),
         island_parent: Vec::new(),
         island_root: Vec::new(),
         island_settled: Vec::new(),
         island_has_dynamic: Vec::new(),
         island_group: Vec::new(),
      }
   }

   /// The current gravity vector.
   /// 当前重力向量。
   #[must_use]
   pub fn gravity(&self) -> Vec3 {
      self.gravity
   }

   /// Override the gravity vector.
   /// Non-finite input is ignored.
   /// 覆盖重力向量。
   /// 非有限输入会被忽略。
   pub fn set_gravity(&mut self, gravity: Vec3) {
      if gravity.is_finite() {
         self.gravity = gravity;
      }
   }

   /// The edge length of a broadphase cell.
   /// 宽阶段单元格的边长。
   ///
   /// It should be near the size of a typical Body.
   /// 它应接近典型 Body 的尺寸。
   pub fn set_broadphase_cell_size(&mut self, cell_size: f32) {
      if cell_size.is_finite() && cell_size > 0.0 {
         self.grid = SpatialGrid::new(cell_size);
      }
   }

   /// Velocity iterations per step.
   /// 每步的速度迭代次数。
   pub fn set_velocity_iterations(&mut self, iterations: u32) {
      self.settings.velocity_iterations = iterations.max(1);
   }

   /// Friction of Static geometry, combined with each Body's own.
   /// Static geometry 的摩擦系数，与各 Body 自身的系数合并。
   ///
   /// It is not a per-mesh property: terrain is usually one material, and a Body's own coefficient already varies per Body.
   /// 它不是逐网格属性：地形通常只有一种材质，而 Body 自身的系数已经逐 Body 变化。
   pub fn set_static_friction(&mut self, friction: f32) {
      if friction.is_finite() && friction >= 0.0 {
         self.static_friction = friction;
      }
   }

   /// Register immovable triangle geometry and return its handle.
   /// 注册不可移动的三角形几何并返回其句柄。
   ///
   /// `positions` is a flat list in which every three consecutive entries form one triangle — the layout [`he3d_math::Vec3`] arrays use everywhere else.
   /// Every registered mesh welds into one shared topology, so an edge that lies between two separate meshes still counts as an Internal edge; that is what keeps a Body sliding across streamed terrain from catching on the seam between two tiles.
   /// `positions` 是扁平列表，每三个连续条目构成一个三角形——与其他地方使用 [`he3d_math::Vec3`] 数组的布局一致。
   /// 每个已注册网格都焊接进同一套共享拓扑，因此位于两个独立网格之间的边仍算作 Internal edge；这正是让划过流式地形的 Body 不会卡在瓦片接缝上的原因。
   ///
   /// Returns `None` for malformed input: an empty list, a length that is not a multiple of three, or a coordinate that is not finite.
   /// 畸形输入返回 `None`：空列表、长度不是三的倍数，或存在非有限坐标。
   pub fn add_static_mesh(&mut self, positions: &[Vec3]) -> Option<StaticMeshId> {
      if positions.is_empty() || positions.len() % 3 != 0 {
         return None;
      }
      if !positions.iter().all(|vertex| vertex.is_finite()) {
         return None;
      }

      let triangle_count = positions.len() / 3;
      let mut owned = Vec::with_capacity(positions.len());
      let mut triangle_bounds = Vec::with_capacity(triangle_count);
      let mut welded = Vec::with_capacity(triangle_count);
      for triangle_index in 0..triangle_count {
         let base = triangle_index * 3;
         let vertices = [positions[base], positions[base + 1], positions[base + 2]];
         let ids = [
            self.topology.weld(vertices[0]),
            self.topology.weld(vertices[1]),
            self.topology.weld(vertices[2]),
         ];
         self.topology.add_triangle(ids);
         triangle_bounds.push(Aabb::new(
            vertices[0].min(vertices[1]).min(vertices[2]),
            vertices[0].max(vertices[1]).max(vertices[2]),
         ));
         welded.push(ids);
         owned.extend(vertices);
      }
      let mut bounds = *triangle_bounds.first()?;
      for triangle_aabb in &triangle_bounds[1..] {
         bounds = bounds.merged(triangle_aabb);
      }

      let mesh = StaticMesh {
         positions: owned,
         triangle_bounds,
         welded,
         bounds,
      };
      if let Some(index) = self.static_free.pop() {
         let slot = &mut self.static_meshes[index as usize];
         slot.mesh = Some(mesh);
         Some(StaticMeshId::new(index, slot.generation))
      } else {
         let index = u32::try_from(self.static_meshes.len()).ok()?;
         self.static_meshes.push(StaticSlot {
            generation: 0,
            mesh: Some(mesh),
         });
         Some(StaticMeshId::new(index, 0))
      }
   }

   /// Remove a piece of Static geometry, invalidating its handle.
   /// 移除一块 Static geometry 并使其句柄失效。
   ///
   /// Its triangles release their edges from the shared topology, so a tile that is streamed out stops making its neighbour's boundary look internal.
   /// 它的三角形会从共享拓扑中释放各自的边，因此被流式卸载的瓦片不再让邻居的边界显得像内部边。
   pub fn remove_static_mesh(&mut self, id: StaticMeshId) -> bool {
      let Some(slot) = self.static_meshes.get_mut(id.index() as usize) else {
         return false;
      };
      if slot.generation != id.generation() {
         return false;
      }
      let Some(mesh) = slot.mesh.take() else {
         return false;
      };
      let bounds = mesh.bounds;
      for triangle in &mesh.welded {
         self.topology.remove_triangle(*triangle);
      }
      slot.generation = slot.generation.wrapping_add(1);
      self.static_free.push(id.index());

      // Anything asleep inside the region this geometry occupied has just lost its support, so it must be returned to simulation.
      // Without this a Body left asleep on streamed-out terrain would hang in mid-air.
      //
      // The test is geometric rather than "which constraints mentioned this mesh", because a sleeping Body stops producing constraints at all — by the time it is asleep there is nothing left to consult.
      // 处于该几何所占区域内的睡眠 Body 刚刚失去了支撑，因此必须被唤回模拟。
      // 若没有这一步，睡在被流式卸载地形上的 Body 会悬在半空。
      // 判据用几何而不是"哪些约束提到过这块网格"，因为睡眠中的 Body 根本不再产生约束——等它睡着时已经没有可查询的记录了。
      for body_slot in &mut self.slots {
         let Some(body) = body_slot.get_mut() else {
            continue;
         };
         if body.sleeping && body.motion.is_simulated() && body.aabb().overlaps(&bounds) {
            body.wake();
         }
      }
      true
   }

   /// Whether a handle still addresses a registered Static mesh.
   /// 句柄是否仍指向一块已注册的 Static mesh。
   #[must_use]
   pub fn contains_static_mesh(&self, id: StaticMeshId) -> bool {
      self
         .static_meshes
         .get(id.index() as usize)
         .is_some_and(|slot| slot.generation == id.generation() && slot.mesh.is_some())
   }

   /// How many pieces of Static geometry are registered.
   /// 已注册的 Static geometry 数量。
   #[must_use]
   pub fn static_mesh_count(&self) -> usize {
      self
         .static_meshes
         .iter()
         .filter(|slot| slot.mesh.is_some())
         .count()
   }

   /// The Contacts found by the most recent [`World::step`].
   /// 最近一次 [`World::step`] 发现的 Contact。
   ///
   /// The list is rebuilt on every step, so it must be read before the next one.
   /// 该列表每步重建，因此必须在下一步之前读取。
   #[must_use]
   pub fn contacts(&self) -> &[Contact] {
      &self.contacts
   }

   /// The nearest thing a ray meets within `max_distance`, or nothing at all.
   /// 射线在 `max_distance` 内遇到的最近对象，若没有则为空。
   ///
   /// The ray's direction is normalized first, so a hit's distance is a world length.
   /// A ray tests every Body's shape and every Static triangle and returns the closest hit, whatever kind it is; hit filtering is out of scope, per `docs/physics-spec.md` §10.
   /// Static triangles are scanned linearly against their own bounds, matching how the narrow phase reaches them; a triangle BVH is deferred because the measured cost at the current scale does not earn one.
   /// 射线的方向会先归一化，因此命中的距离是世界长度。
   /// 射线测试每个 Body 的形状与每个 Static 三角形，返回最近的命中，无论它是哪一种；命中过滤不在范围内，见 `docs/physics-spec.md` §10。
   /// Static 三角形对其自身包围盒做线性扫描，与窄相位访问它们的方式一致；三角形 BVH 被推迟，因为在当前规模下实测成本不足以挣得它。
   ///
   /// # Examples
   /// 示例
   /// ```
   /// use he3d_math::Vec3;
   /// use he3d_physics::{BodyDesc, Ray, RayTarget, Shape, World};
   ///
   /// let mut world = World::new();
   /// let ball = world.add_body(BodyDesc::dynamic(Shape::sphere(1.0), 1.0).at(Vec3::new(0.0, 0.0, 5.0)));
   ///
   /// let hit = world.cast_ray(Ray::new(Vec3::ZERO, Vec3::Z), 100.0).expect("the ray meets the ball");
   /// assert_eq!(hit.target, RayTarget::Body(ball));
   /// assert!((hit.distance - 4.0).abs() < 1e-4);
   /// ```
   #[must_use]
   pub fn cast_ray(&self, ray: Ray, max_distance: f32) -> Option<RayHit> {
      query::cast_ray(&self.slots, &self.static_meshes, ray, max_distance)
   }

   /// The number of live Bodies.
   /// 存活 Body 的数量。
   #[must_use]
   pub fn len(&self) -> usize {
      self
         .slots
         .iter()
         .filter(|slot| slot.get().is_some())
         .count()
   }

   /// Whether the World holds no Bodies.
   /// World 是否不持有任何 Body。
   #[must_use]
   pub fn is_empty(&self) -> bool {
      self.slots.iter().all(|slot| slot.get().is_none())
   }

   /// Insert a Body and return its handle.
   /// 插入一个 Body 并返回其句柄。
   ///
   /// # Panics
   /// Panics once the World holds more than `u32::MAX` Bodies.
   /// That is unreachable in practice, since every Body occupies a slot in a `Vec` that could not grow that large on any real machine.
   /// 当 World 持有的 Body 数超过 `u32::MAX` 时 panic。
   /// 实践中不可达：每个 Body 占用 `Vec` 中的一个槽位，没有任何真实机器能让它增长到那么大。
   pub fn add_body(&mut self, desc: BodyDesc) -> BodyId {
      let body = Body::from_desc(&desc);
      if let Some(index) = self.free.pop() {
         let slot = &mut self.slots[index as usize];
         slot.body = Some(body);
         BodyId::new(index, slot.generation)
      } else {
         let index = u32::try_from(self.slots.len()).expect("body count fits u32");
         self.slots.push(Slot {
            generation: 0,
            body: Some(body),
         });
         BodyId::new(index, 0)
      }
   }

   /// Remove a Body, invalidating its handle.
   /// Returns whether it existed.
   /// 移除一个 Body 并使其句柄失效。
   /// 返回它原先是否存在。
   pub fn remove_body(&mut self, id: BodyId) -> bool {
      let Some(slot) = self.slots.get_mut(id.index() as usize) else {
         return false;
      };
      if slot.generation != id.generation() || slot.get().is_none() {
         return false;
      }
      slot.body = None;
      slot.generation = slot.generation.wrapping_add(1);
      self.free.push(id.index());
      true
   }

   /// Whether a handle still addresses a live Body.
   /// 句柄是否仍指向一个存活的 Body。
   #[must_use]
   pub fn contains(&self, id: BodyId) -> bool {
      self.body(id).is_some()
   }

   /// Every live Body's handle, in slot order.
   /// 所有存活 Body 的句柄，按槽位顺序。
   ///
   /// # Panics
   /// Panics once the World holds more than `u32::MAX` Bodies; see [`World::add_body`].
   /// 当 World 持有的 Body 数超过 `u32::MAX` 时 panic；见 [`World::add_body`]。
   pub fn ids(&self) -> impl Iterator<Item = BodyId> + '_ {
      self.slots.iter().enumerate().filter_map(|(index, slot)| {
         slot.get().map(|_| {
            BodyId::new(
               u32::try_from(index).expect("slot count fits u32"),
               slot.generation,
            )
         })
      })
   }

   /// The Body's shape.
   /// Body 的形状。
   #[must_use]
   pub fn shape(&self, id: BodyId) -> Option<Shape> {
      self.body(id).map(|body| body.shape)
   }

   /// The Body's motion mode.
   /// Body 的运动模式。
   #[must_use]
   pub fn motion(&self, id: BodyId) -> Option<MotionType> {
      self.body(id).map(|body| body.motion)
   }

   /// The Body's mass properties.
   /// Body 的质量属性。
   #[must_use]
   pub fn mass_properties(&self, id: BodyId) -> Option<MassProperties> {
      self.body(id).map(|body| body.mass)
   }

   /// The Body's position.
   /// Body 的位置。
   #[must_use]
   pub fn position(&self, id: BodyId) -> Option<Vec3> {
      self.body(id).map(|body| body.position)
   }

   /// The Body's orientation.
   /// Body 的朝向。
   #[must_use]
   pub fn orientation(&self, id: BodyId) -> Option<Quat> {
      self.body(id).map(|body| body.orientation)
   }

   /// The Body's world-space bounding box.
   /// Body 的世界空间包围盒。
   #[must_use]
   pub fn aabb(&self, id: BodyId) -> Option<Aabb> {
      self.body(id).map(Body::aabb)
   }

   /// The Body's linear velocity.
   /// Body 的线速度。
   #[must_use]
   pub fn velocity(&self, id: BodyId) -> Option<Vec3> {
      self.body(id).map(|body| body.linear_velocity)
   }

   /// The Body's angular velocity.
   /// Body 的角速度。
   #[must_use]
   pub fn angular_velocity(&self, id: BodyId) -> Option<Vec3> {
      self.body(id).map(|body| body.angular_velocity)
   }

   /// The Body's friction coefficient.
   /// Body 的摩擦系数。
   #[must_use]
   pub fn friction(&self, id: BodyId) -> Option<f32> {
      self.body(id).map(|body| body.friction)
   }

   /// The Body's restitution.
   /// Body 的弹性。
   #[must_use]
   pub fn restitution(&self, id: BodyId) -> Option<f32> {
      self.body(id).map(|body| body.restitution)
   }

   /// Whether the Body is currently asleep.
   /// Body 当前是否处于睡眠。
   #[must_use]
   pub fn is_sleeping(&self, id: BodyId) -> Option<bool> {
      self.body(id).map(|body| body.sleeping)
   }

   /// Move a Body to a position and orientation, waking it.
   /// 将 Body 移动到某位置与朝向，并唤醒它。
   ///
   /// Returns whether the Body exists and accepts the change.
   /// A Static Body is repositioned but stays static, and a non-finite or degenerate input is rejected.
   /// 返回 Body 是否存在并接受该改动。
   /// Static Body 会被重新摆放但仍保持静止；非有限或退化的输入会被拒绝。
   pub fn set_pose(&mut self, id: BodyId, position: Vec3, orientation: Quat) -> bool {
      if !position.is_finite() || !orientation.is_finite() || orientation.length_squared() <= 0.0 {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      body.position = position;
      body.orientation = orientation.normalize();
      if body.motion.is_simulated() {
         body.wake();
      }
      true
   }

   /// Set a Body's linear velocity, waking it.
   /// 设置 Body 的线速度，并唤醒它。
   pub fn set_velocity(&mut self, id: BodyId, velocity: Vec3) -> bool {
      if !velocity.is_finite() {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      if !body.motion.moves() {
         return false;
      }
      body.linear_velocity = velocity;
      if body.motion.is_simulated() {
         body.wake();
      }
      true
   }

   /// Set a Body's angular velocity, waking it.
   /// 设置 Body 的角速度，并唤醒它。
   pub fn set_angular_velocity(&mut self, id: BodyId, angular_velocity: Vec3) -> bool {
      if !angular_velocity.is_finite() {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      if !body.motion.moves() {
         return false;
      }
      body.angular_velocity = angular_velocity;
      if body.motion.is_simulated() {
         body.wake();
      }
      true
   }

   /// Set a Body's friction coefficient, clamped to be non-negative.
   /// 设置 Body 的摩擦系数，钳制为非负。
   pub fn set_friction(&mut self, id: BodyId, friction: f32) -> bool {
      if !friction.is_finite() {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      body.friction = friction.max(0.0);
      true
   }

   /// Set a Body's restitution, clamped to be non-negative.
   /// 设置 Body 的弹性，钳制为非负。
   pub fn set_restitution(&mut self, id: BodyId, restitution: f32) -> bool {
      if !restitution.is_finite() {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      body.restitution = restitution.max(0.0);
      true
   }

   /// Set a Body's linear and angular damping, clamped to be non-negative.
   /// 设置 Body 的线性与角阻尼，钳制为非负。
   pub fn set_damping(&mut self, id: BodyId, linear: f32, angular: f32) -> bool {
      if !linear.is_finite() || !angular.is_finite() {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      body.linear_damping = linear.max(0.0);
      body.angular_damping = angular.max(0.0);
      true
   }

   /// Change how much of the World's gravity reaches a Body.
   /// 改变 World 的重力有多少作用于此 Body。
   ///
   /// The live counterpart of [`BodyDesc::with_gravity_scale`], for a Body whose owner changes its mind mid-scene — a flight model that hands the aircraft back to gravity when it crashes, for instance.
   /// [`BodyDesc::with_gravity_scale`] 的运行时对应物，用于所有者在中途改变主意的 Body——例如飞行模型在飞机坠毁时把它交还给重力。
   ///
   /// Returns `false` for a dead handle.
   /// Negative or non-finite values are clamped to zero, and a zero scale wakes the Body, since a Body asleep under no gravity is not necessarily asleep under the new one.
   /// 句柄失效时返回 `false`。
   /// 负值或非有限值被钳制为零；系数为零时会唤醒该 Body，因为在无重力下睡着的 Body 在新重力下未必还该睡。
   pub fn set_gravity_scale(&mut self, id: BodyId, scale: f32) -> bool {
      let scale = if scale.is_finite() && scale > 0.0 {
         scale
      } else {
         0.0
      };
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      // An exact comparison is the point here: this is a change detector, not a measurement, and any difference at all is a change worth waking for.
      // 这里就是要精确比较：它是变更检测而非测量，任何差异都是值得唤醒的变化。
      #[expect(clippy::float_cmp)]
      let changed = body.gravity_scale != scale;
      if changed {
         body.wake();
      }
      body.gravity_scale = scale;
      true
   }

   /// Add a force at the Body's center of mass, acting for the next step only.
   /// 在 Body 质心施加一个只作用于下一步的力。
   ///
   /// Only a Dynamic Body accumulates force; Static and Kinematic Bodies ignore gravity and forces entirely.
   /// 只有 Dynamic Body 会累积力；Static 与 Kinematic 完全忽略重力与力。
   pub fn apply_force(&mut self, id: BodyId, force: Vec3) -> bool {
      if !force.is_finite() {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      if !body.motion.is_simulated() {
         return false;
      }
      body.force += force;
      body.wake();
      true
   }

   /// Add a torque, acting for the next step only.
   /// 施加一个只作用于下一步的力矩。
   pub fn apply_torque(&mut self, id: BodyId, torque: Vec3) -> bool {
      if !torque.is_finite() {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      if !body.motion.is_simulated() {
         return false;
      }
      body.torque += torque;
      body.wake();
      true
   }

   /// Apply an instantaneous linear impulse at the center of mass.
   /// 在质心施加一个瞬时线冲量。
   pub fn apply_impulse(&mut self, id: BodyId, impulse: Vec3) -> bool {
      if !impulse.is_finite() {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      if !body.motion.is_simulated() {
         return false;
      }
      body.linear_velocity += impulse * body.mass.inv_mass;
      body.wake();
      true
   }

   /// Apply an instantaneous angular impulse.
   /// 施加一个瞬时角冲量。
   pub fn apply_angular_impulse(&mut self, id: BodyId, impulse: Vec3) -> bool {
      if !impulse.is_finite() {
         return false;
      }
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      if !body.motion.is_simulated() {
         return false;
      }
      let delta = body.apply_inv_inertia(impulse);
      body.angular_velocity += delta;
      body.wake();
      true
   }

   /// Return a sleeping Body to simulation.
   /// 将处于睡眠的 Body 唤回模拟。
   pub fn wake(&mut self, id: BodyId) -> bool {
      let Some(body) = self.body_mut(id) else {
         return false;
      };
      body.wake();
      true
   }

   /// Clear the force and torque accumulators of every Body.
   /// 清空所有 Body 的力与力矩累积。
   pub fn clear_forces(&mut self) {
      for slot in &mut self.slots {
         if let Some(body) = slot.get_mut() {
            body.force = Vec3::ZERO;
            body.torque = Vec3::ZERO;
         }
      }
   }

   /// Advance the simulation by one fixed time step.
   /// 以固定步长推进模拟一步。
   ///
   /// Non-finite or non-positive input is ignored, so a caller handing over a degenerate frame delta cannot corrupt the World.
   /// Forces and torques accumulated before the call act for exactly this step and are then cleared.
   /// 非有限或非正输入会被忽略，因此调用方传入退化的帧间隔也不会破坏 World。
   /// 调用前累积的力与力矩恰好作用这一步，随后被清除。
   pub fn step(&mut self, delta_time: f32) {
      if !delta_time.is_finite() || delta_time <= 0.0 {
         return;
      }
      self.integrate_velocities(delta_time);
      self.detect_contacts();
      self.carry_impulses();
      solver::prepare(&self.slots, &mut self.constraints, delta_time);
      solver::warm_start(&mut self.slots, &self.constraints);
      solver::solve_velocities(&mut self.slots, &mut self.constraints, self.settings);
      solver::solve_positions(&mut self.slots, &mut self.constraints);
      self.integrate_positions(delta_time);
      self.publish_contacts();
      core::mem::swap(&mut self.constraints, &mut self.previous);
      self.update_sleep(delta_time);
      self.refresh_movement();
      self.clear_forces();
   }

   /// Record which Bodies are still moving, for the next step's wake decisions.
   /// 记录哪些 Body 仍在运动，供下一步的唤醒决策使用。
   ///
   /// This must run after the solve: the point of it is to read a velocity that is not the raw post-gravity one.
   /// 它必须在求解之后运行：其意义正是读取一个不是"刚加上重力"的速度。
   fn refresh_movement(&mut self) {
      for slot in &mut self.slots {
         if let Some(body) = slot.get_mut() {
            body.refresh_movement(SLEEP_LINEAR_THRESHOLD, SLEEP_ANGULAR_THRESHOLD);
         }
      }
   }

   /// Apply gravity, accumulated force, damping, and accumulated torque.
   /// 施加重力、累积力、阻尼与累积力矩。
   fn integrate_velocities(&mut self, delta_time: f32) {
      let gravity = self.gravity;
      for slot in &mut self.slots {
         let Some(body) = slot.get_mut() else {
            continue;
         };
         if !body.motion.is_simulated() || body.sleeping {
            continue;
         }
         // Gravity is an acceleration, but a Body with no mass is immovable by definition and must not be dragged along by it either, and a Body whose gravity scale is zero has its motion owned by something else.
         // 重力是加速度，但质量为零的 Body 按定义无法被移动，因此也不能被重力拖走；而重力系数为零的 Body，其运动由别的东西拥有。
         if body.mass.inv_mass > 0.0 && body.gravity_scale > 0.0 {
            body.linear_velocity += gravity * (body.gravity_scale * delta_time);
         }
         body.linear_velocity += body.force * body.mass.inv_mass * delta_time;
         body.linear_velocity *= 1.0 / (1.0 + body.linear_damping * delta_time);

         let angular_acceleration = body.apply_inv_inertia(body.torque);
         body.angular_velocity += angular_acceleration * delta_time;
         body.angular_velocity *= 1.0 / (1.0 + body.angular_damping * delta_time);
      }
   }

   /// Move every Body that moves by its own velocity, plus whatever positional correction asked for.
   /// 按自身速度移动每一个会移动的 Body，并叠加位置修正所要求的那部分。
   fn integrate_positions(&mut self, delta_time: f32) {
      for slot in &mut self.slots {
         let Some(body) = slot.get_mut() else {
            continue;
         };
         if !body.motion.moves() || body.sleeping {
            continue;
         }
         let linear = body.linear_velocity + body.pseudo_linear_velocity;
         body.position += linear * delta_time;

         // Rotate by the exact angle rather than by a truncated quaternion derivative, so a spinning Body accumulates neither drift nor shrink.
         // 按精确角度旋转，而不是截断的四元数导数，因此旋转的 Body 既不累积漂移也不收缩。
         let spin = body.angular_velocity + body.pseudo_angular_velocity;
         let speed = spin.length();
         if speed > 1e-8 {
            let axis = spin / speed;
            let step = Quat::from_axis_angle(axis, speed * delta_time);
            body.orientation = (step * body.orientation).normalize();
         }
         body.clear_pseudo_velocities();
      }
   }

   /// Narrow-phase every broadphase candidate into a contact constraint.
   ///
   /// A sleeping Body is left out of the solver entirely.
   /// It is woken only by a neighbour that is itself moving, so a stack resting on Static geometry stays asleep instead of being woken by its own floor every step.
   /// 每个宽阶段候选对都经过窄相位生成接触约束。
   /// 处于睡眠的 Body 完全不进入求解器；只有当邻居自身在运动时才会唤醒它，因此搁在静态几何上的堆叠不会被自己的地板每步唤醒。
   fn detect_contacts(&mut self) {
      self.active.clear();
      self.aabbs.clear();
      for (index, slot) in self.slots.iter().enumerate() {
         if let Some(body) = slot.get() {
            let Ok(index) = u32::try_from(index) else {
               break;
            };
            self.active.push(BodyId::new(index, slot.generation));
            self.aabbs.push(body.aabb());
         }
      }

      let static_friction = self.static_friction;
      let Self {
         slots,
         grid,
         active,
         aabbs,
         constraints,
         wakers,
         static_meshes,
         topology,
         ..
      } = self;
      grid.rebuild(aabbs);
      constraints.clear();
      wakers.clear();

      grid.for_each_candidate(aabbs, |first_compact, second_compact| {
         let first_index = active[first_compact as usize].index() as usize;
         let second_index = active[second_compact as usize].index() as usize;
         let (Some(first), Some(second)) = (slots[first_index].get(), slots[second_index].get())
         else {
            return;
         };
         if !solver::is_solvable(first, second) {
            return;
         }
         match (first.sleeping, second.sleeping) {
            (true, true) => return,
            (true, false) => {
               // Only a neighbour that was still moving after the last solve can disturb a sleeper.
               // Asking the live velocity would call every awake Body "moving", because gravity has just been added.
               // 只有在上一步求解后仍在运动的邻居才能扰动睡眠者。
               // 直接看实时速度会把每个清醒的 Body 都判为"在动"，因为重力刚刚被加上。
               if second.was_moving {
                  wakers.push(first_index);
               } else {
                  return;
               }
            }
            (false, true) => {
               if first.was_moving {
                  wakers.push(second_index);
               } else {
                  return;
               }
            }
            (false, false) => {}
         }

         let Some(manifold) = narrowphase::collide(
            first.shape,
            first.position,
            first.orientation,
            second.shape,
            second.position,
            second.orientation,
         ) else {
            return;
         };

         let mut constraint = ContactConstraint {
            id_a: active[first_compact as usize],
            id_b: ContactPartner::Body(active[second_compact as usize]),
            a: u32::try_from(first_index).unwrap_or(0),
            b: u32::try_from(second_index).unwrap_or(0),
            partner_key: 0,
            normal: manifold.normal,
            tangents: tangent_basis(manifold.normal),
            friction: solver::combine_friction(first.friction, second.friction),
            point_count: 0,
            points: [ConstraintPoint::default(); MAX_MANIFOLD_POINTS],
         };
         for (slot, point) in manifold.points[..manifold.count].iter().enumerate() {
            constraint.points[slot] = ConstraintPoint {
               position: point.position,
               anchor_a: point.position - first.position,
               anchor_b: point.position - second.position,
               depth: point.depth,
               feature: point.feature,
               ..ConstraintPoint::default()
            };
         }
         constraint.point_count = u8::try_from(manifold.count).unwrap_or(0);
         constraints.push(constraint);
      });

      detect_static_contacts(slots, static_friction, static_meshes, topology, constraints);
      for index in wakers.drain(..) {
         if let Some(body) = slots[index].get_mut() {
            body.wake();
         }
      }
   }

   /// Carry accumulated impulses across from the previous step, matching points by Body pair and feature id.
   /// 按 Body 对与特征 id 匹配接触点，从上一步沿用累积冲量。
   fn carry_impulses(&mut self) {
      // Both lists are sorted, so the match is a binary search per pair.
      // 两个列表都已排序，因此每对的匹配是一次二分查找。
      self
         .constraints
         .sort_unstable_by_key(ContactConstraint::sort_key);
      for current in &mut self.constraints {
         let key = current.sort_key();
         let Ok(found) = self
            .previous
            .binary_search_by_key(&key, ContactConstraint::sort_key)
         else {
            continue;
         };
         let old = &self.previous[found];
         for point in current.points_mut() {
            if let Some(previous) = old
               .points()
               .iter()
               .find(|entry| entry.feature == point.feature)
            {
               point.normal_impulse = previous.normal_impulse;
               point.tangent_impulse = previous.tangent_impulse;
            }
         }
      }
   }

   /// Publish this step's Contacts with their lifecycle phase.
   /// 发布本步的 Contact 及其生命周期阶段。
   fn publish_contacts(&mut self) {
      let Self {
         constraints,
         previous,
         contacts,
         ..
      } = self;
      contacts.clear();

      for current in constraints.iter() {
         let key = current.sort_key();
         let old = previous
            .binary_search_by_key(&key, ContactConstraint::sort_key)
            .ok()
            .map(|found| &previous[found]);
         for point in current.points() {
            let existed = old.is_some_and(|entry| {
               entry
                  .points()
                  .iter()
                  .any(|candidate| candidate.feature == point.feature)
            });
            contacts.push(Contact {
               a: current.id_a,
               b: current.id_b,
               point: point.position,
               normal: current.normal,
               depth: point.depth,
               phase: if existed {
                  ContactPhase::Persist
               } else {
                  ContactPhase::Begin
               },
            });
         }
      }

      // Anything that touched last step and does not now is ending.
      // The Bodies are named from the handle captured last step, so an `End` still refers to a Body that has since been removed.
      // 上一步相碰、本步不再相碰的接触正在结束。
      // Body 用上一步捕获的句柄命名，因此 `End` 仍能指代一个此后已被移除的 Body。
      for old in previous.iter() {
         let key = old.sort_key();
         let current = constraints
            .binary_search_by_key(&key, ContactConstraint::sort_key)
            .ok()
            .map(|found| &constraints[found]);
         for point in old.points() {
            let survives = current.is_some_and(|entry| {
               entry
                  .points()
                  .iter()
                  .any(|candidate| candidate.feature == point.feature)
            });
            if !survives {
               contacts.push(Contact {
                  a: old.id_a,
                  b: old.id_b,
                  point: point.position,
                  normal: old.normal,
                  depth: point.depth,
                  phase: ContactPhase::End,
               });
            }
         }
      }
   }

   /// Accumulate rest time, then settle and wake whole Islands.
   /// 累积静止时间，然后以整个 Island 为单位入睡与唤醒。
   ///
   /// Sleep is per Island, not per Body.
   /// A stack whose members slept independently would leave the ones above a disturbed Body frozen in mid-air; grouping them by the contact graph and moving the whole group together is what prevents that.
   /// 睡眠以 Island 而非 Body 为单位。
   /// 若一叠箱子的成员各自独立入睡，被扰动的 Body 上方的那些就会僵在半空；按接触图分组并整组一起移动，正是防止这件事的手段。
   fn update_sleep(&mut self, delta_time: f32) {
      self.dissolve_woken_groups();

      for slot in &mut self.slots {
         let Some(body) = slot.get_mut() else {
            continue;
         };
         if !body.motion.is_simulated() || body.sleeping {
            continue;
         }
         if body.is_below_sleep_threshold(SLEEP_LINEAR_THRESHOLD, SLEEP_ANGULAR_THRESHOLD) {
            body.sleep_timer += delta_time;
         } else {
            body.sleep_timer = 0.0;
         }
      }

      self.sleep_settled_islands();
   }

   /// Wake every Island that has at least one woken member, and release its group.
   ///
   /// A Body that has been woken keeps its group tag precisely so that this pass can find its companions.
   /// That single rule covers every way a disturbance can start — an impulse, a velocity, a new contact — without each call site having to know about Islands.
   /// 唤醒每个至少有一个成员已醒来的 Island，并释放其组。
   /// 已被唤醒的 Body 会刻意保留组标记，正是为了让这一趟能找到它的同伴。
   /// 这一条规则覆盖了扰动可能开始的每一种方式——冲量、速度、新接触——而无需每个调用点都了解 Island。
   fn dissolve_woken_groups(&mut self) {
      self.woken_groups.clear();
      for slot in &self.slots {
         let Some(body) = slot.get() else {
            continue;
         };
         if !body.sleeping
            && body.sleep_group != NO_SLEEP_GROUP
            && !self.woken_groups.contains(&body.sleep_group)
         {
            self.woken_groups.push(body.sleep_group);
         }
      }
      if self.woken_groups.is_empty() {
         return;
      }
      for slot in &mut self.slots {
         let Some(body) = slot.get_mut() else {
            continue;
         };
         if self.woken_groups.contains(&body.sleep_group) {
            body.wake();
            body.sleep_group = NO_SLEEP_GROUP;
         }
      }
      for group in self.woken_groups.drain(..) {
         if let Some(members) = self.sleep_groups.get_mut(group as usize) {
            members.clear();
         }
         self.free_sleep_groups.push(group);
      }
   }

   /// Group the awake dynamic Bodies by the contact graph and sleep any group that has come entirely to rest.
   /// 按接触图把清醒的 Dynamic Body 分组，并让整体已稳定的组入睡。
   fn sleep_settled_islands(&mut self) {
      let count = self.slots.len();
      let Ok(count_u32) = u32::try_from(count) else {
         return;
      };
      if count == 0 {
         return;
      }

      self.island_parent.clear();
      self.island_parent.extend(0..count_u32);
      self.island_root.clear();
      self.island_root.resize(count, NO_SLEEP_GROUP);
      self.island_settled.clear();
      self.island_settled.resize(count, true);
      self.island_has_dynamic.clear();
      self.island_has_dynamic.resize(count, false);
      self.island_group.clear();
      self.island_group.resize(count, NO_SLEEP_GROUP);

      // Union across Body-versus-Body contacts.
      // Static geometry and Kinematic Bodies transmit nothing: they never sleep, and a Body resting on a floor must be able to sleep without the floor joining its Island.
      // 沿 Body 对 Body 的接触做并集。
      // Static geometry 与 Kinematic Body 不传递任何东西：它们从不入睡，而搁在地板上的 Body 必须能在不把地板拉进自己 Island 的前提下入睡。
      for constraint in &self.constraints {
         if constraint.b == STATIC_PARTNER {
            continue;
         }
         let first = find_root(&mut self.island_parent, constraint.a);
         let second = find_root(&mut self.island_parent, constraint.b);
         if first != second {
            self.island_parent[second as usize] = first;
         }
      }

      for index in 0..count {
         let Some(body) = self.slots[index].get() else {
            continue;
         };
         if !body.motion.is_simulated() || body.sleeping {
            continue;
         }
         let root = find_root(&mut self.island_parent, u32::try_from(index).unwrap_or(0));
         self.island_root[index] = root;
         self.island_has_dynamic[root as usize] = true;
         if body.sleep_timer < SLEEP_TIME {
            self.island_settled[root as usize] = false;
         }
      }

      for index in 0..count {
         let root = self.island_root[index];
         if root == NO_SLEEP_GROUP
            || !self.island_has_dynamic[root as usize]
            || !self.island_settled[root as usize]
         {
            continue;
         }
         let mut group = self.island_group[root as usize];
         if group == NO_SLEEP_GROUP {
            group = self.allocate_sleep_group();
            self.island_group[root as usize] = group;
         }
         if let Some(members) = self.sleep_groups.get_mut(group as usize) {
            members.push(u32::try_from(index).unwrap_or(0));
         }
         if let Some(body) = self.slots[index].get_mut() {
            body.put_to_sleep();
            body.sleep_group = group;
         }
      }
   }

   /// Reserve a sleep group, reusing a released one when possible.
   /// 预留一个睡眠组，尽可能复用已释放的组。
   fn allocate_sleep_group(&mut self) -> u32 {
      if let Some(group) = self.free_sleep_groups.pop() {
         return group;
      }
      let index = u32::try_from(self.sleep_groups.len()).unwrap_or(u32::MAX);
      self.sleep_groups.push(Vec::new());
      index
   }

   /// Borrow a live Body.
   fn body(&self, id: BodyId) -> Option<&Body> {
      let slot = self.slots.get(id.index() as usize)?;
      if slot.generation != id.generation() {
         return None;
      }
      slot.get()
   }

   /// Mutably borrow a live Body.
   fn body_mut(&mut self, id: BodyId) -> Option<&mut Body> {
      let slot = self.slots.get_mut(id.index() as usize)?;
      if slot.generation != id.generation() {
         return None;
      }
      slot.get_mut()
   }
}

/// Narrow-phase every Body against every registered piece of Static geometry.
/// 对每个 Body 与每块已注册 Static geometry 做窄相位。
///
/// Static geometry is not in the broadphase grid: each mesh carries its own bounds and each of its triangles carries tighter ones, so a Body only reaches the triangles it can actually touch.
/// Static geometry 不在宽阶段网格中：每个 mesh 携带自身包围盒，其每个三角形又携带更紧的包围盒，因此 Body 只会触及它真正够得着的三角形。
fn detect_static_contacts(
   slots: &[Slot],
   static_friction: f32,
   static_meshes: &[StaticSlot],
   topology: &StaticTopology,
   constraints: &mut Vec<ContactConstraint>,
) {
   // Static geometry is not in the grid.
   // Each mesh carries its own bounds, and each of its triangles carries tighter ones, so a Body only reaches the triangles it can actually touch.
   // Static geometry 不在网格中。
   // 每个 mesh 携带自身包围盒，其每个三角形又携带更紧的包围盒，因此 Body 只会触及它真正够得着的三角形。
   for (mesh_index, slot) in static_meshes.iter().enumerate() {
      let Some(mesh) = slot.mesh.as_ref() else {
         continue;
      };
      let Ok(mesh_index) = u32::try_from(mesh_index) else {
         break;
      };
      let mesh_id = StaticMeshId::new(mesh_index, slot.generation);
      for (body_index, body_slot) in slots.iter().enumerate() {
         let Some(body) = body_slot.get() else {
            continue;
         };
         // A sleeping Body stays asleep on immovable geometry: nothing there can move, so nothing there can wake it.
         // 睡眠中的 Body 会在不可移动几何上保持睡眠：那里没有任何东西会移动，因此也没有任何东西能唤醒它。
         if body.sleeping || !body.motion.moves() {
            continue;
         }
         let body_bounds = body.aabb();
         if !body_bounds.overlaps(&mesh.bounds) {
            continue;
         }
         let Ok(body_index) = u32::try_from(body_index) else {
            break;
         };
         for triangle_index in 0..mesh.triangle_count() {
            if !body_bounds.overlaps(&mesh.triangle_bounds[triangle_index]) {
               continue;
            }
            let triangle = mesh.triangle(triangle_index);
            let hit = match body.shape {
               Shape::Box { half_extents } => {
                  let frame =
                     narrowphase::BoxFrame::new(body.position, body.orientation, half_extents);
                  narrowphase::box_triangle(&frame, triangle)
               }
               Shape::Sphere { radius } => {
                  narrowphase::sphere_triangle(body.position, radius, triangle).map(|manifold| {
                     narrowphase::TriangleHit {
                        manifold,
                        source_edge: None,
                     }
                  })
               }
               Shape::Capsule {
                  half_height,
                  radius,
               } => {
                  let (start, end) =
                     narrowphase::capsule_segment(body.position, body.orientation, half_height);
                  narrowphase::capsule_triangle(start, end, radius, triangle).map(|manifold| {
                     narrowphase::TriangleHit {
                        manifold,
                        source_edge: None,
                     }
                  })
               }
            };
            let Some(hit) = hit else {
               continue;
            };

            // An Internal edge is where two triangles of the shared topology meet, so there is no real corner there for a Body to catch on.
            // Discarding the contact is what lets a Body slide across a triangulated surface instead of tripping over its seams.
            // Internal edge 是共享拓扑中两个三角形相接之处，那里并不存在可供 Body 卡住的真实棱角。
            // 丢弃该接触正是让 Body 能在三角化表面上滑行、而不是被其接缝绊住的原因。
            if let Some(edge) = hit.source_edge {
               let ids = mesh.welded[triangle_index];
               if topology.is_internal_edge(ids[edge], ids[(edge + 1) % 3]) {
                  continue;
               }
            }

            let key =
               (u64::from(mesh_id.index()) << 32) | u64::try_from(triangle_index).unwrap_or(0);
            let mut constraint = ContactConstraint {
               id_a: BodyId::new(body_index, body_slot.generation),
               id_b: ContactPartner::Static(mesh_id),
               a: body_index,
               b: STATIC_PARTNER,
               partner_key: key,
               normal: hit.manifold.normal,
               tangents: tangent_basis(hit.manifold.normal),
               friction: solver::combine_friction(body.friction, static_friction),
               point_count: 0,
               points: [ConstraintPoint::default(); MAX_MANIFOLD_POINTS],
            };
            for (slot, point) in hit.manifold.points[..hit.manifold.count].iter().enumerate() {
               constraint.points[slot] = ConstraintPoint {
                  position: point.position,
                  anchor_a: point.position - body.position,
                  // Immovable geometry has no center of mass, and the solver never applies an impulse on that side, so the second anchor is only ever read as a zero contribution.
                  // 不可移动几何没有质心，且求解器永不在该侧施加冲量，因此第二个锚点只会被当作零贡献读取。
                  anchor_b: Vec3::ZERO,
                  depth: point.depth,
                  feature: point.feature,
                  ..ConstraintPoint::default()
               };
            }
            constraint.point_count = u8::try_from(hit.manifold.count).unwrap_or(0);
            constraints.push(constraint);
         }
      }
   }
}
/// Walk a union-find tree to its root, halving the path on the way.
/// 沿并查集树走到根，途中做路径减半。
fn find_root(parent: &mut [u32], start: u32) -> u32 {
   let mut index = start;
   loop {
      let Some(&up) = parent.get(index as usize) else {
         return index;
      };
      if up == index {
         return index;
      }
      let grandparent = parent.get(up as usize).copied().unwrap_or(up);
      parent[index as usize] = grandparent;
      index = grandparent;
   }
}

#[cfg(test)]
// Test fixtures convert small loop counters into world coordinates and settle times into step counts; both sit far inside f32's exact integer range and the step count is bounded by the test's own duration.
// 测试夹具把小整数循环计数转换为世界坐标、把稳定时长换算为步数；两者都远在 f32 的精确整数范围内，且步数受测试自身时长的约束。
#[allow(
   clippy::cast_precision_loss,
   clippy::cast_possible_truncation,
   clippy::cast_sign_loss
)]
mod tests {
   use super::{DEFAULT_GRAVITY, SLEEP_TIME, World};
   use crate::body::{BodyDesc, BodyId};
   use crate::contact::{ContactPartner, ContactPhase};
   use crate::shape::Shape;
   use he3d_math::Vec3;

   fn free_fall_world() -> (World, BodyId) {
      let mut world = World::new();
      let ball =
         world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(0.0, 10.0, 0.0)));
      (world, ball)
   }

   /// A wide static slab whose top surface sits at `top`.
   fn ground(world: &mut World, top: f32) -> BodyId {
      let half = Vec3::new(20.0, 0.5, 20.0);
      world.add_body(BodyDesc::stationary(Shape::cuboid(half)).at(Vec3::new(0.0, top - 0.5, 0.0)))
   }

   fn settle(world: &mut World, seconds: f32) {
      let dt = 1.0 / 120.0;
      let steps = (seconds / dt) as u32;
      for _ in 0..steps {
         world.step(dt);
      }
   }

   #[test]
   fn gravity_accelerates_a_dynamic_body() {
      let (mut world, ball) = free_fall_world();
      world.step(0.5);
      let velocity = world.velocity(ball).expect("ball exists");
      assert!((velocity.y + 4.905).abs() < 1e-3, "got {velocity:?}");
   }

   #[test]
   fn a_static_body_never_moves() {
      let mut world = World::new();
      let slab = world.add_body(BodyDesc::stationary(Shape::cuboid(Vec3::splat(5.0))));
      for _ in 0..120 {
         world.step(1.0 / 60.0);
      }
      assert_eq!(world.position(slab), Some(Vec3::ZERO));
      assert_eq!(world.velocity(slab), Some(Vec3::ZERO));
   }

   #[test]
   fn a_kinematic_body_ignores_gravity_and_holds_its_velocity() {
      let mut world = World::new();
      let platform = world.add_body(BodyDesc::kinematic(Shape::cuboid(Vec3::splat(0.5))));
      assert!(world.set_velocity(platform, Vec3::new(2.0, 0.0, 0.0)));
      for _ in 0..120 {
         world.step(1.0 / 60.0);
      }
      let position = world.position(platform).expect("platform exists");
      assert!((position.x - 4.0).abs() < 1e-3, "got {position:?}");
      assert!(position.y.abs() < 1e-6, "got {position:?}");
   }

   #[test]
   fn a_kinematic_body_ignores_forces_and_keeps_its_speed() {
      let mut world = World::new();
      let platform = world.add_body(BodyDesc::kinematic(Shape::cuboid(Vec3::splat(0.5))));
      world.set_velocity(platform, Vec3::new(1.0, 0.0, 0.0));
      assert!(!world.apply_force(platform, Vec3::new(1000.0, 0.0, 0.0)));
      world.step(0.1);
      let velocity = world.velocity(platform).expect("platform exists");
      assert!((velocity.x - 1.0).abs() < 1e-6, "got {velocity:?}");
   }

   #[test]
   fn damping_decays_velocity_by_the_discrete_factor_each_step() {
      let mut world = World::new();
      world.set_gravity(Vec3::ZERO);
      let dt = 1.0 / 60.0;
      let damping = 2.0;
      let body = world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.5), 1.0)
            .with_velocity(Vec3::new(10.0, 0.0, 0.0))
            .with_damping(damping, 0.0),
      );
      let factor = 1.0 / (1.0 + damping * dt);
      let mut expected = 10.0;
      for _ in 0..120 {
         world.step(dt);
         expected *= factor;
         let speed = world.velocity(body).expect("body exists").x;
         assert!(speed > 0.0, "damping must not reverse the direction");
         assert!(
            (speed - expected).abs() < 1e-3,
            "got {speed}, expected {expected}"
         );
      }
   }

   #[test]
   fn a_zero_mass_dynamic_body_ignores_gravity() {
      let mut world = World::new();
      let ghost = world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 0.0).at(Vec3::Y));
      world.step(1.0);
      assert_eq!(world.position(ghost), Some(Vec3::Y));
   }

   #[test]
   fn removing_a_body_invalidates_its_handle_without_disturbing_others() {
      let mut world = World::new();
      let first = world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0));
      let second = world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0));
      assert!(world.remove_body(first));
      assert!(!world.contains(first));
      assert!(world.contains(second));
      assert!(!world.remove_body(first), "double removal must fail");

      let third = world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0));
      assert_eq!(third.index(), first.index());
      assert_ne!(third.generation(), first.generation());
      assert!(!world.contains(first));
   }

   #[test]
   fn a_degenerate_step_is_ignored() {
      let (mut world, ball) = free_fall_world();
      let before = world.position(ball).expect("ball exists");
      world.step(0.0);
      world.step(-1.0);
      world.step(f32::NAN);
      world.step(f32::INFINITY);
      assert_eq!(world.position(ball), Some(before));
   }

   #[test]
   fn a_force_acts_for_exactly_one_step() {
      let mut world = World::new();
      world.set_gravity(Vec3::ZERO);
      let body = world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0));
      assert!(world.apply_force(body, Vec3::new(0.0, 100.0, 0.0)));
      world.step(0.1);
      let after_push = world.velocity(body).expect("body exists").y;
      assert!((after_push - 10.0).abs() < 1e-3, "got {after_push}");
      world.step(0.1);
      let after_idle = world.velocity(body).expect("body exists").y;
      assert!(
         (after_idle - after_push).abs() < 1e-3,
         "the force must not persist: {after_idle}"
      );
   }

   #[test]
   fn a_spinning_body_keeps_a_unit_quaternion() {
      let mut world = World::new();
      world.set_gravity(Vec3::ZERO);
      let body = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
            .with_angular_velocity(Vec3::new(3.0, 1.0, 2.0)),
      );
      for _ in 0..2000 {
         world.step(1.0 / 120.0);
         let orientation = world.orientation(body).expect("body exists");
         assert!(
            (orientation.length() - 1.0).abs() < 1e-4,
            "quaternion drifted: {}",
            orientation.length()
         );
      }
   }

   #[test]
   fn ids_reports_every_live_body() {
      let mut world = World::new();
      let a = world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0));
      let b = world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0));
      let mut ids: alloc::vec::Vec<_> = world.ids().collect();
      ids.sort_unstable();
      assert_eq!(ids, alloc::vec![a.min(b), a.max(b)]);
      assert_eq!(world.len(), 2);
      assert!(!world.is_empty());
      world.remove_body(a);
      assert_eq!(world.len(), 1);
   }

   // ---------------------------------------------------------------- collision

   #[test]
   fn a_ball_lands_on_the_ground_and_stops() {
      let mut world = World::new();
      let slab = ground(&mut world, 0.0);
      let ball = world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.5), 1.0)
            .at(Vec3::new(0.0, 3.0, 0.0))
            .with_friction(0.6),
      );
      settle(&mut world, 3.0);

      let position = world.position(ball).expect("ball exists");
      assert!(
         (position.y - 0.5).abs() < 0.05,
         "the ball should rest on the slab, got {position:?}"
      );
      let speed = world.velocity(ball).expect("ball exists").length();
      assert!(speed < 0.1, "the ball should be at rest, got {speed}");
      assert_eq!(world.position(slab), Some(Vec3::new(0.0, -0.5, 0.0)));
   }

   #[test]
   fn a_ball_does_not_fall_through_the_ground() {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let ball = world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.5), 1.0)
            .at(Vec3::new(0.0, 3.0, 0.0))
            .with_velocity(Vec3::new(0.0, -20.0, 0.0)),
      );
      settle(&mut world, 2.0);
      let y = world.position(ball).expect("ball exists").y;
      assert!(y > -0.5, "the ball tunnelled through, got y={y}");
   }

   #[test]
   fn a_single_box_lands_flat_and_stops() {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let cube = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0).at(Vec3::new(0.0, 2.0, 0.0)),
      );
      settle(&mut world, 3.0);
      let position = world.position(cube).expect("cube exists");
      assert!(
         (position.y - 0.5).abs() < 0.05,
         "cube should rest at 0.5, got {position:?}"
      );
      assert!(
         world.velocity(cube).expect("cube exists").length() < 0.1,
         "cube should be at rest"
      );
   }

   #[test]
   fn a_stack_of_boxes_settles_without_sinking() {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let mut cubes = alloc::vec::Vec::new();
      for level in 0..5 {
         let y = 0.5 + level as f32 * 1.01;
         cubes.push(
            world.add_body(
               BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
                  .at(Vec3::new(0.0, y, 0.0))
                  .with_friction(0.9),
            ),
         );
      }
      settle(&mut world, 6.0);

      for (level, cube) in cubes.iter().enumerate() {
         let y = world.position(*cube).expect("cube exists").y;
         let expected = 0.5 + level as f32;
         assert!(
            (y - expected).abs() < 0.15,
            "level {level} should rest near {expected}, got {y}"
         );
      }
   }

   #[test]
   fn a_resting_stack_does_not_jitter() {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let mut cubes = alloc::vec::Vec::new();
      for level in 0..4 {
         cubes.push(
            world.add_body(
               BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
                  .at(Vec3::new(0.0, 0.5 + level as f32 * 1.01, 0.0))
                  .with_friction(0.9),
            ),
         );
      }
      settle(&mut world, 4.0);
      let before: alloc::vec::Vec<_> = cubes
         .iter()
         .map(|cube| world.position(*cube).expect("cube exists"))
         .collect();
      settle(&mut world, 2.0);
      for (index, cube) in cubes.iter().enumerate() {
         let after = world.position(*cube).expect("cube exists");
         assert!(
            (after - before[index]).length() < 0.02,
            "level {index} drifted from {:?} to {after:?}",
            before[index]
         );
      }
   }

   #[test]
   fn a_resting_stack_falls_asleep() {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let mut cubes = alloc::vec::Vec::new();
      for level in 0..3 {
         cubes.push(
            world.add_body(
               BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
                  .at(Vec3::new(0.0, 0.5 + level as f32 * 1.01, 0.0))
                  .with_friction(0.9),
            ),
         );
      }
      settle(&mut world, SLEEP_TIME + 6.0);
      for cube in &cubes {
         assert_eq!(
            world.is_sleeping(*cube),
            Some(true),
            "a settled stack should sleep"
         );
      }
   }

   #[test]
   fn a_sleeping_stack_wakes_when_a_neighbour_arrives() {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let cube = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
            .at(Vec3::new(0.0, 0.5, 0.0))
            .with_friction(0.9),
      );
      settle(&mut world, SLEEP_TIME + 4.0);
      assert_eq!(world.is_sleeping(cube), Some(true));

      let projectile = world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.4), 1.0)
            .at(Vec3::new(-3.0, 0.5, 0.0))
            .with_velocity(Vec3::new(12.0, 0.0, 0.0)),
      );
      settle(&mut world, 0.5);
      assert_eq!(
         world.is_sleeping(cube),
         Some(false),
         "the impact should have woken the settled cube"
      );

      // And the impact should have actually moved it.
      let moved = world.position(cube).expect("cube exists").x;
      assert!(moved.abs() > 1e-3, "the cube should have been pushed");
      let _ = projectile;
   }

   #[test]
   fn contacts_report_begin_then_persist_then_end() {
      let mut world = World::new();
      let slab = ground(&mut world, 0.0);
      world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(0.0, 1.2, 0.0)));

      // Fall until it touches: the first contact must be a Begin.
      let mut saw_begin = false;
      for _ in 0..240 {
         world.step(1.0 / 120.0);
         if world
            .contacts()
            .iter()
            .any(|contact| contact.phase == ContactPhase::Begin)
         {
            saw_begin = true;
            break;
         }
      }
      assert!(saw_begin, "landing must report a Begin phase");

      // Keep resting: the same contact must now be a Persist, not a new Begin.
      settle(&mut world, 0.5);
      assert!(
         world
            .contacts()
            .iter()
            .any(|contact| contact.phase == ContactPhase::Persist),
         "a resting contact must report Persist"
      );

      // Tear the ground away and the contact must end.
      assert!(world.remove_body(slab));
      world.step(1.0 / 120.0);
      assert!(
         world
            .contacts()
            .iter()
            .any(|contact| contact.phase == ContactPhase::End),
         "removing the ground must report an End"
      );
   }

   #[test]
   fn contacts_are_not_reported_for_bodies_that_are_apart() {
      let mut world = World::new();
      world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(0.0, 10.0, 0.0)));
      world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(50.0, 10.0, 0.0)));
      world.step(1.0 / 120.0);
      assert_eq!(world.contacts(), []);
   }

   #[test]
   fn two_static_bodies_are_never_solved_against_each_other() {
      let mut world = World::new();
      world.add_body(BodyDesc::stationary(Shape::cuboid(Vec3::splat(1.0))));
      world.add_body(
         BodyDesc::stationary(Shape::cuboid(Vec3::splat(1.0))).at(Vec3::new(0.5, 0.0, 0.0)),
      );
      world.step(1.0 / 120.0);
      assert_eq!(world.contacts(), []);
   }

   #[test]
   fn a_settled_tower_sleeps_and_then_does_not_move_at_all() {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let mut cubes = alloc::vec::Vec::new();
      for level in 0..4 {
         cubes.push(
            world.add_body(
               BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
                  .at(Vec3::new(0.0, 0.5 + level as f32 * 1.02, 0.0))
                  .with_friction(1.0),
            ),
         );
      }
      settle(&mut world, 8.0);
      for (index, cube) in cubes.iter().enumerate() {
         assert_eq!(
            world.is_sleeping(*cube),
            Some(true),
            "level {index} of a settled tower should sleep"
         );
         let y = world.position(*cube).expect("cube exists").y;
         assert!(
            (y - (0.5 + index as f32)).abs() < 0.1,
            "level {index} sits at {y}"
         );
      }

      // Once asleep nothing may move at all.
      // 入睡之后不应有任何移动。
      let resting: alloc::vec::Vec<_> = cubes
         .iter()
         .map(|cube| world.position(*cube).expect("cube exists"))
         .collect();
      settle(&mut world, 10.0);
      for (index, cube) in cubes.iter().enumerate() {
         let after = world.position(*cube).expect("cube exists");
         assert!(
            (after - resting[index]).length() < 1e-5,
            "level {index} moved from {:?} to {after:?} after sleeping",
            resting[index]
         );
      }
   }

   #[test]
   fn a_taller_tower_holds_its_height_without_sinking() {
      // Six boxes is the height this solver holds at the default iteration count.
      // Ten needs about twenty iterations and twenty is still unstable; see the tall-stack limitation in `docs/physics-spec.md`.
      // 六层是本求解器在默认迭代次数下能撑住的高度。
      // 十层约需二十次迭代，二十层仍不稳定；见 `docs/physics-spec.md` 中的高塔限制。
      let mut world = World::new();
      ground(&mut world, 0.0);
      let mut cubes = alloc::vec::Vec::new();
      for level in 0..6 {
         cubes.push(
            world.add_body(
               BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
                  .at(Vec3::new(0.0, 0.5 + level as f32 * 1.02, 0.0))
                  .with_friction(1.0),
            ),
         );
      }
      settle(&mut world, 12.0);
      for (index, cube) in cubes.iter().enumerate() {
         let y = world.position(*cube).expect("cube exists").y;
         let expected = 0.5 + index as f32;
         assert!(
            (y - expected).abs() < 0.1,
            "level {index} should rest near {expected}, got {y}"
         );
      }
   }

   // ------------------------------------------------------- static geometry

   /// A flat grid of triangles at `y = 0`, split the same way terrain tiles are: each cell becomes two triangles sharing a diagonal.
   /// `y = 0` 处的平坦三角形网格，按与地形瓦片相同的方式切分：每个格子分成两个共享一条对角线的三角形。
   fn flat_terrain_cells(start_x: usize, cells: usize, step: f32) -> alloc::vec::Vec<Vec3> {
      let mut positions = alloc::vec::Vec::new();
      for z in 0..cells {
         for x in start_x..(start_x + cells) {
            let x0 = x as f32 * step;
            let x1 = x0 + step;
            let z0 = z as f32 * step;
            let z1 = z0 + step;
            positions.extend([
               Vec3::new(x0, 0.0, z0),
               Vec3::new(x0, 0.0, z1),
               Vec3::new(x1, 0.0, z0),
               Vec3::new(x1, 0.0, z0),
               Vec3::new(x0, 0.0, z1),
               Vec3::new(x1, 0.0, z1),
            ]);
         }
      }
      positions
   }

   fn flat_terrain(cells: usize, step: f32) -> alloc::vec::Vec<Vec3> {
      flat_terrain_cells(0, cells, step)
   }

   #[test]
   fn a_body_rests_on_static_terrain() {
      let mut world = World::new();
      world
         .add_static_mesh(&flat_terrain(2, 2.0))
         .expect("terrain");
      let ball =
         world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(1.0, 3.0, 1.0)));
      settle(&mut world, 3.0);
      let y = world.position(ball).expect("ball exists").y;
      assert!(
         (y - 0.5).abs() < 0.1,
         "the ball should rest on the terrain, got y={y}"
      );
   }

   #[test]
   fn a_contact_with_static_terrain_names_the_mesh() {
      let mut world = World::new();
      let mesh = world
         .add_static_mesh(&flat_terrain(1, 2.0))
         .expect("terrain");
      world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(1.0, 0.4, 1.0)));
      settle(&mut world, 0.2);
      assert!(
         world
            .contacts()
            .iter()
            .any(|contact| contact.b == ContactPartner::Static(mesh)),
         "the terrain contact must name the mesh"
      );
   }

   #[test]
   fn a_box_slides_across_triangle_seams_without_catching() {
      let mut world = World::new();
      world
         .add_static_mesh(&flat_terrain(4, 2.0))
         .expect("terrain");
      let cube = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
            .at(Vec3::new(0.5, 0.55, 0.5))
            .with_friction(0.0)
            .with_velocity(Vec3::new(4.0, 0.0, 0.0)),
      );
      settle(&mut world, 1.0);
      let position = world.position(cube).expect("cube exists");
      // Frictionless travel would be four units.
      // Without internal-edge filtering the diagonals would trip the box and eat most of that.
      // 无摩擦滑行应前进四个单位。
      // 若没有内部边过滤，对角线会绊住方块并吃掉大部分位移。
      assert!(
         position.x > 3.5,
         "the box caught on a triangle seam, ended at {position:?}"
      );
      assert!(
         (position.y - 0.5).abs() < 0.1,
         "the box sank or floated, ended at {position:?}"
      );
   }

   #[test]
   fn a_box_slides_across_a_seam_between_two_separate_meshes() {
      let mut world = World::new();
      // Two tiles that meet exactly, the way streamed terrain does.
      // Neither tile contains the shared boundary edge, so only the shared topology can tell that it is an Internal edge.
      // 两块精确相接的瓦片，正如流式地形。
      // 共享的边界边不在任何单块瓦片内部，因此只有共享拓扑才能判断它是 Internal edge。
      world
         .add_static_mesh(&flat_terrain_cells(0, 2, 2.0))
         .expect("left tile");
      world
         .add_static_mesh(&flat_terrain_cells(2, 2, 2.0))
         .expect("right tile");
      let cube = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
            .at(Vec3::new(0.5, 0.55, 0.5))
            .with_friction(0.0)
            .with_velocity(Vec3::new(4.0, 0.0, 0.0)),
      );
      settle(&mut world, 1.5);
      let position = world.position(cube).expect("cube exists");
      assert!(
         position.x > 5.0,
         "the box caught on the seam between two meshes, ended at {position:?}"
      );
      assert!(
         (position.y - 0.5).abs() < 0.1,
         "the box sank or floated, ended at {position:?}"
      );
   }

   #[test]
   fn removing_static_geometry_lets_bodies_fall_through() {
      let mut world = World::new();
      let mesh = world
         .add_static_mesh(&flat_terrain(2, 2.0))
         .expect("terrain");
      let ball =
         world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(1.0, 0.5, 1.0)));
      settle(&mut world, 1.0);
      assert!((world.position(ball).expect("ball").y - 0.5).abs() < 0.1);

      assert!(world.remove_static_mesh(mesh));
      assert!(!world.contains_static_mesh(mesh));
      assert_eq!(world.static_mesh_count(), 0);
      assert!(!world.remove_static_mesh(mesh), "double removal must fail");
      settle(&mut world, 1.0);
      assert!(
         world.position(ball).expect("ball").y < -1.0,
         "the ball should fall once its terrain is gone"
      );
   }

   #[test]
   fn removing_one_mesh_releases_only_its_own_edges() {
      let mut world = World::new();
      world
         .add_static_mesh(&flat_terrain_cells(0, 2, 2.0))
         .expect("left tile");
      let right = world
         .add_static_mesh(&flat_terrain_cells(1, 2, 2.0))
         .expect("right tile");
      // The two tiles overlap along x = 2, so that boundary is internal while both exist: a Body sliding over it must not catch.
      // 两块瓦片在 x = 2 处重叠，因此只要两者都存在，该边界就是内部边：滑过它的 Body 不会被绊住。
      let cube = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
            .at(Vec3::new(0.5, 0.55, 0.5))
            .with_friction(0.0)
            .with_velocity(Vec3::new(4.0, 0.0, 0.0)),
      );
      settle(&mut world, 0.6);
      assert!(
         world.position(cube).expect("cube").x > 2.0,
         "the box should have crossed the shared boundary"
      );

      // Dropping the second tile must not corrupt the first one's bookkeeping.
      // 移除第二块瓦片不应破坏第一块的簿记。
      assert!(world.remove_static_mesh(right));
      settle(&mut world, 0.6);
      assert!(world.position(cube).expect("cube").is_finite());
   }

   #[test]
   fn malformed_static_geometry_is_rejected() {
      let mut world = World::new();
      assert!(world.add_static_mesh(&[]).is_none(), "empty input");
      assert!(
         world.add_static_mesh(&[Vec3::ZERO, Vec3::X]).is_none(),
         "not a whole number of triangles"
      );
      assert!(
         world
            .add_static_mesh(&[Vec3::ZERO, Vec3::X, Vec3::new(f32::NAN, 0.0, 0.0)])
            .is_none(),
         "a non-finite coordinate"
      );
      assert_eq!(world.static_mesh_count(), 0);
   }

   // --------------------------------------------------------------- ledges

   /// Settle a block on a ledge with the given fraction of its base hanging off, returning its final position, its tilt in degrees, and whether it slept.
   /// 让方块以给定比例的底面悬空搁在台沿上，返回其最终位置、倾斜角度（度），以及是否入睡。
   fn ledge_block(overhang: f32, platform_half: Vec3, block_half: f32) -> (Vec3, f32, bool) {
      let mut world = World::new();
      world.add_body(BodyDesc::stationary(Shape::cuboid(platform_half)).at(Vec3::ZERO));
      // The base spans `[c - half, c + half]` and the ledge edge is at `platform_half.x`, so the fraction hanging off solves to this centre.
      // 底面覆盖 `[c - half, c + half]`，台沿位于 `platform_half.x`，因此悬空比例对应的中心位置如下。
      let centre_x = platform_half.x - block_half + overhang * block_half * 2.0;
      let block = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(block_half)), 1.0)
            .at(Vec3::new(
               centre_x,
               platform_half.y + block_half + 0.02,
               0.0,
            ))
            .with_friction(0.6),
      );
      for _ in 0..600 {
         world.step(1.0 / 120.0);
      }
      let position = world.position(block).expect("block exists");
      let up = world.orientation(block).expect("block exists") * Vec3::Y;
      let tilt = up.dot(Vec3::Y).clamp(-1.0, 1.0).acos().to_degrees();
      (position, tilt, world.is_sleeping(block) == Some(true))
   }

   #[test]
   fn a_block_tips_off_once_its_centre_passes_the_edge() {
      // A block on a ledge is supported while its centre of mass is over the ledge and tips off once it is not.
      // The threshold is exactly the platform edge, and the block's centre sitting on that edge is half of its base, so this pins both sides of it.
      // 搁在台沿上的方块，质心还在台上时受支撑，一旦越过台沿就倾覆。
      // 阈值恰好是台沿，而质心落在台沿上正是底面的一半，因此这个测试把两侧都钉住。
      let platform_half = Vec3::new(3.0, 0.5, 3.0);
      let block_half = 0.5;
      let resting_height = platform_half.y + block_half;

      let mut worst_tilt = 0.0_f32;
      for overhang in [0.1_f32, 0.2, 0.3, 0.4] {
         let (position, tilt, sleeping) = ledge_block(overhang, platform_half, block_half);
         assert!(
            (position.y - resting_height).abs() < 0.02,
            "{overhang} overhang should rest on the ledge, got y={}",
            position.y
         );
         assert!(sleeping, "{overhang} overhang should settle and sleep");
         worst_tilt = worst_tilt.max(tilt);
      }
      // Resting on a lip must not mean propped up at an angle by an asymmetric contact.
      // 搁在台沿上不应意味着被不对称接触顶成斜的。
      assert!(
         worst_tilt < 0.01,
         "a block resting on a ledge must stay level, it tilted {worst_tilt} deg"
      );

      for overhang in [0.6_f32, 0.7, 0.8, 0.9] {
         let (position, _, _) = ledge_block(overhang, platform_half, block_half);
         assert!(
            position.y < 0.0,
            "{overhang} overhang is past the centre of mass and must fall, got y={}",
            position.y
         );
      }
   }

   /// Tip a resting cube onto the next face with `impulse` about z, and report how many steps it spent pivoting on an edge and how fast it twisted about x while it did.
   ///
   /// The pivot's contact points sit on the edge itself, so the correct motion is a pure rotation about z and the contact points do not slide along it.
   /// Anything the cube picks up about x is the solver's, not the physics'.
   /// 用绕 z 的 `impulse` 把一个静止的方块掀到下一个面上，并报告它花了几步在棱上翻转、以及翻转期间绕 x 转得多快。
   /// 翻转的接触点就落在那条棱上，因此正确的运动是纯绕 z 旋转、接触点不沿棱滑动。
   /// 方块绕 x 拿到的任何东西都来自求解器，而不是物理。
   fn edge_pivot_twist(impulse: f32) -> (u32, f32) {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let cube = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
            .at(Vec3::new(0.0, 0.5, 0.0))
            .with_friction(1.0),
      );
      settle(&mut world, 2.0);
      world.apply_angular_impulse(cube, Vec3::new(0.0, 0.0, impulse));
      let mut edge_steps = 0;
      let mut worst_twist = 0.0_f32;
      for _ in 0..2400 {
         world.step(1.0 / 120.0);
         let touching = world
            .contacts()
            .iter()
            .filter(|contact| contact.involves(cube))
            .count();
         if touching == 2 {
            edge_steps += 1;
            let spin = world.angular_velocity(cube).expect("the cube exists");
            worst_twist = worst_twist.max(spin.x.abs());
         }
      }
      (edge_steps, worst_twist)
   }

   #[test]
   fn a_cube_pivoting_on_its_edge_does_not_twist_about_the_edge() {
      // Two contact points apart along an axis share a couple about it: solving them one after the other leaves them unequal, and their difference is a torque nothing applied.
      // A cube rolled onto its edge is exactly this case, and the twist it picks up is rotation about the axis it is rolling over.
      // 沿某条轴分开的两个接触点共享一个绕该轴的力偶：逐个求解会让它们不等，而它们的差就是一个无人施加的力矩。
      // 滚到棱上的方块正是这种情形，它拿到的扭转就是绕滚动轴的旋转。
      //
      // The two points are solved as one block, so the couple is gone at the default iteration count; measured, 4.2e-7 rad/s against 2.0e-4 rad/s without it, and better than raising the count to 24, which costs 65% more on a contact-dense step.
      // Four-point manifolds cannot be blocked — theirs is singular — so what is left after this is the residual of a face landing, which is a separate and already documented limit.
      // 两点作为一个块求解，因此默认迭代次数下该力偶就不存在了；实测 4.2e-7 rad/s，而没有它时是 2.0e-4 rad/s，并且优于把迭代次数提到 24——后者在接触密集的一步上要多花 65%。
      // 四点流形无法成块——它的块是奇异的——因此这里剩下的是面着地时的残差，那是一个独立且已有记录的局限。
      let (edge_steps, twist) = edge_pivot_twist(0.6);
      assert!(
         edge_steps > 10,
         "the cube must actually pivot on an edge, it did for {edge_steps} steps"
      );
      assert!(
         twist < 1e-5,
         "the cube twisted {twist} rad/s about the edge it was pivoting on"
      );
   }

   #[test]
   fn a_block_sliding_towards_a_ledge_leaves_it_instead_of_hanging() {
      // The realistic version of the same question: a block that slides off a ledge has to actually leave it once its centre goes past the edge, rather than being held up by a contact on the lip.
      // 同一个问题的现实版本：滑向台沿的方块，质心越过台沿后必须真的离开，而不是被台沿上的接触顶住悬挂。
      //
      // Both surfaces are nearly frictionless, so the block still carries real speed by the time it reaches the lip.
      // With ordinary friction it coasts to a halt just short of the edge, and the test would be measuring friction rather than the ledge.
      // 两个表面都近乎无摩擦，因此方块抵达台沿时仍有可观速度。
      // 用普通摩擦系数它会在离边缘一步之遥处停下，那时测的就是摩擦而不是台沿了。
      let mut world = World::new();
      world.add_body(
         BodyDesc::stationary(Shape::cuboid(Vec3::new(3.0, 0.5, 3.0)))
            .at(Vec3::ZERO)
            .with_friction(0.02),
      );
      let block = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
            .at(Vec3::new(0.0, 1.02, 0.0))
            .with_friction(0.02)
            .with_velocity(Vec3::new(3.0, 0.0, 0.0)),
      );

      let mut left_the_ledge = false;
      for _ in 0..600 {
         world.step(1.0 / 120.0);
         let position = world.position(block).expect("block exists");
         if position.x > 3.0 && position.y < 0.0 {
            left_the_ledge = true;
            break;
         }
      }
      assert!(
         left_the_ledge,
         "the block should have slid off the ledge, ended at {:?}",
         world.position(block).expect("block exists")
      );
   }

   /// Settle a plank across a gap between two platforms, returning its final height, its tilt in degrees, and whether it slept.
   /// 让一块板横跨两个平台之间的缝隙，返回其最终高度、倾斜角度（度），以及是否入睡。
   fn spanning_plank(gap: f32, plank_half: f32, platform_half: f32) -> (f32, f32, bool) {
      let mut world = World::new();
      let offset = gap / 2.0 + platform_half;
      for side in [-1.0_f32, 1.0] {
         world.add_body(
            BodyDesc::stationary(Shape::cuboid(Vec3::new(platform_half, 0.5, 3.0))).at(Vec3::new(
               side * offset,
               0.0,
               0.0,
            )),
         );
      }
      let plank = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::new(plank_half, 0.25, 0.5)), 1.0)
            .at(Vec3::new(0.0, 0.5 + 0.25 + 0.02, 0.0))
            .with_friction(0.8),
      );
      for _ in 0..600 {
         world.step(1.0 / 120.0);
      }
      let y = world.position(plank).expect("plank exists").y;
      let up = world.orientation(plank).expect("plank exists") * Vec3::Y;
      let tilt = up.dot(Vec3::Y).clamp(-1.0, 1.0).acos().to_degrees();
      (y, tilt, world.is_sleeping(plank) == Some(true))
   }

   #[test]
   fn a_plank_spanning_a_gap_rests_level_until_the_gap_outgrows_it() {
      // Two supports at once, which is where a solver that mishandles several simultaneous contacts shows up: the plank sags or tips instead of sharing load.
      // Even with only a sliver of overlap on each side it must stay level.
      // 同时有两个支撑，这正是处理不好多接触的求解器会露馅的地方：板会下垂或倾斜，而不是分担载荷。
      // 即使每侧只剩一丝重叠，它也必须保持水平。
      let plank_half = 0.9;
      let platform_half = 1.5;
      let resting = 0.5 + 0.25;
      for gap in [0.4_f32, 0.8, 1.2, 1.6, 1.7] {
         let (y, tilt, sleeping) = spanning_plank(gap, plank_half, platform_half);
         assert!(
            (y - resting).abs() < 0.02,
            "gap {gap} should rest at {resting}, got y={y}"
         );
         assert!(tilt < 0.01, "gap {gap} tilted the plank by {tilt} deg");
         assert!(sleeping, "gap {gap} should settle and sleep");
      }
      // At twice the half-width there is no overlap left to stand on.
      // 缝隙达到半宽的两倍时，已无任何可站立的重叠。
      let (y, _, _) = spanning_plank(1.9, plank_half, platform_half);
      assert!(
         y < 0.0,
         "a gap wider than the plank must let it through, got y={y}"
      );
   }

   #[test]
   fn a_sphere_rests_on_a_ledge_up_to_the_edge_and_falls_past_it() {
      // A sphere touches a flat top at exactly the point below its centre, so the same centre-past-the-edge rule applies — through a different narrowphase path than a box takes.
      // 球与平顶的接触点恰在其球心正下方，因此"质心越过边缘"这条规则同样适用，只是走的窄相位路径与盒不同。
      let radius = 0.4;
      let resting = 0.5 + radius;
      for centre_x in [2.0_f32, 2.6, 2.99] {
         let mut world = World::new();
         world
            .add_body(BodyDesc::stationary(Shape::cuboid(Vec3::new(3.0, 0.5, 3.0))).at(Vec3::ZERO));
         let ball = world.add_body(
            BodyDesc::dynamic(Shape::sphere(radius), 1.0)
               .at(Vec3::new(centre_x, resting + 0.02, 0.0))
               .with_friction(0.6),
         );
         for _ in 0..600 {
            world.step(1.0 / 120.0);
         }
         let y = world.position(ball).expect("ball exists").y;
         assert!(
            (y - resting).abs() < 0.02,
            "centre at {centre_x} is over the ledge and should rest, got y={y}"
         );
         assert_eq!(world.is_sleeping(ball), Some(true));
      }

      let mut world = World::new();
      world.add_body(BodyDesc::stationary(Shape::cuboid(Vec3::new(3.0, 0.5, 3.0))).at(Vec3::ZERO));
      let ball = world.add_body(
         BodyDesc::dynamic(Shape::sphere(radius), 1.0)
            .at(Vec3::new(3.01, resting + 0.02, 0.0))
            .with_friction(0.6),
      );
      for _ in 0..600 {
         world.step(1.0 / 120.0);
      }
      assert!(
         world.position(ball).expect("ball exists").y < 0.0,
         "a sphere whose centre is past the edge must fall"
      );
   }

   #[test]
   fn a_capsule_comes_to_rest_on_static_geometry() {
      // The capsule stands on its bottom cap, so the world reaches it through the capsule-versus-triangle path.
      // 胶囊以底帽站立，因此 World 通过胶囊-三角形路径访问它。
      let mut world = World::new();
      let mesh = [
         Vec3::new(-5.0, 0.0, -5.0),
         Vec3::new(5.0, 0.0, -5.0),
         Vec3::new(5.0, 0.0, 5.0),
         Vec3::new(-5.0, 0.0, -5.0),
         Vec3::new(5.0, 0.0, 5.0),
         Vec3::new(-5.0, 0.0, 5.0),
      ];
      world.add_static_mesh(&mesh).expect("a flat quad");
      let pill = world.add_body(
         BodyDesc::dynamic(Shape::capsule(1.0, 0.5), 1.0)
            .at(Vec3::new(0.0, 2.5, 0.0))
            .with_friction(0.6),
      );
      for _ in 0..600 {
         world.step(1.0 / 120.0);
      }
      let y = world.position(pill).expect("pill exists").y;
      // The ground is at 0, and the cap adds half_height + radius below the centre.
      assert!((y - 1.5).abs() < 0.03, "got y={y}");
      assert_eq!(world.is_sleeping(pill), Some(true));
   }

   #[test]
   fn a_sphere_rolling_across_mesh_seams_keeps_its_speed() {
      // A sphere's contact normal always runs from the closest point of the triangle to its centre, so for coplanar triangles it is the plane normal whichever feature that closest point lands on.
      // There is therefore no spurious normal to filter, and a seam between two separate meshes is as smooth as the interior of one triangle — unlike a box, whose manifold can be generated from a triangle edge axis.
      // 球的接触法线始终由三角形上的最近点指向球心，因此对共面三角形而言，无论最近点落在哪个特征上，法线都是平面法线。
      // 因此不存在需要过滤的伪法线，两块独立网格之间的接缝与单个三角形内部一样平滑——这与盒不同，盒的流形可能由三角形棱轴生成。
      let mut world = World::new();
      world
         .add_static_mesh(&flat_terrain_cells(0, 2, 2.0))
         .expect("left tile");
      world
         .add_static_mesh(&flat_terrain_cells(2, 2, 2.0))
         .expect("right tile");
      let ball = world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.4), 1.0)
            .at(Vec3::new(0.5, 0.4005, 0.5))
            .with_friction(0.0)
            .with_velocity(Vec3::new(4.0, 0.0, 0.0)),
      );

      let mut lowest = f32::MAX;
      for _ in 0..180 {
         world.step(1.0 / 120.0);
         lowest = lowest.min(world.position(ball).expect("ball exists").y);
      }
      let velocity = world.velocity(ball).expect("ball exists");
      assert!(
         (velocity.x - 4.0).abs() < 0.05,
         "the seam stole horizontal speed, vx={}",
         velocity.x
      );
      assert!(
         lowest > 0.39,
         "the ball dipped into the seam, lowest y={lowest}"
      );
   }

   /// Slide a box diagonally off the corner of a platform, along `x = z`.
   ///
   /// The supported region is the intersection of the box's base with the platform, which near the threshold is a tiny square out at the platform's corner.
   /// 让方块沿 `x = z` 斜向滑出平台的角。
   /// 支撑区是底面与平台的交集，在阈值附近它是平台角落处的一小块方形。
   fn corner_slide(centre: f32) -> (Vec3, f32) {
      let mut world = World::new();
      world.add_body(BodyDesc::stationary(Shape::cuboid(Vec3::new(3.0, 0.5, 3.0))).at(Vec3::ZERO));
      let block = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::new(0.5, 0.25, 0.5)), 1.0)
            .at(Vec3::new(centre, 0.5 + 0.25 + 0.02, centre))
            .with_friction(0.7),
      );
      for _ in 0..900 {
         world.step(1.0 / 120.0);
      }
      let position = world.position(block).expect("block exists");
      let up = world.orientation(block).expect("block exists") * Vec3::Y;
      (
         position,
         up.dot(Vec3::Y).clamp(-1.0, 1.0).acos().to_degrees(),
      )
   }

   #[test]
   fn a_box_leaves_a_platform_corner_once_its_centre_passes_it() {
      // The diagonal version of the ledge test.
      // Two differences make it harder: the supported region is a rectangle rather than a half-plane, so the tipping direction is diagonal in the body's own frame; and just below the threshold that region shrinks to a 0.1 x 0.1 patch at the platform's corner.
      //
      // The rule is the same one: the box stands while its centre is inside the supported region and tips once it is not.
      // The corner of the platform is at x = z = 3, so the centre crossing that line is exactly the threshold.
      // 台沿测试的对角版本。
      // 两点让它更难：支撑区是矩形而非半平面，因此倾覆方向在刚体自身坐标系里是对角线；而就在阈值之下，该区域会收缩到平台角落处 0.1 x 0.1 的一小块。
      //
      // 规则相同：质心位于支撑区内时方块站立，不在时倾覆。
      // 平台的角在 x = z = 3，因此质心越过这条线正是阈值。
      let resting = 0.5 + 0.25;
      for centre in [1.0_f32, 2.0, 2.5, 2.9, 2.99] {
         let (position, tilt) = corner_slide(centre);
         assert!(
            (position.y - resting).abs() < 0.02,
            "centre {centre} is over the platform and should rest, got y={}",
            position.y
         );
         assert!(
            tilt < 1.0,
            "centre {centre} should stay level, tilted {tilt} deg"
         );
      }

      for centre in [3.2_f32, 3.4, 4.0] {
         let (position, _) = corner_slide(centre);
         assert!(
            position.y < 0.0,
            "centre {centre} is past the platform corner and must fall, got y={}",
            position.y
         );
      }
   }

   #[test]
   fn a_stack_pushed_over_a_ledge_leaves_no_body_hanging() {
      // A contact on the lip has to turn into a torque, not into permanent support.
      // The failure this guards against is a Body left hovering beside the ledge.
      //
      // Hovering means: still at platform height, but with its centre outside the platform.
      // A Body could in principle be cantilevered there legitimately, but only by being balanced — and a rigid one balanced that far out has already broken the tipping rule, so it counts as the same failure.
      // 台沿上的接触必须转化为力矩，而不是永久支撑。
      // 这里防的失效是某个 Body 悬在台沿旁边。
      //
      // 悬空指：仍处于平台高度，但质心已在平台之外。
      // 原则上 Body 可以被合法地悬挑在那里，但那要求它处于平衡——而刚体在那么外面还平衡得住，本身就已经违反了倾覆规则，因此归为同一种失效。
      for friction in [0.05_f32, 0.8] {
         let mut world = World::new();
         world.add_body(
            BodyDesc::stationary(Shape::cuboid(Vec3::new(4.0, 0.5, 3.0)))
               .at(Vec3::ZERO)
               .with_friction(friction),
         );
         let mut stack = alloc::vec::Vec::new();
         for level in 0..4 {
            stack.push(
               world.add_body(
                  BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
                     .at(Vec3::new(2.0, 0.5 + level as f32 * 1.01, 0.0))
                     .with_friction(friction)
                     .with_velocity(Vec3::new(2.0, 0.0, 0.0)),
               ),
            );
         }
         settle(&mut world, 10.0);

         let mut left_the_platform = 0;
         for (level, body) in stack.iter().enumerate() {
            let position = world.position(*body).expect("body exists");
            if position.y < 0.0 {
               left_the_platform += 1;
            }
            assert!(
               position.y < 0.0 || position.x.abs() <= 4.0,
               "friction {friction}: level {level} is hanging past the ledge at {position:?}"
            );
         }
         assert!(
            left_the_platform > 0,
            "friction {friction}: the stack never reached the ledge, so nothing was tested"
         );
      }
   }

   /// The outcome of dropping a box onto the overhanging half of a dynamic slab.
   /// 把一个方块放到动态板悬挑的一半上之后的结果。
   struct SlabOutcome {
      slab_y: f32,
      slab_tilt: f32,
      box_y: f32,
      box_slept: bool,
   }

   /// A dynamic slab centred at x = -0.5 rests on a static ledge that ends at x = 0, so it overhangs by 1.5.
   /// A box sits at `box_x` on that overhang.
   ///
   /// Both bodies have finite mass and both can rotate, so whether the assembly stands is decided by their **combined** centre of mass: `(1 * -0.5 + box_mass * box_x) / (1 + box_mass)`.
   /// 一块中心在 x = -0.5 的动态板搁在终止于 x = 0 的静态台沿上，因此悬挑 1.5。
   /// 方块位于悬挑上的 `box_x` 处。
   ///
   /// 两个刚体都有有限质量、都能转动，因此整个组合能否站立由其**联合质心**决定：`(1 * -0.5 + box_mass * box_x) / (1 + box_mass)`。
   fn loaded_slab(box_x: f32, box_mass: f32, friction: f32) -> SlabOutcome {
      let mut world = World::new();
      world.add_body(
         BodyDesc::stationary(Shape::cuboid(Vec3::new(2.0, 0.5, 3.0)))
            .at(Vec3::new(-2.0, -0.5, 0.0))
            .with_friction(friction),
      );
      let slab = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::new(2.0, 0.1, 1.0)), 1.0)
            .at(Vec3::new(-0.5, 0.1 + 0.02, 0.0))
            .with_friction(friction),
      );
      let box_body = world.add_body(
         BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.4)), box_mass)
            .at(Vec3::new(box_x, 0.2 + 0.4 + 0.04, 0.0))
            .with_friction(friction),
      );
      settle(&mut world, 10.0);

      let slab_position = world.position(slab).expect("slab exists");
      let box_position = world.position(box_body).expect("box exists");
      SlabOutcome {
         slab_y: slab_position.y,
         slab_tilt: {
            let up = world.orientation(slab).expect("slab exists") * Vec3::Y;
            up.dot(Vec3::Y).clamp(-1.0, 1.0).acos().to_degrees()
         },
         box_y: box_position.y,
         box_slept: world.is_sleeping(box_body) == Some(true),
      }
   }

   #[test]
   fn a_loaded_dynamic_slab_tips_by_its_combined_centre_of_mass() {
      // Dynamic against dynamic, and the hardest of the edge cases: the platform itself has finite mass and can rotate, so the box's weight and the slab's own weight together decide whether the assembly stands.
      // Getting this right needs torque to cross the contact in both directions — the box must be able to push the slab over, and the slab must be able to carry the box.
      // 动态对动态，也是这些边缘情形里最难的：平台自身有有限质量、可以转动，因此方块的重量与板自身的重量共同决定组合能否站立。
      // 要做对这件事，力矩必须能在接触处双向传递——方块要能压翻板，板也要能带住方块。
      //
      // The static ledge ends at x = 0, so the assembly tips exactly when its combined centre of mass passes it.
      // 静态台沿终止于 x = 0，因此当联合质心越过它时组合恰好倾覆。
      let combined = |box_x: f32, mass: f32| (1.0 * -0.5 + mass * box_x) / (1.0 + mass);

      let stable = [
         (-0.5_f32, 1.0_f32),
         (0.0, 1.0),
         (0.4, 1.0),
         (0.0, 3.0),
         (0.1, 3.0),
      ];
      for (box_x, mass) in stable {
         let centre = combined(box_x, mass);
         assert!(centre < 0.0, "the stable case must be inside the ledge");
         let outcome = loaded_slab(box_x, mass, 0.6);
         assert!(
            outcome.slab_y > -0.5,
            "combined centre {centre} is over the ledge, so the slab should stand; got slab y={}",
            outcome.slab_y
         );
         assert!(
            outcome.box_y > 0.0,
            "the box rode a standing slab, so it should still be up; got y={}",
            outcome.box_y
         );
         assert!(
            outcome.slab_tilt < 2.0,
            "a standing slab should stay level, tilted {} deg",
            outcome.slab_tilt
         );
         // With the load comfortably inside the ledge, the whole assembly settles and the box is allowed to sleep.
         // 当载荷稳稳位于台沿之内时，整个组合会稳定下来，方块也可以入睡。
         if centre < -0.2 {
            assert!(
               outcome.box_slept,
               "a box riding a settled slab should sleep, centre was {centre}"
            );
         }
      }

      let tipping = [(0.6_f32, 1.0_f32), (1.0, 1.0), (1.4, 1.0), (0.3, 3.0)];
      for (box_x, mass) in tipping {
         let centre = combined(box_x, mass);
         assert!(centre > 0.0, "the tipping case must be past the ledge");
         let outcome = loaded_slab(box_x, mass, 0.6);
         assert!(
            outcome.slab_y < -1.0,
            "combined centre {centre} is past the ledge, so the slab should go over; got slab y={}",
            outcome.slab_y
         );
         assert!(
            outcome.slab_tilt > 5.0,
            "the slab should have rotated rather than slid off, tilted only {} deg",
            outcome.slab_tilt
         );
         // And it must not be held up anywhere near the ledge.
         assert!(
            outcome.box_y < 0.0,
            "the box should have followed the slab down, got y={}",
            outcome.box_y
         );
      }
   }

   // -------------------------------------------------------------- gravity

   #[test]
   fn gravity_scale_decides_how_much_gravity_reaches_a_body() {
      // A Body whose motion something else owns opts out with a scale of zero.
      // The default stays full gravity, and the scale multiplies rather than replaces.
      // 运动由别的东西拥有的 Body 用零系数退出。
      // 默认仍是完整重力，而系数是相乘而非替换。
      let mut world = World::new();
      let normal =
         world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(0.0, 10.0, 0.0)));
      let weightless = world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.5), 1.0)
            .at(Vec3::new(10.0, 10.0, 0.0))
            .with_gravity_scale(0.0),
      );
      let heavy = world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.5), 1.0)
            .at(Vec3::new(20.0, 10.0, 0.0))
            .with_gravity_scale(2.0),
      );

      world.step(1.0 / 120.0);
      let vertical = |body| world.velocity(body).expect("body exists").y;
      let one_step = DEFAULT_GRAVITY.y / 120.0;

      assert!(
         vertical(weightless).abs() < 1e-6,
         "a zero scale must ignore gravity entirely, got {}",
         vertical(weightless)
      );
      assert!(
         (vertical(normal) - one_step).abs() < 1e-4,
         "the default must be full gravity, got {}",
         vertical(normal)
      );
      assert!(
         (vertical(heavy) - 2.0 * one_step).abs() < 1e-4,
         "the scale must multiply, got {}",
         vertical(heavy)
      );
   }

   #[test]
   fn gravity_scale_can_be_changed_on_a_live_body() {
      // The case the setter exists for: something owns the Body's motion for a while and then hands it back.
      // It also has to wake the Body, because a Body asleep under no gravity is not necessarily asleep under the new one.
      // 该 setter 存在的理由：某个东西在一段时间内拥有此 Body 的运动，随后交还回去。
      // 它还必须唤醒该 Body，因为在无重力下睡着的 Body 在新重力下未必还该睡。
      let mut world = World::new();
      let body =
         world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(Vec3::new(0.0, 10.0, 0.0)));

      assert!(world.set_gravity_scale(body, 0.0));
      for _ in 0..600 {
         world.step(1.0 / 120.0);
      }
      let held = world.position(body).expect("body exists").y;
      assert!(
         (held - 10.0).abs() < 1e-3,
         "a zero scale must hover, got y={held}"
      );
      assert_eq!(world.is_sleeping(body), Some(true), "it should have slept");

      assert!(world.set_gravity_scale(body, 1.0));
      assert_eq!(
         world.is_sleeping(body),
         Some(false),
         "changing the scale must wake the Body"
      );
      world.step(1.0 / 120.0);
      assert!(
         world.velocity(body).expect("body exists").y < 0.0,
         "gravity must reach it again"
      );

      let dead = world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0));
      assert!(world.remove_body(dead));
      assert!(
         !world.set_gravity_scale(dead, 1.0),
         "a dead handle must be rejected"
      );
   }

   #[test]
   fn a_negative_or_non_finite_gravity_scale_is_clamped_to_zero() {
      // Anti-gravity is not a thing this engine offers, and a NaN scale would put a NaN into every velocity it touched.
      // 反重力不是本引擎提供的东西，而 NaN 系数会把它碰到的每一个速度都变成 NaN。
      let mut world = World::new();
      let negative = world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.5), 1.0)
            .at(Vec3::new(0.0, 10.0, 0.0))
            .with_gravity_scale(-1.0),
      );
      let nan = world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.5), 1.0)
            .at(Vec3::new(10.0, 10.0, 0.0))
            .with_gravity_scale(f32::NAN),
      );

      world.step(1.0 / 120.0);
      for body in [negative, nan] {
         let velocity = world.velocity(body).expect("body exists");
         assert!(
            velocity.is_finite() && velocity.y.abs() < 1e-6,
            "expected no gravity, got {velocity:?}"
         );
      }
   }

   // ------------------------------------------------------------- islands

   #[test]
   fn disturbing_one_member_wakes_the_whole_island() {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let mut cubes = alloc::vec::Vec::new();
      for level in 0..4 {
         cubes.push(
            world.add_body(
               BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
                  .at(Vec3::new(0.0, 0.5 + level as f32 * 1.01, 0.0))
                  .with_friction(0.9),
            ),
         );
      }
      settle(&mut world, 8.0);
      for cube in &cubes {
         assert_eq!(
            world.is_sleeping(*cube),
            Some(true),
            "the stack should be asleep before it is disturbed"
         );
      }

      // Disturb the bottom box only.
      // Sleeping per Body would leave the rest of the stack frozen in mid-air; the Island has to come back as a unit.
      // 只扰动最下面的方块。
      // 若按 Body 独立睡眠，堆叠的其余部分会僵在半空；Island 必须整体苏醒。
      assert!(world.apply_impulse(cubes[0], Vec3::new(3.0, 0.0, 0.0)));
      world.step(1.0 / 120.0);
      for (index, cube) in cubes.iter().enumerate() {
         assert_eq!(
            world.is_sleeping(*cube),
            Some(false),
            "level {index} of the Island should have woken"
         );
      }
   }

   #[test]
   fn an_island_sleeps_as_a_unit() {
      let mut world = World::new();
      ground(&mut world, 0.0);
      let mut cubes = alloc::vec::Vec::new();
      for level in 0..5 {
         cubes.push(
            world.add_body(
               BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
                  .at(Vec3::new(0.0, 0.5 + level as f32 * 1.01, 0.0))
                  .with_friction(0.9),
            ),
         );
      }
      // At no point may part of the stack be asleep while the rest is not.
      // 堆叠的任何一部分都不允许在其余部分清醒时先行入睡。
      let mut slept = false;
      for step in 0..1200 {
         world.step(1.0 / 120.0);
         let asleep = cubes
            .iter()
            .filter(|cube| world.is_sleeping(**cube) == Some(true))
            .count();
         assert!(
            asleep == 0 || asleep == cubes.len(),
            "the Island must sleep as a unit, {asleep} of {} were asleep at step {step}",
            cubes.len()
         );
         if asleep == cubes.len() {
            slept = true;
            break;
         }
      }
      assert!(slept, "the stack never slept");
   }
}

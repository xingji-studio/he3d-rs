use alloc::vec::Vec;

use gp3d::{BodyDesc, BodyHandle, Motion};
use he3d_core::{Mesh, ObjectId, Scene, Transform};
use he3d_math::Vec3;

use crate::StepResult;

/// Default gravity, matching the reference engine's `(0, -9.8, 0)`.
/// 默认重力，与参考引擎的 `(0, -9.8, 0)` 一致。
const DEFAULT_GRAVITY: Vec3 = Vec3::new(0.0, -9.8, 0.0);

/// A physics world mapping render objects by stable [`ObjectId`] onto the
/// standalone [`gp3d`] engine's generational body handles.
/// 一个通过稳定的 [`ObjectId`] 将渲染对象映射到独立 [`gp3d`] 引擎代际刚体句柄上的物理世界。
///
/// Bodies and colliders are owned by gp3d.
/// 刚体和碰撞体由 gp3d 持有。
/// After stepping, call [`PhysicsWorld::synchronize`] to write body transforms back into the scene so the renderer sees the physics state.
/// 步进后，调用 [`PhysicsWorld::synchronize`] 将刚体变换写回场景，使渲染器看到物理状态。
///
/// gp3d and `he3d-math` both wrap `glam`, so `Vec3`/`Quat` interoperate directly.
/// gp3d 与 `he3d-math` 都包装 `glam`，因此 `Vec3`/`Quat` 可直接互操作。
pub struct PhysicsWorld {
   scene: gp3d::PhysicsScene,
   bodies: Vec<(ObjectId, BodyHandle)>,
}

impl PhysicsWorld {
   /// Create an empty world with the reference default gravity `(0, -9.8, 0)`.
   /// 创建一个空世界，重力为参考默认值 `(0, -9.8, 0)`。
   #[must_use]
   pub fn new() -> Self {
      let mut scene = gp3d::PhysicsScene::new();
      scene.set_gravity(DEFAULT_GRAVITY);
      Self {
         scene,
         bodies: Vec::new(),
      }
   }

   /// Override the world gravity vector. 覆盖世界重力向量。
   pub fn set_gravity(&mut self, gravity: Vec3) {
      self.scene.set_gravity(gravity);
   }

   /// The current world gravity. 当前世界重力。
   #[must_use]
   pub fn gravity(&self) -> Vec3 {
      self.scene.gravity()
   }

   /// Returns `false` if the object already has a body, leaving the world unchanged.
   /// 若对象已拥有刚体则返回 `false`，世界保持不变。
   pub fn add_dynamic_box(
      &mut self,
      object: ObjectId,
      half_extents: Vec3,
      transform: Transform,
   ) -> bool {
      self.add_box(object, half_extents, transform, Motion::Dynamic)
   }

   /// Returns `false` if the object already has a body, leaving the world unchanged.
   /// 若对象已拥有刚体则返回 `false`，世界保持不变。
   pub fn add_static_box(
      &mut self,
      object: ObjectId,
      half_extents: Vec3,
      transform: Transform,
   ) -> bool {
      self.add_box(object, half_extents, transform, Motion::Static)
   }

   /// Returns `false` if the object already has a body, leaving the world unchanged.
   /// 若对象已拥有刚体则返回 `false`，世界保持不变。
   pub fn add_kinematic_box(
      &mut self,
      object: ObjectId,
      half_extents: Vec3,
      transform: Transform,
   ) -> bool {
      self.add_box(object, half_extents, transform, Motion::Kinematic)
   }

   /// Register a body, rejecting a duplicate `ObjectId` (the reference `AddBody`
   /// also rejects already-registered objects).
   /// 注册刚体；拒绝重复的 `ObjectId`（参考实现 `AddBody` 同样拒绝已注册对象）。
   fn add_box(
      &mut self,
      object: ObjectId,
      half_extents: Vec3,
      transform: Transform,
      motion: Motion,
   ) -> bool {
      if self.handle(object).is_some() {
         return false;
      }
      let desc = BodyDesc {
         motion,
         collider: gp3d::Collider::Box { half_extents },
         position: transform.position,
         orientation: transform.orientation,
         ..BodyDesc::default()
      };
      match self.scene.add_body(desc) {
         Ok(handle) => {
            self.bodies.push((object, handle));
            true
         }
         Err(_) => false,
      }
   }

   /// Add a static triangle-mesh collider from a [`Mesh`]. The mesh vertices are
   /// copied; the reference `MeshCollider` reads live mesh geometry, and gp3d
   /// stores its own copy, so re-adding after the mesh changes is the norm.
   /// 从 [`Mesh`] 添加静态三角网格碰撞体。网格顶点会被复制；参考 `MeshCollider`
   /// 读取实时网格几何，而 gp3d 保存自己的副本，因此网格变化后重新添加是常态。
   pub fn add_static_mesh(&mut self, object: ObjectId, mesh: &Mesh, transform: Transform) -> bool {
      if self.handle(object).is_some() {
         return false;
      }
      let vertices = mesh.vertices().to_vec();
      if vertices.is_empty() || vertices.len() % 3 != 0 {
         return false;
      }
      match self
         .scene
         .add_static_mesh(vertices, transform.position, transform.orientation)
      {
         Ok(handle) => {
            self.bodies.push((object, handle));
            true
         }
         Err(_) => false,
      }
   }

   /// Teleport a body, keeping its velocity. Returns `false` for unknown objects.
   /// 瞬移刚体并保持其速度。对未知对象返回 `false`。
   pub fn set_transform(&mut self, object: ObjectId, transform: Transform) -> bool {
      match self.handle(object) {
         Some(handle) => {
            self
               .scene
               .set_transform(handle, transform.position, transform.orientation)
         }
         None => false,
      }
   }

   /// Read the current body transform. Returns `None` for unknown objects.
   /// 读取当前刚体变换。对未知对象返回 `None`。
   #[must_use]
   pub fn transform(&self, object: ObjectId) -> Option<Transform> {
      let handle = self.handle(object)?;
      let position = self.scene.position(handle)?;
      let orientation = self.scene.orientation(handle)?;
      Some(Transform::from_position_orientation(position, orientation))
   }

   /// Override the body's friction and restitution coefficients.
   /// 覆盖刚体的摩擦系数和恢复系数。
   pub fn set_material(&mut self, object: ObjectId, friction: f32, restitution: f32) -> bool {
      match self.handle(object) {
         Some(handle) => self.scene.set_material(handle, friction, restitution),
         None => false,
      }
   }

   /// Override the body's linear and angular damping. 覆盖刚体的线性和角向阻尼。
   pub fn set_damping(&mut self, object: ObjectId, linear: f32, angular: f32) -> bool {
      let Some(handle) = self.handle(object) else {
         return false;
      };
      self.scene.set_linear_damping(handle, linear);
      self.scene.set_angular_damping(handle, angular);
      true
   }

   /// Whether the body is currently touching another collider.
   /// 刚体当前是否与另一碰撞体接触。
   #[must_use]
   pub fn has_contacts(&self, object: ObjectId) -> bool {
      match self.handle(object) {
         Some(handle) => self.scene.has_contacts(handle),
         None => false,
      }
   }

   /// Remove a body. Returns `false` for unknown objects.
   /// 移除一个刚体。对未知对象返回 `false`。
   pub fn remove_body(&mut self, object: ObjectId) -> bool {
      let Some(position) = self.bodies.iter().position(|(id, _)| *id == object) else {
         return false;
      };
      let (_, handle) = self.bodies.swap_remove(position);
      self.scene.remove_body(handle)
   }

   /// Whether the object currently has a registered body.
   /// 对象当前是否注册了刚体。
   #[must_use]
   pub fn has_body(&self, object: ObjectId) -> bool {
      self.handle(object).is_some()
   }

   /// Override the body's linear velocity. 覆盖刚体线速度。
   pub fn set_velocity(&mut self, object: ObjectId, velocity: Vec3) -> bool {
      match self.handle(object) {
         Some(handle) => self.scene.set_velocity(handle, velocity),
         None => false,
      }
   }

   /// The body's linear velocity, or zero for unknown objects.
   /// 刚体线速度；未知对象返回零向量。
   #[must_use]
   pub fn velocity(&self, object: ObjectId) -> Vec3 {
      match self.handle(object) {
         Some(handle) => self.scene.velocity(handle),
         None => Vec3::ZERO,
      }
   }

   /// Override the body's angular velocity. 覆盖刚体角速度。
   pub fn set_angular_velocity(&mut self, object: ObjectId, velocity: Vec3) -> bool {
      match self.handle(object) {
         Some(handle) => self.scene.set_angular_velocity(handle, velocity),
         None => false,
      }
   }

   /// The body's angular velocity, or zero for unknown objects.
   /// 刚体角速度；未知对象返回零向量。
   #[must_use]
   pub fn angular_velocity(&self, object: ObjectId) -> Vec3 {
      match self.handle(object) {
         Some(handle) => self.scene.angular_velocity(handle),
         None => Vec3::ZERO,
      }
   }

   /// Apply an instantaneous force for the next step only. 施加仅作用于下一步的瞬时力。
   pub fn add_force(&mut self, object: ObjectId, force: Vec3) -> bool {
      match self.handle(object) {
         Some(handle) => self.scene.add_force(handle, force),
         None => false,
      }
   }

   /// Apply an instantaneous torque for the next step only. 施加仅作用于下一步的瞬时力矩。
   pub fn add_torque(&mut self, object: ObjectId, torque: Vec3) -> bool {
      match self.handle(object) {
         Some(handle) => self.scene.add_torque(handle, torque),
         None => false,
      }
   }

   /// Apply an instantaneous impulse. 施加瞬时冲量。
   pub fn add_impulse(&mut self, object: ObjectId, impulse: Vec3) -> bool {
      match self.handle(object) {
         Some(handle) => self.scene.add_impulse(handle, impulse),
         None => false,
      }
   }

   /// Apply an instantaneous angular impulse. 施加瞬时角冲量。
   pub fn add_angular_impulse(&mut self, object: ObjectId, impulse: Vec3) -> bool {
      match self.handle(object) {
         Some(handle) => self.scene.add_angular_impulse(handle, impulse),
         None => false,
      }
   }

   /// Advance the simulation by one fixed step using the reference default of 4
   /// solver iterations. Forces and torques applied with [`PhysicsWorld::add_force`]
   /// / [`PhysicsWorld::add_torque`] act for one step only and are then cleared.
   /// 以参考默认的 4 次求解器迭代推进模拟。由 [`PhysicsWorld::add_force`] /
   /// [`PhysicsWorld::add_torque`] 施加的力与力矩只作用一步并被清除。
   pub fn step(&mut self, delta_time: f32) -> StepResult {
      self.scene.step(delta_time, 4)
   }

   /// Write body transforms back into the scene's render objects.
   /// 将刚体变换写回场景的渲染对象。
   pub fn synchronize(&mut self, scene: &mut Scene) {
      for &(object, handle) in &self.bodies {
         let Some(render_object) = scene.objects.get_mut(object) else {
            continue;
         };
         let Some(position) = self.scene.position(handle) else {
            continue;
         };
         let Some(orientation) = self.scene.orientation(handle) else {
            continue;
         };
         render_object.transform.position = position;
         render_object.transform.orientation = orientation;
      }
   }

   fn handle(&self, object: ObjectId) -> Option<BodyHandle> {
      self
         .bodies
         .iter()
         .find(|(id, _)| *id == object)
         .map(|(_, handle)| *handle)
   }
}

impl Default for PhysicsWorld {
   fn default() -> Self {
      Self::new()
   }
}

#[cfg(test)]
mod tests {
   use alloc::vec;

   use super::*;
   use he3d_core::{ColorRgba8, Material, Mesh, RenderObject, Rgb, Texture};
   use he3d_math::Quat;

   const DT: f32 = 1.0 / 60.0;

   fn spawn_box(scene: &mut Scene, position: Vec3) -> ObjectId {
      let mesh = scene
         .meshes
         .insert(Mesh::cube(1.0, 1.0, 1.0).expect("cube"));
      let solid = scene.textures.insert(Texture::solid(ColorRgba8::WHITE));
      scene.spawn(RenderObject {
         mesh,
         texture: Some(solid),
         transform: Transform::from_position(position),
         material: Material::new(Rgb::WHITE),
         visible: true,
      })
   }

   /// A flat quad mesh floor with a render object; use with `add_static_mesh`.
   /// 一个带渲染对象的平坦四边形网格地面；配合 `add_static_mesh` 使用。
   fn spawn_quad_floor(scene: &mut Scene, half: f32) -> (ObjectId, Mesh) {
      let h = half;
      let quad = Mesh::from_vertices(vec![
         Vec3::new(-h, 0.0, -h),
         Vec3::new(h, 0.0, h),
         Vec3::new(h, 0.0, -h),
         Vec3::new(-h, 0.0, -h),
         Vec3::new(-h, 0.0, h),
         Vec3::new(h, 0.0, h),
      ])
      .expect("quad");
      let mesh_id = scene.meshes.insert(quad.clone());
      let solid = scene
         .textures
         .insert(Texture::solid(ColorRgba8::from_rgb(77, 179, 77)));
      let object = scene.spawn(RenderObject {
         mesh: mesh_id,
         texture: Some(solid),
         transform: Transform::default(),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      (object, quad)
   }

   fn run(world: &mut PhysicsWorld, steps: usize) {
      for _ in 0..steps {
         assert_eq!(world.step(DT), StepResult::Completed);
      }
   }

   #[test]
   fn registration_and_removal() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::ZERO);
      let mut world = PhysicsWorld::new();

      assert!(world.add_dynamic_box(cube, Vec3::splat(0.5), Transform::default()));
      assert!(world.has_body(cube));
      // Duplicate registration is rejected. 重复注册被拒绝。
      assert!(!world.add_dynamic_box(cube, Vec3::splat(0.5), Transform::default()));
      assert!(!world.add_static_box(cube, Vec3::splat(0.5), Transform::default()));

      assert!(world.remove_body(cube));
      assert!(!world.has_body(cube));
      assert!(!world.remove_body(cube));
      assert!(!world.add_force(cube, Vec3::X));
   }

   #[test]
   fn transform_and_velocity_passthrough() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::new(0.0, 1.0, 0.0));
      let mut world = PhysicsWorld::new();
      world.add_dynamic_box(
         cube,
         Vec3::splat(0.5),
         Transform::from_position(Vec3::new(0.0, 1.0, 0.0)),
      );

      assert!(world.set_velocity(cube, Vec3::new(2.0, 0.0, 0.0)));
      assert_eq!(world.velocity(cube), Vec3::new(2.0, 0.0, 0.0));

      // Teleport keeps velocity. 瞬移保持速度。
      let teleport =
         Transform::from_position_orientation(Vec3::new(3.0, 4.0, 5.0), Quat::from_rotation_y(1.0));
      assert!(world.set_transform(cube, teleport));
      assert_eq!(world.transform(cube), Some(teleport));
      assert_eq!(world.velocity(cube), Vec3::new(2.0, 0.0, 0.0));

      assert!(world.set_angular_velocity(cube, Vec3::new(0.0, 0.0, 1.0)));
      assert_eq!(world.angular_velocity(cube), Vec3::new(0.0, 0.0, 1.0));

      // Unknown objects are no-ops. 未知对象为无操作。
      let stale = spawn_box(&mut scene, Vec3::ZERO);
      let _ = scene.objects.remove(stale);
      assert!(!world.set_transform(stale, Transform::default()));
      assert_eq!(world.transform(stale), None);
   }

   #[test]
   fn gravity_pulls_dynamic_body_down() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::new(0.0, 10.0, 0.0));
      let mut world = PhysicsWorld::new();
      world.add_dynamic_box(
         cube,
         Vec3::splat(0.5),
         Transform::from_position(Vec3::new(0.0, 10.0, 0.0)),
      );
      run(&mut world, 60);
      world.synchronize(&mut scene);
      let y = scene.objects.get(cube).unwrap().transform.position.y;
      assert!(y < 9.0, "body should have fallen, got y={y}");
      assert!(y > 0.0, "body should not have fallen through, got y={y}");
   }

   #[test]
   fn cube_rests_on_static_mesh_ground() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::new(0.0, 5.0, 0.0));
      let (floor, floor_mesh) = spawn_quad_floor(&mut scene, 3.0);
      let mut world = PhysicsWorld::new();
      world.add_dynamic_box(
         cube,
         Vec3::splat(0.5),
         Transform::from_position(Vec3::new(0.0, 5.0, 0.0)),
      );
      assert!(world.add_static_mesh(
         floor,
         &floor_mesh,
         Transform::from_position(Vec3::new(0.0, -0.4, 0.0)),
      ));

      run(&mut world, 300);
      world.synchronize(&mut scene);
      let y = scene.objects.get(cube).unwrap().transform.position.y;
      assert!(
         (y - 0.1).abs() < 0.05,
         "cube should rest near y=0.1, got y={y}"
      );
   }

   #[test]
   fn restitution_bounces_on_fast_impact() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::new(0.0, 5.0, 0.0));
      let (floor, floor_mesh) = spawn_quad_floor(&mut scene, 6.0);
      let mut world = PhysicsWorld::new();
      world.add_dynamic_box(
         cube,
         Vec3::splat(0.5),
         Transform::from_position(Vec3::new(0.0, 5.0, 0.0)),
      );
      world.set_material(cube, 0.5, 0.5);
      world.add_static_mesh(
         floor,
         &floor_mesh,
         Transform::from_position(Vec3::new(0.0, -0.4, 0.0)),
      );

      let mut max_vy = 0.0f32;
      for _ in 0..120 {
         world.step(DT);
         max_vy = max_vy.max(world.velocity(cube).y);
      }
      assert!(max_vy > 0.5, "fast impact must bounce, got max_vy={max_vy}");
   }

   #[test]
   fn force_acts_for_one_step_then_is_cleared() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::ZERO);
      let mut world = PhysicsWorld::new();
      world.set_gravity(Vec3::ZERO);
      world.add_dynamic_box(cube, Vec3::splat(0.5), Transform::default());

      assert!(world.add_force(cube, Vec3::new(60.0, 0.0, 0.0)));
      assert_eq!(world.step(DT), StepResult::Completed);
      let after_force = world.velocity(cube).x;
      assert!(after_force > 0.9, "got {after_force}");

      assert_eq!(world.step(DT), StepResult::Completed);
      let after_idle = world.velocity(cube).x;
      assert!(after_idle < after_force);
      assert!(after_idle > after_force * 0.99);
   }

   #[test]
   fn damping_decays_velocity() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::ZERO);
      let mut world = PhysicsWorld::new();
      world.set_gravity(Vec3::ZERO);
      world.add_dynamic_box(cube, Vec3::splat(0.5), Transform::default());
      assert!(world.set_damping(cube, 0.5, 0.5));
      assert!(world.set_velocity(cube, Vec3::new(10.0, 0.0, 0.0)));
      assert_eq!(world.step(DT), StepResult::Completed);
      let vx = world.velocity(cube).x;
      assert!(vx > 9.0 && vx < 10.0, "got {vx}");
   }

   #[test]
   fn contacts_detect_overlap_and_separation() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::ZERO);
      let other = spawn_box(&mut scene, Vec3::ZERO);
      let mut world = PhysicsWorld::new();
      world.add_dynamic_box(cube, Vec3::splat(0.5), Transform::default());
      world.add_dynamic_box(other, Vec3::splat(0.5), Transform::default());

      assert!(world.has_contacts(cube));
      assert!(world.has_contacts(other));

      assert!(world.set_transform(other, Transform::from_position(Vec3::new(10.0, 0.0, 0.0)),));
      assert!(!world.has_contacts(cube));
   }

   #[test]
   // A kinematic body ignores gravity, so `position.y` stays exactly zero by
   // construction; the exact equality is intentional.
   // 运动学刚体忽略重力，因此 `position.y` 在构造上严格保持为零；此精确相等是有意为之。
   #[expect(clippy::float_cmp)]
   fn kinematic_body_moves_and_ignores_gravity() {
      let mut scene = Scene::new();
      let mover = spawn_box(&mut scene, Vec3::ZERO);
      let mut world = PhysicsWorld::new();
      world.add_kinematic_box(mover, Vec3::splat(0.5), Transform::default());
      assert!(world.set_velocity(mover, Vec3::new(1.0, 0.0, 0.0)));
      run(&mut world, 10);
      let position = world.transform(mover).unwrap().position;
      assert!(
         (position.x - 10.0 * DT).abs() < 1e-4,
         "got x={}",
         position.x
      );
      assert_eq!(position.y, 0.0);
   }

   #[test]
   fn step_rejects_invalid_input() {
      let mut world = PhysicsWorld::new();
      assert_eq!(world.step(0.0), StepResult::InvalidInput);
      assert_eq!(world.step(-1.0), StepResult::InvalidInput);
      assert_eq!(world.step(f32::NAN), StepResult::InvalidInput);
   }

   #[test]
   fn synchronize_writes_orientation_and_position() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::new(0.0, 2.0, 0.0));
      let mut world = PhysicsWorld::new();
      world.add_dynamic_box(
         cube,
         Vec3::splat(0.5),
         Transform::from_position(Vec3::new(0.0, 2.0, 0.0)),
      );
      assert!(world.set_angular_velocity(cube, Vec3::new(0.0, 0.0, 2.0)));
      run(&mut world, 30);
      world.synchronize(&mut scene);
      let orientation = scene.objects.get(cube).unwrap().transform.orientation;
      assert_ne!(orientation, Quat::IDENTITY, "cube should have rotated");
      assert!(orientation.is_finite());
   }

   #[test]
   fn add_static_mesh_registers_and_rejects_duplicates() {
      let mut scene = Scene::new();
      let cube = spawn_box(&mut scene, Vec3::new(0.0, 5.0, 0.0));
      let (floor, floor_mesh) = spawn_quad_floor(&mut scene, 3.0);
      let mut world = PhysicsWorld::new();
      assert!(world.add_static_mesh(
         floor,
         &floor_mesh,
         Transform::from_position(Vec3::new(0.0, -0.4, 0.0)),
      ));
      assert!(world.has_body(floor));
      // Duplicate registration for the same object is rejected.
      // 同一对象的重复注册被拒绝。
      assert!(!world.add_static_mesh(
         floor,
         &floor_mesh,
         Transform::from_position(Vec3::new(0.0, -0.4, 0.0)),
      ));
      assert!(world.add_dynamic_box(
         cube,
         Vec3::splat(0.5),
         Transform::from_position(Vec3::new(0.0, 5.0, 0.0)),
      ));
      run(&mut world, 300);
      world.synchronize(&mut scene);
      let y = scene.objects.get(cube).unwrap().transform.position.y;
      assert!(
         (y - 0.1).abs() < 0.05,
         "cube should rest near y=0.1, got y={y}"
      );
   }
}

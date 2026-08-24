//! Flight simulator core logic: terrain noise, mesh generation, flight controls, camera follow, and streamed terrain tiles.
//! 飞行模拟核心逻辑：地形噪声、网格生成、飞行控制、相机跟随和流式地形块。

use he3d::{
   Camera, ColorRgba8, Error, EulerRot, MAX_VERTEX_COUNT, Material, Mesh, MeshId, ObjectId,
   PhysicsWorld, Quat, RenderObject, Rgb, Scene, Texture, Transform, Vec2, Vec3,
};

/// Default gravity used by the he3d-cpp flight simulator.
/// he3d-cpp 飞行模拟器使用的默认重力。
pub const FLIGHT_GRAVITY: Vec3 = Vec3::new(0.0, -100.0, 0.0);

/// Deterministic pseudo-random noise at an integer grid point.
/// 整数网格点上的确定性伪随机噪声。
pub fn flight_noise_grid(x: i32, z: i32) -> f32 {
   let n = x.wrapping_add(z.wrapping_mul(57));
   let n = (n << 13) ^ n;
   let mixed = n
      .wrapping_mul(n.wrapping_mul(n).wrapping_mul(15_731).wrapping_add(789_221))
      .wrapping_add(1_376_312_589);
   1.0 - (mixed & 0x7fff_ffff) as f32 / 1_073_741_824.0
}

fn smooth_step(t: f32) -> f32 {
   t * t * (3.0 - 2.0 * t)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
   a + (b - a) * t
}

/// Sample smoothed value noise at a floating-point terrain coordinate.
/// 在浮点地形坐标上采样平滑后的值噪声。
pub fn flight_noise(x: f32, z: f32) -> f32 {
   let ix = x.floor() as i32;
   let iz = z.floor() as i32;
   let fx = x - ix as f32;
   let fz = z - iz as f32;
   let sx = smooth_step(fx);
   let sz = smooth_step(fz);

   let n00 = flight_noise_grid(ix, iz);
   let n10 = flight_noise_grid(ix + 1, iz);
   let n01 = flight_noise_grid(ix, iz + 1);
   let n11 = flight_noise_grid(ix + 1, iz + 1);
   let nx0 = lerp(n00, n10, sx);
   let nx1 = lerp(n01, n11, sx);
   lerp(nx0, nx1, sz)
}

/// Procedural terrain height at a world-space x/z coordinate.
/// 世界空间 x/z 坐标处的程序化地形高度。
pub fn terrain_height_at(x: f32, z: f32) -> f32 {
   let large = flight_noise(x * 0.05, z * 0.05) * 8.0;
   let small = flight_noise(x * 0.2, z * 0.2) * 1.5;
   large + small
}

/// Create a terrain mesh tile centered at the given world coordinate.
/// 创建以给定世界坐标为中心的地形网格块。
///
/// The tile is `grid_count × grid_count` samples with `step` spacing.
/// 该地形块为 `grid_count × grid_count` 个采样点、间距为 `step`。
/// Heights are sampled from [`terrain_height_at`] in world space and stored in local coordinates (the tile transform carries the world offset).
/// 高度在世界空间通过 [`terrain_height_at`] 采样，并以局部坐标存储（地形块的变换携带世界偏移）。
pub fn create_plane(grid_count: i32, step: f32, world_x: f32, world_z: f32) -> Result<Mesh, Error> {
   if grid_count < 2 || !step.is_finite() || step <= 0.0 {
      return Err(Error::InvalidSize);
   }
   let cell_count = grid_count - 1;
   let tri_count = (cell_count as usize)
      .checked_mul(cell_count as usize)
      .and_then(|v| v.checked_mul(2))
      .ok_or(Error::Overflow)?;
   let vert_count = tri_count.checked_mul(3).ok_or(Error::Overflow)?;
   if vert_count > MAX_VERTEX_COUNT {
      return Err(Error::InvalidSize);
   }

   let half = cell_count as f32 * step * 0.5;
   let max_cached_grid = 32;
   let use_height_cache = grid_count <= max_cached_grid;
   let mut cache = Vec::new();
   if use_height_cache {
      cache.resize((grid_count * grid_count) as usize, 0.0);
      for z in 0..grid_count {
         let wz = z as f32 * step - half + world_z;
         for x in 0..grid_count {
            let wx = x as f32 * step - half + world_x;
            cache[(z * grid_count + x) as usize] = terrain_height_at(wx, wz);
         }
      }
   }

   let mut vertices = Vec::with_capacity(vert_count);
   for z in 0..cell_count {
      for x in 0..cell_count {
         let lx0 = x as f32 * step - half;
         let lz0 = z as f32 * step - half;
         let lx1 = (x + 1) as f32 * step - half;
         let lz1 = (z + 1) as f32 * step - half;
         let wx0 = lx0 + world_x;
         let wz0 = lz0 + world_z;
         let wx1 = lx1 + world_x;
         let wz1 = lz1 + world_z;

         let h1 = if use_height_cache {
            cache[(z * grid_count + x) as usize]
         } else {
            terrain_height_at(wx0, wz0)
         };
         let h2 = if use_height_cache {
            cache[((z + 1) * grid_count + x) as usize]
         } else {
            terrain_height_at(wx0, wz1)
         };
         let h3 = if use_height_cache {
            cache[(z * grid_count + x + 1) as usize]
         } else {
            terrain_height_at(wx1, wz0)
         };
         let h4 = if use_height_cache {
            cache[((z + 1) * grid_count + x + 1) as usize]
         } else {
            terrain_height_at(wx1, wz1)
         };

         let v1 = Vec3::new(lx0, h1, lz0);
         let v2 = Vec3::new(lx0, h2, lz1);
         let v3 = Vec3::new(lx1, h3, lz0);
         let v4 = Vec3::new(lx1, h4, lz1);

         vertices.extend([v1, v2, v3, v3, v2, v4]);
      }
   }
   Mesh::from_vertices(vertices)
}

/// Built-in aircraft mesh, used when `biplane.obj` cannot be loaded.
/// 内置飞机网格，在无法加载 `biplane.obj` 时使用。
pub fn fallback_aircraft_mesh() -> Result<Mesh, Error> {
   const VERTICES: [Vec3; 39] = [
      Vec3::new(0.07, 0.0, 0.45),
      Vec3::new(0.0, 0.07, 0.45),
      Vec3::new(0.07, 0.0, -0.45),
      Vec3::new(0.0, 0.07, 0.45),
      Vec3::new(0.0, 0.07, -0.45),
      Vec3::new(0.07, 0.0, -0.45),
      Vec3::new(0.0, 0.07, 0.45),
      Vec3::new(-0.07, 0.0, 0.45),
      Vec3::new(0.0, 0.07, -0.45),
      Vec3::new(-0.07, 0.0, 0.45),
      Vec3::new(-0.07, 0.0, -0.45),
      Vec3::new(0.0, 0.07, -0.45),
      Vec3::new(-0.07, 0.0, 0.45),
      Vec3::new(0.0, -0.07, 0.45),
      Vec3::new(-0.07, 0.0, -0.45),
      Vec3::new(0.0, -0.07, 0.45),
      Vec3::new(0.0, -0.07, -0.45),
      Vec3::new(-0.07, 0.0, -0.45),
      Vec3::new(0.0, -0.07, 0.45),
      Vec3::new(0.07, 0.0, 0.45),
      Vec3::new(0.0, -0.07, -0.45),
      Vec3::new(0.07, 0.0, 0.45),
      Vec3::new(0.07, 0.0, -0.45),
      Vec3::new(0.0, -0.07, -0.45),
      Vec3::new(-0.35, 0.15, -0.06),
      Vec3::new(0.35, 0.15, -0.06),
      Vec3::new(-0.35, 0.15, 0.06),
      Vec3::new(0.35, 0.15, -0.06),
      Vec3::new(0.35, 0.15, 0.06),
      Vec3::new(-0.35, 0.15, 0.06),
      Vec3::new(-0.315, -0.15, -0.06),
      Vec3::new(0.315, -0.15, -0.06),
      Vec3::new(-0.315, -0.15, 0.06),
      Vec3::new(0.315, -0.15, -0.06),
      Vec3::new(0.315, -0.15, 0.06),
      Vec3::new(-0.315, -0.15, 0.06),
      Vec3::new(0.0, 0.0, -0.5),
      Vec3::new(0.0, 0.1, -0.6),
      Vec3::new(0.0, 0.0, -0.6),
   ];
   const UVS: [Vec2; 39] = [
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(1.0, 1.0),
      Vec2::new(0.0, 1.0),
      Vec2::new(0.0, 0.0),
      Vec2::new(1.0, 0.0),
      Vec2::new(0.5, 1.0),
   ];
   Mesh::new(VERTICES.to_vec(), UVS.to_vec())
}

/// Smoothed control values for pitch, yaw, and roll.
/// 俯仰、偏航和滚转的平滑控制值。
#[derive(Clone, Copy, Debug, Default)]
pub struct FlightControls {
   pub pitch: f32,
   pub yaw: f32,
   pub roll: f32,
}

/// Ease a digital key pair into a smooth cubic control value.
/// 将一组数字按键平滑为三次曲线控制值。
///
/// `current` is smoothed state that eases toward the pressed key's direction
/// `current` 是朝按键方向缓动、松开后
/// and decays back to zero when released. The returned value is `±|v|³`.
/// 衰减回零的平滑状态。返回值为 `±|v|³`。
pub fn input_curve(
   current: &mut f32,
   positive: bool,
   negative: bool,
   delta_time: f32,
   acceleration: f32,
   recovery: f32,
) -> f32 {
   let target = if positive {
      1.0
   } else if negative {
      -1.0
   } else {
      0.0
   };
   if target != 0.0 {
      *current += target * acceleration * delta_time;
   } else if *current > 0.0 {
      *current = (*current - recovery * delta_time).max(0.0);
   } else if *current < 0.0 {
      *current = (*current + recovery * delta_time).min(0.0);
   }
   *current = current.clamp(-1.0, 1.0);
   let sign = if *current >= 0.0 { 1.0 } else { -1.0 };
   let amount = current.abs();
   sign * amount * amount * amount
}

/// Normalize an angle to the `-π..π` range.
/// 将角度归一化到 `-π..π` 区间。
pub fn normalize_angle(angle: f32) -> f32 {
   if (-core::f32::consts::PI..=core::f32::consts::PI).contains(&angle) {
      return angle;
   }
   let mut result = angle - (angle / core::f32::consts::TAU) as i32 as f32 * core::f32::consts::TAU;
   if result > core::f32::consts::PI {
      result -= core::f32::consts::TAU;
   }
   if result < -core::f32::consts::PI {
      result += core::f32::consts::TAU;
   }
   result
}

/// Build a quaternion from an axis and a radians angle.
/// 由旋转轴和以弧度表示的角度构建四元数。
pub fn axis_angle_quat(axis: Vec3, angle: f32) -> Quat {
   let axis = axis.normalize_or_zero();
   let half = angle * 0.5;
   let (sin, cos) = half.sin_cos();
   Quat::from_xyzw(axis.x * sin, axis.y * sin, axis.z * sin, cos)
}

/// Cruise speed maintained while airborne, matching the he3d-cpp simulator.
/// 空中保持的巡航速度，与 he3d-cpp 模拟器一致。
pub const CRUISE_SPEED: f32 = 15.0;

/// Exponential velocity decay applied while the plane touches the ground.
/// 飞机接触地面时施加的指数速度衰减。
pub const GROUND_CONTACT_DECAY: f32 = 8.0;

/// Exponential damping applied to a crashed wreck so the physics step can settle it against the terrain without letting it slide or spin forever.
/// 施加于坠毁残骸的指数阻尼，使物理步进能够让其贴合地形静止，而不会永久滑行或旋转。
pub const CRASH_DAMPING: f32 = 6.0;

/// Update the aircraft's orientation from controls and write its velocity.
/// 根据控制输入更新飞机朝向并写入其速度。
///
/// `controlled_orientation` is the player-owned heading. It is used instead of the physics body's own rotation so ground collisions can never roll or pitch the plane away from player control.
/// `controlled_orientation` 是玩家持有的朝向。使用它而非物理刚体自身的旋转，这样地面碰撞永远不会让飞机滚转或俯仰偏离玩家控制。
/// This matches the he3d-cpp simulator, which rebuilds its scene every step and only lets controls change heading.
/// 这与 he3d-cpp 模拟器一致，它每步重建场景，仅允许控制改变朝向。
/// The angular velocity is also cleared each step so impacts cannot make the plane spin.
/// 每步还会清零角速度，使撞击不会导致飞机旋转。
///
/// While `contacted` is true the plane is out of control: the heading is frozen (player input no longer changes it) and the velocity decays exponentially, so ground contact stops the plane promptly.
/// 当 `contacted` 为真时飞机失去控制：朝向被冻结（玩家输入不再改变它），速度以指数方式衰减，因此触地能迅速停下飞机。
/// Returns the current world-space forward direction.
/// 返回当前世界空间的前向方向。
pub fn update_flight_controls(
   physics: &mut PhysicsWorld,
   plane: ObjectId,
   controlled_orientation: &mut Quat,
   controls: &mut FlightControls,
   keys: &[bool; 256],
   delta_time: f32,
   contacted: bool,
) -> Option<Vec3> {
   let position = physics.transform(plane)?.position;

   // Ground impacts must never leave the plane spinning out of control.
   // 地面撞击绝不能使飞机失去控制地旋转。
   physics.set_angular_velocity(plane, Vec3::ZERO);
   if contacted {
      // Touching down: lose control authority immediately, freeze the heading and decelerate until the crash check stops the plane.
      // 触地：立即失去控制权，冻结朝向并减速，直到坠毁判定停住飞机。
      let forward = *controlled_orientation * Vec3::Z;
      physics.set_transform(
         plane,
         Transform::from_position_orientation(position, *controlled_orientation),
      );
      let velocity = physics.velocity(plane);
      let decay = (-GROUND_CONTACT_DECAY * delta_time).exp();
      physics.set_velocity(plane, velocity * decay);
      return Some(forward);
   }

   const INPUT_ACCELERATION: f32 = 4.5;
   const INPUT_RECOVERY: f32 = 2.5;
   let pitch = input_curve(
      &mut controls.pitch,
      keys[b'w' as usize],
      keys[b's' as usize],
      delta_time,
      INPUT_ACCELERATION,
      INPUT_RECOVERY,
   );
   let yaw = input_curve(
      &mut controls.yaw,
      keys[b'e' as usize],
      keys[b'q' as usize],
      delta_time,
      INPUT_ACCELERATION,
      INPUT_RECOVERY,
   );
   let roll = input_curve(
      &mut controls.roll,
      keys[b'a' as usize],
      keys[b'd' as usize],
      delta_time,
      INPUT_ACCELERATION,
      INPUT_RECOVERY,
   );

   let orientation = *controlled_orientation;

   let pitch_delta = Quat::from_rotation_x(pitch * 1.35 * delta_time);
   let right = orientation * Vec3::X;
   let local_up = orientation * Vec3::Y;
   let world_up = Vec3::Y;
   let bank_influence = right.y.abs().clamp(0.0, 1.0);
   let yaw_axis =
      (world_up * (1.0 - bank_influence) + local_up * bank_influence).normalize_or_zero();
   let yaw_delta = axis_angle_quat(yaw_axis, yaw * 0.95 * delta_time);
   let roll_delta = Quat::from_rotation_z(roll * 1.75 * delta_time);

   let new_orientation = (yaw_delta * orientation * pitch_delta * roll_delta).normalize();
   let forward = new_orientation * Vec3::Z;
   *controlled_orientation = new_orientation;

   physics.set_transform(
      plane,
      Transform::from_position_orientation(position, new_orientation),
   );
   physics.set_velocity(plane, forward * CRUISE_SPEED);
   Some(forward)
}

/// Chase-camera state: smoothed yaw plus a fixed body offset behind the plane.
/// 追尾相机状态：平滑偏航角加上飞机后方的固定机身偏移。
#[derive(Clone, Copy, Debug)]
pub struct FlightCamera {
   pub yaw: f32,
   pub offset: Vec3,
}

impl Default for FlightCamera {
   fn default() -> Self {
      Self {
         yaw: 0.0,
         offset: Vec3::new(0.0, 0.7, -1.8),
      }
   }
}

/// Follow the aircraft from behind with smoothed yaw and a fixed pitch.
/// 以平滑偏航和固定俯仰从后方跟随飞机。
pub fn update_camera(
   camera: &mut Camera,
   chase: &mut FlightCamera,
   plane_position: Vec3,
   forward: Vec3,
   delta_time: f32,
) {
   let horizontal_forward = Vec3::new(forward.x, 0.0, forward.z);
   if horizontal_forward.length_squared() > 0.001 {
      let target_yaw = horizontal_forward.x.atan2(horizontal_forward.z);
      chase.yaw += normalize_angle(target_yaw - chase.yaw) * (5.0 * delta_time).min(1.0);
   }

   // Fixed downward pitch (0.15 rad) so the plane sits near screen center.
   // 固定向下俯仰（0.15 弧度），使飞机位于屏幕中心附近。
   // `EulerRot::ZYX` takes angles in (Z, Y, X) axis order, so the pitch goes in the `z` slot.
   // `EulerRot::ZYX` 按 (Z, Y, X) 轴序接收角度，因此俯仰放在 `z` 槽位。
   // This matches he3d-cpp's `FromEuler({pitch, yaw, 0})`.
   // 这与 he3d-cpp 的 `FromEuler({pitch, yaw, 0})` 一致。
   camera.orientation = Quat::from_euler(EulerRot::ZYX, 0.0, chase.yaw, 0.15);
   let yaw_rotation = Quat::from_rotation_y(chase.yaw);
   let target_position = plane_position + yaw_rotation * chase.offset;
   let delta = target_position - camera.position;
   if delta.length_squared() > 100.0 {
      camera.position = target_position;
   } else {
      camera.position += delta * (8.0 * delta_time).min(1.0);
   }
}

fn tile_in_range(grid_x: i32, grid_z: i32, center_x: i32, center_z: i32, radius: i32) -> bool {
   grid_x >= center_x - radius
      && grid_x <= center_x + radius
      && grid_z >= center_z - radius
      && grid_z <= center_z + radius
}

/// A terrain tile slot: owns one render object and its static mesh collider.
/// 一个地形块槽位：拥有一个渲染对象及其静态网格碰撞体。
#[derive(Clone, Copy, Debug)]
pub struct TerrainTile {
   pub mesh_id: MeshId,
   pub object_id: ObjectId,
   pub grid_x: i32,
   pub grid_z: i32,
   pub active: bool,
}

/// Streams square terrain tiles around the aircraft, refreshing at most one tile per call.
/// 在飞机周围流式加载方形地形块，每次调用最多刷新一个地形块。
/// Mirrors he3d-cpp's budget: terrain generation is spread across frames instead of blocking one frame.
/// 与 he3d-cpp 的预算一致：地形生成分摊到多帧执行，而非阻塞单帧。
pub struct TerrainManager {
   pub tiles: Vec<TerrainTile>,
   pub center_x: i32,
   pub center_z: i32,
   pub tile_radius: i32,
   pub grid_count: i32,
   pub step: f32,
   pub tile_size: f32,
   dirty: bool,
   pending: Vec<(i32, i32)>,
   pending_cursor: usize,
   placeholder_mesh: MeshId,
}

impl TerrainManager {
   pub fn new(
      scene: &mut Scene,
      placeholder_mesh: MeshId,
      tile_radius: i32,
      grid_count: i32,
      step: f32,
   ) -> Self {
      let tile_size = (grid_count - 1) as f32 * step;
      let count = ((tile_radius * 2 + 1) as usize).pow(2);
      let mut tiles = Vec::with_capacity(count);
      let terrain_solid = scene
         .textures
         .insert(Texture::solid(ColorRgba8::from_rgb(77, 179, 77)));
      for _ in 0..count {
         let object_id = scene.spawn(RenderObject {
            mesh: placeholder_mesh,
            texture: Some(terrain_solid),
            transform: Transform::default(),
            material: Material::new(Rgb::WHITE),
            visible: false,
         });
         tiles.push(TerrainTile {
            mesh_id: placeholder_mesh,
            object_id,
            grid_x: 0,
            grid_z: 0,
            active: false,
         });
      }
      Self {
         tiles,
         center_x: i32::MAX,
         center_z: i32::MAX,
         tile_radius,
         grid_count,
         step,
         tile_size,
         dirty: true,
         pending: Vec::new(),
         pending_cursor: 0,
         placeholder_mesh,
      }
   }

   fn has_active_tile(&self, grid_x: i32, grid_z: i32) -> bool {
      self
         .tiles
         .iter()
         .any(|tile| tile.active && tile.grid_x == grid_x && tile.grid_z == grid_z)
   }

   /// Recenter on the aircraft when it crosses a tile boundary.
   /// 当飞机跨越地形块边界时重新居中。
   pub fn update(&mut self, plane_position: Vec3) {
      let grid_x = (plane_position.x / self.tile_size).floor() as i32;
      let grid_z = (plane_position.z / self.tile_size).floor() as i32;
      if grid_x != self.center_x || grid_z != self.center_z || self.dirty {
         self.center_x = grid_x;
         self.center_z = grid_z;
         self.dirty = false;
         self.queue_tiles();
      }
   }

   fn queue_tiles(&mut self) {
      self.pending.clear();
      self.pending_cursor = 0;

      let radius = self.tile_radius;
      let center_x = self.center_x;
      let center_z = self.center_z;
      for tile in &mut self.tiles {
         if tile.active && !tile_in_range(tile.grid_x, tile.grid_z, center_x, center_z, radius) {
            tile.active = false;
         }
      }

      for offset_x in -self.tile_radius..=self.tile_radius {
         for offset_z in -self.tile_radius..=self.tile_radius {
            let grid_x = self.center_x + offset_x;
            let grid_z = self.center_z + offset_z;
            if !self.has_active_tile(grid_x, grid_z) && self.pending.len() < self.tiles.len() {
               self.pending.push((grid_x, grid_z));
            }
         }
      }
   }

   /// Generate or refresh at most one queued terrain tile.
   /// 生成或刷新最多一个排队中的地形块。
   pub fn refresh_one(&mut self, scene: &mut Scene, physics: &mut PhysicsWorld) {
      if self.pending_cursor >= self.pending.len() {
         return;
      }
      let (grid_x, grid_z) = self.pending[self.pending_cursor];
      self.pending_cursor += 1;
      if self.has_active_tile(grid_x, grid_z) {
         return;
      }

      for tile in &mut self.tiles {
         if tile.active {
            continue;
         }
         // Free the previous per-tile mesh before replacing it. The shared
         // placeholder is kept so inactive tiles can keep referencing it.
         // 在替换前释放旧的逐块网格。共享占位网格保留，使未激活块仍可引用它。
         if tile.mesh_id != self.placeholder_mesh {
            let _ = scene.meshes.remove(tile.mesh_id);
         }
         tile.grid_x = grid_x;
         tile.grid_z = grid_z;
         let world_x = grid_x as f32 * self.tile_size;
         let world_z = grid_z as f32 * self.tile_size;

         let Ok(mesh) = create_plane(self.grid_count, self.step, world_x, world_z) else {
            tile.active = false;
            return;
         };
         let mesh_id = scene.meshes.insert(mesh);
         let world_transform = Transform::from_position(Vec3::new(world_x, 0.0, world_z));
         if let Some(object) = scene.objects.get_mut(tile.object_id) {
            object.mesh = mesh_id;
            object.transform = world_transform;
            object.visible = true;
         }
         tile.mesh_id = mesh_id;

         // Refresh the static collider for the new tile geometry.
         // 为新地形块几何刷新静态碰撞体。
         physics.remove_body(tile.object_id);
         if let Some(mesh_ref) = scene.meshes.get(mesh_id) {
            tile.active = physics.add_static_mesh(tile.object_id, mesh_ref, world_transform);
         } else {
            tile.active = false;
         }
         return;
      }
   }
}

#[cfg(test)]
mod tests {
   use super::*;
   use he3d::Vec3;

   #[test]
   fn noise_is_deterministic_and_bounded() {
      let a = flight_noise(12.7, -3.2);
      let b = flight_noise(12.7, -3.2);
      assert_eq!(a, b);
      assert!((-1.0..=1.0).contains(&a));
   }

   #[test]
   fn terrain_height_is_finite_everywhere() {
      for x in [-50.0, -1.0, 0.0, 1.0, 50.0] {
         for z in [-50.0, -1.0, 0.0, 1.0, 50.0] {
            assert!(terrain_height_at(x, z).is_finite());
         }
      }
   }

   #[test]
   fn plane_has_expected_triangle_count() {
      let mesh = create_plane(7, 12.0, 0.0, 0.0).expect("plane");
      assert_eq!(mesh.triangle_count(), (7 - 1) * (7 - 1) * 2);
      assert_eq!(mesh.vertex_count(), (7 - 1) * (7 - 1) * 6);
   }

   #[test]
   fn plane_rejects_small_grids() {
      assert!(create_plane(1, 12.0, 0.0, 0.0).is_err());
      assert!(create_plane(7, 0.0, 0.0, 0.0).is_err());
      assert!(create_plane(7, f32::NAN, 0.0, 0.0).is_err());
   }

   #[test]
   fn fallback_aircraft_is_valid() {
      let mesh = fallback_aircraft_mesh().expect("fallback");
      assert_eq!(mesh.vertex_count(), 39);
      assert_eq!(mesh.triangle_count(), 13);
   }

   #[test]
   fn input_curve_eases_toward_target() {
      let mut current = 0.0;
      let value = input_curve(&mut current, true, false, 1.0, 4.5, 2.5);
      assert_eq!(value, 1.0);
      // Released: current decays slowly enough to remain inside (0,1).
      // 松开后：current 衰减得足够慢，仍保持在 (0,1) 区间内。
      let value = input_curve(&mut current, false, false, 0.1, 4.5, 2.5);
      assert!(value > 0.0 && value < 1.0, "got {value}");
      let mut current = 2.0;
      let value = input_curve(&mut current, false, true, 1.0, 4.5, 2.5);
      assert_eq!(value, -1.0, "clamped to -1");
   }

   #[test]
   fn normalize_angle_wraps_to_pi() {
      assert_eq!(normalize_angle(0.0), 0.0);
      assert!((normalize_angle(4.0) - (4.0 - core::f32::consts::TAU)).abs() < 1e-5);
      assert!((normalize_angle(-4.0) - (-4.0 + core::f32::consts::TAU)).abs() < 1e-5);
      assert!(normalize_angle(100.0).abs() <= core::f32::consts::PI);
   }

   #[test]
   fn camera_follows_plane_behind() {
      let mut camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 90.0);
      let mut chase = FlightCamera::default();
      let forward = Vec3::Z;
      for _ in 0..60 {
         update_camera(&mut camera, &mut chase, Vec3::ZERO, forward, 1.0 / 60.0);
      }
      // Camera settles behind (-Z) and above (+Y) the plane at the origin.
      // 相机最终位于原点飞机的后方 (-Z) 和上方 (+Y)。
      assert!(camera.position.z < 0.0, "camera should trail the plane");
      assert!(camera.position.y > 0.0, "camera should sit above the plane");
   }

   #[test]
   fn camera_has_no_roll_when_flying_straight() {
      let mut camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 90.0);
      let mut chase = FlightCamera::default();
      update_camera(&mut camera, &mut chase, Vec3::ZERO, Vec3::Z, 1.0 / 60.0);
      // The fixed downward pitch must never roll the view: the up vector keeps zero X component and the camera right axis stays horizontal.
      // 固定的向下俯仰绝不能使视图滚转：上向量保持 X 分量为零，且相机右轴保持水平。
      // Regression: the pitch angle was once passed as the roll slot of `EulerRot::ZYX`, tipping the whole scene by 8.6°.
      // 回归测试：俯仰角曾一度被传入 `EulerRot::ZYX` 的滚转槽位，导致整个场景倾斜 8.6°。
      let up = camera.orientation * Vec3::Y;
      let right = camera.orientation * Vec3::X;
      assert!(up.x.abs() < 1e-4, "camera up rolled, got {up:?}");
      assert!(
         right.y.abs() < 1e-4,
         "camera right not horizontal, got {right:?}"
      );
      // Pitch is really present: the forward vector dips slightly below +Z.
      // 俯仰确实生效：前向向量略微下倾到 +Z 下方。
      let forward = camera.orientation * Vec3::Z;
      assert!(forward.y < 0.0, "camera should look slightly downward");
   }

   fn build_plane_scene(physics: &mut PhysicsWorld, scene: &mut Scene) -> ObjectId {
      let mesh = fallback_aircraft_mesh().expect("plane");
      let mesh_id = scene.meshes.insert(mesh);
      let solid = scene.textures.insert(Texture::solid(ColorRgba8::WHITE));
      let plane = scene.spawn(RenderObject {
         mesh: mesh_id,
         texture: Some(solid),
         transform: Transform::from_position(Vec3::new(0.0, 13.0, 0.0)),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      physics.add_dynamic_box(
         plane,
         Vec3::new(0.5, 0.225, 0.7),
         Transform::from_position(Vec3::new(0.0, 13.0, 0.0)),
      );
      plane
   }

   #[test]
   fn flight_controls_set_forward_velocity() {
      let mut physics = PhysicsWorld::new();
      physics.set_gravity(FLIGHT_GRAVITY);
      let mut scene = Scene::new();
      let plane = build_plane_scene(&mut physics, &mut scene);
      let mut controls = FlightControls::default();
      let mut orientation = Quat::IDENTITY;
      let keys = [false; 256];

      // Airborne: cruise speed is held step after step.
      // 空中飞行：逐帧保持巡航速度。
      for _ in 0..30 {
         let forward = update_flight_controls(
            &mut physics,
            plane,
            &mut orientation,
            &mut controls,
            &keys,
            1.0 / 120.0,
            false,
         )
         .expect("plane exists");
         assert!(forward.length() > 0.99);
         assert!((physics.velocity(plane).length() - CRUISE_SPEED).abs() < 0.01);
      }
   }

   #[test]
   fn controls_pitch_from_w_key() {
      let mut physics = PhysicsWorld::new();
      physics.set_gravity(FLIGHT_GRAVITY);
      let mut scene = Scene::new();
      let plane = build_plane_scene(&mut physics, &mut scene);
      let mut controls = FlightControls::default();
      let mut orientation = Quat::IDENTITY;
      let mut keys = [false; 256];
      keys[b'w' as usize] = true;

      let before = orientation;
      for _ in 0..30 {
         update_flight_controls(
            &mut physics,
            plane,
            &mut orientation,
            &mut controls,
            &keys,
            1.0 / 120.0,
            false,
         );
         physics.step(1.0 / 120.0);
      }
      assert_ne!(before, orientation, "holding W should pitch the plane");
   }

   #[test]
   fn grounded_plane_decelerates_to_stop() {
      let mut physics = PhysicsWorld::new();
      physics.set_gravity(FLIGHT_GRAVITY);
      let mut scene = Scene::new();
      let plane = build_plane_scene(&mut physics, &mut scene);
      let mut controls = FlightControls::default();
      let mut orientation = Quat::IDENTITY;
      let keys = [false; 256];

      // Fly at cruise speed first, then touch ground.
      // 先以巡航速度飞行，再触地。
      update_flight_controls(
         &mut physics,
         plane,
         &mut orientation,
         &mut controls,
         &keys,
         1.0 / 120.0,
         false,
      );
      assert!(
         (physics.velocity(plane).length() - CRUISE_SPEED).abs() < 0.01,
         "airborne plane should cruise at {CRUISE_SPEED}"
      );

      for _ in 0..120 {
         update_flight_controls(
            &mut physics,
            plane,
            &mut orientation,
            &mut controls,
            &keys,
            1.0 / 120.0,
            true,
         );
      }
      assert!(
         physics.velocity(plane).length() < 1.0,
         "grounded plane should slow down, got {:?}",
         physics.velocity(plane)
      );
   }

   #[test]
   fn terrain_manager_fills_neighborhood() {
      let mut scene = Scene::new();
      let placeholder = scene.meshes.insert(Mesh::triangle(1.0, 1.0).unwrap());
      let mut physics = PhysicsWorld::new();
      let mut terrain = TerrainManager::new(&mut scene, placeholder, 1, 7, 12.0);

      terrain.update(Vec3::ZERO);
      let mut budget = 0;
      while budget < 100 {
         terrain.refresh_one(&mut scene, &mut physics);
         let active = terrain.tiles.iter().filter(|tile| tile.active).count();
         if active == 9 {
            break;
         }
         budget += 1;
      }
      assert_eq!(terrain.tiles.iter().filter(|tile| tile.active).count(), 9);
      // Center tile is at the plane's grid cell.
      // 中心地形块位于飞机的网格单元。
      assert!(
         terrain
            .tiles
            .iter()
            .any(|tile| tile.active && tile.grid_x == 0 && tile.grid_z == 0)
      );
      // Physics registered a collider for every active tile.
      // 物理系统为每个激活的地形块注册了碰撞体。
      for tile in terrain.tiles.iter().filter(|tile| tile.active) {
         assert!(physics.has_body(tile.object_id));
      }
   }

   #[test]
   fn terrain_manager_recycles_out_of_range_tiles() {
      let mut scene = Scene::new();
      let placeholder = scene.meshes.insert(Mesh::triangle(1.0, 1.0).unwrap());
      let mut physics = PhysicsWorld::new();
      let mut terrain = TerrainManager::new(&mut scene, placeholder, 1, 7, 12.0);

      terrain.update(Vec3::new(0.5, 0.0, 0.5));
      for _ in 0..20 {
         terrain.refresh_one(&mut scene, &mut physics);
      }
      assert_eq!(terrain.tiles.iter().filter(|tile| tile.active).count(), 9);

      // Move far away: old tiles deactivate, then a fresh neighborhood forms.
      // 飞远后：旧地形块停用，随后形成新的邻域。
      terrain.update(Vec3::new(300.0, 0.0, 300.0));
      for _ in 0..20 {
         terrain.refresh_one(&mut scene, &mut physics);
      }
      assert_eq!(terrain.tiles.iter().filter(|tile| tile.active).count(), 9);
      let (center_x, center_z) = (terrain.center_x, terrain.center_z);
      assert!(
         terrain.tiles.iter().all(|tile| !tile.active
            || tile_in_range(
               tile.grid_x,
               tile.grid_z,
               center_x,
               center_z,
               terrain.tile_radius
            )),
         "all active tiles must be inside the radius"
      );
   }
}

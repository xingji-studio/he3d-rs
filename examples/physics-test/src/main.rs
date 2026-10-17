//! `PhysicsTest` demo: a controllable dynamic cube over a static floor.
//! 物理测试示例：静态地板上一个可操控的动态方块。
//!
//! The engine owns the cube's `BodyId`; this application owns the link between
//! that Body and the `RenderObject` that draws it, including the per-frame copy of
//! the Body's placement into the render transform. See `docs/adr/0001`.
//! 引擎拥有方块的 `BodyId`；Body 与绘制它的 `RenderObject` 之间的联系、以及每帧把 Body
//! 摆放复制进渲染变换的动作，都由本应用拥有。见 `docs/adr/0001`。
use he3d::{
   BackendConfig, BodyDesc, BodyId, Camera, ColorRgba8, ContactPhase, Event, KEY_ESCAPE, KeyCode,
   Material, Mesh, ObjectId, Platform, Quat, RenderObject, Renderer, RendererConfig, Rgb, Scene,
   Shape, Texture, Transform, Vec3, World,
};
use he3d_platform_sdl3::SdlBackend;

const WINDOW_WIDTH: u32 = 640;
const WINDOW_HEIGHT: u32 = 360;

const FLOOR_POS: Vec3 = Vec3::new(0.0, -0.6, 4.0);
const FLOOR_HALF: Vec3 = Vec3::new(3.5, 0.1, 3.5);
const CUBE_RESET_POS: Vec3 = Vec3::new(0.0, 3.0, 4.0);
const CUBE_HALF: f32 = 0.5;

/// Torque the arrow keys apply, about the horizontal Z axis.
/// 方向键施加的力矩，绕水平的 Z 轴。
///
/// A cube resting on the floor resists a roll until the torque exceeds `m·g·half_side`, here `1 × 9.81 × 0.5 ≈ 4.9`; below that it cannot turn at all, which is why the old 4.0 did nothing on its own.
/// 静置在地板上的方块要克服 `m·g·半边长`（此处 `1 × 9.81 × 0.5 ≈ 4.9`）才会翻滚；低于它根本转不动——这正是原先 4.0 单独施加时毫无反应的原因。
/// 6.0 leaves margin and starts the roll within a fraction of a second.
/// 6.0 留有余量，零点几秒内就开始翻滚。
const ARROW_TORQUE: f32 = 6.0;

/// The left and right arrows arrive as SDL keycodes, passed through unchanged.
/// They do not fit the 256-entry ASCII-only `keys` array below, so they are tracked separately.
/// 左右方向键以 SDL 键码的形式到达，原样传递。
/// 它们放不下下面只覆盖 ASCII 的 256 项 `keys` 数组，因此单独跟踪。
const KEY_LEFT: KeyCode = 0x4000_0050;
const KEY_RIGHT: KeyCode = 0x4000_004f;

fn main() {
   let frames = parse_frames(std::env::args());

   let config = BackendConfig::new("HE3D PhysicsTest", WINDOW_WIDTH, WINDOW_HEIGHT);
   let mut backend = match SdlBackend::new(config) {
      Ok(backend) => backend,
      Err(error) => {
         eprintln!("backend init failed: {error}");
         std::process::exit(1);
      }
   };

   let mut renderer = match Renderer::new(RendererConfig::new(WINDOW_WIDTH, WINDOW_HEIGHT)) {
      Ok(renderer) => renderer,
      Err(error) => {
         eprintln!("renderer init failed: {error}");
         std::process::exit(1);
      }
   };

   let mut scene = Scene::new();
   // A gentle key light from just behind and above the camera, so the cube and the
   // floor read with some depth instead of blowing out to white.
   // 从相机正后方稍上方打一束柔和的主光，使立方体与地面有立体感，又不至于过曝成白色。
   scene.light.direction = Vec3::new(0.0, 0.5, -1.0).normalize();
   scene.light.color = Rgb::new(0.9, 0.9, 0.9);
   scene.light.ambient = 0.2;
   let floor_mesh = scene
      .meshes
      .insert(Mesh::cube(7.0, 0.2, 7.0).expect("floor mesh"));
   let cube_mesh = scene
      .meshes
      .insert(Mesh::cube(1.0, 1.0, 1.0).expect("cube mesh"));
   let floor_solid = scene
      .textures
      .insert(Texture::solid(ColorRgba8::from_rgb(64, 140, 89)));
   let cube_solid = scene
      .textures
      .insert(Texture::solid(ColorRgba8::from_rgb(89, 166, 255)));
   // The floor never moves, so it needs no per-frame synchronisation.
   // 地面永不移动，因此不需要每帧同步。
   scene.spawn(RenderObject {
      mesh: floor_mesh,
      texture: Some(floor_solid),
      transform: Transform::from_position(FLOOR_POS),
      material: Material::new(Rgb::WHITE),
      visible: true,
   });
   let cube = scene.spawn(RenderObject {
      mesh: cube_mesh,
      texture: Some(cube_solid),
      transform: Transform::from_position(CUBE_RESET_POS),
      material: Material::new(Rgb::WHITE),
      visible: true,
   });
   let camera = Camera::new(Vec3::new(0.0, 1.4, -5.0), Quat::IDENTITY, 50.0);

   let mut world = World::new();
   // The floor is a static box: the mesh it draws is exactly this cuboid, so the
   // collider and the picture agree by construction.
   // 地面是静态盒：它绘制的网格正是这个长方体，因此碰撞体与画面在构造上一致。
   let floor_body = world.add_body(BodyDesc::stationary(Shape::cuboid(FLOOR_HALF)).at(FLOOR_POS));
   // Friction low enough to slide when pushed, restitution high enough that a drop
   // visibly bounces, and light damping so a held key does not accelerate forever.
   // 摩擦低到能被推得滑动，弹性高到落地有可见回弹，阻尼轻到按住方向键不会无限加速。
   let cube_body = world.add_body(
      BodyDesc::dynamic(Shape::cuboid(Vec3::splat(CUBE_HALF)), 1.0)
         .at(CUBE_RESET_POS)
         .with_friction(1.0)
         .with_restitution(0.5)
         .with_damping(0.05, 0.05),
   );
   // The cube starts at rest. It used to be launched at 1 m/s along +x, which is
   // why it landed 0.77 m right of centre — the fall is long enough to carry it
   // there, and the velocity then dies against friction. That was inherited from a
   // demo this file no longer has any reason to mirror, so the spawn is a plain
   // vertical drop onto the middle of the floor.
   // 方块自静止开始。它曾被以 1 m/s 沿 +x 抛出，这正是它落点偏离中心右侧 0.77 米的原因——
   // 下落时间足够把它带到那里，随后速度被摩擦耗尽。那个做法继承自一个本文件已无任何理由
   // 去效仿的演示，因此现在就是在底板正中垂直落下。
   let _ = floor_body;

   let mut keys = [false; 256];
   let mut turn_left = false;
   let mut turn_right = false;
   let mut jump_was_down = false;
   let mut grounded = false;
   let mut last_time = backend.time_seconds();
   let mut accumulator = 0.0_f64;
   const PHYSICS_STEP: f64 = 1.0 / 120.0;
   const MAX_PHYSICS_STEPS_PER_FRAME: u32 = 8;
   let mut frame = 0u32;

   loop {
      for event in backend.poll_events() {
         match event {
            Event::KeyDown(KEY_ESCAPE) => backend.request_close(),
            Event::KeyDown(KEY_LEFT) | Event::KeyUp(KEY_LEFT) => {
               turn_left = matches!(event, Event::KeyDown(_));
            }
            Event::KeyDown(KEY_RIGHT) | Event::KeyUp(KEY_RIGHT) => {
               turn_right = matches!(event, Event::KeyDown(_));
            }
            Event::KeyDown(code) | Event::KeyUp(code)
               if (0..keys.len()).contains(&(code as usize)) =>
            {
               keys[code as usize] = matches!(event, Event::KeyDown(_));
            }
            _ => {}
         }
      }
      if backend.should_close() {
         break;
      }
      if let Some(limit) = frames {
         if frame >= limit {
            break;
         }
      }

      let now = backend.time_seconds();
      let frame_delta = now - last_time;
      last_time = now;

      // Drive physics from a fixed clock. The engine exposes only a fixed-step
      // entry point, so the accumulator and the per-frame cap live here.
      // 使用固定物理时钟。引擎只暴露固定步长入口，因此累加器与每帧步数上限都在这里。
      let frame_delta = if frame_delta.is_finite() {
         frame_delta.clamp(0.0, 0.25)
      } else {
         0.0
      };
      accumulator += frame_delta;

      if keys[b'r' as usize] {
         reset_cube(&mut world, cube_body);
         grounded = false;
      }

      let mut physics_steps = 0;
      while accumulator >= PHYSICS_STEP && physics_steps < MAX_PHYSICS_STEPS_PER_FRAME {
         apply_input(
            &mut world,
            cube_body,
            &keys,
            i32::from(turn_right) - i32::from(turn_left),
            grounded,
            &mut jump_was_down,
         );
         world.step(PHYSICS_STEP as f32);
         accumulator -= PHYSICS_STEP;
         physics_steps += 1;
         grounded = is_grounded(&world, cube_body);
      }
      if physics_steps == MAX_PHYSICS_STEPS_PER_FRAME && accumulator >= PHYSICS_STEP {
         accumulator = 0.0;
      }
      sync_pose(&world, &mut scene, cube, cube_body);

      if scene.objects.get(cube).unwrap().transform.position.y < -5.0 {
         reset_cube(&mut world, cube_body);
         grounded = false;
      }

      let cube_color = if grounded {
         Rgb::new(0.95, 0.75, 0.25)
      } else {
         Rgb::new(0.35, 0.65, 1.0)
      };
      if let Some(object) = scene.objects.get_mut(cube) {
         object.material.tint = cube_color;
      }

      renderer
         .render(&scene, &camera, Rgb::new(0.08, 0.10, 0.13))
         .expect("render frame");
      backend
         .present(renderer.width(), renderer.height(), renderer.pixels())
         .expect("present frame");
      frame += 1;
      // No frame-rate lock: render as fast as the software rasterizer can;
      // simulation time is kept independent by the fixed-step accumulator.
      // 不限制帧率：以软件光栅化器所能达到的最快速度渲染；模拟时间由固定步长累加器独立维持。
   }
}

/// Copy a Body's placement into the render object that draws it.
/// 将 Body 的摆放复制进绘制它的渲染对象。
///
/// The engine deliberately provides no binding layer, so this is where the
/// association between the two halves of the scene is honoured.
/// 引擎刻意不提供绑定层，因此场景两半之间的关联在这里被兑现。
fn sync_pose(world: &World, scene: &mut Scene, object: ObjectId, body: BodyId) {
   let (Some(position), Some(orientation)) = (world.position(body), world.orientation(body)) else {
      return;
   };
   if let Some(render_object) = scene.objects.get_mut(object) {
      render_object.transform.position = position;
      render_object.transform.orientation = orientation;
      // Scale belongs to the renderer alone; physics never sees it.
      // 缩放只属于渲染器；物理永远看不到它。
   }
}

/// Apply one step of player input.
/// 应用一步玩家输入。
///
/// `A`/`D` push the cube sideways and impart no spin.
/// The left and right arrows roll it about the horizontal Z axis and push nothing.
/// The two axes are separate knobs, so sliding the cube no longer rolls it.
/// `turn` is `-1` for the left arrow, `+1` for the right, and `0` for neither.
/// Its signs match what `A`/`D` used to impart, so the cube rolls the way it always did.
/// `A`/`D` 把方块往左右推，不施加自转。
/// 左右箭头绕水平的 Z 轴使其翻滚、不施加推力。
/// 两个轴互不干扰，因此把方块推过地板时它不会再翻滚。
/// `turn` 为 `-1`（左箭头）、`+1`（右箭头）或 `0`（都没按）。
/// 其符号与原先 `A`/`D` 施加的一致，方块的滚动方向与以往相同。
fn apply_input(
   world: &mut World,
   cube: BodyId,
   keys: &[bool; 256],
   turn: i32,
   grounded: bool,
   jump_was_down: &mut bool,
) {
   if keys[b'a' as usize] {
      world.apply_force(cube, Vec3::new(-14.0, 0.0, 0.0));
   }
   if keys[b'd' as usize] {
      world.apply_force(cube, Vec3::new(14.0, 0.0, 0.0));
   }
   if turn < 0 {
      world.apply_torque(cube, Vec3::new(0.0, 0.0, ARROW_TORQUE));
   } else if turn > 0 {
      world.apply_torque(cube, Vec3::new(0.0, 0.0, -ARROW_TORQUE));
   }

   let jump_down = keys[b' ' as usize];
   if jump_down && !*jump_was_down && grounded {
      world.apply_impulse(cube, Vec3::new(0.0, 5.0, 0.0));
   }
   *jump_was_down = jump_down;
}

/// Return the cube to its starting pose, at rest.
/// 将方块恢复到起始位姿，且处于静止。
///
/// A reset means "as if the demo had just started", so it must not hand the cube a
/// velocity the spawn does not: doing that made pressing `R` look like the engine
/// was shoving the cube sideways.
/// 重置意味着"就像演示刚刚启动"，因此不能给方块一个 spawn 时没有的速度：那样做会让按下
/// `R` 看起来像是引擎在把方块往侧面推。
fn reset_cube(world: &mut World, cube: BodyId) {
   world.set_pose(cube, CUBE_RESET_POS, Quat::IDENTITY);
   world.set_velocity(cube, Vec3::ZERO);
   world.set_angular_velocity(cube, Vec3::ZERO);
}

/// Whether the cube is touching anything right now.
/// 方块当前是否与任何东西接触。
///
/// This is the whole answer: a live Contact involving the cube, straight from the
/// step that just ran. No geometry is recomputed and no tolerance is tuned.
/// 这就是全部答案：刚跑完的那一步产出的、涉及方块的一个存活 Contact。无需重算几何，
/// 也无需调参任何容差。
fn is_grounded(world: &World, cube: BodyId) -> bool {
   world
      .contacts()
      .iter()
      .any(|contact| contact.involves(cube) && contact.phase != ContactPhase::End)
}

fn parse_frames(mut args: impl Iterator<Item = String>) -> Option<u32> {
   let mut frames = None;
   while let Some(arg) = args.next() {
      if arg == "--frames" {
         frames = args.next().and_then(|value| value.parse().ok());
      }
   }
   frames
}

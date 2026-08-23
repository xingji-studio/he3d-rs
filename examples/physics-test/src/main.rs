//! `PhysicsTest` demo: a controllable dynamic cube over a static floor.
//! 物理测试示例：静态地板上一个可操控的动态方块。
use he3d::{
   BackendConfig, Camera, ColorRgba8, Event, KEY_ESCAPE, Material, Mesh, ObjectId, PhysicsWorld,
   Platform, Quat, RenderObject, Renderer, RendererConfig, Rgb, Scene, Texture, Transform, Vec3,
   create_backend,
};

const WINDOW_WIDTH: u32 = 640;
const WINDOW_HEIGHT: u32 = 360;

const FLOOR_POS: Vec3 = Vec3::new(0.0, -0.6, 4.0);
const FLOOR_HALF: Vec3 = Vec3::new(3.5, 0.1, 3.5);
const CUBE_RESET_POS: Vec3 = Vec3::new(0.0, 3.0, 4.0);
const CUBE_HALF: f32 = 0.5;

fn main() {
   let frames = parse_frames(std::env::args());

   let config = BackendConfig {
      title: "HE3D PhysicsTest",
      width: WINDOW_WIDTH,
      height: WINDOW_HEIGHT,
   };
   let backend = match create_backend(config) {
      Ok(backend) => backend,
      Err(error) => {
         eprintln!("backend init failed: {error}");
         std::process::exit(1);
      }
   };
   run(backend, frames);
}

fn run(mut backend: Box<dyn Platform>, frames: Option<u32>) {
   let mut renderer = match Renderer::new(RendererConfig::new(WINDOW_WIDTH, WINDOW_HEIGHT)) {
      Ok(renderer) => renderer,
      Err(error) => {
         eprintln!("renderer init failed: {error}");
         std::process::exit(1);
      }
   };

   let mut scene = Scene::new();
   // The reference default light (0,-1,1) leaves every camera-facing surface backlit (only the 15% ambient), so the demo renders almost pitch black.
   // 参考默认光源 (0,-1,1) 会让所有面向相机的表面处于背光状态（仅有 15% 环境光），导致演示几乎渲染成漆黑一片。
   // Use a gentle key light from just behind and above the camera so the cube and floor are visible with some depth, without blowing out to white.
   // 从相机正后方稍上方打一束柔和的主光，使立方体与地面可见且富有立体感，又不至于过曝成白色。
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
   let floor = scene.spawn(RenderObject {
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

   let mut physics = PhysicsWorld::new();
   // The reference PhysicsTest uses a `MeshCollider` floor (box-box SAT is unstable).
   // 参考 PhysicsTest 使用 `MeshCollider` 地面（盒-盒 SAT 不稳定）。
   physics.add_static_mesh(
      floor,
      scene.meshes.get(floor_mesh).expect("floor mesh"),
      Transform::from_position(FLOOR_POS),
   );
   physics.add_dynamic_box(
      cube,
      Vec3::splat(CUBE_HALF),
      Transform::from_position(CUBE_RESET_POS),
   );
   // Match the reference PhysicsTest's cube material: modest friction, a noticeable bounce, and light damping so the push doesn't run away.
   // 匹配参考 PhysicsTest 的立方体材质：适中的摩擦力、明显的弹性和轻微阻尼，使推力不会失控。
   physics.set_material(cube, 0.45, 0.5);
   physics.set_damping(cube, 0.05, 0.05);
   physics.set_velocity(cube, Vec3::new(1.0, 0.0, 0.0));

   let mut keys = [false; 256];
   let mut jump_was_down = false;
   let mut grounded = false;
   let mut last_time = backend.time_seconds();
   let mut frame = 0u32;

   loop {
      for event in backend.poll_events() {
         match event {
            Event::KeyDown(KEY_ESCAPE) => backend.request_close(),
            Event::KeyDown(code) | Event::KeyUp(code) => {
               if (0..keys.len()).contains(&(code as usize)) {
                  keys[code as usize] = matches!(event, Event::KeyDown(_));
               }
            }
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
      let delta = ((now - last_time) as f32).clamp(0.0, 0.05);
      last_time = now;

      if keys[b'r' as usize] {
         reset_cube(&mut physics, cube);
         grounded = false;
      }

      apply_input(&mut physics, cube, &keys, grounded, &mut jump_was_down);

      physics.step(delta);
      physics.synchronize(&mut scene);

      grounded = is_grounded(&physics, cube);
      if scene.objects.get(cube).unwrap().transform.position.y < -5.0 {
         reset_cube(&mut physics, cube);
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
      // No frame-rate lock: render as fast as the software rasterizer can.
      // 不限制帧率：以软件光栅化器所能达到的最快速度渲染。
   }
}

fn apply_input(
   physics: &mut PhysicsWorld,
   cube: ObjectId,
   keys: &[bool; 256],
   grounded: bool,
   jump_was_down: &mut bool,
) {
   if keys[b'a' as usize] {
      physics.add_force(cube, Vec3::new(-14.0, 0.0, 0.0));
      physics.add_torque(cube, Vec3::new(0.0, 0.0, 4.0));
   }
   if keys[b'd' as usize] {
      physics.add_force(cube, Vec3::new(14.0, 0.0, 0.0));
      physics.add_torque(cube, Vec3::new(0.0, 0.0, -4.0));
   }

   let jump_down = keys[b' ' as usize];
   if jump_down && !*jump_was_down && grounded {
      physics.add_impulse(cube, Vec3::new(0.0, 5.0, 0.0));
   }
   *jump_was_down = jump_down;
}

fn reset_cube(physics: &mut PhysicsWorld, cube: ObjectId) {
   physics.set_transform(cube, Transform::from_position(CUBE_RESET_POS));
   physics.set_velocity(cube, Vec3::new(1.0, 0.0, 0.0));
   physics.set_angular_velocity(cube, Vec3::ZERO);
}

fn is_grounded(physics: &PhysicsWorld, cube: ObjectId) -> bool {
   // The reference PhysicsTest checks an unrotated AABB overlap against the floor
   // with tolerances, plus a slow vertical velocity. Ported verbatim: it is stable
   // at rest, whereas recomputed contacts require positive penetration each step.
   // 参考 PhysicsTest 使用未旋转的 AABB 与地面做带容差的重叠判定，并附加较慢的
   // 垂直速度条件。这里原样移植：它在静止时稳定，而重算的接触要求每步都有正穿透。
   let Some(position) = physics.transform(cube).map(|transform| transform.position) else {
      return false;
   };
   let cube_min = position - Vec3::splat(CUBE_HALF);
   let cube_max = position + Vec3::splat(CUBE_HALF);
   let floor_min = FLOOR_POS - FLOOR_HALF;
   let floor_max = FLOOR_POS + FLOOR_HALF;
   let overlaps_x = cube_max.x >= floor_min.x - 0.02 && cube_min.x <= floor_max.x + 0.02;
   let overlaps_z = cube_max.z >= floor_min.z - 0.02 && cube_min.z <= floor_max.z + 0.02;
   let near_floor = cube_min.y <= floor_max.y + 0.06 && cube_min.y >= floor_max.y - 0.35;
   overlaps_x && overlaps_z && near_floor && physics.velocity(cube).y <= 0.25
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

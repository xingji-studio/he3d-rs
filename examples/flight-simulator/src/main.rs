mod flight_sim;

use flight_sim::{
   FlightCamera, FlightControls, TerrainManager, fallback_aircraft_mesh, update_camera,
   update_flight_controls,
};
use he3d_assets::{decode_image, parse_obj};
use he3d_core::{Camera, Material, Mesh, RenderObject, Rgb, Scene, Transform};
use he3d_math::{Quat, Vec3};
use he3d_physics::PhysicsWorld;
use he3d_platform::{BackendConfig, Event, KEY_ESCAPE, PlatformFactory};
use he3d_renderer::{Renderer, RendererConfig};

const WINDOW_WIDTH: u32 = 800;
const WINDOW_HEIGHT: u32 = 600;

const OBJ_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/biplane.obj");
const BMP_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/biplane.bmp");

const TILE_RADIUS: i32 = 1;
const TERRAIN_STEP: f32 = 12.0;
const TERRAIN_GRID_COUNT: i32 = 7;

/// Spawn point and reset position for the aircraft.
/// 飞机的出生点与重置位置。
const SPAWN_POS: Vec3 = Vec3::new(0.0, 13.0, 0.0);

/// Below this ground speed (with contact) the plane is considered crashed.
/// 低于该地面速度（且存在接触）时判定飞机坠毁。
const CRASH_SPEED: f32 = 3.0;

fn main() {
   let config = BackendConfig {
      title: "HE3D Flight Simulator",
      width: WINDOW_WIDTH,
      height: WINDOW_HEIGHT,
   };
   #[cfg(feature = "sdl3")]
   run::<he3d_platform_sdl3::SdlBackend>(config);
   #[cfg(feature = "terminal")]
   run::<he3d_platform_std::TerminalBackend>(config);
}

fn run<B: PlatformFactory>(config: BackendConfig<'_>) {
   let frames = parse_frames(std::env::args());

   let mut backend = match B::create(config) {
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

   let plane_mesh = match read_obj() {
      Some(mesh) => mesh,
      None => match fallback_aircraft_mesh() {
         Ok(mesh) => mesh,
         Err(error) => {
            eprintln!("fallback aircraft mesh failed: {error}");
            std::process::exit(1);
         }
      },
   };
   let plane_texture = read_texture();

   let mut scene = Scene::new();
   scene.light.direction = Vec3::new(1.0, 1.0, 0.5).normalize();
   scene.light.color = Rgb::new(1.5, 1.4, 1.2);

   let plane_mesh_id = scene.meshes.insert(plane_mesh);
   let plane_texture_id = plane_texture.map(|texture| scene.textures.insert(texture));
   let plane = scene.spawn(RenderObject {
      mesh: plane_mesh_id,
      texture: plane_texture_id,
      transform: Transform::from_position(SPAWN_POS),
      material: Material::new(Rgb::WHITE),
      visible: true,
   });

   let placeholder_mesh = scene
      .meshes
      .insert(Mesh::triangle(1.0, 1.0).expect("triangle"));
   let mut terrain = TerrainManager::new(
      &mut scene,
      placeholder_mesh,
      TILE_RADIUS,
      TERRAIN_GRID_COUNT,
      TERRAIN_STEP,
   );

   let mut physics = PhysicsWorld::new();
   physics.set_gravity(flight_sim::FLIGHT_GRAVITY);
   physics.add_dynamic_box(
      plane,
      Vec3::new(0.5, 0.225, 0.7),
      Transform::from_position(SPAWN_POS),
   );
   // Reference material: modest friction, slight bounce off the terrain.
   // 参考材质：适中的摩擦力，落地有轻微回弹。
   physics.set_material(plane, 0.5, 0.1);

   let mut camera = Camera::new(Vec3::new(0.0, 12.0, 5.0), Quat::IDENTITY, 90.0);
   let mut chase = FlightCamera::default();
   let mut controls = FlightControls::default();
   let mut keys = [false; 256];

   let mut last_time = backend.time_seconds();
   let mut fps_start = last_time;
   let mut fps_frames = 0u32;
   let mut accumulator = 0.0f64;
   const PHYSICS_STEP: f64 = 1.0 / 120.0;
   let mut previous_position = SPAWN_POS;
   let mut previous_orientation = Quat::IDENTITY;
   let mut controlled_orientation = Quat::IDENTITY;
   let mut crashed = false;
   // Ground-contact stickiness: once the plane touches down it stays in the decelerating "grounded" mode even across bounces, so intermittent contact cannot let it resume cruise speed.
   // 着地“粘滞”效应：飞机一旦触地就保持在减速的“着地”模式，即使发生弹跳也不变，因此间歇性接触不会让它恢复巡航速度。
   // It only lifts off again after a short period without any contact (~0.08 s), long enough to tell a bounce from a real departure.
   // 只有在持续一段时间（约 0.08 秒）无接触后才会再次起飞，足以区分弹跳与真正的离地。
   let mut grounded = false;
   let mut lift_off_frames = 0u32;
   const LIFT_OFF_FRAMES: u32 = 10;
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

      if keys[b'r' as usize] {
         reset_plane(&mut physics, plane);
         controlled_orientation = Quat::IDENTITY;
         controls = FlightControls::default();
         crashed = false;
         grounded = false;
         lift_off_frames = 0;
      }

      let now = backend.time_seconds();
      let frame_delta = now - last_time;
      last_time = now;
      // Clamp, never zero: a long frame (pause, stall, window drag) queues up to
      // 0.25 s of simulation instead of freezing the physics in place. The fixed
      // accumulator drains it at up to 8 substeps per frame, so the sim catches up
      // without a spiral of death.
      // 采用钳制而非清零：长帧（暂停、卡顿、拖动窗口）最多积压 0.25 秒模拟，
      // 而不是让物理原地冻结。固定累加器每帧最多消耗 8 个子步，因此模拟会追平
      // 时间而不会陷入"死亡螺旋"。
      let frame_delta = if frame_delta < 0.0 || frame_delta.is_nan() {
         0.0
      } else {
         frame_delta.min(0.25)
      };

      accumulator += frame_delta;
      let mut iterations = 0;
      while accumulator >= PHYSICS_STEP && iterations < 8 {
         previous_position = physics.transform(plane).unwrap().position;
         previous_orientation = physics.transform(plane).unwrap().orientation;

         if let Some(position) = physics.transform(plane) {
            terrain.update(position.position);
            terrain.refresh_one(&mut scene, &mut physics);
            if crashed {
               // The wreck is no longer controlled: hand the plane back to the physics step so it settles against the terrain.
               // 残骸不再受控：将飞机交还给物理步进，使其贴合地形静止下来。
               // Gravity pulls it down, contact keeps it from sinking, residual spin is damped out.
               // 重力使其下压，接触防止其下沉，残余自旋被衰减。
               // Strong exponential damping bleeds off the remaining velocity and angular velocity so it comes to rest where it hit instead of sliding forever.
               // 强指数阻尼会逐渐消耗残余的线速度与角速度，使其在撞击处静止而非永久滑行。
               let damp = (-flight_sim::CRASH_DAMPING * PHYSICS_STEP as f32).exp();
               let velocity = physics.velocity(plane) * damp;
               physics.set_velocity(plane, velocity);
               let spin = physics.angular_velocity(plane) * damp;
               physics.set_angular_velocity(plane, spin);
            } else {
               let contacted = physics.has_contacts(plane);
               if contacted {
                  grounded = true;
                  lift_off_frames = 0;
               } else if grounded {
                  lift_off_frames += 1;
                  if lift_off_frames >= LIFT_OFF_FRAMES {
                     grounded = false;
                     lift_off_frames = 0;
                  }
               }
               update_flight_controls(
                  &mut physics,
                  plane,
                  &mut controlled_orientation,
                  &mut controls,
                  &keys,
                  PHYSICS_STEP as f32,
                  grounded,
               );
               if grounded && physics.velocity(plane).length() < CRASH_SPEED {
                  crashed = true;
                  physics.set_velocity(plane, Vec3::ZERO);
                  physics.set_angular_velocity(plane, Vec3::ZERO);
               }
            }
         }
         physics.step(PHYSICS_STEP as f32);

         accumulator -= PHYSICS_STEP;
         iterations += 1;
      }
      if iterations == 8 && accumulator >= PHYSICS_STEP {
         accumulator = 0.0;
      }
      let alpha = (accumulator / PHYSICS_STEP) as f32;
      let live = physics.transform(plane).unwrap();
      let presentation_position = previous_position.lerp(live.position, alpha);
      let presentation_orientation = previous_orientation
         .slerp(live.orientation, alpha)
         .normalize();
      let forward = presentation_orientation * Vec3::Z;

      if let Some(object) = scene.objects.get_mut(plane) {
         object.transform.position = presentation_position;
         object.transform.orientation = presentation_orientation;
      }

      let render_delta = if frame_delta > 0.0 {
         frame_delta as f32
      } else {
         PHYSICS_STEP as f32
      };
      update_camera(
         &mut camera,
         &mut chase,
         presentation_position,
         forward,
         render_delta,
      );

      let draw_radius = terrain.tile_size * 1.45;
      for tile in &terrain.tiles {
         if let Some(object) = scene.objects.get_mut(tile.object_id) {
            let tile_pos = Vec3::new(
               tile.grid_x as f32 * terrain.tile_size,
               0.0,
               tile.grid_z as f32 * terrain.tile_size,
            );
            let delta = tile_pos - camera.position;
            let within_radius = delta.x * delta.x + delta.z * delta.z <= draw_radius * draw_radius;
            object.visible = tile.active && within_radius;
         }
      }

      renderer
         .render(&scene, &camera, Rgb::new(0.45, 0.75, 1.0))
         .expect("render frame");
      backend
         .present(renderer.width(), renderer.height(), renderer.pixels())
         .expect("present frame");
      frame += 1;

      fps_frames += 1;
      let fps_elapsed = now - fps_start;
      if crashed {
         backend.set_title("HE3D Flight Simulator - CRASHED (press R)");
      } else if fps_elapsed >= 1.0 {
         let fps = ((fps_frames as f64) / fps_elapsed).round() as u32;
         backend.set_title(&format!("HE3D Flight Simulator - FPS {fps}"));
         fps_start = now;
         fps_frames = 0;
      }
      // No frame-rate lock: render as fast as the software rasterizer can, matching the reference simulator (SetFrameRateLimit(0)).
      // 不限制帧率：以软件光栅化器所能达到的最快速度渲染，与参考模拟器一致（SetFrameRateLimit(0)）。
   }
}

fn read_obj() -> Option<Mesh> {
   let bytes = std::fs::read(OBJ_PATH).ok()?;
   match parse_obj(&bytes) {
      Ok(mesh) => Some(mesh),
      Err(error) => {
         eprintln!("failed to parse biplane.obj: {error}");
         None
      }
   }
}

fn reset_plane(physics: &mut PhysicsWorld, plane: he3d_core::ObjectId) {
   physics.set_transform(plane, Transform::from_position(SPAWN_POS));
   physics.set_velocity(plane, Vec3::ZERO);
   physics.set_angular_velocity(plane, Vec3::ZERO);
}

fn read_texture() -> Option<he3d_core::Texture> {
   let bytes = std::fs::read(BMP_PATH).ok()?;
   match decode_image(&bytes) {
      Ok(texture) => Some(texture),
      Err(error) => {
         eprintln!("failed to decode biplane.bmp: {error}");
         None
      }
   }
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

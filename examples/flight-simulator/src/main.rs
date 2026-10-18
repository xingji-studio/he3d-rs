//! Flight Simulator demo: streamed procedural terrain and a textured biplane.
//! 飞行模拟示例：流式程序化地形与带纹理双翼机。
mod flight_sim;

use flight_sim::{
   FlightCamera, FlightControls, TerrainManager, fallback_aircraft_mesh, update_camera,
   update_flight_controls,
};
use he3d::{
   BackendConfig, BodyDesc, BodyId, Camera, ContactPhase, Event, KEY_ESCAPE, Material, Mesh,
   Platform, Quat, RenderObject, Renderer, RendererConfig, Rgb, Scene, Shape, Texture, Transform,
   Vec3, World, decode_image, parse_obj,
};
use he3d_platform_sdl3::SdlBackend;

const WINDOW_WIDTH: u32 = 800;
const WINDOW_HEIGHT: u32 = 600;

const OBJ_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/biplane.obj");
const BMP_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../assets/biplane.bmp");

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
   let frames = parse_frames(std::env::args());

   let config = BackendConfig::new("HE3D Flight Simulator", WINDOW_WIDTH, WINDOW_HEIGHT);
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

   // The Body is the engine's; `plane` names the render object that draws it, and
   // the two are related only here. See `docs/adr/0001`.
   // Body 归引擎所有；`plane` 是绘制它的渲染对象的名字，两者只在这里发生联系。见
   // `docs/adr/0001`。
   let mut world = World::new();
   world.set_gravity(flight_sim::FLIGHT_GRAVITY);
   // The plane is one Body against immovable terrain, so a handful of iterations
   // is plenty and keeps the frame budget for the rasterizer.
   // 飞机是单个 Body 与不可移动地形作用，少数几次迭代就足够，把帧预算留给光栅化器。
   world.set_velocity_iterations(4);
   let plane_body = world.add_body(
      BodyDesc::dynamic(Shape::cuboid(Vec3::new(0.5, 0.225, 0.7)), 1.0)
         .at(SPAWN_POS)
         .with_friction(0.5)
         .with_restitution(0.1)
         .with_damping(0.1, 0.1)
         // The flight model owns this Body's velocity: it writes the whole vector
         // every step, so gravity reaching it could only ever leak into the position
         // as a steady sink. The wreck gets gravity back below.
         // 飞行模型拥有此 Body 的速度：它每步重写整个向量，因此重力作用于它只会以恒定下沉的
         // 形式漏进位置。残骸会在下面把重力要回去。
         .with_gravity_scale(0.0),
   );

   let mut camera = Camera::perspective(Vec3::new(0.0, 12.0, 5.0), Quat::IDENTITY, 90.0);
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

      if keys[b'r' as usize] {
         reset_plane(&mut world, plane_body);
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
         previous_position = world.position(plane_body).unwrap();
         previous_orientation = world.orientation(plane_body).unwrap();

         if let Some(position) = world.position(plane_body) {
            terrain.update(position);
            terrain.refresh_one(&mut scene, &mut world);
            if crashed {
               // The wreck is no longer controlled: hand the plane back to the physics step so it settles against the terrain.
               // 残骸不再受控：将飞机交还给物理步进，使其贴合地形静止下来。
               // Gravity pulls it down, contact keeps it from sinking, residual spin is damped out.
               // 重力使其下压，接触防止其下沉，残余自旋被衰减。
               // Strong exponential damping bleeds off the remaining velocity and angular velocity so it comes to rest where it hit instead of sliding forever.
               // 强指数阻尼会逐渐消耗残余的线速度与角速度，使其在撞击处静止而非永久滑行。
               let damp = (-flight_sim::CRASH_DAMPING * PHYSICS_STEP as f32).exp();
               let velocity = world.velocity(plane_body).unwrap_or(Vec3::ZERO) * damp;
               world.set_velocity(plane_body, velocity);
               let spin = world.angular_velocity(plane_body).unwrap_or(Vec3::ZERO) * damp;
               world.set_angular_velocity(plane_body, spin);
            } else {
               // "Am I touching anything" is answered by the Contact list the last
               // step produced; no geometry is recomputed here.
               // "我碰到东西了吗"由上一步产出的 Contact 列表回答；这里不重算任何几何。
               let contacted = world.contacts().iter().any(|contact| {
                  contact.involves(plane_body) && contact.phase != ContactPhase::End
               });
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
                  &mut world,
                  plane_body,
                  &mut controlled_orientation,
                  &mut controls,
                  &keys,
                  PHYSICS_STEP as f32,
                  grounded,
               );
               if grounded
                  && world.velocity(plane_body).unwrap_or(Vec3::ZERO).length() < CRASH_SPEED
               {
                  crashed = true;
                  // Hand the wreck back to gravity so it falls and settles against
                  // the terrain instead of hanging where it stopped.
                  // 把残骸交还给重力，使其下坠并贴合地形，而不是停在原地悬着。
                  world.set_gravity_scale(plane_body, 1.0);
                  world.set_velocity(plane_body, Vec3::ZERO);
                  world.set_angular_velocity(plane_body, Vec3::ZERO);
               }
            }
         }
         world.step(PHYSICS_STEP as f32);

         accumulator -= PHYSICS_STEP;
         iterations += 1;
      }
      if iterations == 8 && accumulator >= PHYSICS_STEP {
         accumulator = 0.0;
      }
      let alpha = (accumulator / PHYSICS_STEP) as f32;
      let live_position = world.position(plane_body).unwrap_or(previous_position);
      let live_orientation = world
         .orientation(plane_body)
         .unwrap_or(previous_orientation);
      let presentation_position = previous_position.lerp(live_position, alpha);
      let presentation_orientation = previous_orientation
         .slerp(live_orientation, alpha)
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
      // No frame-rate lock: render as fast as the software rasterizer can;
      // simulation time is kept independent by the fixed-step accumulator.
      // 不限制帧率：以软件光栅化器所能达到的最快速度渲染；模拟时间由固定步长累加器独立
      // 维持。
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

fn reset_plane(world: &mut World, plane: BodyId) {
   world.set_pose(plane, SPAWN_POS, Quat::IDENTITY);
   world.set_velocity(plane, Vec3::ZERO);
   world.set_angular_velocity(plane, Vec3::ZERO);
   // Back to the state a fresh aircraft is in: flying, so the flight model owns it.
   // 回到一架全新飞机的状态：在飞，因此由飞行模型拥有。
   world.set_gravity_scale(plane, 0.0);
}

fn read_texture() -> Option<Texture> {
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

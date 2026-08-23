use he3d_assets::{decode_image, parse_obj};
use he3d_core::{Camera, Material, RenderObject, Rgb, Scene, Transform};
use he3d_math::{Quat, Vec3};
use he3d_platform::{BackendConfig, Event, KEY_ESCAPE, PlatformFactory};
use he3d_renderer::{Renderer, RendererConfig};

const WINDOW_WIDTH: u32 = 640;
const WINDOW_HEIGHT: u32 = 360;

const OBJ_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/biplane.obj");
const BMP_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/biplane.bmp");

fn main() {
   let config = BackendConfig {
      title: "HE3D Biplane",
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

   let obj_bytes = match std::fs::read(OBJ_PATH) {
      Ok(bytes) => bytes,
      Err(error) => {
         eprintln!("failed to read {OBJ_PATH}: {error}");
         std::process::exit(1);
      }
   };
   let mesh = match parse_obj(&obj_bytes) {
      Ok(mesh) => mesh,
      Err(error) => {
         eprintln!("failed to parse biplane.obj: {error}");
         std::process::exit(1);
      }
   };
   let bmp_bytes = match std::fs::read(BMP_PATH) {
      Ok(bytes) => bytes,
      Err(error) => {
         eprintln!("failed to read {BMP_PATH}: {error}");
         std::process::exit(1);
      }
   };
   let texture = match decode_image(&bmp_bytes) {
      Ok(texture) => texture,
      Err(error) => {
         eprintln!("failed to decode biplane.bmp: {error}");
         std::process::exit(1);
      }
   };

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

   let mut scene = Scene::new();
   // The reference FlightSimulator sun: strong warm key light.
   // 参考 FlightSimulator 的太阳：强烈的暖色主光。
   scene.light.direction = Vec3::new(1.0, 1.0, 0.5).normalize();
   scene.light.color = Rgb::new(1.5, 1.4, 1.2);
   let mesh_id = scene.meshes.insert(mesh);
   let texture_id = scene.textures.insert(texture);
   let object = scene.spawn(RenderObject {
      mesh: mesh_id,
      texture: Some(texture_id),
      transform: Transform::from_position(Vec3::ZERO),
      material: Material::new(Rgb::new(1.0, 1.0, 1.0)),
      visible: true,
   });
   let camera = Camera::new(Vec3::new(0.0, 1.2, -4.0), Quat::IDENTITY, 60.0);

   let mut angle = 0.0f32;
   let mut last_time = backend.time_seconds();
   let mut frame = 0u32;

   loop {
      for event in backend.poll_events() {
         if let Event::KeyDown(KEY_ESCAPE) = event {
            backend.request_close();
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
      let delta = ((now - last_time) as f32).clamp(0.0, 0.1);
      last_time = now;
      angle += delta * 0.5;
      if let Some(object) = scene.objects.get_mut(object) {
         object.transform.orientation = Quat::from_rotation_y(angle);
      }

      renderer
         .render(&scene, &camera, Rgb::new(0.35, 0.6, 0.9))
         .expect("render frame");
      backend
         .present(renderer.width(), renderer.height(), renderer.pixels())
         .expect("present frame");
      frame += 1;
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

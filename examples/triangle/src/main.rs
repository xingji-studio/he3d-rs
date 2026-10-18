//! Triangle demo: a rotating textured triangle.
//! 三角形示例：一个旋转的带纹理三角形。
use he3d::{
   BackendConfig, Camera, ColorRgba8, Event, KEY_ESCAPE, Material, Mesh, Platform, Quat,
   RenderObject, Renderer, RendererConfig, Rgb, Scene, Texture, Transform, Vec3,
};
use he3d_platform_sdl3::SdlBackend;

const WINDOW_WIDTH: u32 = 640;
const WINDOW_HEIGHT: u32 = 360;

fn main() {
   let frames = parse_frames(std::env::args());

   let config = BackendConfig::new("HE3D TriangleTest", WINDOW_WIDTH, WINDOW_HEIGHT);
   let mut backend = match SdlBackend::new(config) {
      Ok(backend) => backend,
      Err(error) => {
         eprintln!("backend init failed: {error}");
         std::process::exit(1);
      }
   };

   let mut scene = Scene::new();
   scene.light.direction = Vec3::new(1.0, 1.0, -1.0).normalize();
   let triangle_mesh = Mesh::triangle(1.6, 1.4).expect("triangle mesh");
   let mesh = scene.meshes.insert(triangle_mesh);
   let solid = scene
      .textures
      .insert(Texture::solid(ColorRgba8::from_rgb(255, 89, 38)));
   let object = scene.spawn(RenderObject {
      mesh,
      texture: Some(solid),
      transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
      material: Material::new(Rgb::WHITE),
      visible: true,
   });
   let camera = Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 70.0);

   let mut renderer = match Renderer::new(RendererConfig::new(WINDOW_WIDTH, WINDOW_HEIGHT)) {
      Ok(renderer) => renderer,
      Err(error) => {
         eprintln!("renderer init failed: {error}");
         std::process::exit(1);
      }
   };

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
      angle += delta;

      if let Some(object) = scene.objects.get_mut(object) {
         object.transform.orientation = Quat::from_rotation_z(angle);
      }

      renderer
         .render(&scene, &camera, Rgb::new(0.08, 0.10, 0.14))
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

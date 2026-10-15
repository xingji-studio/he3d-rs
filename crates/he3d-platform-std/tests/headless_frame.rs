//! A rendered frame reaches the stdout backend, headlessly.
//! 渲染出来的一帧能到达 stdout 后端，且不需要显示设备。
//!
//! This is the path an application can rely on when there is no display: the
//! backend needs nothing from the system, so it works wherever a terminal does, and
//! skips input configuration entirely when stdin is not a TTY (as in CI).
//! 这是没有显示设备时应用可以依赖的路径：后端不需要系统中的任何东西，因此有终端的地方
//! 就能用；标准输入不是 TTY 时（例如 CI）会完全跳过输入配置。

use he3d_platform::{BackendConfig, Platform};
use he3d_platform_std::StdBackend;
use he3d_renderer::{Camera, Renderer, RendererConfig, Rgb, Scene};

#[test]
fn std_backend_drives_a_headless_frame() {
   let mut backend = StdBackend::new(BackendConfig::new("smoke", 64, 36)).expect("std backend");
   assert!(!backend.should_close());

   // Non-TTY (CI) input is skipped, so no synthetic events arrive.
   // 非 TTY（CI）输入会被跳过，因此不会产生合成事件。
   assert_eq!(backend.poll_events(), [] as [he3d_platform::Event; 0]);

   let scene = Scene::new();
   let mut renderer = Renderer::new(RendererConfig::new(64, 36)).expect("renderer");
   renderer
      .render(&scene, &Camera::default(), Rgb::new(0.1, 0.1, 0.1))
      .expect("render");
   backend.present(64, 36, renderer.pixels()).expect("present");

   backend.request_close();
   assert!(backend.should_close());
}

//! Integration smoke test for the `he3d` facade.
//! `he3d` 门面 crate 的集成冒烟测试。
//!
//! Requires the `std` backend feature, so it runs headlessly in CI (stdin is not a
//! TTY there, which the backend skips configuring).
//! 需要 `std` 后端特性，因此在 CI 中可无头运行（CI 中标准输入不是 TTY，后端会跳过输入配置）。
#![cfg(feature = "std")]

use he3d::{
   BackendConfig, Camera, ColorRgba8, Event, Renderer, RendererConfig, Rgb, Scene, create_backend,
};

#[test]
fn std_backend_drives_a_headless_frame() {
   let mut backend = create_backend(BackendConfig::new("smoke", 64, 36)).expect("std backend");
   assert!(!backend.should_close());

   // Non-TTY (CI) input is skipped, so no synthetic events arrive.
   // 非 TTY（CI）输入会被跳过，因此不会产生合成事件。
   assert!(backend.poll_events().is_empty());

   let scene = Scene::new();
   let mut renderer = Renderer::new(RendererConfig::new(64, 36)).expect("renderer");
   renderer
      .render(&scene, &Camera::default(), Rgb::new(0.1, 0.1, 0.1))
      .expect("render");
   backend.present(64, 36, renderer.pixels()).expect("present");

   backend.request_close();
   assert!(backend.should_close());
}

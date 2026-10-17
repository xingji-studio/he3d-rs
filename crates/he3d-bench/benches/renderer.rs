//! Software-renderer benchmarks.
//!
//! Rendering is steady state by nature: `Renderer::render` reuses its framebuffers and never mutates the `Scene`, so a plain repeated call measures the frame cost without drift (TAA keeps history across frames, which is the intended per-frame behaviour, not an artifact).
//!
//! The sweeps separate the drivers of frame cost instead of varying them together: triangle/object counts run at a fixed 64x36, resolution runs a fixed small scene, and an empty scene at each resolution isolates the fixed per-frame cost — clearing the colour and inverse-depth buffers and presenting — from the cost of geometry.
//! A screen-filling quad holds geometry constant at two triangles, so its cost minus the empty-scene cost is the per-fragment (fill) cost, with `solid` and `tex` separating the constant-texture fast path from sampling a real texture.

#![allow(clippy::pedantic)]

use criterion::{Criterion, criterion_group, criterion_main};
use he3d_bench::workloads;
use he3d_core::{Rgb, Scene};
use he3d_renderer::{AntiAliasing, Renderer, RendererConfig};
use std::hint::black_box;

const CLEAR: Rgb = Rgb::new(0.05, 0.06, 0.08);

/// A config for `width`x`height` with the given AA mode.
/// The struct is `#[non_exhaustive]`, so the field is set after construction rather than in a literal.
fn config(width: u32, height: u32, aa: AntiAliasing) -> RendererConfig {
   let mut config = RendererConfig::new(width, height);
   config.anti_aliasing = aa;
   config
}

fn renderer(c: &mut Criterion) {
   // Construction: validates the size and allocates every buffer.
   for (w, h) in [(64u32, 36u32), (320, 180), (1280, 720)] {
      c.bench_function(&format!("new/{w}x{h}"), |b| {
         b.iter(|| black_box(Renderer::new(RendererConfig::new(w, h)).expect("renderer")));
      });
   }

   // Empty scene: the fixed per-frame cost with no geometry at all.
   for (w, h) in [(64u32, 36u32), (320, 180), (1280, 720)] {
      let mut r = Renderer::new(RendererConfig::new(w, h)).expect("renderer");
      let scene = Scene::new();
      let camera = workloads::default_camera();
      c.bench_function(&format!("render/empty/{w}x{h}"), |b| {
         b.iter(|| {
            r.render(black_box(&scene), black_box(&camera), CLEAR)
               .expect("render")
         });
      });
   }

   // Resolution sweep on a fixed small scene.
   for (w, h) in [(64u32, 36u32), (320, 180), (1280, 720)] {
      let mut r = Renderer::new(RendererConfig::new(w, h)).expect("renderer");
      let (scene, camera) = workloads::render_scene_small();
      c.bench_function(&format!("render/scene8/{w}x{h}"), |b| {
         b.iter(|| {
            r.render(black_box(&scene), black_box(&camera), CLEAR)
               .expect("render")
         });
      });
   }

   // Triangle-count sweep at a fixed resolution.
   for tris in [12usize, 1000, 10000] {
      let mut r = Renderer::new(RendererConfig::new(64, 36)).expect("renderer");
      let (scene, camera) = workloads::render_scene_tris(tris);
      c.bench_function(&format!("render/tris/{tris}"), |b| {
         b.iter(|| {
            r.render(black_box(&scene), black_box(&camera), CLEAR)
               .expect("render")
         });
      });
   }

   // Object-count sweep at a fixed resolution.
   for n in [1usize, 100, 1000] {
      let mut r = Renderer::new(RendererConfig::new(64, 36)).expect("renderer");
      let (scene, camera) = workloads::render_scene_objects(n);
      c.bench_function(&format!("render/n_objects/{n}"), |b| {
         b.iter(|| {
            r.render(black_box(&scene), black_box(&camera), CLEAR)
               .expect("render")
         });
      });
   }

   // Fill-rate sweep: one screen-filling quad, so frame cost is the fixed clear plus one per-fragment cost for every pixel.
   // Triangle/object counts are held constant at two triangles, which is what separates fill from geometry.
   for (w, h) in [(320u32, 180u32), (1280, 720)] {
      for (label, textured) in [("solid", false), ("tex", true)] {
         let mut r = Renderer::new(RendererConfig::new(w, h)).expect("renderer");
         let (scene, camera) = workloads::render_scene_screen_fill(w, h, textured);
         c.bench_function(&format!("render/fill/{label}/{w}x{h}"), |b| {
            b.iter(|| {
               r.render(black_box(&scene), black_box(&camera), CLEAR)
                  .expect("render")
            });
         });
      }
   }

   // Anti-aliasing: same scene and resolution, one mode each.
   for (label, aa) in [
      ("none", AntiAliasing::None),
      ("fxaa", AntiAliasing::Fxaa),
      ("taa", AntiAliasing::Taa),
      ("msaa", AntiAliasing::Msaa),
      ("ssaa4", AntiAliasing::Ssaa(4)),
   ] {
      let mut r = Renderer::new(config(64, 36, aa)).expect("renderer");
      let (scene, camera) = workloads::render_scene_small();
      c.bench_function(&format!("render/aa/{label}"), |b| {
         b.iter(|| {
            r.render(black_box(&scene), black_box(&camera), CLEAR)
               .expect("render")
         });
      });
   }
}

criterion_group!(renderer_group, renderer);
criterion_main!(renderer_group);

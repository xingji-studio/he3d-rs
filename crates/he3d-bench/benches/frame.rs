//! Simulation and rendering in one frame.
//!
//! The physics and renderer benchmarks each measure one side of a frame, and `docs/physics-spec.md` §14 risk 4 records that the two had never been measured together: they share one CPU, so the combined cost is not guaranteed to be their sum.
//! This target steps and draws one scene of two thousand Bodies in the same closure, and measures the draw alone from the same scene so the step's share is a subtraction rather than an assumption.
//!
//! The scene drifts as it is stepped, so setup (build, warm, sync) is untimed and only the single following frame is measured, the same way `benches/physics.rs` handles an active scene.
//! The routine returns its input so its drop lands outside the timed section.

#![allow(clippy::pedantic)]

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use he3d_bench::workloads::{self, STEP, SimRenderScene};
use he3d_core::Rgb;
use he3d_renderer::{Renderer, RendererConfig};
use std::hint::black_box;

/// Frame width; the budget the docs quote.
const WIDTH: u32 = 1280;

/// Frame height.
const HEIGHT: u32 = 720;

/// Clear colour, matching `benches/renderer.rs`.
const CLEAR: Rgb = Rgb::new(0.05, 0.06, 0.08);

/// Leading steps taken (untimed) before the timed frame, matching `benches/physics.rs`.
const WARMUP: u32 = 16;

/// How many Bodies are simulated and drawn.
const BODIES: usize = 2000;

/// A warmed scene and a renderer sized for one frame.
fn setup() -> (SimRenderScene, Renderer) {
   let mut scene = workloads::sim_render_scene(BODIES);
   for _ in 0..WARMUP {
      scene.world.step(STEP);
   }
   scene.sync();
   let renderer = Renderer::new(RendererConfig::new(WIDTH, HEIGHT)).expect("renderer");
   (scene, renderer)
}

fn frame(c: &mut Criterion) {
   // Both sides of a frame: the number risk 4 asks for.
   c.bench_function("frame/sim_render/2000", |b| {
      b.iter_batched(
         setup,
         |(mut scene, mut renderer)| {
            scene.world.step(STEP);
            scene.sync();
            renderer
               .render(black_box(&scene.scene), black_box(&scene.camera), CLEAR)
               .expect("render");
            (scene, renderer)
         },
         BatchSize::LargeInput,
      );
   });

   // The draw alone, from the same scene, so the step's share of the combined frame is measured rather than assumed.
   c.bench_function("frame/render_only/2000", |b| {
      b.iter_batched(
         setup,
         |(scene, mut renderer)| {
            renderer
               .render(black_box(&scene.scene), black_box(&scene.camera), CLEAR)
               .expect("render");
            (scene, renderer)
         },
         BatchSize::LargeInput,
      );
   });

   // The pose copy alone: the application-owned cost docs/adr/0001 places outside the physics crate.
   c.bench_function("frame/sync_only/2000", |b| {
      b.iter_batched(
         setup,
         |(mut scene, renderer)| {
            scene.sync();
            (scene, renderer)
         },
         BatchSize::LargeInput,
      );
   });
}

// 30 samples: the setup rebuilds a two-thousand-Body world and a 1280x720 renderer every iteration, so the default 100 would make the suite take all afternoon.
criterion_group! {
   name = frame_group;
   config = Criterion::default().sample_size(30);
   targets = frame
}
criterion_main!(frame_group);

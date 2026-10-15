//! Heap and stack accounting, printed as tables.
//!
//! Run with `cargo run -p he3d-bench --bin memreport [--release]`.
//!
//! This binary installs [`CountingAlloc`] as the global allocator and prints two
//! tables:
//!
//! * **Heap** — for each scenario, how many allocations it made, how many bytes it
//!   requested, and the peak it added. The interesting zeros are the ones that mean
//!   "reused": a settled [`World`] stepped many times, or a [`Renderer`] drawing many
//!   frames, should allocate *nothing*, because both reuse their buffers.
//! * **Sizeof** — `size_of` for the engine's core types, i.e. the stack footprint of
//!   holding one.
//!
//! Bytes are what the allocation asked for, so they are a lower bound on resident
//! memory (allocator headers and size-class rounding are not counted) and an exact
//! count of what the engine requested.

#![allow(clippy::pedantic)]

use he3d_bench::alloc::{self, CountingAlloc};
use he3d_bench::workloads;
use he3d_core::{Camera, ColorRgba8, Mesh, Rgb, Scene, Texture, Transform};
use he3d_math::{Mat4, Quat, Vec3};
use he3d_physics::{Aabb, BodyDesc, Contact, World};
use he3d_renderer::{Renderer, RendererConfig};

/// The counting allocator only exists where the numbers are read.
#[global_allocator]
static ALLOC: CountingAlloc = CountingAlloc;

fn main() {
   println!("HE3D memory report");
   println!("==================");
   println!();
   println!("Heap accounting (requested bytes; excludes allocator bookkeeping)");
   println!(
      "{:<40} {:>8} {:>9} {:>12} {:>12} {:>12}",
      "scenario", "allocs", "reallocs", "alloc_bytes", "peak_added", "live_delta"
   );
   println!("{}", "-".repeat(97));

   // --- Construction: what one built object costs in heap. ------------------
   measure_build("build/World::new", workloads::world_empty);
   measure_build("build/world_freefall/2000", || {
      workloads::world_freefall(2000)
   });
   measure_build("build/world_lattice_on_slab/2000", || {
      workloads::world_lattice_on_slab(2000)
   });
   measure_build("build/world_static_terrain/10000", || {
      workloads::world_static_terrain(10_000, 8)
   });
   measure_build("build/Renderer/1280x720", || {
      Renderer::new(RendererConfig::new(1280, 720)).expect("renderer")
   });
   measure_build("build/render_scene_tris/10000", || {
      workloads::render_scene_tris(10_000).0
   });

   // --- Steady state: what repeated work costs. Zero means "reused". --------
   let mut lattice = workloads::world_settled_asleep(2000);
   let asleep = workloads::sleeping_count(&lattice);
   println!();
   println!("  (resting_on_slab/2000 settled: {asleep} of 2000 Bodies asleep)");
   measure_run("step/world_settled_asleep/2000  x100", || {
      for _ in 0..100 {
         lattice.step(workloads::STEP);
      }
   });

   let mut freefall = workloads::world_freefall(2000);
   for _ in 0..10 {
      freefall.step(workloads::STEP);
   }
   measure_run("step/world_freefall/2000  x100", || {
      for _ in 0..100 {
         freefall.step(workloads::STEP);
      }
   });

   let mut renderer = Renderer::new(RendererConfig::new(640, 360)).expect("renderer");
   let (scene, camera) = workloads::render_scene_small();
   for _ in 0..2 {
      let _ = renderer.render(&scene, &camera, Rgb::new(0.0, 0.0, 0.0));
   }
   measure_run("render/scene8/640x360  x100", || {
      for _ in 0..100 {
         let _ = renderer.render(&scene, &camera, Rgb::new(0.0, 0.0, 0.0));
      }
   });

   // --- Assets: decode cost. ------------------------------------------------
   let obj = workloads::biplane_obj_bytes();
   measure_run("parse_obj/biplane", || {
      let _ = he3d_assets::parse_obj(obj).expect("parse obj");
   });
   let bmp = workloads::biplane_bmp_bytes();
   measure_run("decode_image/biplane_bmp", || {
      let _ = he3d_assets::decode_image(bmp).expect("decode bmp");
   });

   // --- Stack: size_of for the core types. ----------------------------------
   println!();
   println!("size_of core types (stack footprint of holding one)");
   println!("{:<24} {:>10}", "type", "bytes");
   println!("{}", "-".repeat(36));
   let types: &[(&str, usize)] = &[
      ("World", size_of::<World>()),
      ("Renderer", size_of::<Renderer>()),
      ("RendererConfig", size_of::<RendererConfig>()),
      ("Scene", size_of::<Scene>()),
      ("Mesh", size_of::<Mesh>()),
      ("Texture", size_of::<Texture>()),
      ("BodyDesc", size_of::<BodyDesc>()),
      ("Contact", size_of::<Contact>()),
      ("Aabb", size_of::<Aabb>()),
      ("Transform", size_of::<Transform>()),
      ("Camera", size_of::<Camera>()),
      ("Rgb", size_of::<Rgb>()),
      ("ColorRgba8", size_of::<ColorRgba8>()),
      ("Vec3", size_of::<Vec3>()),
      ("Quat", size_of::<Quat>()),
      ("Mat4", size_of::<Mat4>()),
   ];
   for (name, bytes) in types {
      println!("{name:<24} {bytes:>10}");
   }
}

/// Measure a construction, keep the result alive long enough to report its heap,
/// then drop it.
fn measure_build<T>(name: &str, build: impl FnOnce() -> T) -> T {
   let base = alloc::stats().live_bytes;
   alloc::reset_counters();
   let value = build();
   let after = alloc::stats();
   print_row(name, base, &after);
   value
}

/// Measure a run (loops, or a call whose result is dropped).
fn measure_run(name: &str, run: impl FnOnce()) {
   let base = alloc::stats().live_bytes;
   alloc::reset_counters();
   run();
   let after = alloc::stats();
   print_row(name, base, &after);
}

/// Print one heap row. `base` is the live size at the start of the window, so
/// `peak_added` and `live_delta` are relative to it.
fn print_row(name: &str, base: usize, after: &alloc::AllocStats) {
   let peak_added = after.peak_bytes.saturating_sub(base);
   let live_delta = after.live_bytes as isize - base as isize;
   println!(
      "{name:<40} {:>8} {:>9} {:>12} {:>12} {:>+12}",
      after.alloc_calls, after.realloc_calls, after.alloc_bytes, peak_added, live_delta,
   );
}

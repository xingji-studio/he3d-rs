//! Physics benchmarks.
//!
//! # Measuring an *active* scene
//!
//! Two things make a naive `b.iter(|| world.step(dt))` lie about an active scene:
//!
//! * the first step of a freshly built scene has no contacts yet, and contact count
//!   climbs for several steps as Bodies settle into each other, so a step taken at
//!   step 0 is far cheaper than one taken in the developed, fully-awake state; and
//! * criterion repeats the closure against the *same* value, so a long run drifts
//!   toward sleep and reports a mix of awake and resting steps.
//!
//! [`bench_active`] sidesteps both with `iter_batched`: setup builds the scene and
//! takes `warmup` untimed steps to bring it into the developed state, and only the
//! single step after that is timed. The number is therefore "the cost of one step,
//! taken while the scene is fully awake", reproduced from scratch every iteration.

#![allow(clippy::pedantic)]

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use he3d_bench::workloads::{self, CUBE_HALF, STEP};
use he3d_math::Vec3;
use he3d_physics::{BodyDesc, Shape, World};
use std::hint::black_box;

/// Leading steps taken (untimed) before the timed step.
const WARMUP: u32 = 16;

/// Time exactly one `step`, taken `warmup` steps into a freshly built scene.
fn bench_active<F>(c: &mut Criterion, name: &str, warmup: u32, build: F)
where
   F: Fn() -> World,
{
   c.bench_function(name, |b| {
      b.iter_batched(
         || {
            let mut world = build();
            for _ in 0..warmup {
               world.step(STEP);
            }
            world
         },
         |mut world| world.step(STEP),
         BatchSize::LargeInput,
      );
   });
}

fn physics(c: &mut Criterion) {
   // Fixed overhead with nothing to do.
   let mut empty = workloads::world_empty();
   c.bench_function("step/empty", |b| {
      b.iter(|| black_box(&mut empty).step(STEP));
   });

   // Active: freefall, no contacts ever form, so no warm-up is needed.
   for n in [8usize, 64, 256, 1000, 2000] {
      bench_active(c, &format!("step/freefall/{n}"), 0, || {
         workloads::world_freefall(n)
      });
   }

   // Active: the docs/physics-spec.md §12 lattice, fully awake and in contact.
   for n in [200usize, 500, 2000] {
      bench_active(c, &format!("step/lattice_on_slab/{n}"), WARMUP, || {
         workloads::world_lattice_on_slab(n)
      });
   }

   // Resting: a single layer settled and asleep. Repeated steps are stable here,
   // so a plain `iter` is honest.
   let mut resting = workloads::world_settled_asleep(2000);
   c.bench_function("step/settled_asleep/2000", |b| {
      b.iter(|| black_box(&mut resting).step(STEP));
   });

   // Active: Bodies resting on a large *static* triangle mesh.
   for tris in [100usize, 1000, 10000] {
      bench_active(
         c,
         &format!("step/static_terrain/{tris}"),
         WARMUP,
         move || workloads::world_static_terrain(tris, 16),
      );
   }

   // Active: broadphase cell-size sweep at the §12 size.
   for cell in [0.5_f32, 2.0, 8.0] {
      bench_active(c, &format!("step/cell_size/{cell}"), WARMUP, move || {
         workloads::world_cell_size(2000, cell)
      });
   }

   // Construction throughput.
   for n in [200usize, 2000] {
      c.bench_function(&format!("build/lattice/{n}"), |b| {
         b.iter(|| black_box(workloads::world_lattice_on_slab(n)));
      });
   }

   // Per-Body insertion cost into a growing world.
   c.bench_function("add_body/2000", |b| {
      b.iter(|| {
         let mut world = World::new();
         for i in 0..2000usize {
            let position = Vec3::new((i % 50) as f32, (i / 50) as f32, 0.0);
            black_box(
               world.add_body(BodyDesc::dynamic(Shape::cuboid(CUBE_HALF), 1.0).at(position)),
            );
         }
      });
   });
}

// 30 samples: enough for a stable median, few enough that the heavy lattice setups
// do not make the suite take all afternoon.
criterion_group! {
   name = physics_group;
   config = Criterion::default().sample_size(30);
   targets = physics
}
criterion_main!(physics_group);

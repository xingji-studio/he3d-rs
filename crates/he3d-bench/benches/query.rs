//! Raycast benchmarks.
//!
//! A raycast is a query, not a step, so it is measured on a plain `iter` over a fixed World: nothing about the scene changes between calls.
//! The two sweeps separate the two costs a ray pays — the number of Bodies it must test against their shapes, and the number of Static triangles it must scan behind their bounds.

#![allow(clippy::pedantic)]

use criterion::{Criterion, criterion_group, criterion_main};
use he3d_bench::workloads;
use he3d_math::Vec3;
use he3d_physics::Ray;
use std::hint::black_box;

fn query(c: &mut Criterion) {
   // Static terrain: a ray straight down over a grid of triangles, against `tris` alone.
   for tris in [100usize, 1000, 10000] {
      let world = workloads::world_static_terrain(tris, 0);
      let ray = Ray::new(Vec3::new(0.5, 100.0, 0.5), Vec3::new(0.0, -1.0, 0.0));
      c.bench_function(&format!("query/static_terrain/{tris}"), |b| {
         b.iter(|| black_box(&world).cast_ray(black_box(ray), 1000.0));
      });
   }

   // Bodies: a ray swept through a lattice, so it tests every Body's shape.
   for n in [100usize, 1000, 2000] {
      let world = workloads::world_lattice_on_slab(n);
      let ray = Ray::new(Vec3::new(-100.0, 0.5, 0.0), Vec3::X);
      c.bench_function(&format!("query/bodies/{n}"), |b| {
         b.iter(|| black_box(&world).cast_ray(black_box(ray), 1000.0));
      });
   }
}

criterion_group!(query_group, query);
criterion_main!(query_group);

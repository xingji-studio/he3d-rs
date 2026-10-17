//! Asset-decoding benchmarks.
//!
//! Two arms: the real fixtures the engine ships and examples load (`biplane.obj`, `biplane.bmp`), which catch regressions against real data, and a synthetic OBJ whose triangle count can be scaled, which shows how decode cost grows.

#![allow(clippy::pedantic)]

use criterion::{Criterion, criterion_group, criterion_main};
use he3d_bench::workloads;
use std::hint::black_box;

fn assets(c: &mut Criterion) {
   // Real OBJ fixture from examples/assets.
   let obj = workloads::biplane_obj_bytes();
   c.bench_function("parse_obj/biplane", |b| {
      b.iter(|| black_box(he3d_assets::parse_obj(black_box(obj)).expect("parse obj")));
   });

   // Real BMP fixture from examples/assets.
   let bmp = workloads::biplane_bmp_bytes();
   c.bench_function("decode_image/biplane_bmp", |b| {
      b.iter(|| black_box(he3d_assets::decode_image(black_box(bmp)).expect("decode bmp")));
   });

   // Scaling arm: synthetic OBJ, triangle count swept.
   for tris in [100usize, 1000, 10000] {
      let bytes = workloads::synthetic_obj(tris);
      c.bench_function(&format!("parse_obj/synthetic/{tris}"), |b| {
         b.iter(|| black_box(he3d_assets::parse_obj(black_box(&bytes)).expect("parse obj")));
      });
   }
}

criterion_group!(assets_group, assets);
criterion_main!(assets_group);

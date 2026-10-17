//! Core/math micro-benchmarks.
//!
//! These cover the operations the rest of the engine leans on hardest.
//! `glam` inlines them aggressively, so the absolute times are tiny; they exist to catch a regression that a change of representation (or a lost `#[inline]`) would cause.

#![allow(clippy::pedantic)]

use criterion::{Criterion, criterion_group, criterion_main};
use he3d_core::Transform;
use he3d_math::{Mat4, Quat, Vec3};
use std::hint::black_box;

fn core_math(c: &mut Criterion) {
   let rotation = Quat::from_rotation_y(0.5);
   let point = Vec3::new(1.0, 2.0, 3.0);

   c.bench_function("math/quat_mul", |b| {
      b.iter(|| black_box(rotation * rotation));
   });

   c.bench_function("math/quat_rotate_vec3", |b| {
      b.iter(|| black_box(rotation * point));
   });

   c.bench_function("math/vec3_normalize", |b| {
      b.iter(|| black_box(Vec3::new(3.0, 4.0, 5.0).normalize()));
   });

   c.bench_function("math/vec3_cross", |b| {
      b.iter(|| black_box(Vec3::X.cross(Vec3::Y)));
   });

   c.bench_function("math/mat4_mul", |b| {
      b.iter(|| black_box(Mat4::IDENTITY * Mat4::IDENTITY));
   });

   let transform = Transform::from_position_orientation(point, rotation);
   c.bench_function("math/transform_to_matrix", |b| {
      b.iter(|| black_box(transform.to_matrix()));
   });
}

criterion_group!(core_math_group, core_math);
criterion_main!(core_math_group);

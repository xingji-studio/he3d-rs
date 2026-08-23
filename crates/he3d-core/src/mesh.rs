use alloc::vec;
use alloc::vec::Vec;
use he3d_math::scalar::{cos, sin};
use he3d_math::{Vec2, Vec3};

use crate::Error;

/// Maximum number of vertices a mesh may declare.
/// 一个网格可声明的最大顶点数。
pub const MAX_VERTEX_COUNT: usize = 16_777_216;

/// A triangle mesh owning its vertices, UVs, and per-triangle normals.
/// 一个拥有顶点、UV 和逐三角形法线的三角形网格。
///
/// Storage is a flat vertex list where every three consecutive vertices form a triangle (no index buffer).
/// 存储是一个扁平的顶点列表，每三个连续顶点构成一个三角形（没有索引缓冲）。
/// `uvs.len() == vertices.len()` and `triangle_normals.len() == vertices.len()
/// 3` are invariants maintained by [`Mesh::new`]. / `uvs.len() == vertices.len()` 与 `triangle_normals.len() == vertices.len() / 3` 是由 [`Mesh::new`] 维持的不变式。
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
   vertices: Vec<Vec3>,
   uvs: Vec<Vec2>,
   triangle_normals: Vec<Vec3>,
}

impl Mesh {
   /// Construct a mesh, validating invariants and computing flat normals.
   /// 构造一个网格，校验不变式并计算平面法线。
   ///
   /// # Errors
   /// 顶点列表为空、超过 [`MAX_VERTEX_COUNT`]、顶点数不是 3 的倍数，或 UV 数量与顶点数不符时返回相应错误。
   /// Returns an error when the vertex list is empty or exceeds [`MAX_VERTEX_COUNT`],
   /// when the vertex count is not a multiple of three, or when the UV count
   /// does not match the vertex count.
   pub fn new(vertices: Vec<Vec3>, uvs: Vec<Vec2>) -> Result<Self, Error> {
      if vertices.is_empty() || vertices.len() > MAX_VERTEX_COUNT {
         return Err(Error::InvalidSize);
      }
      if vertices.len() % 3 != 0 {
         return Err(Error::InvalidData);
      }
      if uvs.len() != vertices.len() {
         return Err(Error::BufferMismatch);
      }
      let mut mesh = Self {
         vertices,
         uvs,
         triangle_normals: Vec::new(),
      };
      mesh.recalculate_normals();
      Ok(mesh)
   }

   /// Construct a mesh from vertices only, defaulting UVs to zero.
   /// 仅从顶点构造网格，UV 默认置零。
   ///
   /// # Errors
   /// 顶点列表为空、超过 [`MAX_VERTEX_COUNT`]，或顶点数不是 3 的倍数时返回相应错误。
   /// Returns an error when the vertex list is empty or exceeds [`MAX_VERTEX_COUNT`],
   /// or when the vertex count is not a multiple of three.
   pub fn from_vertices(vertices: Vec<Vec3>) -> Result<Self, Error> {
      let count = vertices.len();
      if count == 0 || count > MAX_VERTEX_COUNT {
         return Err(Error::InvalidSize);
      }
      if count % 3 != 0 {
         return Err(Error::InvalidData);
      }
      Self::new(vertices, vec![Vec2::ZERO; count])
   }

   /// The number of vertices.
   /// 顶点数量。
   #[must_use]
   pub fn vertex_count(&self) -> usize {
      self.vertices.len()
   }

   /// The number of triangles.
   /// 三角形数量。
   #[must_use]
   pub fn triangle_count(&self) -> usize {
      self.vertices.len() / 3
   }

   /// Whether the mesh has at least one full triangle.
   /// 网格是否至少包含一个完整三角形。
   #[must_use]
   pub fn is_valid(&self) -> bool {
      !self.vertices.is_empty() && self.vertices.len() % 3 == 0
   }

   /// The raw vertex list.
   /// 原始顶点列表。
   #[must_use]
   pub fn vertices(&self) -> &[Vec3] {
      &self.vertices
   }

   /// The per-vertex UV list.
   /// 逐顶点 UV 列表。
   #[must_use]
   pub fn uvs(&self) -> &[Vec2] {
      &self.uvs
   }

   /// The per-triangle flat normals.
   /// 逐三角形平面法线。
   #[must_use]
   pub fn triangle_normals(&self) -> &[Vec3] {
      &self.triangle_normals
   }

   /// Recompute flat per-triangle normals from the current vertices.
   /// 根据当前顶点重新计算逐三角形平面法线。
   pub fn recalculate_normals(&mut self) {
      let triangle_count = self.vertices.len() / 3;
      self.triangle_normals.clear();
      self.triangle_normals.reserve(triangle_count);
      for tri in self.vertices.chunks_exact(3) {
         let e1 = tri[1] - tri[0];
         let e2 = tri[2] - tri[0];
         self.triangle_normals.push(e1.cross(e2).normalize_or_zero());
      }
   }

   /// A single triangle lying in the XY plane.
   /// 一个位于 XY 平面上的三角形。
   ///
   /// # Errors
   /// `width` 或 `height` 不是正有限数时返回 [`Error::InvalidSize`]。
   /// Returns [`Error::InvalidSize`] when `width` or `height` is not a positive finite number.
   pub fn triangle(width: f32, height: f32) -> Result<Self, Error> {
      if !positive_finite(width) || !positive_finite(height) {
         return Err(Error::InvalidSize);
      }
      let hx = width * 0.5;
      let hy = height * 0.5;
      let vertices = vec![
         Vec3::new(-hx, -hy, 0.0),
         Vec3::new(0.0, hy, 0.0),
         Vec3::new(hx, -hy, 0.0),
      ];
      let uvs = vec![
         Vec2::new(0.0, 0.0),
         Vec2::new(0.5, 1.0),
         Vec2::new(1.0, 0.0),
      ];
      Self::new(vertices, uvs)
   }

   /// A flat quad on the XZ plane (two triangles).
   /// XZ 平面上的一个平面四边形（两个三角形）。
   ///
   /// # Errors
   /// `width` 或 `depth` 不是正有限数时返回 [`Error::InvalidSize`]。
   /// Returns [`Error::InvalidSize`] when `width` or `depth` is not a positive finite number.
   pub fn plane(width: f32, depth: f32) -> Result<Self, Error> {
      if !positive_finite(width) || !positive_finite(depth) {
         return Err(Error::InvalidSize);
      }
      let hx = width * 0.5;
      let hz = depth * 0.5;
      let vertices = vec![
         Vec3::new(-hx, 0.0, -hz),
         Vec3::new(hx, 0.0, hz),
         Vec3::new(hx, 0.0, -hz),
         Vec3::new(-hx, 0.0, -hz),
         Vec3::new(-hx, 0.0, hz),
         Vec3::new(hx, 0.0, hz),
      ];
      let uvs = vec![
         Vec2::new(0.0, 0.0),
         Vec2::new(1.0, 1.0),
         Vec2::new(1.0, 0.0),
         Vec2::new(0.0, 0.0),
         Vec2::new(0.0, 1.0),
         Vec2::new(1.0, 1.0),
      ];
      Self::new(vertices, uvs)
   }

   /// An axis-aligned cube (12 triangles, 36 vertices).
   /// 一个轴对齐的立方体（12 个三角形，36 个顶点）。
   ///
   /// # Errors
   /// 任一尺寸不是正有限数时返回 [`Error::InvalidSize`]。
   /// Returns [`Error::InvalidSize`] when any dimension is not a positive finite number.
   pub fn cube(width: f32, height: f32, depth: f32) -> Result<Self, Error> {
      if !positive_finite(width) || !positive_finite(height) || !positive_finite(depth) {
         return Err(Error::InvalidSize);
      }
      let hx = width * 0.5;
      let hy = height * 0.5;
      let hz = depth * 0.5;
      let p = [
         Vec3::new(-hx, -hy, -hz),
         Vec3::new(hx, -hy, -hz),
         Vec3::new(hx, hy, -hz),
         Vec3::new(-hx, hy, -hz),
         Vec3::new(-hx, -hy, hz),
         Vec3::new(hx, -hy, hz),
         Vec3::new(hx, hy, hz),
         Vec3::new(-hx, hy, hz),
      ];
      let (uv0, uv1, uv2, uv3) = (
         Vec2::new(0.0, 0.0),
         Vec2::new(1.0, 0.0),
         Vec2::new(1.0, 1.0),
         Vec2::new(0.0, 1.0),
      );

      let mut vertices = Vec::with_capacity(36);
      let mut uvs = Vec::with_capacity(36);
      let mut emit = |a: usize, b: usize, c: usize, uv: [Vec2; 3]| {
         push_tri(&mut vertices, &mut uvs, p[a], p[b], p[c], uv);
      };
      emit(4, 5, 6, [uv0, uv1, uv2]);
      emit(4, 6, 7, [uv0, uv2, uv3]);
      emit(1, 0, 3, [uv0, uv1, uv2]);
      emit(1, 3, 2, [uv0, uv2, uv3]);
      emit(0, 4, 7, [uv0, uv1, uv2]);
      emit(0, 7, 3, [uv0, uv2, uv3]);
      emit(5, 1, 2, [uv0, uv1, uv2]);
      emit(5, 2, 6, [uv0, uv2, uv3]);
      emit(3, 7, 6, [uv0, uv1, uv2]);
      emit(3, 6, 2, [uv0, uv2, uv3]);
      emit(0, 1, 5, [uv0, uv1, uv2]);
      emit(0, 5, 4, [uv0, uv2, uv3]);

      Self::new(vertices, uvs)
   }

   /// A UV sphere tessellated into `segments` × `rings` quads.
   /// 一个被细分为 `segments` × `rings` 个四边形的 UV 球体。
   ///
   /// Sizing is checked: the total vertex count must fit within [`MAX_VERTEX_COUNT`] without overflowing.
   /// 尺寸经过校验：顶点总数必须在不溢出的前提下落在 [`MAX_VERTEX_COUNT`] 之内。
   ///
   /// # Errors
   /// `radius` 不是正有限数、总顶点数超过 [`MAX_VERTEX_COUNT`]，或计算过程溢出时返回相应错误。
   /// Returns an error when `radius` is not a positive finite number, when the total
   /// vertex count exceeds [`MAX_VERTEX_COUNT`], or when the computation overflows.
   // UV coordinates are built from loop indices bounded by `segments`/`rings`; the
   // f32 mantissa is far larger than any realistic tessellation count, so the
   // conversion cannot meaningfully lose precision here.
   // UV 坐标由受 `segments`/`rings` 约束的循环索引构造；f32 的有效数字远大于任何
   // 现实的细分数量，因此这里的转换不会产生有意义的精度损失。
   #[expect(clippy::cast_precision_loss)]
   pub fn sphere(radius: f32, segments: u32, rings: u32) -> Result<Self, Error> {
      if !positive_finite(radius) {
         return Err(Error::InvalidSize);
      }
      let segments = segments.max(3);
      let rings = rings.max(2);
      let total = (segments as usize)
         .checked_mul(rings as usize)
         .and_then(|v| v.checked_mul(6))
         .ok_or(Error::Overflow)?;
      if total == 0 || total > MAX_VERTEX_COUNT {
         return Err(Error::InvalidSize);
      }

      let tau = core::f32::consts::TAU;
      let pi = core::f32::consts::PI;
      let mut vertices = Vec::with_capacity(total);
      let mut uvs = Vec::with_capacity(total);

      for ring in 0..rings {
         let v0 = ring as f32 / rings as f32;
         let v1 = (ring + 1) as f32 / rings as f32;
         let lat0 = (v0 - 0.5) * pi;
         let lat1 = (v1 - 0.5) * pi;
         let y0 = sin(lat0) * radius;
         let y1 = sin(lat1) * radius;
         let r0 = cos(lat0) * radius;
         let r1 = cos(lat1) * radius;

         for segment in 0..segments {
            let u0 = segment as f32 / segments as f32;
            let u1 = (segment + 1) as f32 / segments as f32;
            let lon0 = u0 * tau;
            let lon1 = u1 * tau;

            let p00 = Vec3::new(sin(lon0) * r0, y0, cos(lon0) * r0);
            let p01 = Vec3::new(sin(lon1) * r0, y0, cos(lon1) * r0);
            let p10 = Vec3::new(sin(lon0) * r1, y1, cos(lon0) * r1);
            let p11 = Vec3::new(sin(lon1) * r1, y1, cos(lon1) * r1);

            push_tri(
               &mut vertices,
               &mut uvs,
               p00,
               p11,
               p10,
               [Vec2::new(u0, v0), Vec2::new(u1, v1), Vec2::new(u0, v1)],
            );
            push_tri(
               &mut vertices,
               &mut uvs,
               p00,
               p01,
               p11,
               [Vec2::new(u0, v0), Vec2::new(u1, v0), Vec2::new(u1, v1)],
            );
         }
      }

      Self::new(vertices, uvs)
   }
}

fn positive_finite(value: f32) -> bool {
   value.is_finite() && value > 0.0
}

fn push_tri(
   vertices: &mut Vec<Vec3>,
   uvs: &mut Vec<Vec2>,
   a: Vec3,
   b: Vec3,
   c: Vec3,
   uv: [Vec2; 3],
) {
   vertices.extend([a, b, c]);
   uvs.extend(uv);
}

#[cfg(test)]
mod tests {
   use alloc::vec;

   use super::*;

   #[test]
   fn triangle_is_valid() {
      let mesh = Mesh::triangle(1.0, 1.0).unwrap();
      assert_eq!(mesh.vertex_count(), 3);
      assert_eq!(mesh.triangle_count(), 1);
      assert!(mesh.is_valid());
   }

   #[test]
   fn cube_has_twelve_triangles() {
      let mesh = Mesh::cube(1.0, 1.0, 1.0).unwrap();
      assert_eq!(mesh.vertex_count(), 36);
      assert_eq!(mesh.triangle_count(), 12);
   }

   #[test]
   fn rejects_empty_mesh() {
      assert_eq!(Mesh::new(Vec::new(), Vec::new()), Err(Error::InvalidSize));
   }

   #[test]
   fn from_vertices_empty_is_invalid_size() {
      assert_eq!(Mesh::from_vertices(Vec::new()), Err(Error::InvalidSize));
   }

   #[test]
   fn rejects_non_triangle_vertex_count() {
      let v = vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z];
      assert_eq!(Mesh::from_vertices(v), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_uv_length_mismatch() {
      let v = vec![Vec3::ZERO, Vec3::X, Vec3::Y];
      let uv = vec![Vec2::ZERO];
      assert_eq!(Mesh::new(v, uv), Err(Error::BufferMismatch));
   }

   #[test]
   fn rejects_zero_or_negative_primitive_sizes() {
      assert_eq!(Mesh::triangle(0.0, 1.0), Err(Error::InvalidSize));
      assert_eq!(Mesh::plane(-1.0, 1.0), Err(Error::InvalidSize));
      assert_eq!(Mesh::cube(1.0, 1.0, f32::NAN), Err(Error::InvalidSize));
      assert_eq!(Mesh::sphere(0.0, 8, 4), Err(Error::InvalidSize));
   }

   #[test]
   fn sphere_rejects_overflow() {
      assert!(Mesh::sphere(1.0, u32::MAX, u32::MAX).is_err());
   }

   #[test]
   fn sphere_vertex_count_matches_tessellation() {
      let segments = 16u32;
      let rings = 8u32;
      let mesh = Mesh::sphere(0.5, segments, rings).unwrap();
      assert_eq!(mesh.vertex_count(), (segments * rings * 6) as usize);
      assert_eq!(mesh.triangle_normals().len(), mesh.triangle_count());
   }
}

use alloc::vec::Vec;
use he3d_core::{Error, Mesh};
use he3d_math::{Vec2, Vec3};

/// Parse a Wavefront OBJ mesh from memory.
/// 从内存解析 Wavefront OBJ 网格。
///
/// Supported: `v`, `vt`, and `f` faces with `v`, `v/vt`, `v/vt/vn`, or `v//vn` corner syntax.
/// OBJ puts the texture origin at the bottom left, so V is flipped into this renderer's top-left convention on the way in.
/// Faces with three or more vertices are triangulated as a fan.
/// `vn` and other lines (`o`, `s`, `mtllib`, `usemtl`, comments) are ignored.
/// Negative indices are rejected by skipping that face.
/// 支持 `v`、`vt` 以及 `f` 面，角语法为 `v`、`v/vt`、`v/vt/vn` 或 `v//vn`。
/// OBJ 的纹理原点在左下角，因此导入时 V 被翻转到本渲染器左上角的约定。
/// 具有三个及以上顶点的面按扇形三角化。
/// `vn` 及其他行（`o`、`s`、`mtllib`、`usemtl`、注释）被忽略。
/// 负索引通过跳过该面来拒绝。
///
/// # Errors
/// Returns an error when the input is not valid UTF-8, when parsing fails, or when the produced mesh is invalid (empty, non-triangle vertex count, or over budget).
/// 输入不是合法 UTF-8、解析失败，或产出的网格无效（空、顶点数不是 3 的倍数、超过顶点上限）时返回相应错误。
///
/// # Examples
/// 示例
/// ```
/// use he3d_assets::parse_obj;
///
/// let obj = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
/// let mesh = parse_obj(obj)?;
/// assert_eq!(mesh.triangle_count(), 1);
/// # Ok::<(), he3d_core::Error>(())
/// ```
pub fn parse_obj(bytes: &[u8]) -> Result<Mesh, Error> {
   let text = core::str::from_utf8(bytes).map_err(|_| Error::InvalidData)?;

   let mut positions: Vec<Vec3> = Vec::new();
   let mut uvs: Vec<Vec2> = Vec::new();
   let mut faces: Vec<Face> = Vec::new();

   for line in text.lines() {
      let line = line.trim();
      if line.is_empty() {
         continue;
      }
      if let Some(rest) = line.strip_prefix("v ") {
         let parts: Vec<&str> = rest.split_whitespace().take(3).collect();
         if parts.len() == 3 {
            if let (Ok(x), Ok(y), Ok(z)) = (
               parse_f32(parts[0]),
               parse_f32(parts[1]),
               parse_f32(parts[2]),
            ) {
               positions.push(Vec3::new(x, y, z));
            }
         }
      } else if let Some(rest) = line.strip_prefix("vt ") {
         let parts: Vec<&str> = rest.split_whitespace().take(2).collect();
         if parts.len() == 2 {
            if let (Ok(u), Ok(raw_v)) = (parse_f32(parts[0]), parse_f32(parts[1])) {
               uvs.push(Vec2::new(u, 1.0 - raw_v));
            }
         }
      } else if let Some(rest) = line.strip_prefix("f ") {
         if let Some(face) = parse_face(rest) {
            faces.push(face);
         }
      }
   }

   if positions.is_empty() || faces.is_empty() {
      return Err(Error::InvalidData);
   }

   let mut vertices = Vec::new();
   let mut mesh_uvs = Vec::new();

   for face in &faces {
      if face.0.len() < 3 {
         continue;
      }
      let valid = face.0.iter().all(|corner| {
         (corner.vertex as usize) < positions.len()
            && corner.uv.is_none_or(|uv| (uv as usize) < uvs.len())
      });
      if !valid {
         continue;
      }

      let pivot = face.0[0];
      for i in 1..face.0.len() - 1 {
         let a = pivot;
         let b = face.0[i];
         let c = face.0[i + 1];
         vertices.push(positions[a.vertex as usize]);
         vertices.push(positions[b.vertex as usize]);
         vertices.push(positions[c.vertex as usize]);
         for corner in [a, b, c] {
            mesh_uvs.push(corner.uv.map_or(Vec2::ZERO, |uv| uvs[uv as usize]));
         }
      }
   }

   if vertices.is_empty() {
      return Err(Error::InvalidData);
   }
   Mesh::new(vertices, mesh_uvs)
}

/// A face corner: 1-based OBJ indices converted to 0-based.
/// 面角：将 1 起始的 OBJ 索引转换为 0 起始。
#[derive(Clone, Copy)]
struct Corner {
   vertex: u32,
   uv: Option<u32>,
}

/// A face: its corners in order.
/// 面：按顺序排列的角。
struct Face(Vec<Corner>);

fn parse_face(rest: &str) -> Option<Face> {
   let mut corners = Vec::with_capacity(4);
   for token in rest.split_whitespace() {
      // Negative (relative) indices are not supported.
      // 不支持负（相对）索引。
      if token.starts_with('-') {
         return None;
      }
      let mut segments = token.split('/');
      let vertex = segments.next()?.parse::<u32>().ok()?;
      if vertex == 0 {
         return None;
      }
      let uv = match segments.next() {
         None | Some("") => None,
         Some(text) => {
            if text.starts_with('-') {
               return None;
            }
            let parsed = text.parse::<u32>().ok()?;
            if parsed == 0 {
               return None;
            }
            Some(parsed - 1)
         }
      };
      corners.push(Corner {
         vertex: vertex - 1,
         uv,
      });
   }
   if corners.len() < 3 {
      return None;
   }
   Some(Face(corners))
}

fn parse_f32(text: &str) -> Result<f32, core::num::ParseFloatError> {
   text.parse::<f32>()
}

#[cfg(test)]
mod tests {
   use super::*;

   const TRIANGLE_OBJ: &[u8] = b"\
v 0 0 0\n\
v 1 0 0\n\
v 0 1 0\n\
vt 0 0\n\
vt 1 0\n\
vt 0 1\n\
f 1/1 2/2 3/3\n";

   #[test]
   fn parses_triangle_face() {
      let mesh = parse_obj(TRIANGLE_OBJ).unwrap();
      assert_eq!(mesh.vertex_count(), 3);
      assert_eq!(mesh.triangle_count(), 1);
      assert_eq!(mesh.vertices()[0], Vec3::ZERO);
      assert_eq!(mesh.vertices()[2], Vec3::new(0.0, 1.0, 0.0));
   }

   #[test]
   fn flips_v_uv_coordinate() {
      let mesh = parse_obj(TRIANGLE_OBJ).unwrap();
      // Raw vt v=0 becomes 1.0, v=1 becomes 0.0.
      // 原始 vt v=0 变为 1.0，v=1 变为 0.0。
      assert_eq!(mesh.uvs()[0], Vec2::new(0.0, 1.0));
      assert_eq!(mesh.uvs()[2], Vec2::new(0.0, 0.0));
   }

   #[test]
   fn triangulates_quad_fan() {
      let obj = b"\
v 0 0 0\n\
v 1 0 0\n\
v 1 1 0\n\
v 0 1 0\n\
f 1 2 3 4\n";
      let mesh = parse_obj(obj).unwrap();
      assert_eq!(mesh.vertex_count(), 6);
      assert_eq!(mesh.triangle_count(), 2);
   }

   #[test]
   fn parses_v_vt_vn_corner_syntax() {
      let obj = b"\
v 0 0 0\n\
v 1 0 0\n\
v 0 1 0\n\
vt 0 0\n\
vt 1 0\n\
vt 0 1\n\
vn 0 0 1\n\
vn 0 0 1\n\
vn 0 0 1\n\
f 1/1/1 2/2/2 3/3/3\n";
      let mesh = parse_obj(obj).unwrap();
      assert_eq!(mesh.vertex_count(), 3);
   }

   #[test]
   fn rejects_negative_indices_by_skipping_face() {
      let obj = b"\
v 0 0 0\n\
v 1 0 0\n\
v 0 1 0\n\
f 1 2 -1\n";
      assert_eq!(parse_obj(obj), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_zero_uv_index_without_underflow() {
      // `f 1/0 ...` must not underflow `u32` (debug panic / release wrap); the whole face is rejected like any other invalid corner.
      // `f 1/0 ...` 不得让 `u32` 下溢（debug 下 panic / release 下回绕）；该面会被像其它非法角一样整体拒绝。
      let obj = b"\
v 0 0 0\n\
v 1 0 0\n\
v 0 1 0\n\
vt 0 0\n\
vt 1 0\n\
vt 0 1\n\
f 1/0 2/1 3/2\n";
      assert_eq!(parse_obj(obj), Err(Error::InvalidData));
   }

   #[test]
   fn ignores_out_of_range_face() {
      let obj = b"\
v 0 0 0\n\
v 1 0 0\n\
v 0 1 0\n\
f 1 2 99\n";
      assert_eq!(parse_obj(obj), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_empty_input() {
      assert_eq!(parse_obj(b""), Err(Error::InvalidData));
      assert_eq!(parse_obj(b"# only a comment\n"), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_non_utf8() {
      assert_eq!(parse_obj(&[0xff, 0xfe, 0x00]), Err(Error::InvalidData));
   }
}

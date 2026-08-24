use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::vec::Vec;
use he3d_core::{Error, Mesh};
use he3d_math::{Vec2, Vec3};
use serde::{Deserialize, Serialize};

const GLB_MAGIC: u32 = 0x4654_6c67; // GLB magic, "glTF". / GLB 魔数，即 "glTF"。
const CHUNK_JSON: u32 = 0x4e4f_534a; // JSON chunk type. / JSON 块类型。
const CHUNK_BIN: u32 = 0x004e_4942; // BIN chunk type, "BIN\0". / BIN 块类型，即 "BIN\0"。

const COMPONENT_U8: u32 = 5121;
const COMPONENT_U16: u32 = 5123;
const COMPONENT_U32: u32 = 5125;
const COMPONENT_F32: u32 = 5126;

const MODE_TRIANGLES: u32 = 4;

/// Parse a `.gltf` JSON document plus external buffer bytes into a mesh.
/// 将 `.gltf` JSON 文档及外部缓冲字节解析为网格。
///
/// Restricted subset, matching the reference loader's range: exactly one mesh with one triangle primitive, `POSITION` (vec3 f32) and optional `TEXCOORD_0` (vec2 f32) attributes, optional u8/u16/u32 indices, tightly packed buffer views (no strides), and buffers supplied as byte slices.
/// 受限子集，与参考加载器的范围一致：恰好一个网格含一个三角形图元，`POSITION`（vec3 f32）与可选的 `TEXCOORD_0`（vec2 f32）属性、可选的 u8/u16/u32 索引、紧密打包的缓冲视图（无步长），缓冲以字节切片提供。
/// Buffers with a `uri` (external files or data URIs) are rejected; use [`parse_glb`] for embedded buffer data.
/// 带 `uri` 的缓冲（外部文件或 data URI）会被拒绝；请使用 [`parse_glb`] 处理内嵌缓冲数据。
///
/// # Errors
/// 文档不符合受限子集、缓冲缺失或越界、属性/索引声明与缓冲长度不符时返回相应错误。
/// Returns an error when the document does not match the restricted subset, when a
/// buffer or accessor is missing or out of range, or when declared counts do not
/// match the buffer lengths.
pub fn parse_gltf(json_bytes: &[u8], external_buffers: &[&[u8]]) -> Result<Mesh, Error> {
   let doc: Gltf = serde_json::from_slice(json_bytes).map_err(|_| Error::InvalidData)?;

   if doc.buffers.is_empty() || doc.meshes.len() != 1 {
      return Err(Error::InvalidData);
   }
   if doc.buffers.iter().any(|buffer| buffer.uri.is_some()) {
      return Err(Error::UnsupportedFormat);
   }

   let primitives = &doc.meshes[0].primitives;
   if primitives.len() != 1 {
      return Err(Error::InvalidData);
   }
   let primitive = &primitives[0];
   if primitive.mode != MODE_TRIANGLES {
      return Err(Error::UnsupportedFormat);
   }

   let position_index = primitive
      .attributes
      .get("POSITION")
      .copied()
      .ok_or(Error::InvalidData)?;
   let position_accessor = doc
      .accessors
      .get(position_index)
      .ok_or(Error::InvalidData)?;
   let uv_accessor = match primitive.attributes.get("TEXCOORD_0") {
      Some(&index) => Some(doc.accessors.get(index).ok_or(Error::InvalidData)?),
      None => None,
   };
   let index_accessor = match primitive.indices {
      Some(index) => Some(doc.accessors.get(index).ok_or(Error::InvalidData)?),
      None => None,
   };

   let positions = read_vec3(position_accessor, &doc, external_buffers)?;
   let uvs = match uv_accessor {
      Some(accessor) => read_vec2(accessor, &doc, external_buffers)?,
      None => Vec::new(),
   };
   let indices = match index_accessor {
      Some(accessor) => read_indices(accessor, &doc, external_buffers)?,
      None => Vec::new(),
   };

   let output_count = if indices.is_empty() {
      positions.len()
   } else {
      indices.len()
   };
   if output_count == 0 || output_count % 3 != 0 {
      return Err(Error::InvalidData);
   }

   let mut vertices = Vec::with_capacity(output_count);
   let mut mesh_uvs = Vec::with_capacity(output_count);
   for i in 0..output_count {
      let vertex_index = if indices.is_empty() {
         i
      } else {
         indices[i] as usize
      };
      let position = positions.get(vertex_index).ok_or(Error::InvalidData)?;
      vertices.push(*position);
      // A missing TEXCOORD_0 attribute means no UVs (fill zero); a declared
      // accessor whose index escapes the data is corrupt and must not be
      // silently degraded to zero.
      // TEXCOORD_0 属性缺失意味着没有 UV（填零）；声明了访问器但其索引越界属于
      // 数据损坏，不得静默降级为零。
      let uv = if uvs.is_empty() {
         Vec2::ZERO
      } else {
         *uvs.get(vertex_index).ok_or(Error::InvalidData)?
      };
      mesh_uvs.push(uv);
   }

   Mesh::new(vertices, mesh_uvs)
}

/// Parse a GLB container: the JSON chunk becomes the glTF document and the BIN chunk becomes buffer 0.
/// 解析 GLB 容器：JSON 块成为 glTF 文档，BIN 块成为缓冲 0。
///
/// The container header (magic, version 2, declared length), the JSON-first chunk
/// ordering, and single-JSON/single-BIN cardinality are validated; a duplicate or
/// misplaced chunk is rejected rather than silently last-wins.
/// 会校验容器头（魔数、版本 2、声明长度）、JSON 在首位的块顺序，以及 JSON/BIN
/// 各至多一个的数量约束；重复或错位的块会被拒绝，而不是静默地后者覆盖前者。
///
/// # Errors
/// 容器头不合法（魔数/版本/声明长度错误）、块顺序或数量错误，或内部 JSON 文档
/// 无法解析时返回相应错误。
/// Returns an error when the container header (magic/version/declared length) is
/// invalid, when chunk ordering or cardinality is wrong, or when the embedded JSON
/// document does not parse.
pub fn parse_glb(bytes: &[u8]) -> Result<Mesh, Error> {
   if bytes.len() < 12 {
      return Err(Error::InvalidData);
   }
   if read_u32(bytes, 0) != GLB_MAGIC {
      return Err(Error::InvalidData);
   }
   if read_u32(bytes, 4) != 2 {
      return Err(Error::UnsupportedFormat);
   }
   let declared = read_u32(bytes, 8) as usize;
   if declared > bytes.len() {
      return Err(Error::InvalidData);
   }

   let mut offset = 12usize;
   let mut json: Option<&[u8]> = None;
   let mut bin: Option<&[u8]> = None;

   while offset + 8 <= declared {
      let chunk_length = read_u32(bytes, offset) as usize;
      let chunk_type = read_u32(bytes, offset + 4);
      // Saturating arithmetic keeps a hostile length from wrapping `usize` on
      // 32-bit targets and re-parsing the same bytes.
      // 饱和运算可防止恶意长度在 32 位目标上回绕 `usize` 并重复解析相同字节。
      let next = offset.saturating_add(8).saturating_add(chunk_length);
      if next > declared {
         return Err(Error::InvalidData);
      }
      let data = bytes.get(offset + 8..next).ok_or(Error::InvalidData)?;
      match chunk_type {
         CHUNK_JSON => {
            if json.is_some() {
               return Err(Error::InvalidData);
            }
            json = Some(data);
         }
         CHUNK_BIN => {
            if json.is_none() || bin.is_some() {
               return Err(Error::InvalidData);
            }
            bin = Some(data);
         }
         _ => {}
      }
      offset = next;
   }

   let json = json.ok_or(Error::InvalidData)?;
   let buffers: &[&[u8]] = match bin {
      Some(bin) => &[bin],
      None => &[],
   };
   parse_gltf(json, buffers)
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Gltf {
   #[serde(default)]
   buffers: Vec<Buffer>,
   #[serde(default)]
   buffer_views: Vec<BufferView>,
   #[serde(default)]
   accessors: Vec<Accessor>,
   #[serde(default)]
   meshes: Vec<GltfMesh>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Buffer {
   uri: Option<String>,
   #[allow(dead_code)]
   byte_length: usize,
}

#[derive(Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
struct BufferView {
   #[serde(rename = "buffer")]
   buffer_index: usize,
   #[serde(default)]
   byte_offset: usize,
   byte_length: usize,
   #[serde(default)]
   byte_stride: Option<usize>,
}

#[derive(Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
struct Accessor {
   #[serde(default)]
   buffer_view: Option<usize>,
   #[serde(default)]
   byte_offset: usize,
   component_type: u32,
   count: usize,
   #[serde(rename = "type")]
   data_type: String,
}

#[derive(Deserialize)]
struct GltfMesh {
   #[serde(default)]
   primitives: Vec<Primitive>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Primitive {
   #[serde(default)]
   attributes: BTreeMap<String, usize>,
   #[serde(default)]
   indices: Option<usize>,
   #[serde(default = "default_triangles")]
   mode: u32,
}

fn default_triangles() -> u32 {
   MODE_TRIANGLES
}

fn accessor_bytes<'a>(
   accessor: &Accessor,
   doc: &Gltf,
   external_buffers: &[&'a [u8]],
) -> Result<&'a [u8], Error> {
   let view_index = accessor.buffer_view.ok_or(Error::InvalidData)?;
   let view = doc.buffer_views.get(view_index).ok_or(Error::InvalidData)?;
   if let Some(stride) = view.byte_stride {
      if stride != 0 {
         return Err(Error::UnsupportedFormat);
      }
   }
   let buffer = external_buffers
      .get(view.buffer_index)
      .ok_or(Error::InvalidData)?;

   let element_size = component_size(accessor.component_type)
      .checked_mul(type_components(&accessor.data_type)?)
      .ok_or(Error::Overflow)?;
   let start = view
      .byte_offset
      .checked_add(accessor.byte_offset)
      .ok_or(Error::Overflow)?;
   let end = start
      .checked_add(
         element_size
            .checked_mul(accessor.count)
            .ok_or(Error::Overflow)?,
      )
      .ok_or(Error::Overflow)?;
   buffer.get(start..end).ok_or(Error::InvalidData)
}

fn component_size(component_type: u32) -> usize {
   match component_type {
      COMPONENT_U8 => 1,
      COMPONENT_U16 | 5122 => 2,
      COMPONENT_U32 | COMPONENT_F32 => 4,
      _ => 0,
   }
}

fn type_components(accessor_type: &str) -> Result<usize, Error> {
   match accessor_type {
      "SCALAR" => Ok(1),
      "VEC2" => Ok(2),
      "VEC3" => Ok(3),
      _ => Err(Error::UnsupportedFormat),
   }
}

fn read_vec3(
   accessor: &Accessor,
   doc: &Gltf,
   external_buffers: &[&[u8]],
) -> Result<Vec<Vec3>, Error> {
   if accessor.component_type != COMPONENT_F32 || accessor.data_type != "VEC3" {
      return Err(Error::UnsupportedFormat);
   }
   let bytes = accessor_bytes(accessor, doc, external_buffers)?;
   if bytes.len() != accessor.count * 12 {
      return Err(Error::InvalidData);
   }
   let mut out = Vec::with_capacity(accessor.count);
   for chunk in bytes.chunks_exact(12) {
      out.push(Vec3::new(
         f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]),
         f32::from_le_bytes([chunk[4], chunk[5], chunk[6], chunk[7]]),
         f32::from_le_bytes([chunk[8], chunk[9], chunk[10], chunk[11]]),
      ));
   }
   Ok(out)
}

fn read_vec2(
   accessor: &Accessor,
   doc: &Gltf,
   external_buffers: &[&[u8]],
) -> Result<Vec<Vec2>, Error> {
   if accessor.component_type != COMPONENT_F32 || accessor.data_type != "VEC2" {
      return Err(Error::UnsupportedFormat);
   }
   let bytes = accessor_bytes(accessor, doc, external_buffers)?;
   if bytes.len() != accessor.count * 8 {
      return Err(Error::InvalidData);
   }
   let mut out = Vec::with_capacity(accessor.count);
   for chunk in bytes.chunks_exact(8) {
      let u = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
      let v = f32::from_le_bytes([chunk[4], chunk[5], chunk[6], chunk[7]]);
      out.push(Vec2::new(u, 1.0 - v));
   }
   Ok(out)
}

fn read_indices(
   accessor: &Accessor,
   doc: &Gltf,
   external_buffers: &[&[u8]],
) -> Result<Vec<u32>, Error> {
   if accessor.data_type != "SCALAR" {
      return Err(Error::UnsupportedFormat);
   }
   let bytes = accessor_bytes(accessor, doc, external_buffers)?;
   // Validate the component type and the declared count before allocating, so a
   // malicious `count` cannot trigger a huge allocation for a tiny buffer.
   // 在分配前校验分量类型与声明的 count，使恶意的 `count` 无法用极小的缓冲
   // 触发巨大的分配。
   let element_size = match accessor.component_type {
      COMPONENT_U8 => 1,
      COMPONENT_U16 => 2,
      COMPONENT_U32 => 4,
      _ => return Err(Error::UnsupportedFormat),
   };
   let expected = accessor
      .count
      .checked_mul(element_size)
      .ok_or(Error::Overflow)?;
   if bytes.len() != expected {
      return Err(Error::InvalidData);
   }
   let mut out = Vec::with_capacity(accessor.count);
   match accessor.component_type {
      COMPONENT_U8 => out.extend(bytes.iter().map(|&b| u32::from(b))),
      COMPONENT_U16 => {
         for chunk in bytes.chunks_exact(2) {
            out.push(u32::from(u16::from_le_bytes([chunk[0], chunk[1]])));
         }
      }
      COMPONENT_U32 => {
         for chunk in bytes.chunks_exact(4) {
            out.push(u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
         }
      }
      _ => unreachable!("component type validated above"),
   }
   Ok(out)
}

fn read_u32(bytes: &[u8], offset: usize) -> u32 {
   u32::from_le_bytes([
      bytes[offset],
      bytes[offset + 1],
      bytes[offset + 2],
      bytes[offset + 3],
   ])
}

#[cfg(test)]
mod tests {
   use alloc::string::ToString;

   use super::*;

   struct Sample {
      json: Vec<u8>,
      bin: Vec<u8>,
   }

   fn f32_bytes(values: &[f32]) -> Vec<u8> {
      let mut out = Vec::new();
      for v in values {
         out.extend_from_slice(&v.to_le_bytes());
      }
      out
   }

   fn u16_bytes(values: &[u16]) -> Vec<u8> {
      let mut out = Vec::new();
      for v in values {
         out.extend_from_slice(&v.to_le_bytes());
      }
      out
   }

   fn pad4(len: usize) -> usize {
      (4 - len % 4) % 4
   }

   fn sample() -> Sample {
      let positions = [0f32, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
      let uvs = [0f32, 1.0, 1.0, 0.0, 0.0, 0.0];
      let indices: [u16; 3] = [0, 1, 2];

      let mut bin = Vec::new();
      let mut views = Vec::new();
      let mut accessors = Vec::new();
      let mut push = |bytes: &[u8]| -> usize {
         let offset = bin.len();
         bin.extend_from_slice(bytes);
         views.push(BufferView {
            buffer_index: 0,
            byte_offset: offset,
            byte_length: bytes.len(),
            byte_stride: None,
         });
         views.len() - 1
      };

      let pos_view = push(&f32_bytes(&positions));
      accessors.push(Accessor {
         buffer_view: Some(pos_view),
         byte_offset: 0,
         component_type: COMPONENT_F32,
         count: 3,
         data_type: "VEC3".to_string(),
      });
      let uv_view = push(&f32_bytes(&uvs));
      accessors.push(Accessor {
         buffer_view: Some(uv_view),
         byte_offset: 0,
         component_type: COMPONENT_F32,
         count: 3,
         data_type: "VEC2".to_string(),
      });
      let index_view = push(&u16_bytes(&indices));
      accessors.push(Accessor {
         buffer_view: Some(index_view),
         byte_offset: 0,
         component_type: COMPONENT_U16,
         count: 3,
         data_type: "SCALAR".to_string(),
      });

      let json = serde_json::json!({
          "asset": {"version": "2.0"},
          "buffers": [{"byteLength": bin.len()}],
          "bufferViews": views,
          "accessors": accessors,
          "meshes": [{
              "primitives": [{
                  "attributes": {"POSITION": 0, "TEXCOORD_0": 1},
                  "indices": 2,
                  "mode": 4
              }]
          }]
      })
      .to_string()
      .into_bytes();

      Sample { json, bin }
   }

   // Test fixtures are tiny, so the `usize`→`u32` narrowing below cannot truncate.
   // 测试夹具都很小，因此下面的 `usize`→`u32` 收窄不会截断。
   #[expect(clippy::cast_possible_truncation)]
   fn glb_from(sample: &Sample) -> Vec<u8> {
      let json_pad = pad4(sample.json.len());
      let bin_pad = pad4(sample.bin.len());
      let total = 12 + 8 + sample.json.len() + json_pad + 8 + sample.bin.len() + bin_pad;
      let mut out = Vec::new();
      out.extend_from_slice(&GLB_MAGIC.to_le_bytes());
      out.extend_from_slice(&2u32.to_le_bytes());
      out.extend_from_slice(&(total as u32).to_le_bytes());
      out.extend_from_slice(&((sample.json.len() + json_pad) as u32).to_le_bytes());
      out.extend_from_slice(&CHUNK_JSON.to_le_bytes());
      out.extend_from_slice(&sample.json);
      out.resize(out.len() + json_pad, b' ');
      out.extend_from_slice(&((sample.bin.len() + bin_pad) as u32).to_le_bytes());
      out.extend_from_slice(&CHUNK_BIN.to_le_bytes());
      out.extend_from_slice(&sample.bin);
      out.resize(out.len() + bin_pad, 0);
      out
   }

   #[test]
   fn parses_gltf_with_external_buffer() {
      let sample = sample();
      let mesh =
         parse_gltf(&sample.json, &[&sample.bin]).unwrap_or_else(|e| panic!("parse failed: {e:?}"));
      assert_eq!(mesh.vertex_count(), 3);
      assert_eq!(mesh.vertices()[0], Vec3::ZERO);
      assert_eq!(mesh.vertices()[2], Vec3::new(0.0, 1.0, 0.0));
      // Raw vt v=1.0 flips to 0.0; raw v=0.0 flips to 1.0.
      // 原始 vt v=1.0 翻转为 0.0；原始 v=0.0 翻转为 1.0。
      assert_eq!(mesh.uvs()[0], Vec2::new(0.0, 0.0));
      assert_eq!(mesh.uvs()[1], Vec2::new(1.0, 1.0));
   }

   #[test]
   fn parses_glb() {
      let sample = sample();
      let mesh = parse_glb(&glb_from(&sample)).expect("glb");
      assert_eq!(mesh.vertex_count(), 3);
      assert_eq!(mesh.vertices()[0], Vec3::ZERO);
   }

   #[test]
   fn rejects_non_glb() {
      assert_eq!(parse_glb(b"not a glb at all"), Err(Error::InvalidData));
      assert_eq!(parse_glb(&[]), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_non_version_2_glb() {
      let sample = sample();
      let mut glb = glb_from(&sample);
      glb[4..8].copy_from_slice(&1u32.to_le_bytes());
      assert_eq!(parse_glb(&glb), Err(Error::UnsupportedFormat));
   }

   #[test]
   fn rejects_glb_with_duplicate_json_chunk() {
      let sample = sample();
      let mut glb = glb_from(&sample);
      // Overwrite the BIN chunk type with a second JSON chunk.
      // 将 BIN 块类型改写为第二个 JSON 块。
      let json_len = 12 + 8 + sample.json.len() + pad4(sample.json.len());
      glb[json_len + 4..json_len + 8].copy_from_slice(&CHUNK_JSON.to_le_bytes());
      assert_eq!(parse_glb(&glb), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_glb_with_truncated_chunk() {
      let sample = sample();
      let glb = glb_from(&sample);
      let truncated = &glb[..glb.len() - 4];
      assert_eq!(parse_glb(truncated), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_bad_json() {
      assert_eq!(parse_gltf(b"{}", &[]), Err(Error::InvalidData));
      assert_eq!(parse_gltf(b"garbage", &[]), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_missing_buffer() {
      let sample = sample();
      assert_eq!(parse_gltf(&sample.json, &[]), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_buffer_with_uri() {
      let mut sample = sample();
      // Inject a uri by re-serializing with a modified buffer.
      // 通过修改缓冲后重新序列化注入 uri。
      let mut doc: serde_json::Value = serde_json::from_slice(&sample.json).unwrap();
      doc["buffers"][0]["uri"] = serde_json::json!("data:application/octet-stream;base64,AAAA");
      sample.json = serde_json::to_vec(&doc).unwrap();
      assert_eq!(
         parse_gltf(&sample.json, &[&sample.bin]),
         Err(Error::UnsupportedFormat)
      );
   }

   #[test]
   fn rejects_out_of_range_index() {
      let mut sample = sample();
      // Patch the index accessor data: change the last u16 index to 99.
      // 修补索引访问器数据：将最后一个 u16 索引改为 99。
      let indices_offset = 36 + 24; // positions (36) + uvs (24) / 顶点位置（36）+ UV（24）
      let bin = &mut sample.bin;
      bin[indices_offset + 4..indices_offset + 6].copy_from_slice(&99u16.to_le_bytes());
      assert_eq!(
         parse_gltf(&sample.json, &[&sample.bin]),
         Err(Error::InvalidData)
      );
   }
}

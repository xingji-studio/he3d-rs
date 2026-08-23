#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

//! HE3D assets.
//! HE3D 资源。
//!
//! Byte-based decoders for OBJ, glTF/GLB, and images (BMP/PNG/JPEG). All parsers take `&[u8]`.
//! 基于字节的 OBJ、glTF/GLB 及图像（BMP/PNG/JPEG）解码器。所有解析器均接收 `&[u8]`。
//! Resource path resolution belongs to the example or backend layer, never here.
//! 资源路径的解析属于示例层或后端层，绝不在此处理。

extern crate alloc;

mod image;
mod obj;

pub use image::decode_image;
pub use obj::parse_obj;

#[cfg(feature = "gltf")]
mod gltf;
#[cfg(feature = "gltf")]
pub use gltf::{parse_glb, parse_gltf};

pub use he3d_core::{ColorRgba8, Error, Mesh, Texture};

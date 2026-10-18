#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

//! HE3D core.
//! HE3D 核心。
//!
//! Owns the scene graph, resources (meshes, textures, materials), and stable generational IDs.
//! 拥有场景图、资源（网格、纹理、材质）以及稳定的代际 ID。
//! A [`Scene`] owns its resources directly through [`Arena`]s; IDs are cheap handles that never dangle.
//! 一个 [`Scene`] 直接通过 [`Arena`] 拥有其资源；ID 是永不悬垂的廉价句柄。
//!
//! Everything here is `no_std + alloc`. The optional `std` feature only adds convenience impls (e.g. `std::error::Error`); it never enables `std` for any dependency by default.
//! 此处所有内容都是 `no_std + alloc`。可选的 `std` 特性仅添加便捷实现（例如 `std::error::Error`）；它永远不会默认启用任何依赖的 `std`。

extern crate alloc;

mod arena;
mod camera;
mod color;
mod error;
mod light;
mod material;
mod mesh;
mod scene;
mod texture;
mod transform;

pub use arena::{Arena, Id};
pub use camera::{Camera, Projection};
pub use color::{ColorRgba8, Rgb};
pub use error::Error;
pub use light::DirectionalLight;
pub use material::Material;
pub use mesh::{MAX_VERTEX_COUNT, Mesh};
pub use scene::{RenderObject, Scene};
pub use texture::Texture;
pub use transform::Transform;

/// Stable handle to a [`Mesh`].
/// 指向 [`Mesh`] 的稳定句柄。
pub type MeshId = Id<Mesh>;
/// Stable handle to a [`Texture`].
/// 指向 [`Texture`] 的稳定句柄。
pub type TextureId = Id<Texture>;
/// Stable handle to a [`RenderObject`].
/// 指向 [`RenderObject`] 的稳定句柄。
pub type ObjectId = Id<RenderObject>;

#[cfg(test)]
mod tests {
   use alloc::vec;
   use alloc::vec::Vec;

   #[test]
   fn alloc_is_available() {
      let mut v: Vec<u32> = vec![0, 1, 2];
      v.push(3);
      assert_eq!(v, [0, 1, 2, 3]);
   }
}

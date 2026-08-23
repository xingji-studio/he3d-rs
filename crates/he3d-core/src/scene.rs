use crate::{
   Arena, DirectionalLight, Material, Mesh, MeshId, ObjectId, Texture, TextureId, Transform,
};

/// A renderable instance: a mesh, optional texture, transform, and material.
/// 一个可渲染实例：一个网格、可选纹理、变换和材质。
#[derive(Clone, Debug)]
pub struct RenderObject {
   /// The mesh to draw.
   /// 要绘制的网格。
   pub mesh: MeshId,
   /// Optional texture applied on top of the material tint.
   /// 叠加在材质色调上的可选纹理。
   pub texture: Option<TextureId>,
   /// Placement in world space.
   /// 世界空间中的放置。
   pub transform: Transform,
   /// Surface shading parameters.
   /// 表面着色参数。
   pub material: Material,
   /// Whether the object participates in rendering.
   /// 对象是否参与渲染。
   pub visible: bool,
}

/// Owns meshes, textures, render objects, and the scene light.
/// 拥有网格、纹理、渲染对象和场景光。
///
/// Resources live in [`Arena`]s; the scene exposes the arenas directly so callers can insert assets and get stable handles back.
/// 资源存放在 [`Arena`] 中；场景直接暴露这些 arena，以便调用方可以插入资源并获得稳定的句柄。
pub struct Scene {
   /// Mesh storage.
   /// 网格存储。
   pub meshes: Arena<Mesh>,
   /// Texture storage.
   /// 纹理存储。
   pub textures: Arena<Texture>,
   /// Render-object storage.
   /// 渲染对象存储。
   pub objects: Arena<RenderObject>,
   /// The single directional light of the scene.
   /// 场景中唯一的方向光。
   pub light: DirectionalLight,
}

impl Scene {
   /// Create an empty scene.
   /// 创建一个空场景。
   #[must_use]
   pub fn new() -> Self {
      Self {
         meshes: Arena::new(),
         textures: Arena::new(),
         objects: Arena::new(),
         light: DirectionalLight::default(),
      }
   }

   /// Insert a render object and return its handle.
   /// 插入一个渲染对象并返回其句柄。
   pub fn spawn(&mut self, object: RenderObject) -> ObjectId {
      self.objects.insert(object)
   }
}

impl Default for Scene {
   fn default() -> Self {
      Self::new()
   }
}

#[cfg(test)]
mod tests {
   use super::*;
   use crate::{Material, Rgb, Transform};

   #[test]
   fn spawn_returns_reachable_object() {
      let mut scene = Scene::new();
      let mesh = scene.meshes.insert(Mesh::triangle(1.0, 1.0).unwrap());
      let object = scene.spawn(RenderObject {
         mesh,
         texture: None,
         transform: Transform::default(),
         material: Material::new(Rgb::new(1.0, 0.5, 0.25)),
         visible: true,
      });
      let fetched = scene.objects.get(object).unwrap();
      assert_eq!(fetched.mesh, mesh);
      assert!(fetched.visible);
      assert_eq!(fetched.material.tint, Rgb::new(1.0, 0.5, 0.25));
   }

   #[test]
   fn removed_object_does_not_leak_into_scene() {
      let mut scene = Scene::new();
      let mesh = scene.meshes.insert(Mesh::plane(1.0, 1.0).unwrap());
      let object = scene.spawn(RenderObject {
         mesh,
         texture: None,
         transform: Transform::default(),
         material: Material::default(),
         visible: true,
      });
      assert_eq!(scene.objects.len(), 1);
      let _ = scene.objects.remove(object);
      assert!(scene.objects.is_empty());
   }
}

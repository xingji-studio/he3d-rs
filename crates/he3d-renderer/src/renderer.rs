use alloc::vec;
use alloc::vec::Vec;
use he3d_core::{Camera, ColorRgba8, Error, Mesh, RenderObject, Rgb, Scene, Texture};
use he3d_math::scalar::tan;
use he3d_math::{Vec2, Vec3, conv};

use crate::{AntiAliasing, RendererConfig};

/// Hard cap on framebuffer pixels, matching the reference renderer's `kMaximumRendererPixels`.
/// 帧缓冲像素数上限，与参考渲染器的 `kMaximumRendererPixels` 一致。
const MAX_FRAMEBUFFER_PIXELS: u64 = 16_777_216;

/// The software rasterizer.
/// 软件光栅化器。
///
/// Owns the color buffer and the inverse-depth buffer. Renders the whole scene on every [`Renderer::render`] call and hands the result back through [`Renderer::pixels`].
/// 拥有颜色缓冲与逆深度缓冲。每次调用 [`Renderer::render`] 都会渲染整个场景，并通过 [`Renderer::pixels`] 交回结果。
pub struct Renderer {
   width: u32,
   height: u32,
   anti_aliasing: AntiAliasing,
   color_buf: Vec<ColorRgba8>,
   depth_buf: Vec<f32>,
   /// Built-in 2×2 magenta/black checkerboard shown for objects without a texture.
   /// 内建的 2×2 紫黑格子，用于没有纹理的对象。
   missing_texture: Texture,
}

impl Renderer {
   /// Allocate a framebuffer, validating size and budget.
   /// 分配帧缓冲，校验尺寸与预算。
   ///
   /// # Errors
   /// 尺寸为 0、超过 [`MAX_FRAMEBUFFER_PIXELS`] 预算，或尺寸乘积溢出时返回相应错误。
   /// Returns an error when a dimension is zero, when the pixel budget
   /// exceeds `MAX_FRAMEBUFFER_PIXELS`, or when the product overflows.
   ///
   /// # Panics
   /// 内建 2×2 回退棋盘纹理构造失败时 panic（静态上不可能发生）。
   /// Panics only if the built-in 2×2 fallback texture cannot be constructed,
   /// which is statically impossible.
   pub fn new(config: RendererConfig) -> Result<Self, Error> {
      if config.width == 0 || config.height == 0 {
         return Err(Error::InvalidSize);
      }
      let pixel_count = u64::from(config.width)
         .checked_mul(u64::from(config.height))
         .ok_or(Error::Overflow)?;
      if pixel_count > MAX_FRAMEBUFFER_PIXELS {
         return Err(Error::InvalidSize);
      }
      let count = usize::try_from(pixel_count).expect("framebuffer budget fits host usize");
      let missing_texture = Texture::new(
         2,
         2,
         vec![
            ColorRgba8::MAGENTA,
            ColorRgba8::BLACK,
            ColorRgba8::BLACK,
            ColorRgba8::MAGENTA,
         ],
      )
      .expect("built-in 2x2 checkerboard is always valid");
      Ok(Self {
         width: config.width,
         height: config.height,
         anti_aliasing: config.anti_aliasing,
         color_buf: vec![ColorRgba8::BLACK; count],
         depth_buf: vec![0.0; count],
         missing_texture,
      })
   }

   /// Framebuffer width in pixels.
   /// 帧缓冲宽度（像素）。
   #[must_use]
   pub fn width(&self) -> u32 {
      self.width
   }

   /// Framebuffer height in pixels.
   /// 帧缓冲高度（像素）。
   #[must_use]
   pub fn height(&self) -> u32 {
      self.height
   }

   /// The rendered frame, top-to-bottom, left-to-right.
   /// 渲染完成的帧，从上到下、从左到右。
   #[must_use]
   pub fn pixels(&self) -> &[ColorRgba8] {
      &self.color_buf
   }

   /// Draw the scene into the framebuffer.
   /// 将场景绘制到帧缓冲中。
   ///
   /// # Errors
   /// `camera.near` 或 `camera.fov_y_degrees` 不是有限正数时返回 [`Error::InvalidSize`]。
   /// Returns [`Error::InvalidSize`] when `camera.near` or `camera.fov_y_degrees`
   /// is not a positive finite number.
   pub fn render(&mut self, scene: &Scene, camera: &Camera, clear_color: Rgb) -> Result<(), Error> {
      if !camera.near.is_finite() || camera.near <= 0.0 {
         return Err(Error::InvalidSize);
      }
      if !camera.fov_y_degrees.is_finite() || camera.fov_y_degrees <= 0.0 {
         return Err(Error::InvalidSize);
      }
      match self.anti_aliasing {
         AntiAliasing::None => {}
      }
      self.clear(clear_color);

      let light = Light {
         direction: scene.light.direction.normalize_or_zero(),
         color: scene.light.color,
         ambient: scene.light.ambient,
      };

      // Clone the tiny 2×2 fallback once per frame (not per object) so objects
      // without a texture can borrow it without aliasing `self`.
      // 每帧只克隆一次 2×2 回退贴图（而非每个对象一次），使无纹理对象能借用它而不与 `self` 冲突。
      let missing = self.missing_texture.clone();
      for (_, object) in scene.objects.iter() {
         if !object.visible {
            continue;
         }
         let Some(mesh) = scene.meshes.get(object.mesh) else {
            continue;
         };
         let texture = match object.texture.and_then(|id| scene.textures.get(id)) {
            Some(texture) => texture,
            None => &missing,
         };
         self.draw_mesh(mesh, object, texture, camera, light);
      }
      Ok(())
   }

   fn clear(&mut self, clear_color: Rgb) {
      self.color_buf.fill(clear_color.to_rgba8());
      self.depth_buf.fill(0.0);
   }

   // Framebuffer dimensions are bounded by MAX_FRAMEBUFFER_PIXELS, so the
   // u32→f32 conversions in this function cannot meaningfully lose precision.
   // 帧缓冲尺寸受 MAX_FRAMEBUFFER_PIXELS 约束，因此本函数中的 u32→f32
   // 转换不会产生有意义的精度损失。
   #[expect(clippy::cast_precision_loss)]
   fn draw_mesh(
      &mut self,
      mesh: &Mesh,
      object: &RenderObject,
      texture: &Texture,
      camera: &Camera,
      light: Light,
   ) {
      let cam_inv_rot = camera.orientation.inverse();
      let cam_pos = camera.position;
      let obj_rot = object.transform.orientation;
      let obj_pos = object.transform.position;
      let scale = object.transform.scale;
      let tint = object.material.tint;

      let fov_scale = 1.0 / tan(conv::deg_to_rad(camera.fov_y_degrees) * 0.5);
      let half_w = self.width as f32 * 0.5;
      let half_h = self.height as f32 * 0.5;
      let scale_x = fov_scale * half_w / (self.width as f32 / self.height as f32);
      let scale_y = fov_scale * half_h;
      let near = camera.near;

      let vertices = mesh.vertices();
      let uvs = mesh.uvs();
      let normals = mesh.triangle_normals();

      let mut tri_index = 0;
      for tri in vertices.chunks_exact(3) {
         let world = [
            obj_rot * (tri[0] * scale) + obj_pos,
            obj_rot * (tri[1] * scale) + obj_pos,
            obj_rot * (tri[2] * scale) + obj_pos,
         ];
         let view = [
            cam_inv_rot * (world[0] - cam_pos),
            cam_inv_rot * (world[1] - cam_pos),
            cam_inv_rot * (world[2] - cam_pos),
         ];

         let ab = view[1] - view[0];
         let ac = view[2] - view[0];
         if ab.cross(ac).dot(view[0]) >= 0.0 {
            tri_index += 1;
            continue;
         }

         let clip_in = [
            ClipVertex {
               view: view[0],
               uv: uvs[tri_index * 3],
            },
            ClipVertex {
               view: view[1],
               uv: uvs[tri_index * 3 + 1],
            },
            ClipVertex {
               view: view[2],
               uv: uvs[tri_index * 3 + 2],
            },
         ];
         let clipped = clip_triangle_to_near(&clip_in, near);
         if clipped.len() < 3 {
            tri_index += 1;
            continue;
         }

         let Some(&local_normal) = normals.get(tri_index) else {
            tri_index += 1;
            continue;
         };
         // For a T·R·S transform, the correct normal transform is the inverse-transpose of (R·S), which reduces to R·S⁻¹. Uniform scale (`S = I`) collapses to the plain rotation.
         // 对于 T·R·S 变换，正确的法线变换是 (R·S) 的逆转置，化简后为 R·S⁻¹。均匀缩放（`S = I`）时退化为纯旋转。
         let inv_scale = scale.recip();
         let n_world = obj_rot * (local_normal * inv_scale).normalize_or_zero();
         let diffuse = n_world.dot(light.direction).max(0.0);
         let lit = tint * light.color * (light.ambient + diffuse);

         for k in 1..clipped.len() - 1 {
            let tri_view = [clipped[0].view, clipped[k].view, clipped[k + 1].view];
            let screen = project_triangle(&tri_view, half_w, half_h, scale_x, scale_y);
            if triangle_outside_viewport(&screen, self.width, self.height) {
               continue;
            }
            if screen_triangle_area(&screen).abs() <= 0.0001 {
               continue;
            }
            let tri_uv = [clipped[0].uv, clipped[k].uv, clipped[k + 1].uv];
            self.rasterize_textured(&tri_view, &screen, &tri_uv, lit, texture);
         }
         tri_index += 1;
      }
   }

   // The casts below are deliberate rasterizer math: screen coordinates are
   // bounded by the framebuffer size, fixed-point lighting mirrors the reference
   // renderer, and the x/y-paired names are idiomatic here.
   // 下面的转换都是刻意为之的光栅化数学：屏幕坐标受帧缓冲尺寸约束，定点光照
   // 与参考渲染器一致，x/y 成对命名在此处是惯用法。
   #[expect(
      clippy::cast_possible_truncation,
      clippy::cast_precision_loss,
      clippy::cast_sign_loss,
      clippy::similar_names
   )]
   fn rasterize_textured(
      &mut self,
      view: &[Vec3; 3],
      screen: &[Vec2; 3],
      uvs: &[Vec2; 3],
      light: Rgb,
      tex: &Texture,
   ) {
      let left_f = screen[0].x.min(screen[1].x).min(screen[2].x);
      let right_f = screen[0].x.max(screen[1].x).max(screen[2].x);
      let top_f = screen[0].y.min(screen[1].y).min(screen[2].y);
      let bottom_f = screen[0].y.max(screen[1].y).max(screen[2].y);

      let width_i = i64::from(self.width);
      let height_i = i64::from(self.height);
      let x_start = (left_f as i64).clamp(0, width_i - 1);
      let x_end = (right_f as i64).saturating_add(1).clamp(0, width_i - 1);
      let y_start = (top_f as i64).clamp(0, height_i - 1);
      let y_end = (bottom_f as i64).saturating_add(1).clamp(0, height_i - 1);
      if x_start > x_end || y_start > y_end {
         return;
      }

      let (x0, y0) = (screen[0].x, screen[0].y);
      let (x1, y1) = (screen[1].x, screen[1].y);
      let (x2, y2) = (screen[2].x, screen[2].y);

      let area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
      if area.abs() < 0.0001 {
         return;
      }
      let inv_a = 1.0 / area;

      let dw0 = (y1 - y2) * inv_a;
      let dw1 = (y2 - y0) * inv_a;
      let dw2 = -dw0 - dw1;
      let iz0 = 1.0 / view[0].z;
      let iz1 = 1.0 / view[1].z;
      let iz2 = 1.0 / view[2].z;
      let uz0 = uvs[0] * iz0;
      let uz1 = uvs[1] * iz1;
      let uz2 = uvs[2] * iz2;
      let diz_dx = dw0 * iz0 + dw1 * iz1 + dw2 * iz2;
      let duz_dx = dw0 * uz0.x + dw1 * uz1.x + dw2 * uz2.x;
      let dvz_dx = dw0 * uz0.y + dw1 * uz1.y + dw2 * uz2.y;

      // Fixed-point light intensity, matching the reference renderer.
      // 定点光照强度，与参考渲染器一致。
      let lr = ((light.r * 256.0) as i32).clamp(0, 512);
      let lg = ((light.g * 256.0) as i32).clamp(0, 512);
      let lb = ((light.b * 256.0) as i32).clamp(0, 512);

      let stride = self.width as usize;
      for y in y_start..=y_end {
         let py = y as f32 + 0.5;
         let px0 = x_start as f32 + 0.5;
         let mut w0 = ((x1 - px0) * (y2 - py) - (y1 - py) * (x2 - px0)) * inv_a;
         let mut w1 = ((x2 - px0) * (y0 - py) - (y2 - py) * (x0 - px0)) * inv_a;
         let mut w2 = 1.0 - w0 - w1;
         let mut ciz = w0 * iz0 + w1 * iz1 + w2 * iz2;
         let mut cuz = w0 * uz0.x + w1 * uz1.x + w2 * uz2.x;
         let mut cvz = w0 * uz0.y + w1 * uz1.y + w2 * uz2.y;

         let row = (y as usize) * stride;
         for x in x_start..=x_end {
            let idx = row + x as usize;
            if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 && ciz > self.depth_buf[idx] {
               self.depth_buf[idx] = ciz;
               let inv = 1.0 / ciz;
               let texel = tex.sample(cuz * inv, cvz * inv);
               let shaded = ColorRgba8::from_rgba(
                  (((i32::from(texel.r) * lr) >> 8).clamp(0, 255)) as u8,
                  (((i32::from(texel.g) * lg) >> 8).clamp(0, 255)) as u8,
                  (((i32::from(texel.b) * lb) >> 8).clamp(0, 255)) as u8,
                  255,
               );
               self.color_buf[idx] = shaded;
            }
            w0 += dw0;
            w1 += dw1;
            w2 += dw2;
            ciz += diz_dx;
            cuz += duz_dx;
            cvz += dvz_dx;
         }
      }
   }
}

/// A view-space vertex carried through near-plane clipping, keeping its UV.
/// 穿过近平面裁剪的视空间顶点，保留其 UV。
#[derive(Clone, Copy)]
struct ClipVertex {
   view: Vec3,
   uv: Vec2,
}

/// Per-frame lighting parameters.
/// 每帧光照参数。
#[derive(Clone, Copy)]
struct Light {
   direction: Vec3,
   color: Rgb,
   ambient: f32,
}

fn lerp_clip(a: ClipVertex, b: ClipVertex, t: f32) -> ClipVertex {
   ClipVertex {
      view: a.view + (b.view - a.view) * t,
      uv: a.uv + (b.uv - a.uv) * t,
   }
}

fn clip_triangle_to_near(input: &[ClipVertex; 3], near: f32) -> Vec<ClipVertex> {
   let mut out = Vec::with_capacity(4);
   for i in 0..3 {
      let a = input[i];
      let b = input[(i + 1) % 3];
      let a_inside = a.view.z > near;
      let b_inside = b.view.z > near;
      match (a_inside, b_inside) {
         (true, true) => out.push(b),
         (true, false) => out.push(lerp_clip(a, b, (near - a.view.z) / (b.view.z - a.view.z))),
         (false, true) => {
            out.push(lerp_clip(a, b, (near - a.view.z) / (b.view.z - a.view.z)));
            out.push(b);
         }
         (false, false) => {}
      }
   }
   out
}

fn project_triangle(
   view: &[Vec3; 3],
   half_w: f32,
   half_h: f32,
   scale_x: f32,
   scale_y: f32,
) -> [Vec2; 3] {
   let mut screen = [Vec2::ZERO; 3];
   for (i, v) in view.iter().enumerate() {
      let inv_z = 1.0 / v.z;
      screen[i] = Vec2::new(
         half_w + v.x * inv_z * scale_x,
         half_h - v.y * inv_z * scale_y,
      );
   }
   screen
}

fn triangle_outside_viewport(screen: &[Vec2; 3], width: u32, height: u32) -> bool {
   // Viewport size is bounded by MAX_FRAMEBUFFER_PIXELS, so u32→f32 is exact enough
   // for these comparisons.
   // 视口尺寸受 MAX_FRAMEBUFFER_PIXELS 约束，因此 u32→f32 对这些比较而言足够精确。
   #[expect(clippy::cast_precision_loss)]
   {
      let w = width as f32;
      let h = height as f32;
      (screen[0].x < 0.0 && screen[1].x < 0.0 && screen[2].x < 0.0)
         || (screen[0].x >= w && screen[1].x >= w && screen[2].x >= w)
         || (screen[0].y < 0.0 && screen[1].y < 0.0 && screen[2].y < 0.0)
         || (screen[0].y >= h && screen[1].y >= h && screen[2].y >= h)
   }
}

fn screen_triangle_area(screen: &[Vec2; 3]) -> f32 {
   (screen[1].x - screen[0].x) * (screen[2].y - screen[0].y)
      - (screen[1].y - screen[0].y) * (screen[2].x - screen[0].x)
}

#[cfg(test)]
mod tests {
   use alloc::vec;

   use super::*;
   use he3d_core::{Material, Transform};
   use he3d_math::{Quat, Vec2, Vec3};

   fn triangle_scene() -> (Scene, Camera, Rgb) {
      let mut scene = Scene::new();
      let mesh = scene
         .meshes
         .insert(Mesh::triangle(1.6, 1.4).expect("triangle mesh"));
      let solid = scene
         .textures
         .insert(Texture::solid(ColorRgba8::from_rgb(255, 89, 38)));
      scene.spawn(RenderObject {
         mesh,
         texture: Some(solid),
         transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 70.0);
      (scene, camera, Rgb::new(0.08, 0.10, 0.14))
   }

   fn render_triangle(width: u32, height: u32) -> Renderer {
      let (scene, camera, clear) = triangle_scene();
      let mut renderer = Renderer::new(RendererConfig::new(width, height)).unwrap();
      renderer.render(&scene, &camera, clear).unwrap();
      renderer
   }

   fn pixel(renderer: &Renderer, x: u32, y: u32) -> ColorRgba8 {
      renderer.pixels()[(y * renderer.width() + x) as usize]
   }

   #[test]
   fn triangle_render_matches_reference_shading() {
      let renderer = render_triangle(64, 64);
      // The triangle faces -Z (flat-shaded with the reference vertex order), and the default light (0,-1,1) hits its back, so it receives only ambient 0.15. The solid texture (255,89,38) is lit in fixed point: (255·38, 89·38, 38·38) >> 8 = (37, 13, 5).
      // 该三角形面朝 -Z（按参考顶点顺序平面着色），默认光源 (0,-1,1) 击中其背面，因此只受环境光 0.15 影响。纯色纹理 (255,89,38) 经定点光照：(255·38, 89·38, 38·38) >> 8 = (37, 13, 5)。
      let center = pixel(&renderer, 32, 32);
      assert_eq!(center, ColorRgba8::from_rgb(37, 13, 5));
   }

   #[test]
   fn diffuse_light_brightens_facing_surface() {
      let (mut scene, camera, clear) = triangle_scene();
      // Light travels along -Z, straight into the triangle's face normal.
      // 光线沿 -Z 传播，正对三角形的面法线。
      scene.light.direction = Vec3::new(0.0, 0.0, -1.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer.render(&scene, &camera, clear).unwrap();
      // tint * (0.15 ambient + 1.0 diffuse) = (1.15, 0.4025, 0.1725).
      // tint * (0.15 环境光 + 1.0 漫反射) = (1.15, 0.4025, 0.1725)。
      let center = pixel(&renderer, 32, 32);
      assert_eq!(center, ColorRgba8::from_rgb(255, 102, 43));
   }

   #[test]
   fn background_color_fills_empty_areas() {
      let renderer = render_triangle(64, 64);
      // Corner far from the triangle keeps the clear color.
      // 远离三角形的角落保持清屏颜色。
      let corner = pixel(&renderer, 2, 2);
      let clear = Rgb::new(0.08, 0.10, 0.14).to_rgba8();
      assert_eq!(corner, clear);
   }

   #[test]
   fn near_triangle_occludes_far_triangle() {
      let mut scene = Scene::new();
      scene.light.direction = Vec3::new(0.0, 0.0, -1.0);
      let mesh = scene.meshes.insert(Mesh::triangle(2.0, 2.0).unwrap());
      let far = Rgb::new(0.2, 0.6, 0.9);
      let near = Rgb::new(0.9, 0.2, 0.3);
      let far_tex = scene.textures.insert(Texture::solid(far.to_rgba8()));
      let near_tex = scene.textures.insert(Texture::solid(near.to_rgba8()));
      scene.spawn(RenderObject {
         mesh,
         texture: Some(far_tex),
         transform: Transform::from_position(Vec3::new(0.0, 0.0, 8.0)),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      scene.spawn(RenderObject {
         mesh,
         texture: Some(near_tex),
         transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 70.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer
         .render(&scene, &camera, Rgb::new(0.05, 0.05, 0.05))
         .unwrap();
      // Center pixel is the near triangle's lit color: near * 1.15.
      // 中心像素为近处三角形的受光颜色：near * 1.15。
      let center = pixel(&renderer, 32, 32);
      assert_eq!(center, ColorRgba8::from_rgb(255, 58, 87));
   }

   #[test]
   fn textured_triangle_lights_and_samples() {
      let mut scene = Scene::new();
      scene.light.direction = Vec3::new(0.0, 0.0, -1.0);
      let mesh = scene.meshes.insert(Mesh::triangle(1.6, 1.4).unwrap());
      let tex_pixels = vec![ColorRgba8::from_rgb(200, 0, 0); 16];
      let texture = scene
         .textures
         .insert(Texture::new(4, 4, tex_pixels).unwrap());
      scene.spawn(RenderObject {
         mesh,
         texture: Some(texture),
         transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 70.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer
         .render(&scene, &camera, Rgb::new(0.05, 0.05, 0.05))
         .unwrap();
      // texel(200,0,0) × (ambient 0.15 + diffuse 1.0) → fixed-point (229, 0, 0).
      // texel(200,0,0) × (环境光 0.15 + 漫反射 1.0) → 定点 (229, 0, 0)。
      let center = pixel(&renderer, 32, 32);
      assert_eq!(center, ColorRgba8::from_rgb(229, 0, 0));
   }

   #[test]
   fn textured_uv_wraps_into_texture() {
      let mut scene = Scene::new();
      // Back-facing winding (-Z normal, like the reference triangle), lit along -Z so it renders and receives diffuse light.
      // 背向绕序（-Z 法线，与参考三角形一致），沿 -Z 方向打光，使其能够渲染并接受漫反射光照。
      scene.light.direction = Vec3::new(0.0, 0.0, -1.0);
      let texture = scene.textures.insert(
         Texture::new(
            2,
            2,
            vec![
               ColorRgba8::from_rgb(255, 0, 0),
               ColorRgba8::from_rgb(0, 255, 0),
               ColorRgba8::from_rgb(0, 0, 255),
               ColorRgba8::WHITE,
            ],
         )
         .unwrap(),
      );
      let vertices = vec![
         Vec3::new(-0.5, -0.5, 3.0),
         Vec3::new(0.0, 0.5, 3.0),
         Vec3::new(0.5, -0.5, 3.0),
      ];
      let uvs = vec![Vec2::splat(3.5); 3];
      let mesh = scene.meshes.insert(Mesh::new(vertices, uvs).unwrap());
      scene.spawn(RenderObject {
         mesh,
         texture: Some(texture),
         transform: Transform::default(),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 70.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer
         .render(&scene, &camera, Rgb::new(0.0, 0.0, 0.0))
         .unwrap();
      // uv=3.5 wraps to 0.5 → texel (1,1) = white; white × 1.15 → 255.
      // uv=3.5 回绕为 0.5 → texel (1,1) = 白色；白色 × 1.15 → 255。
      let center = pixel(&renderer, 32, 32);
      assert_eq!(center, ColorRgba8::WHITE);
   }

   #[test]
   fn invisible_objects_are_skipped() {
      let mut scene = Scene::new();
      let mesh = scene.meshes.insert(Mesh::triangle(1.6, 1.4).unwrap());
      scene.spawn(RenderObject {
         mesh,
         texture: None,
         transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
         material: Material::new(Rgb::new(1.0, 0.35, 0.15)),
         visible: false,
      });
      let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 70.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer
         .render(&scene, &camera, Rgb::new(0.08, 0.10, 0.14))
         .unwrap();
      let center = pixel(&renderer, 32, 32);
      assert_eq!(center, Rgb::new(0.08, 0.10, 0.14).to_rgba8());
   }

   fn fnv1a_pixels(pixels: &[ColorRgba8]) -> u64 {
      let mut hash = 0xcbf2_9ce4_8422_2325u64;
      for p in pixels {
         for byte in [p.r, p.g, p.b, p.a] {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
         }
      }
      hash
   }

   #[test]
   fn triangle_golden_snapshot() {
      let renderer = render_triangle(64, 64);
      // Frozen snapshot of a known scene; guards against accidental renderer regressions. Regenerate by deleting this value and reading the failure output.
      // 已知场景的固定快照，防止渲染器意外回归。如需重新生成，请删除该值并读取失败输出。
      assert_eq!(fnv1a_pixels(renderer.pixels()), 8_090_596_331_343_206_561);
   }

   #[test]
   fn none_texture_renders_missing_checkerboard() {
      let mut scene = Scene::new();
      // Light along -Z so the triangle faces diffuse light; no texture on purpose.
      // 光线沿 -Z 传播使三角形正面受光；故意不给纹理。
      scene.light.direction = Vec3::new(0.0, 0.0, -1.0);
      let mesh = scene.meshes.insert(Mesh::triangle(1.6, 1.4).unwrap());
      scene.spawn(RenderObject {
         mesh,
         texture: None,
         transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 70.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer
         .render(&scene, &camera, Rgb::new(0.05, 0.05, 0.05))
         .unwrap();
      // A 2×2 magenta/black checkerboard: the center texel is black, so the lit center pixel stays black.
      // 2×2 紫黑格子：中心纹素为黑色，因此受光后的中心像素仍为黑色。
      let center = pixel(&renderer, 32, 32);
      assert_eq!(center, ColorRgba8::BLACK);
   }

   #[test]
   fn tint_modulates_solid_texture() {
      let mut scene = Scene::new();
      scene.light.direction = Vec3::new(0.0, 0.0, -1.0);
      let mesh = scene.meshes.insert(Mesh::triangle(1.6, 1.4).unwrap());
      let red = scene
         .textures
         .insert(Texture::solid(ColorRgba8::from_rgb(200, 0, 0)));
      scene.spawn(RenderObject {
         mesh,
         texture: Some(red),
         transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
         // Half-intensity tint halves the lit red channel.
         // 半强度色调将受光的红色通道减半。
         material: Material::new(Rgb::new(0.5, 1.0, 1.0)),
         visible: true,
      });
      let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 70.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer
         .render(&scene, &camera, Rgb::new(0.05, 0.05, 0.05))
         .unwrap();
      // texel(200,0,0) × tint(0.5,1,1) × 1.15 → fixed point (114, 0, 0).
      // texel(200,0,0) × tint(0.5,1,1) × 1.15 → 定点 (114, 0, 0)。
      let center = pixel(&renderer, 32, 32);
      assert_eq!(center, ColorRgba8::from_rgb(114, 0, 0));
   }

   #[test]
   fn invalid_config_is_rejected() {
      assert!(Renderer::new(RendererConfig::new(0, 64)).is_err());
      let overflow = RendererConfig {
         width: u32::MAX,
         height: u32::MAX,
         anti_aliasing: AntiAliasing::None,
      };
      assert!(Renderer::new(overflow).is_err());
   }

   #[test]
   fn rejects_invalid_camera_parameters() {
      let (scene, camera, clear) = triangle_scene();
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();

      let mut bad = camera;
      bad.near = 0.0;
      assert_eq!(
         renderer.render(&scene, &bad, clear),
         Err(Error::InvalidSize)
      );
      bad.near = -1.0;
      assert_eq!(
         renderer.render(&scene, &bad, clear),
         Err(Error::InvalidSize)
      );
      bad.near = f32::NAN;
      assert_eq!(
         renderer.render(&scene, &bad, clear),
         Err(Error::InvalidSize)
      );

      bad = camera;
      bad.fov_y_degrees = 0.0;
      assert_eq!(
         renderer.render(&scene, &bad, clear),
         Err(Error::InvalidSize)
      );
      bad.fov_y_degrees = f32::NAN;
      assert_eq!(
         renderer.render(&scene, &bad, clear),
         Err(Error::InvalidSize)
      );
   }

   #[test]
   fn adversarial_inputs_never_panic() {
      let cameras = [
         Camera::new(Vec3::new(f32::NAN, 0.0, 0.0), Quat::IDENTITY, 70.0),
         Camera::new(Vec3::ZERO, Quat::IDENTITY, f32::NAN),
         Camera::new(Vec3::ZERO, Quat::IDENTITY, 0.0),
         Camera::new(Vec3::ZERO, Quat::IDENTITY, 360.0),
         Camera::new(Vec3::new(1e30, 1e30, -1e30), Quat::IDENTITY, 70.0),
      ];
      let mut renderer = Renderer::new(RendererConfig::new(32, 32)).unwrap();

      for camera in &cameras {
         let mut scene = Scene::new();
         let mesh = scene.meshes.insert(Mesh::triangle(1.6, 1.4).unwrap());
         scene.spawn(RenderObject {
            mesh,
            texture: None,
            transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
            material: Material::new(Rgb::new(1.0, 0.35, 0.15)),
            visible: true,
         });
         let _ = renderer.render(&scene, camera, Rgb::new(0.0, 0.0, 0.0));

         // Extreme object transform.
         // 极端物体变换。
         scene.spawn(RenderObject {
            mesh,
            texture: None,
            transform: Transform {
               position: Vec3::new(f32::NAN, f32::INFINITY, -1e38),
               ..Transform::default()
            },
            material: Material::default(),
            visible: true,
         });
         let _ = renderer.render(&scene, camera, Rgb::new(0.0, 0.0, 0.0));
      }

      // Stale mesh id.
      // 失效的网格 id。
      let mut scene = Scene::new();
      let stale = scene.meshes.insert(Mesh::triangle(1.0, 1.0).unwrap());
      let _ = scene.meshes.remove(stale);
      scene.spawn(RenderObject {
         mesh: stale,
         texture: None,
         transform: Transform::default(),
         material: Material::default(),
         visible: true,
      });
      let _ = renderer.render(&scene, &Camera::default(), Rgb::new(0.0, 0.0, 0.0));

      // Empty scene.
      // 空场景。
      let empty = Scene::new();
      let _ = renderer.render(&empty, &Camera::default(), Rgb::new(0.1, 0.2, 0.3));
   }

   #[test]
   fn fuzz_style_random_scenes_never_panic() {
      fn next_u64(state: &mut u64) -> u64 {
         let mut x = *state;
         x ^= x << 13;
         x ^= x >> 7;
         x ^= x << 17;
         *state = x;
         x
      }
      fn next_f32(state: &mut u64) -> f32 {
         f32::from_bits((next_u64(state) & 0xffff_ffff) as u32)
      }

      let mut renderer = Renderer::new(RendererConfig::new(32, 32)).unwrap();
      let mut state = 0x1234_5678_9abc_def0u64;
      for _ in 0..200 {
         let mut scene = Scene::new();
         let mesh = scene.meshes.insert(Mesh::triangle(1.0, 1.0).unwrap());
         let pos = Vec3::new(
            next_f32(&mut state) * 200.0 - 100.0,
            next_f32(&mut state) * 200.0 - 100.0,
            next_f32(&mut state) * 200.0 + 0.5,
         );
         let rot = Quat::from_rotation_y(next_f32(&mut state) * 100.0);
         let color = Rgb::new(
            next_f32(&mut state).abs(),
            next_f32(&mut state).abs(),
            next_f32(&mut state).abs(),
         );
         scene.spawn(RenderObject {
            mesh,
            texture: None,
            transform: Transform::from_position_orientation(pos, rot),
            material: Material::new(color),
            visible: next_u64(&mut state) & 1 == 1,
         });
         let camera = Camera::new(
            Vec3::new(
               next_f32(&mut state) * 40.0 - 20.0,
               next_f32(&mut state) * 40.0 - 20.0,
               next_f32(&mut state) * 40.0 - 20.0,
            ),
            Quat::from_rotation_y(next_f32(&mut state) * 20.0),
            next_f32(&mut state).abs() % 180.0,
         );
         let clear = Rgb::new(
            next_f32(&mut state).abs().min(1.0),
            next_f32(&mut state).abs().min(1.0),
            next_f32(&mut state).abs().min(1.0),
         );
         let _ = renderer.render(&scene, &camera, clear);
      }
   }
}

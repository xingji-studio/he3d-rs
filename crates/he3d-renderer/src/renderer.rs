use alloc::vec;
use alloc::vec::Vec;
use he3d_core::{Camera, ColorRgba8, Error, Mesh, Projection, RenderObject, Rgb, Scene, Texture};
use he3d_math::scalar::tan;
use he3d_math::{Vec2, Vec3, conv};

use crate::{AntiAliasing, MAX_SSAA_SCALE, RendererConfig};

/// Hard cap on framebuffer pixels: 4096 x 4096.
///
/// The operative limit is memory, not representation: colour and `f32` depth are four bytes per pixel each, so this is 64 MiB + 64 MiB — and 512 MiB once 4x MSAA multiplies the colour side.
/// Beyond that it stops being a software rasterizer anyone can reasonably run.
/// Staying far inside a `u32` for byte offsets is a consequence, not the reason.
/// 真正的约束是内存而非表示范围：颜色与 `f32` 深度各占每像素四字节，因此这里是 64 MiB + 64 MiB——而在 4x MSAA 把颜色侧放大之后是 512 MiB。
/// 超过这个量级，它就不再是任何人能合理运行的软件光栅化器了。
/// 字节偏移远在 `u32` 之内是结果，不是理由。
const MAX_FRAMEBUFFER_PIXELS: u64 = 16_777_216;

/// 4x MSAA sample offsets inside a pixel: the centres of the four sub-pixels of a 2x2 grid, which is the cheapest arrangement that still samples every quadrant.
/// 像素内的 4x MSAA 样本偏移：2x2 网格中四个子像素的中心，这是仍能覆盖每个象限的最省排布。
const MSAA_SAMPLE_OFFSETS: [[f32; 2]; 4] = [[0.25, 0.25], [0.75, 0.25], [0.25, 0.75], [0.75, 0.75]];

/// Which buffer currently holds the finished frame for [`Renderer::pixels`].
/// 当前由 [`Renderer::pixels`] 提交的帧所在的缓冲。
#[derive(Clone, Copy)]
enum PresentSource {
   Color,
   Fxaa,
   Taa,
   Ssaa,
}

/// The software rasterizer.
/// 软件光栅化器。
///
/// Owns the color buffer and the inverse-depth buffer.
/// Renders the whole scene on every [`Renderer::render`] call and hands the result back through [`Renderer::pixels`].
/// 拥有颜色缓冲与逆深度缓冲。
/// 每次调用 [`Renderer::render`] 都会渲染整个场景，并通过 [`Renderer::pixels`] 交回结果。
///
/// Anti-aliasing runs at the end of each frame, and the order is forced by what each stage consumes: MSAA resolves per-sample colour first, the screen-space passes (FXAA, then TAA) work on whole pixels, and SSAA's resolution change has to come last or the passes after it would run at the wrong scale.
/// 抗锯齿在每帧末尾执行，其顺序由各阶段消费什么决定：MSAA 先解析逐样本颜色；屏幕空间阶段（FXAA，随后 TAA）以整像素为输入；SSAA 的分辨率变化必须放在最后，否则其后的阶段会在错误的尺度上运行。
pub struct Renderer {
   /// Internal render width in pixels; equals the output width unless SSAA scales it up.
   /// 内部渲染宽度（像素）；除非 SSAA 放大，否则等于输出宽度。
   width: u32,
   /// Internal render height in pixels; equals the output height unless SSAA scales it up.
   /// 内部渲染高度（像素）；除非 SSAA 放大，否则等于输出高度。
   height: u32,
   /// Output width presented to the caller (the unscaled config width).
   /// 提交给调用方的输出宽度（未缩放的配置宽度）。
   output_width: u32,
   /// Output height presented to the caller (the unscaled config height).
   /// 提交给调用方的输出高度（未缩放的配置高度）。
   output_height: u32,
   anti_aliasing: AntiAliasing,
   /// Supersampling scale, clamped to `1..=MAX_SSAA_SCALE`.
   /// 超采样倍率，钳制在 `1..=MAX_SSAA_SCALE`。
   ssaa_scale: u32,
   color_buf: Vec<ColorRgba8>,
   depth_buf: Vec<f32>,
   fxaa_buf: Vec<ColorRgba8>,
   taa_buf: Vec<ColorRgba8>,
   taa_history: Vec<ColorRgba8>,
   taa_depth: Vec<f32>,
   ssaa_buf: Vec<ColorRgba8>,
   msaa_color_buf: Vec<ColorRgba8>,
   msaa_depth_buf: Vec<f32>,
   taa_valid: bool,
   taa_frame_index: u32,
   presented: PresentSource,
   presented_width: u32,
   presented_height: u32,
   /// Built-in 2×2 magenta/black checkerboard shown for objects without a texture.
   /// 内建的 2×2 紫黑格子，用于没有纹理的对象。
   missing_texture: Texture,
}

impl core::fmt::Debug for Renderer {
   fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
      // Framebuffer contents would flood the debug output; print dimensions instead.
      // 帧缓冲内容会淹没调试输出；改为打印尺寸。
      f.debug_struct("Renderer")
         .field("width", &self.width)
         .field("height", &self.height)
         .field("output_width", &self.output_width)
         .field("output_height", &self.output_height)
         .field("anti_aliasing", &self.anti_aliasing)
         .finish_non_exhaustive()
   }
}

impl Renderer {
   /// Allocate a framebuffer, validating size and budget.
   /// 分配帧缓冲，校验尺寸与预算。
   ///
   /// With SSAA the internal render buffer is `scale`× the output resolution, still capped by `MAX_FRAMEBUFFER_PIXELS`.
   /// 使用 SSAA 时，内部渲染缓冲是输出分辨率的 `scale` 倍，仍受 `MAX_FRAMEBUFFER_PIXELS` 上限约束。
   ///
   /// # Examples
   /// # 示例
   /// ```
   /// use he3d_core::{Camera, Rgb, Scene};
   /// use he3d_renderer::{Renderer, RendererConfig};
   ///
   /// let mut renderer = Renderer::new(RendererConfig::new(64, 36))?;
   /// renderer.render(&Scene::new(), &Camera::default(), Rgb::new(0.1, 0.1, 0.1))?;
   /// assert_eq!(renderer.pixels().len(), 64 * 36);
   /// # Ok::<(), he3d_core::Error>(())
   /// ```
   ///
   /// # Errors
   /// Returns an error when a dimension is zero, when the pixel budget exceeds `MAX_FRAMEBUFFER_PIXELS`, or when the product overflows.
   /// 尺寸为 0、超过 `MAX_FRAMEBUFFER_PIXELS` 预算，或尺寸乘积溢出时返回相应错误。
   ///
   /// # Panics
   /// Panics only if the built-in 2×2 fallback texture cannot be constructed, which is statically impossible.
   /// 内建 2×2 回退棋盘纹理构造失败时 panic（静态上不可能发生）。
   pub fn new(config: RendererConfig) -> Result<Self, Error> {
      if config.width == 0 || config.height == 0 {
         return Err(Error::InvalidSize);
      }
      let ssaa_scale = match config.anti_aliasing {
         AntiAliasing::Ssaa(scale) => scale.clamp(1, MAX_SSAA_SCALE),
         _ => 1,
      };
      let (render_width, render_height, count) =
         renderer_buffer_size(config.width, config.height, ssaa_scale).ok_or(Error::InvalidSize)?;
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
         width: render_width,
         height: render_height,
         output_width: config.width,
         output_height: config.height,
         anti_aliasing: config.anti_aliasing,
         ssaa_scale,
         color_buf: vec![ColorRgba8::BLACK; count],
         depth_buf: vec![0.0; count],
         fxaa_buf: Vec::new(),
         taa_buf: Vec::new(),
         taa_history: Vec::new(),
         taa_depth: Vec::new(),
         ssaa_buf: Vec::new(),
         msaa_color_buf: Vec::new(),
         msaa_depth_buf: Vec::new(),
         taa_valid: false,
         taa_frame_index: 0,
         presented: PresentSource::Color,
         presented_width: config.width,
         presented_height: config.height,
         missing_texture,
      })
   }

   /// Presented framebuffer width in pixels (the output size, down from the internal render size when SSAA is active).
   /// 已提交帧缓冲宽度（像素）——SSAA 生效时由内部渲染尺寸降为输出尺寸。
   #[must_use]
   #[expect(clippy::misnamed_getters)]
   pub fn width(&self) -> u32 {
      self.presented_width
   }

   /// Presented framebuffer height in pixels (the output size, down from the internal render size when SSAA is active).
   /// 已提交帧缓冲高度（像素）——SSAA 生效时由内部渲染尺寸降为输出尺寸。
   #[must_use]
   #[expect(clippy::misnamed_getters)]
   pub fn height(&self) -> u32 {
      self.presented_height
   }

   /// The rendered frame, top-to-bottom, left-to-right.
   /// 渲染完成的帧，从上到下、从左到右。
   #[must_use]
   pub fn pixels(&self) -> &[ColorRgba8] {
      match self.presented {
         PresentSource::Color => &self.color_buf,
         PresentSource::Fxaa => &self.fxaa_buf,
         PresentSource::Taa => &self.taa_buf,
         PresentSource::Ssaa => &self.ssaa_buf,
      }
   }

   /// Draw the scene into the framebuffer.
   /// 将场景绘制到帧缓冲中。
   ///
   /// # Errors
   /// Returns [`Error::InvalidSize`] when `camera.near` or the projection's parameter (the perspective `fov_y_degrees` or the orthographic `height`) is not a positive finite number.
   /// `camera.near` 或投影参数（透视的 `fov_y_degrees`、正交的 `height`）不是有限正数时返回 [`Error::InvalidSize`]。
   pub fn render(&mut self, scene: &Scene, camera: &Camera, clear_color: Rgb) -> Result<(), Error> {
      if !camera.near.is_finite() || camera.near <= 0.0 {
         return Err(Error::InvalidSize);
      }
      let extent = match camera.projection {
         Projection::Perspective { fov_y_degrees } => fov_y_degrees,
         Projection::Orthographic { height } => height,
      };
      if !extent.is_finite() || extent <= 0.0 {
         return Err(Error::InvalidSize);
      }
      self.clear(clear_color);

      let light = Light {
         direction: scene.light.direction.normalize_or_zero(),
         color: scene.light.color,
         ambient: scene.light.ambient,
      };

      let jitter = self.current_taa_jitter();

      // Clone the tiny 2×2 fallback once per frame (not per object) so objects without a texture can borrow it without aliasing `self`.
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
         self.draw_mesh(mesh, object, texture, camera, light, jitter);
      }
      self.present();
      Ok(())
   }

   /// Run the post-processing chain and record which buffer holds the frame.
   /// 执行后处理链，并记录承载帧的缓冲。
   fn present(&mut self) {
      let width = self.width as usize;
      let height = self.height as usize;

      if self.anti_aliasing == AntiAliasing::Msaa {
         self.resolve_msaa();
      }

      let mut present = PresentSource::Color;
      if self.anti_aliasing == AntiAliasing::Fxaa
         && apply_fxaa(&self.color_buf, &mut self.fxaa_buf, width, height)
      {
         present = PresentSource::Fxaa;
      }

      if self.anti_aliasing == AntiAliasing::Taa {
         let source = match present {
            PresentSource::Color => &self.color_buf,
            PresentSource::Fxaa => &self.fxaa_buf,
            PresentSource::Taa | PresentSource::Ssaa => {
               unreachable!("TAA runs before SSAA and never reads its own output")
            }
         };
         if apply_taa(
            source,
            &self.depth_buf,
            &mut self.taa_buf,
            &mut self.taa_history,
            &mut self.taa_depth,
            width,
            height,
            &mut self.taa_valid,
            &mut self.taa_frame_index,
         ) {
            present = PresentSource::Taa;
         }
      } else {
         self.taa_valid = false;
         self.taa_frame_index = 0;
      }

      if self.ssaa_scale > 1 {
         let source = match present {
            PresentSource::Color => &self.color_buf,
            PresentSource::Fxaa => &self.fxaa_buf,
            PresentSource::Taa => &self.taa_buf,
            PresentSource::Ssaa => {
               unreachable!("SSAA downsample runs last and never reads its own output")
            }
         };
         if apply_ssaa(
            source,
            &mut self.ssaa_buf,
            self.width,
            self.height,
            self.ssaa_scale,
            self.output_width,
            self.output_height,
         ) {
            present = PresentSource::Ssaa;
         }
      }

      self.presented = present;
      self.presented_width = if self.ssaa_scale > 1 {
         self.output_width
      } else {
         self.width
      };
      self.presented_height = if self.ssaa_scale > 1 {
         self.output_height
      } else {
         self.height
      };
   }

   fn clear(&mut self, clear_color: Rgb) {
      self.color_buf.fill(clear_color.to_rgba8());
      self.depth_buf.fill(0.0);
      if self.anti_aliasing == AntiAliasing::Msaa && self.ensure_msaa_buffers() {
         self.msaa_color_buf.fill(clear_color.to_rgba8());
         self.msaa_depth_buf.fill(0.0);
      }
   }

   /// Lazily allocate the 4× per-sample buffers, seeded from the color/depth buffers so a frame never shows garbage when MSAA turns on mid-scene.
   /// 懒分配 4× 逐样本缓冲，并以颜色/深度缓冲播种，使 MSAA 中途开启时也不会出现垃圾。
   fn ensure_msaa_buffers(&mut self) -> bool {
      if self.width == 0 || self.height == 0 {
         return false;
      }
      if !self.msaa_color_buf.is_empty() && !self.msaa_depth_buf.is_empty() {
         return true;
      }
      let render_total = self.color_buf.len();
      let sample_total = match u64::try_from(render_total) {
         Ok(total) => total.checked_mul(4),
         Err(_) => None,
      };
      let Some(sample_total) = sample_total else {
         return false;
      };
      if sample_total > MAX_FRAMEBUFFER_PIXELS * 4 {
         return false;
      }
      let sample_total = usize::try_from(sample_total).expect("sample budget fits host usize");
      self.msaa_color_buf = vec![ColorRgba8::BLACK; sample_total];
      self.msaa_depth_buf = vec![0.0; sample_total];
      for (i, (&color, &depth)) in self.color_buf.iter().zip(self.depth_buf.iter()).enumerate() {
         let base = i * 4;
         for sample in 0..4 {
            self.msaa_color_buf[base + sample] = color;
            self.msaa_depth_buf[base + sample] = depth;
         }
      }
      true
   }

   /// Average the four per-sample colors into `color_buf` and keep the deepest sample per pixel in `depth_buf`.
   /// 将四个逐样本颜色平均写入 `color_buf`，并在 `depth_buf` 中保留每像素最深的样本。
   fn resolve_msaa(&mut self) {
      if self.msaa_color_buf.is_empty() || self.msaa_depth_buf.is_empty() {
         return;
      }
      resolve_msaa(
         &mut self.color_buf,
         &mut self.depth_buf,
         &self.msaa_color_buf,
         &self.msaa_depth_buf,
      );
   }

   // Framebuffer dimensions are bounded by MAX_FRAMEBUFFER_PIXELS, so the u32→f32 conversions in this function cannot meaningfully lose precision.
   // 帧缓冲尺寸受 MAX_FRAMEBUFFER_PIXELS 约束，因此本函数中的 u32→f32 转换不会产生有意义的精度损失。
   #[expect(clippy::cast_precision_loss)]
   fn draw_mesh(
      &mut self,
      mesh: &Mesh,
      object: &RenderObject,
      texture: &Texture,
      camera: &Camera,
      light: Light,
      jitter: Vec2,
   ) {
      let cam_inv_rot = camera.orientation.inverse();
      let scale = object.transform.scale;
      let tint = object.material.tint;

      // Fold the object rotation and the world→view rotation into one quaternion, and the translation into one vector, so each vertex costs a single rotation instead of two.
      // Rotations are norm-preserving, so shading can equally be done in view space with the light direction rotated once here.
      // 把物体旋转与世界→视图旋转合成同一个四元数，并把平移合成同一个向量，使每个顶点只需一次旋转而非两次。
      // 旋转保持长度，因此着色同样可以在视图空间完成——只需在此把光照方向旋转一次。
      let view_rot = cam_inv_rot * object.transform.orientation;
      let view_trans = cam_inv_rot * (object.transform.position - camera.position);
      let light_dir = cam_inv_rot * light.direction;

      let half_w = self.width as f32 * 0.5;
      let half_h = self.height as f32 * 0.5;
      // Both projections are isotropic in pixels, so one scale serves both axes and the viewport's aspect ratio is handled implicitly.
      // 两种投影在像素上都是各向同性的，因此一个尺度即可服务两根轴，视口宽高比被隐式处理。
      let (pixel_scale, perspective) = match camera.projection {
         Projection::Perspective { fov_y_degrees } => {
            (half_h / tan(conv::deg_to_rad(fov_y_degrees) * 0.5), true)
         }
         Projection::Orthographic { height } => (half_h / (height * 0.5), false),
      };
      let near = camera.near;

      let vertices = mesh.vertices();
      let uvs = mesh.uvs();
      let normals = mesh.triangle_normals();

      let mut tri_index = 0;
      for tri in vertices.chunks_exact(3) {
         let view = [
            view_rot * (tri[0] * scale) + view_trans,
            view_rot * (tri[1] * scale) + view_trans,
            view_rot * (tri[2] * scale) + view_trans,
         ];

         let ab = view[1] - view[0];
         let ac = view[2] - view[0];
         // The cull direction is the view-space eye vector: the vertex itself for a perspective camera, the constant forward axis for an orthographic one whose eye sits at infinity.
         // 剔除方向是视图空间中的眼向量：透视相机是顶点自身，正交相机（眼在无穷远）是恒定的前向轴。
         let cull = if perspective { view[0] } else { Vec3::Z };
         if ab.cross(ac).dot(cull) >= 0.0 {
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
         // Clip into a fixed stack buffer: a clipped triangle has at most four vertices, so this avoids a heap allocation per triangle.
         // 裁剪写入固定大小的栈缓冲：三角形被裁剪后至多四个顶点，因此避免了逐三角形的堆分配。
         let mut clipped = [ClipVertex {
            view: Vec3::ZERO,
            uv: Vec2::ZERO,
         }; 4];
         let count = clip_triangle_to_near(&clip_in, near, &mut clipped);
         if count < 3 {
            tri_index += 1;
            continue;
         }

         let Some(&local_normal) = normals.get(tri_index) else {
            tri_index += 1;
            continue;
         };
         // For a T·R·S transform, the correct normal transform is the inverse-transpose of (R·S), which reduces to R·S⁻¹.
         // Uniform scale (`S = I`) collapses to the plain rotation.
         // Shading is done in view space, where `view_rot` and `light_dir` share the camera rotation, so their dot product equals the world-space one.
         // 对于 T·R·S 变换，正确的法线变换是 (R·S) 的逆转置，化简后为 R·S⁻¹。
         // 均匀缩放（`S = I`）时退化为纯旋转。
         // 着色在视图空间完成，其中 `view_rot` 与 `light_dir` 共享相机旋转，因此它们的点积与世界空间一致。
         let inv_scale = scale.recip();
         let n_view = view_rot * (local_normal * inv_scale).normalize_or_zero();
         let diffuse = n_view.dot(light_dir).max(0.0);
         let lit = tint * light.color * (light.ambient + diffuse);

         for k in 1..count - 1 {
            let tri_view = [clipped[0].view, clipped[k].view, clipped[k + 1].view];
            let screen =
               project_triangle(&tri_view, half_w, half_h, pixel_scale, jitter, perspective);
            if triangle_outside_viewport(&screen, self.width, self.height) {
               continue;
            }
            if screen_triangle_area(&screen).abs() <= 0.0001 {
               continue;
            }
            let tri_uv = [clipped[0].uv, clipped[k].uv, clipped[k + 1].uv];
            self.rasterize_textured(&tri_view, &screen, &tri_uv, lit, texture, perspective);
         }
         tri_index += 1;
      }
   }

   // Screen coordinates are floats but the framebuffer is indexed with integers, so the bound and index casts below are deliberate rather than incidental.
   // 屏幕坐标是浮点数，而帧缓冲用整数索引，因此下面的边界与索引转换是刻意为之，而非顺手写成。
   #[expect(
      clippy::cast_possible_truncation,
      clippy::cast_precision_loss,
      clippy::cast_sign_loss
   )]
   fn rasterize_textured(
      &mut self,
      view: &[Vec3; 3],
      screen: &[Vec2; 3],
      uvs: &[Vec2; 3],
      light: Rgb,
      tex: &Texture,
      perspective: bool,
   ) {
      let Some(t) = ScreenTriangle::new(view, screen, uvs, perspective) else {
         return;
      };
      let shading = Shading::new(light, tex);

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

      let msaa = self.anti_aliasing == AntiAliasing::Msaa && self.ensure_msaa_buffers();
      let stride = self.width as usize;
      for y in y_start..=y_end {
         let py = y as f32 + 0.5;
         let px0 = x_start as f32 + 0.5;
         let mut w0 = ((t.x1 - px0) * (t.y2 - py) - (t.y1 - py) * (t.x2 - px0)) * t.inv_a;
         let mut w1 = ((t.x2 - px0) * (t.y0 - py) - (t.y2 - py) * (t.x0 - px0)) * t.inv_a;
         let mut w2 = 1.0 - w0 - w1;
         let mut ciz = w0 * t.rz0 + w1 * t.rz1 + w2 * t.rz2;
         let mut cuz = w0 * t.uz0.x + w1 * t.uz1.x + w2 * t.uz2.x;
         let mut cvz = w0 * t.uz0.y + w1 * t.uz1.y + w2 * t.uz2.y;

         let row = (y as usize) * stride;
         for x in x_start..=x_end {
            if msaa {
               self.resolve_msaa_pixel(x, y, &t, &shading, tex, perspective);
            } else {
               let idx = row + x as usize;
               if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 && ciz > self.depth_buf[idx] {
                  self.depth_buf[idx] = ciz;
                  if let Some(color) = shading.flat {
                     self.color_buf[idx] = color;
                  } else {
                     let inv = if perspective { 1.0 / ciz } else { 1.0 };
                     self.color_buf[idx] = shading.sample(tex, cuz * inv, cvz * inv);
                  }
               }
            }
            w0 += t.dw0;
            w1 += t.dw1;
            w2 += t.dw2;
            ciz += t.diz_dx;
            cuz += t.duz_dx;
            cvz += t.dvz_dx;
         }
      }
   }

   /// Depth-test and shade the four MSAA samples covering one pixel, then update that pixel's depth.
   /// 对覆盖单个像素的四个 MSAA 样本做深度测试与着色，然后更新该像素的深度。
   // The same rasterizer casts as `rasterize_textured`, over the sample position, the view depth, and the texture coordinate.
   // 与 `rasterize_textured` 相同的栅格化转换，施加在样本位置、视深度与纹理坐标上。
   #[expect(
      clippy::cast_possible_truncation,
      clippy::cast_precision_loss,
      clippy::cast_sign_loss,
      clippy::many_single_char_names
   )]
   fn resolve_msaa_pixel(
      &mut self,
      x: i64,
      y: i64,
      t: &ScreenTriangle,
      shading: &Shading,
      tex: &Texture,
      perspective: bool,
   ) {
      let idx = y as usize * self.width as usize + x as usize;
      let mask = triangle_sample_mask4(x, y, t.x0, t.y0, t.x1, t.y1, t.x2, t.y2, t.inv_a);
      let mut pixel_max_depth = self.depth_buf[idx];
      for sample in 0..4u32 {
         if mask & ((1 << sample) as u8) == 0 {
            continue;
         }
         let spx = x as f32 + MSAA_SAMPLE_OFFSETS[sample as usize][0];
         let spy = y as f32 + MSAA_SAMPLE_OFFSETS[sample as usize][1];
         let sw0 = ((t.x1 - spx) * (t.y2 - spy) - (t.y1 - spy) * (t.x2 - spx)) * t.inv_a;
         let sw1 = ((t.x2 - spx) * (t.y0 - spy) - (t.y2 - spy) * (t.x0 - spx)) * t.inv_a;
         let sw2 = 1.0 - sw0 - sw1;
         let sample_depth = sw0 * t.rz0 + sw1 * t.rz1 + sw2 * t.rz2;
         let sample_idx = idx * 4 + sample as usize;
         if sample_depth > self.msaa_depth_buf[sample_idx] {
            self.msaa_depth_buf[sample_idx] = sample_depth;
            let inv_sample = if perspective { 1.0 / sample_depth } else { 1.0 };
            let u = (sw0 * t.uz0.x + sw1 * t.uz1.x + sw2 * t.uz2.x) * inv_sample;
            let v = (sw0 * t.uz0.y + sw1 * t.uz1.y + sw2 * t.uz2.y) * inv_sample;
            self.msaa_color_buf[sample_idx] = shading.sample(tex, u, v);
            pixel_max_depth = pixel_max_depth.max(sample_depth);
         }
      }
      self.depth_buf[idx] = pixel_max_depth;
   }

   /// Current TAA sub-pixel jitter in pixels, from the 8-sample Halton(2,3) sequence.
   /// 当前 TAA 亚像素抖动（像素单位），来自 8 样本 Halton(2,3) 序列。
   #[must_use]
   fn current_taa_jitter(&self) -> Vec2 {
      if self.anti_aliasing != AntiAliasing::Taa || self.width == 0 || self.height == 0 {
         return Vec2::ZERO;
      }
      let sample = (self.taa_frame_index & 7) + 1;
      let x = halton(sample, 2) - 0.5;
      let y = halton(sample, 3) - 0.5;
      Vec2::new(x * 0.5, y * 0.5)
   }
}

/// A screen-space triangle, with the edge coefficients and depth and texture interpolants a rasterization pass walks across it.
/// 屏幕空间三角形，以及光栅化遍历时穿过它的边系数与深度、纹理插值量。
struct ScreenTriangle {
   x0: f32,
   y0: f32,
   x1: f32,
   y1: f32,
   x2: f32,
   y2: f32,
   inv_a: f32,
   dw0: f32,
   dw1: f32,
   dw2: f32,
   rz0: f32,
   rz1: f32,
   rz2: f32,
   uz0: Vec2,
   uz1: Vec2,
   uz2: Vec2,
   diz_dx: f32,
   duz_dx: f32,
   dvz_dx: f32,
}

impl ScreenTriangle {
   /// Derive the edge and interpolation coefficients, or return `None` when the triangle is degenerate.
   /// 推导边与插值系数；三角形退化时返回 `None`。
   fn new(
      view: &[Vec3; 3],
      screen: &[Vec2; 3],
      uvs: &[Vec2; 3],
      perspective: bool,
   ) -> Option<Self> {
      let (x0, y0) = (screen[0].x, screen[0].y);
      let (x1, y1) = (screen[1].x, screen[1].y);
      let (x2, y2) = (screen[2].x, screen[2].y);
      let area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
      if area.abs() < 0.0001 {
         return None;
      }
      let inv_a = 1.0 / area;
      let dw0 = (y1 - y2) * inv_a;
      let dw1 = (y2 - y0) * inv_a;
      let dw2 = -dw0 - dw1;
      // Depth is keyed on the reciprocal view depth, which is monotonic in both projections.
      // 深度以视深度的倒数作为键，这在两种投影下都是单调的。
      let rz0 = 1.0 / view[0].z;
      let rz1 = 1.0 / view[1].z;
      let rz2 = 1.0 / view[2].z;
      // Texture weights are the reciprocal depth under perspective and a constant under orthographic, so the sample path divides by the interpolated reciprocal depth only under perspective.
      // 纹理权重在透视下是深度倒数，在正交下是常数，因此采样路径仅在透视下用插值的深度倒数做除法。
      let (wz0, wz1, wz2) = if perspective {
         (rz0, rz1, rz2)
      } else {
         (1.0, 1.0, 1.0)
      };
      let uz0 = uvs[0] * wz0;
      let uz1 = uvs[1] * wz1;
      let uz2 = uvs[2] * wz2;
      Some(Self {
         x0,
         y0,
         x1,
         y1,
         x2,
         y2,
         inv_a,
         dw0,
         dw1,
         dw2,
         rz0,
         rz1,
         rz2,
         uz0,
         uz1,
         uz2,
         diz_dx: dw0 * rz0 + dw1 * rz1 + dw2 * rz2,
         duz_dx: dw0 * uz0.x + dw1 * uz1.x + dw2 * uz2.x,
         dvz_dx: dw0 * uz0.y + dw1 * uz1.y + dw2 * uz2.y,
      })
   }
}

/// Per-triangle shading inputs: fixed-point light intensity, plus a constant texture's pre-shaded colour when there is one.
/// 逐三角形的着色输入：定点光照强度，以及常量纹理预先着色后的颜色（若有）。
struct Shading {
   lr: i32,
   lg: i32,
   lb: i32,
   flat: Option<ColorRgba8>,
}

impl Shading {
   /// Fold the light into fixed-point intensity and pre-shade a constant texture.
   /// 把光照折入定点强度，并预先着色常量纹理。
   // The float-to-integer cast below is the fixed-point conversion itself.
   // 下面的浮点转整数转换正是定点转换本身。
   #[expect(clippy::cast_possible_truncation)]
   fn new(light: Rgb, tex: &Texture) -> Self {
      // Fixed-point light intensity: 8 fractional bits, so the per-texel shade is one integer multiply and shift instead of three float multiplies.
      // 定点光照强度：8 位小数，因此逐纹素着色只需一次整数乘法与移位，而不是三次浮点乘法。
      let lr = ((light.r * 256.0) as i32).clamp(0, 512);
      let lg = ((light.g * 256.0) as i32).clamp(0, 512);
      let lb = ((light.b * 256.0) as i32).clamp(0, 512);

      // A 1x1 texture is constant, so its shaded colour is a per-triangle constant.
      // Hoisting it keeps the constant-texture case out of the per-pixel loop, where the branch would otherwise be paid on every fragment of every ordinary texture.
      // 1x1 纹理恒定，其着色后颜色也是逐三角形常量。
      // 把它提到循环外，避免普通纹理的每个片元都支付这一分支。
      let flat = if tex.pixels().len() == 1 {
         Some(shade(tex.pixels()[0], lr, lg, lb))
      } else {
         None
      };
      Self { lr, lg, lb, flat }
   }

   /// Sample the texture at `u`/`v` and apply the fixed-point light.
   /// 在 `u`/`v` 处采样纹理并施加定点光照。
   fn sample(&self, tex: &Texture, u: f32, v: f32) -> ColorRgba8 {
      shade(tex.sample(u, v), self.lr, self.lg, self.lb)
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

/// Compute the internal render buffer for an output size and SSAA scale, capped by `MAX_FRAMEBUFFER_PIXELS`, or `None` when the request exceeds the budget.
/// 为输出尺寸与 SSAA 倍率计算内部渲染缓冲尺寸，受 `MAX_FRAMEBUFFER_PIXELS` 约束；请求超出预算时返回 `None`。
#[must_use]
fn renderer_buffer_size(width: u32, height: u32, scale: u32) -> Option<(u32, u32, usize)> {
   if width == 0 || height == 0 || !(1..=MAX_SSAA_SCALE).contains(&scale) {
      return None;
   }
   let scaled_w = u64::from(width).checked_mul(u64::from(scale))?;
   let scaled_h = u64::from(height).checked_mul(u64::from(scale))?;
   let total = scaled_w.checked_mul(scaled_h)?;
   if total > MAX_FRAMEBUFFER_PIXELS {
      return None;
   }
   let count = usize::try_from(total).ok()?;
   // `total` is capped by MAX_FRAMEBUFFER_PIXELS, so both dimensions fit u32.
   // `total` 受 MAX_FRAMEBUFFER_PIXELS 约束，因此两个维度都能放进 u32。
   #[expect(clippy::cast_possible_truncation)]
   Some((scaled_w as u32, scaled_h as u32, count))
}

/// Halton sequence value for `index` in a given base.
/// 给定基下 `index` 的 Halton 序列值。
#[expect(clippy::cast_precision_loss)]
fn halton(mut index: u32, base: u32) -> f32 {
   let mut f = 1.0_f32;
   let mut r = 0.0_f32;
   while index > 0 {
      f /= base as f32;
      r += f * (index % base) as f32;
      index /= base;
   }
   r
}

/// Average the 4× MSAA samples into `color` and merge the deepest per pixel into `depth`.
/// No-op when the MSAA buffers are empty.
/// 将 4× MSAA 样本平均写入 `color`，并把每像素最深值合并进 `depth`。
/// MSAA 缓冲为空时不做任何事。
#[expect(clippy::cast_possible_truncation)]
fn resolve_msaa(
   color: &mut [ColorRgba8],
   depth: &mut [f32],
   msaa_color: &[ColorRgba8],
   msaa_depth: &[f32],
) {
   if msaa_color.is_empty() || msaa_depth.is_empty() {
      return;
   }
   for i in 0..color.len() {
      let base = i * 4;
      let mut out = [0u32; 3];
      let mut max_depth = f32::NEG_INFINITY;
      for sample in 0..4 {
         let c = msaa_color[base + sample];
         out[0] += u32::from(c.r);
         out[1] += u32::from(c.g);
         out[2] += u32::from(c.b);
         max_depth = max_depth.max(msaa_depth[base + sample]);
      }
      color[i] = ColorRgba8::from_rgba(
         (out[0] >> 2) as u8,
         (out[1] >> 2) as u8,
         (out[2] >> 2) as u8,
         255,
      );
      depth[i] = max_depth;
   }
}

/// Luma of an 8-bit colour, weighted 77/150/29 for red, green and blue: the Rec.601 coefficients (0.299/0.587/0.114) scaled by 256 and rounded — 76.5, 150.3 and 29.2 — so the whole thing is integer.
/// 8 位颜色的亮度，红绿蓝权重为 77/150/29：Rec.601 系数（0.299/0.587/0.114）乘以 256 并取整——即 76.5、150.3、29.2——因此全程为整数。
fn luma(c: ColorRgba8) -> i32 {
   (i32::from(c.r) * 77 + i32::from(c.g) * 150 + i32::from(c.b) * 29) >> 8
}

/// Blend two channels, weighting the first by 141/256 and the second by 115/256.
///
/// Both weights are positive and sum to exactly 256, so the shift cannot overflow and a smoothed edge keeps the majority of its own colour.
/// 混合两个通道：第一个权重 141/256，第二个 115/256。
/// 两个权重都为正且恰好合计 256，因此移位不会溢出，被平滑的边缘也保留自身颜色的大部分。
#[expect(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn blend_u8(a: u8, b: u8) -> u8 {
   ((i32::from(a) * 141 + i32::from(b) * 115) >> 8) as u8
}

/// FXAA: smooth high-contrast edges by blending the center pixel with the average of its two neighbors along the dominant gradient.
/// FXAA：沿主导梯度，将中心像素与两个相邻像素的平均值混合以平滑高对比度边缘。
#[expect(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn apply_fxaa(
   color: &[ColorRgba8],
   out: &mut Vec<ColorRgba8>,
   width: usize,
   height: usize,
) -> bool {
   const EDGE_THRESHOLD: i32 = 20;
   const EDGE_THRESHOLD_MIN: i32 = 8;

   if width <= 2 || height <= 2 || color.len() < width * height {
      return false;
   }
   let total = width * height;
   if out.len() != total {
      out.resize(total, ColorRgba8::BLACK);
   }
   for x in 0..width {
      out[x] = color[x];
      out[(height - 1) * width + x] = color[(height - 1) * width + x];
   }
   for y in 1..height - 1 {
      out[y * width] = color[y * width];
      out[y * width + width - 1] = color[y * width + width - 1];
   }

   for y in 1..height - 1 {
      let row = y * width;
      for x in 1..width - 1 {
         let idx = row + x;
         let center = color[idx];
         let l_m = luma(center);
         let l_n = luma(color[idx - width]);
         let l_s = luma(color[idx + width]);
         let l_w = luma(color[idx - 1]);
         let l_e = luma(color[idx + 1]);

         let l_min = l_m.min(l_n).min(l_s).min(l_w).min(l_e);
         let l_max = l_m.max(l_n).max(l_s).max(l_w).max(l_e);
         let contrast = l_max - l_min;
         let dynamic_threshold = EDGE_THRESHOLD_MIN.max((l_max * 20) >> 8);
         if contrast < dynamic_threshold || contrast < EDGE_THRESHOLD {
            out[idx] = center;
            continue;
         }

         let horizontal = (l_n + l_s - 2 * l_m).abs();
         let vertical = (l_w + l_e - 2 * l_m).abs();
         let (a, b) = if horizontal >= vertical {
            (color[idx - 1], color[idx + 1])
         } else {
            (color[idx - width], color[idx + width])
         };
         out[idx] = ColorRgba8::from_rgba(
            blend_u8(center.r, ((i32::from(a.r) + i32::from(b.r)) >> 1) as u8),
            blend_u8(center.g, ((i32::from(a.g) + i32::from(b.g)) >> 1) as u8),
            blend_u8(center.b, ((i32::from(a.b) + i32::from(b.b)) >> 1) as u8),
            center.a,
         );
      }
   }
   true
}

/// TAA: clamp the previous frame's color against the current 3×3 neighborhood and blend it in unless the depth/color delta rejects it as disocclusion.
/// TAA：将上一帧颜色钳制到当前 3×3 邻域内并混合，除非深度/颜色差将其判为遮挡变化而拒绝。
#[expect(
   clippy::cast_possible_truncation,
   clippy::cast_possible_wrap,
   clippy::cast_sign_loss,
   clippy::too_many_arguments
)]
fn apply_taa(
   source: &[ColorRgba8],
   depth: &[f32],
   out: &mut Vec<ColorRgba8>,
   history: &mut Vec<ColorRgba8>,
   history_depth: &mut Vec<f32>,
   width: usize,
   height: usize,
   valid: &mut bool,
   frame_index: &mut u32,
) -> bool {
   if width == 0 || height == 0 {
      return false;
   }
   let total = width * height;
   if source.len() < total || depth.len() < total {
      return false;
   }
   if out.len() != total {
      out.resize(total, ColorRgba8::BLACK);
   }
   if history.len() != total {
      history.resize(total, ColorRgba8::BLACK);
   }
   if history_depth.len() != total {
      history_depth.resize(total, 0.0);
   }

   if !*valid {
      out.copy_from_slice(&source[..total]);
      history.copy_from_slice(&source[..total]);
      history_depth.copy_from_slice(&depth[..total]);
      *valid = true;
      *frame_index = frame_index.wrapping_add(1);
      return true;
   }

   for y in 0..height {
      let row = y * width;
      for x in 0..width {
         let idx = row + x;
         let cur = source[idx];
         let hist = history[idx];

         let mut min_r = 255_i32;
         let mut min_g = 255_i32;
         let mut min_b = 255_i32;
         let mut max_r = 0_i32;
         let mut max_g = 0_i32;
         let mut max_b = 0_i32;
         for oy in -1_i32..=1 {
            let sy = (y as i32 + oy).clamp(0, height as i32 - 1) as usize;
            for ox in -1_i32..=1 {
               let sx = (x as i32 + ox).clamp(0, width as i32 - 1) as usize;
               let c = source[sy * width + sx];
               min_r = min_r.min(i32::from(c.r));
               min_g = min_g.min(i32::from(c.g));
               min_b = min_b.min(i32::from(c.b));
               max_r = max_r.max(i32::from(c.r));
               max_g = max_g.max(i32::from(c.g));
               max_b = max_b.max(i32::from(c.b));
            }
         }

         let hist_r = (i32::from(hist.r)).clamp(min_r, max_r);
         let hist_g = (i32::from(hist.g)).clamp(min_g, max_g);
         let hist_b = (i32::from(hist.b)).clamp(min_b, max_b);

         let raw_delta = (i32::from(cur.r) - i32::from(hist.r)).abs()
            + (i32::from(cur.g) - i32::from(hist.g)).abs()
            + (i32::from(cur.b) - i32::from(hist.b)).abs();
         let clamped_delta = (i32::from(cur.r) - hist_r).abs()
            + (i32::from(cur.g) - hist_g).abs()
            + (i32::from(cur.b) - hist_b).abs();
         let depth_delta = (depth[idx] - history_depth[idx]).abs();
         let reject = (depth[idx] > 0.0 && depth_delta > 0.0025_f32.max(depth[idx] * 0.05))
            || raw_delta > 120
            || clamped_delta > 80;

         out[idx] = if reject {
            cur
         } else {
            ColorRgba8::from_rgba(
               ((i32::from(cur.r) * 192 + hist_r * 64) >> 8) as u8,
               ((i32::from(cur.g) * 192 + hist_g * 64) >> 8) as u8,
               ((i32::from(cur.b) * 192 + hist_b * 64) >> 8) as u8,
               cur.a,
            )
         };
      }
   }

   history.copy_from_slice(&out[..total]);
   history_depth.copy_from_slice(&depth[..total]);
   *frame_index = frame_index.wrapping_add(1);
   true
}

/// SSAA: box-downsample a `scale`× render buffer into the output resolution.
/// SSAA：将 `scale` 倍渲染缓冲盒式降采样到输出分辨率。
#[expect(clippy::cast_possible_truncation)]
fn apply_ssaa(
   source: &[ColorRgba8],
   out: &mut Vec<ColorRgba8>,
   src_width: u32,
   src_height: u32,
   scale: u32,
   out_width: u32,
   out_height: u32,
) -> bool {
   if scale <= 1
      || out_width == 0
      || out_height == 0
      || u64::from(src_width) != u64::from(out_width) * u64::from(scale)
      || u64::from(src_height) != u64::from(out_height) * u64::from(scale)
   {
      return false;
   }
   let sample_count = scale * scale;
   let out_total = (out_width as usize) * (out_height as usize);
   let src_width = src_width as usize;
   let scale = scale as usize;
   let out_width = out_width as usize;
   if source.len() < src_width * (src_height as usize) {
      return false;
   }
   if out.len() != out_total {
      out.resize(out_total, ColorRgba8::BLACK);
   }
   for y in 0..(out_height as usize) {
      let base_y = y * scale;
      for x in 0..out_width {
         let base_x = x * scale;
         let mut sum = [0u32; 4];
         for sy in 0..scale {
            let row = (base_y + sy) * src_width;
            for sx in 0..scale {
               let c = source[row + base_x + sx];
               sum[0] += u32::from(c.r);
               sum[1] += u32::from(c.g);
               sum[2] += u32::from(c.b);
               sum[3] += u32::from(c.a);
            }
         }
         out[y * out_width + x] = ColorRgba8::from_rgba(
            (sum[0] / sample_count) as u8,
            (sum[1] / sample_count) as u8,
            (sum[2] / sample_count) as u8,
            (sum[3] / sample_count) as u8,
         );
      }
   }
   true
}

/// Shade a texel with fixed-point light: `(texel * light) >> 8` undoes the 8 fractional bits, so a light of 256 leaves the texel untouched.
/// 用定点光照给纹素着色：`(texel * light) >> 8` 抵消那 8 位小数，因此光照为 256 时纹素原样保留。
#[expect(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn shade(texel: ColorRgba8, lr: i32, lg: i32, lb: i32) -> ColorRgba8 {
   ColorRgba8::from_rgba(
      // `lr`/`lg`/`lb` are non-negative (the caller clamps them to `0..=512`) and a texel channel is a `u8`, so the product is never negative and only the upper bound needs clamping.
      // `lr`/`lg`/`lb` 非负（调用方将其钳制在 `0..=512`），纹素通道又是 `u8`，因此乘积不会为负，只需钳制上界。
      (((i32::from(texel.r) * lr) >> 8).min(255)) as u8,
      (((i32::from(texel.g) * lg) >> 8).min(255)) as u8,
      (((i32::from(texel.b) * lb) >> 8).min(255)) as u8,
      255,
   )
}

/// Which of the four MSAA sample positions fall inside the triangle.
/// 四个 MSAA 样本位置中哪些落在三角形内。
#[expect(clippy::cast_precision_loss, clippy::too_many_arguments)]
fn triangle_sample_mask4(
   x: i64,
   y: i64,
   x0: f32,
   y0: f32,
   x1: f32,
   y1: f32,
   x2: f32,
   y2: f32,
   inv_a: f32,
) -> u8 {
   let mut mask = 0_u8;
   for (i, [ox, oy]) in MSAA_SAMPLE_OFFSETS.iter().enumerate() {
      let px = x as f32 + ox;
      let py = y as f32 + oy;
      let w0 = ((x1 - px) * (y2 - py) - (y1 - py) * (x2 - px)) * inv_a;
      let w1 = ((x2 - px) * (y0 - py) - (y2 - py) * (x0 - px)) * inv_a;
      let w2 = 1.0 - w0 - w1;
      if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 {
         mask |= 1 << i;
      }
   }
   mask
}

fn lerp_clip(a: ClipVertex, b: ClipVertex, t: f32) -> ClipVertex {
   ClipVertex {
      view: a.view + (b.view - a.view) * t,
      uv: a.uv + (b.uv - a.uv) * t,
   }
}

/// Clip a view-space triangle against the near plane, writing the resulting 0–4 vertices into `out` and returning how many were written.
/// `out` must hold 4.
/// 对视空间三角形做近平面裁剪，把得到的 0–4 个顶点写入 `out` 并返回写入数量。
/// `out` 必须能容纳 4 个顶点。
fn clip_triangle_to_near(input: &[ClipVertex; 3], near: f32, out: &mut [ClipVertex; 4]) -> usize {
   let mut count = 0;
   for i in 0..3 {
      let a = input[i];
      let b = input[(i + 1) % 3];
      let a_inside = a.view.z > near;
      let b_inside = b.view.z > near;
      match (a_inside, b_inside) {
         (true, true) => {
            out[count] = b;
            count += 1;
         }
         (true, false) => {
            out[count] = lerp_clip(a, b, (near - a.view.z) / (b.view.z - a.view.z));
            count += 1;
         }
         (false, true) => {
            out[count] = lerp_clip(a, b, (near - a.view.z) / (b.view.z - a.view.z));
            count += 1;
            out[count] = b;
            count += 1;
         }
         (false, false) => {}
      }
   }
   count
}

/// Project view-space vertices to screen space.
/// 把视空间顶点投影到屏幕空间。
///
/// `perspective` divides by the view depth; an orthographic projection is affine and skips the divide.
/// `perspective` 会除以视深度；正交投影是仿射的，因此跳过该除法。
fn project_triangle(
   view: &[Vec3; 3],
   half_w: f32,
   half_h: f32,
   scale: f32,
   jitter: Vec2,
   perspective: bool,
) -> [Vec2; 3] {
   let mut screen = [Vec2::ZERO; 3];
   for (i, v) in view.iter().enumerate() {
      let inv_z = if perspective { 1.0 / v.z } else { 1.0 };
      screen[i] = Vec2::new(
         half_w + v.x * inv_z * scale + jitter.x,
         half_h - v.y * inv_z * scale + jitter.y,
      );
   }
   screen
}

fn triangle_outside_viewport(screen: &[Vec2; 3], width: u32, height: u32) -> bool {
   // Viewport size is bounded by MAX_FRAMEBUFFER_PIXELS, so u32→f32 is exact enough for these comparisons.
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
      let camera = Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 70.0);
      (scene, camera, Rgb::new(0.08, 0.10, 0.14))
   }

   fn render_triangle_with(aa: AntiAliasing, width: u32, height: u32) -> Renderer {
      let (scene, camera, clear) = triangle_scene();
      let mut renderer = Renderer::new(RendererConfig {
         width,
         height,
         anti_aliasing: aa,
      })
      .unwrap();
      renderer.render(&scene, &camera, clear).unwrap();
      renderer
   }

   fn render_triangle(width: u32, height: u32) -> Renderer {
      render_triangle_with(AntiAliasing::None, width, height)
   }

   fn pixel(renderer: &Renderer, x: u32, y: u32) -> ColorRgba8 {
      renderer.pixels()[(y * renderer.width() + x) as usize]
   }

   #[test]
   fn triangle_render_shades_with_fixed_point_light() {
      let renderer = render_triangle(64, 64);
      // The triangle's winding leaves it facing -Z, and the default light (0,-1,1) hits its back, so it receives only the 0.15 ambient term.
      // The solid texture (255,89,38) is then lit in fixed point: (255*38, 89*38, 38*38) >> 8 = (37, 13, 5).
      // 三角形的绕序使它面朝 -Z，而默认光源 (0,-1,1) 击中其背面，因此只受 0.15 的环境光影响。
      // 纯色纹理 (255,89,38) 随后经定点光照：(255*38, 89*38, 38*38) >> 8 = (37, 13, 5)。
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
      let camera = Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 70.0);
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
   fn orthographic_projection_is_independent_of_depth() {
      // The same triangle at z=2 and z=8 covers the same screen rectangle, which is the defining property of a parallel projection.
      // 同一三角形在 z=2 与 z=8 下覆盖相同的屏幕矩形，这是平行投影的定义性质。
      let mesh = Mesh::triangle(1.0, 1.0).expect("triangle mesh is valid");
      let mut frames = Vec::new();
      for z in [2.0, 8.0] {
         let mut scene = Scene::new();
         let mesh_id = scene.meshes.insert(mesh.clone());
         scene.spawn(RenderObject {
            mesh: mesh_id,
            texture: None,
            transform: Transform::from_position(Vec3::new(0.0, 0.0, z)),
            material: Material::new(Rgb::WHITE),
            visible: true,
         });
         let camera = Camera::orthographic(Vec3::ZERO, Quat::IDENTITY, 2.0);
         let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
         renderer
            .render(&scene, &camera, Rgb::new(0.0, 0.0, 0.0))
            .unwrap();
         frames.push(renderer);
      }
      assert_eq!(frames[0].pixels(), frames[1].pixels());
   }

   #[test]
   fn orthographic_culls_a_back_facing_triangle() {
      // A triangle wound the other way has a +Z normal, so a camera looking along +Z sees its back and culls it.
      // 反向绕序的三角形法线为 +Z，因此沿 +Z 看的相机看到背面并将其剔除。
      let mut scene = Scene::new();
      scene.light.direction = Vec3::new(0.0, 0.0, -1.0);
      let mesh = scene.meshes.insert(
         Mesh::from_vertices(vec![
            Vec3::new(-0.5, -0.5, 2.0),
            Vec3::new(0.5, -0.5, 2.0),
            Vec3::new(0.0, 0.5, 2.0),
         ])
         .expect("mesh is valid"),
      );
      scene.spawn(RenderObject {
         mesh,
         texture: None,
         transform: Transform::default(),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      let camera = Camera::orthographic(Vec3::ZERO, Quat::IDENTITY, 2.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer
         .render(&scene, &camera, Rgb::new(0.0, 0.0, 0.0))
         .unwrap();
      assert!(
         renderer
            .pixels()
            .iter()
            .all(|&p| p == ColorRgba8::from_rgb(0, 0, 0))
      );
   }

   #[test]
   fn orthographic_texture_interpolation_is_linear_on_a_sloped_triangle() {
      // The triangle spans z from 2 to 6, so perspective-correct and plain barycentric interpolation disagree at the centre pixel.
      // 该三角形跨越 z=2 到 z=6，因此在中心像素处透视正确插值与普通重心插值结果不同。
      let mut scene = Scene::new();
      scene.light.direction = Vec3::new(0.0, 0.0, -1.0);
      let mesh = scene.meshes.insert(
         Mesh::new(
            vec![
               Vec3::new(-1.0, -1.0, 2.0),
               Vec3::new(0.0, 1.0, 6.0),
               Vec3::new(1.0, -1.0, 2.0),
            ],
            vec![
               Vec2::new(0.0, 0.0),
               Vec2::new(0.0, 1.0),
               Vec2::new(1.0, 0.0),
            ],
         )
         .expect("mesh is valid"),
      );
      let texture = scene.textures.insert(
         Texture::new(
            2,
            1,
            vec![
               ColorRgba8::from_rgb(0, 0, 0),
               ColorRgba8::from_rgb(255, 255, 255),
            ],
         )
         .expect("texture is valid"),
      );
      scene.spawn(RenderObject {
         mesh,
         texture: Some(texture),
         transform: Transform::default(),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
      let camera = Camera::orthographic(Vec3::ZERO, Quat::IDENTITY, 2.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer
         .render(&scene, &camera, Rgb::new(0.0, 0.0, 0.0))
         .unwrap();
      // The centre pixel's affine u is 0.25, which samples the black half of the texture.
      // 中心像素的仿射 u 为 0.25，采样到纹理的黑色半边。
      assert_eq!(pixel(&renderer, 32, 32), ColorRgba8::from_rgb(0, 0, 0));
   }

   #[test]
   fn orthographic_depth_still_orders_by_view_depth() {
      // Two triangles at different depths, orthographically projected, with the nearer one winning the depth test.
      // 两个不同深度的三角形经正交投影后，较近的那个赢得深度测试。
      let mut scene = Scene::new();
      scene.light.direction = Vec3::new(0.0, 0.0, -1.0);
      let mesh = scene
         .meshes
         .insert(Mesh::triangle(2.0, 2.0).expect("triangle mesh is valid"));
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
      let camera = Camera::orthographic(Vec3::ZERO, Quat::IDENTITY, 4.0);
      let mut renderer = Renderer::new(RendererConfig::new(64, 64)).unwrap();
      renderer
         .render(&scene, &camera, Rgb::new(0.0, 0.0, 0.0))
         .unwrap();
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
      let camera = Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 70.0);
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
      // Back-facing winding, so the normal points at -Z; the light is aimed along -Z to match, which lets the quad render and take diffuse light.
      // 背向绕序，因此法线指向 -Z；光源沿 -Z 对齐，使该四边形能够渲染并接受漫反射光照。
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
      let camera = Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 70.0);
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
      let camera = Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 70.0);
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
      // Frozen snapshot of a known scene; guards against accidental renderer regressions.
      // Regenerate by deleting this value and reading the failure output.
      // 已知场景的固定快照，防止渲染器意外回归。
      // 如需重新生成，请删除该值并读取失败输出。
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
      let camera = Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 70.0);
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
      let camera = Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 70.0);
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
   fn ssaa_rejects_oversized_render_buffer() {
      // Output fits the budget, but 4× supersampling pushes past it.
      // 输出尺寸在预算内，但 4× 超采样会超出预算。
      let config = RendererConfig {
         width: 4096,
         height: 4096,
         anti_aliasing: AntiAliasing::Ssaa(4),
      };
      assert!(Renderer::new(config).is_err());
      // 1× Ssaa behaves like None and stays valid.
      // 1× Ssaa 与 None 行为一致且仍有效。
      let config = RendererConfig {
         width: 4096,
         height: 4096,
         anti_aliasing: AntiAliasing::Ssaa(1),
      };
      assert!(Renderer::new(config).is_ok());
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
      bad.projection = Projection::Perspective { fov_y_degrees: 0.0 };
      assert_eq!(
         renderer.render(&scene, &bad, clear),
         Err(Error::InvalidSize)
      );
      bad.projection = Projection::Perspective {
         fov_y_degrees: f32::NAN,
      };
      assert_eq!(
         renderer.render(&scene, &bad, clear),
         Err(Error::InvalidSize)
      );
      bad.projection = Projection::Orthographic { height: 0.0 };
      assert_eq!(
         renderer.render(&scene, &bad, clear),
         Err(Error::InvalidSize)
      );
      bad.projection = Projection::Orthographic {
         height: f32::NEG_INFINITY,
      };
      assert_eq!(
         renderer.render(&scene, &bad, clear),
         Err(Error::InvalidSize)
      );
   }

   #[test]
   fn adversarial_inputs_never_panic() {
      let cameras = [
         Camera::perspective(Vec3::new(f32::NAN, 0.0, 0.0), Quat::IDENTITY, 70.0),
         Camera::perspective(Vec3::ZERO, Quat::IDENTITY, f32::NAN),
         Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 0.0),
         Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 360.0),
         Camera::perspective(Vec3::new(1e30, 1e30, -1e30), Quat::IDENTITY, 70.0),
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
         let camera = Camera::perspective(
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

   #[test]
   fn fuzz_style_random_scenes_never_panic_msaa() {
      let mut renderer = Renderer::new(RendererConfig {
         width: 32,
         height: 32,
         anti_aliasing: AntiAliasing::Msaa,
      })
      .unwrap();
      for _ in 0..100 {
         let mut scene = Scene::new();
         let mesh = scene.meshes.insert(Mesh::triangle(1.0, 1.0).unwrap());
         scene.spawn(RenderObject {
            mesh,
            texture: None,
            transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
            material: Material::default(),
            visible: true,
         });
         let camera = Camera::perspective(Vec3::ZERO, Quat::IDENTITY, 70.0);
         let _ = renderer.render(&scene, &camera, Rgb::new(0.0, 0.0, 0.0));
      }
   }

   #[test]
   fn msaa_averages_edge_samples() {
      // A bright triangle on a dark clear: the edge pixels blend toward the background instead of snapping, so MSAA differs from the raw raster.
      // 深色清屏上的亮色三角形：边缘像素向背景色过渡而非硬切，因此 MSAA 与原始光栅不同。
      let plain = render_triangle(64, 64);
      let msaa = render_triangle_with(AntiAliasing::Msaa, 64, 64);
      assert_ne!(fnv1a_pixels(plain.pixels()), fnv1a_pixels(msaa.pixels()));
      assert_eq!(msaa.width(), 64);
      assert_eq!(msaa.height(), 64);
   }

   #[test]
   fn msaa_resolve_averages_four_samples() {
      let mut color = vec![ColorRgba8::BLACK; 1];
      let mut depth = vec![0.0_f32; 1];
      let msaa_color = vec![
         ColorRgba8::from_rgb(100, 0, 0),
         ColorRgba8::from_rgb(200, 0, 0),
         ColorRgba8::from_rgb(0, 0, 0),
         ColorRgba8::from_rgb(0, 0, 0),
      ];
      let msaa_depth = vec![0.1, 0.5, 0.0, 0.0];
      resolve_msaa(&mut color, &mut depth, &msaa_color, &msaa_depth);
      // (100 + 200 + 0 + 0) >> 2 = 75.
      // (100 + 200 + 0 + 0) >> 2 = 75。
      assert_eq!(color[0], ColorRgba8::from_rgb(75, 0, 0));
      // `resolve_msaa` forwards the deepest sample verbatim; compare with a tolerance because `clippy::float_cmp` is denied workspace-wide.
      // `resolve_msaa` 原样转发最深的样本值；因工作区全局拒绝 `clippy::float_cmp`，这里用容差比较。
      assert!((depth[0] - 0.5).abs() < f32::EPSILON);
   }

   #[test]
   fn fxaa_blends_high_contrast_edges() {
      // A white pixel flanked by black neighbors has high contrast and gets blended.
      // 被黑色邻居夹在中间的白像素对比度很高，会被混合。
      let mut color = vec![ColorRgba8::BLACK; 9];
      color[4] = ColorRgba8::WHITE;
      let mut out = vec![ColorRgba8::BLACK; 9];
      assert!(apply_fxaa(&color, &mut out, 3, 3));
      // blend_u8(255, 0) = (255·141 + 0·115) >> 8 = 140.
      // blend_u8(255, 0) = (255·141 + 0·115) >> 8 = 140。
      assert_eq!(out[4], ColorRgba8::from_rgb(140, 140, 140));
   }

   #[test]
   fn fxaa_leaves_low_contrast_untouched() {
      let color = vec![ColorRgba8::from_rgb(128, 128, 128); 9];
      let mut out = vec![ColorRgba8::BLACK; 9];
      assert!(apply_fxaa(&color, &mut out, 3, 3));
      assert_eq!(out[4], ColorRgba8::from_rgb(128, 128, 128));
   }

   #[test]
   fn taa_first_frame_seeds_history() {
      let source = vec![ColorRgba8::from_rgb(10, 20, 30); 4];
      let depth = vec![0.5; 4];
      let mut out = vec![ColorRgba8::BLACK; 4];
      let mut history = vec![ColorRgba8::BLACK; 4];
      let mut history_depth = vec![0.0; 4];
      let mut valid = false;
      let mut frame_index = 0;
      assert!(apply_taa(
         &source,
         &depth,
         &mut out,
         &mut history,
         &mut history_depth,
         2,
         2,
         &mut valid,
         &mut frame_index,
      ));
      assert!(valid);
      assert_eq!(frame_index, 1);
      assert_eq!(out, source);
      assert_eq!(history, source);
   }

   #[test]
   fn taa_blends_stable_pixels() {
      // A 2×2 mix keeps the neighborhood range [100, 200] wide enough that the history (150) survives the clamp and the color deltas stay under the rejection thresholds.
      // 2×2 的混合使邻域范围 [100, 200] 足够宽，历史值 150 能通过钳制，颜色差也保持在拒绝阈值之下。
      let source = vec![
         ColorRgba8::from_rgb(200, 0, 0),
         ColorRgba8::from_rgb(100, 0, 0),
         ColorRgba8::from_rgb(200, 0, 0),
         ColorRgba8::from_rgb(100, 0, 0),
      ];
      let depth = vec![0.5; 4];
      let mut out = vec![ColorRgba8::BLACK; 4];
      let mut history = vec![ColorRgba8::from_rgb(150, 0, 0); 4];
      let mut history_depth = vec![0.5; 4];
      let mut valid = true;
      let mut frame_index = 0;
      assert!(apply_taa(
         &source,
         &depth,
         &mut out,
         &mut history,
         &mut history_depth,
         2,
         2,
         &mut valid,
         &mut frame_index,
      ));
      // Blend ratio 192:64 → (200·192 + 150·64) >> 8 = 187.
      // 混合比 192:64 → (200·192 + 150·64) >> 8 = 187。
      assert_eq!(out[0], ColorRgba8::from_rgb(187, 0, 0));
      assert_eq!(frame_index, 1);
   }

   #[test]
   fn taa_rejects_moving_depth() {
      let source = vec![ColorRgba8::from_rgb(200, 0, 0); 4];
      let depth = vec![0.5; 4];
      let mut out = vec![ColorRgba8::BLACK; 4];
      // History color is far away; a large depth delta forces a full reject.
      // 历史颜色相差很远；大深度差会强制完全拒绝。
      let mut history = vec![ColorRgba8::from_rgb(10, 0, 0); 4];
      let mut history_depth = vec![0.1; 4];
      let mut valid = true;
      let mut frame_index = 0;
      assert!(apply_taa(
         &source,
         &depth,
         &mut out,
         &mut history,
         &mut history_depth,
         2,
         2,
         &mut valid,
         &mut frame_index,
      ));
      // Depth delta 0.4 > max(0.0025, 0.5·0.05=0.025) and > 0.0, so history is rejected.
      // 深度差 0.4 > max(0.0025, 0.5·0.05=0.025) 且 > 0.0，因此历史被拒绝。
      assert_eq!(out[0], ColorRgba8::from_rgb(200, 0, 0));
   }

   #[test]
   fn halton_sequence_values() {
      // Halton base-2: 1/2, 1/4, 3/4, 1/8, 5/8, ...
      // Halton 基 2：1/2, 1/4, 3/4, 1/8, 5/8, ...
      assert!((halton(1, 2) - 0.5).abs() < 1e-6);
      assert!((halton(2, 2) - 0.25).abs() < 1e-6);
      assert!((halton(3, 2) - 0.75).abs() < 1e-6);
      assert!((halton(1, 3) - 1.0 / 3.0).abs() < 1e-6);
   }

   #[test]
   fn ssaa_downsamples_integer_scale() {
      // 2× scale: every 2×2 block averages to one output pixel.
      // 2× 倍率：每个 2×2 块平均成一个输出像素。
      let source = vec![
         ColorRgba8::from_rgb(0, 0, 0),
         ColorRgba8::from_rgb(255, 0, 0),
         ColorRgba8::from_rgb(0, 0, 0),
         ColorRgba8::from_rgb(255, 0, 0),
      ];
      let mut out = vec![ColorRgba8::BLACK; 1];
      assert!(apply_ssaa(&source, &mut out, 2, 2, 2, 1, 1));
      // (0 + 255 + 0 + 255) / 4 = 127.
      // (0 + 255 + 0 + 255) / 4 = 127。
      assert_eq!(out[0], ColorRgba8::from_rgb(127, 0, 0));
   }

   #[test]
   fn ssaa_presented_size_is_output_size() {
      // Render at 2× internally, present at the requested 64×64.
      // 内部按 2× 渲染，按请求的 64×64 提交。
      let renderer = render_triangle_with(AntiAliasing::Ssaa(2), 64, 64);
      assert_eq!(renderer.width(), 64);
      assert_eq!(renderer.height(), 64);
      assert_eq!(renderer.pixels().len(), 64 * 64);
   }

   #[test]
   fn ssaa_scale_is_clamped() {
      // Out-of-range scales clamp to 1 and 4, still constructing valid renderers.
      // 越界倍率被钳制到 1 和 4，仍能构造有效渲染器。
      assert!(
         Renderer::new(RendererConfig {
            width: 64,
            height: 64,
            anti_aliasing: AntiAliasing::Ssaa(0),
         })
         .is_ok()
      );
      assert!(
         Renderer::new(RendererConfig {
            width: 64,
            height: 64,
            anti_aliasing: AntiAliasing::Ssaa(99),
         })
         .is_ok()
      );
   }

   #[test]
   fn taa_jitter_steps_through_sequence() {
      fn jitter_equals(jitter: Vec2, x: f32, y: f32) -> bool {
         (jitter.x - x).abs() < 1e-6 && (jitter.y - y).abs() < 1e-6
      }

      let (scene, camera, clear) = triangle_scene();
      let mut renderer = Renderer::new(RendererConfig {
         width: 32,
         height: 32,
         anti_aliasing: AntiAliasing::Taa,
      })
      .unwrap();
      // Frame 0: Halton sample 1 → x = (1/2 − 1/2)/2 = 0, y = (1/3 − 1/2)/2 = −1/12.
      // 第 0 帧：Halton 样本 1 → x = (1/2 − 1/2)/2 = 0，y = (1/3 − 1/2)/2 = −1/12。
      assert!(jitter_equals(
         renderer.current_taa_jitter(),
         0.0,
         -1.0 / 12.0
      ));
      renderer.render(&scene, &camera, clear).unwrap();
      assert_eq!(renderer.width(), 32);
      assert_eq!(renderer.height(), 32);
      assert_eq!(renderer.pixels().len(), 32 * 32);
      // Frame 1: Halton sample 2 → x = (1/4 − 1/2)/2 = −1/8, y = (2/3 − 1/2)/2 = 1/12.
      // 第 1 帧：Halton 样本 2 → x = (1/4 − 1/2)/2 = −1/8，y = (2/3 − 1/2)/2 = 1/12。
      assert!(jitter_equals(
         renderer.current_taa_jitter(),
         -1.0 / 8.0,
         1.0 / 12.0
      ));
      renderer.render(&scene, &camera, clear).unwrap();
      // After 8 frames the Halton samples wrap around.
      // 8 帧后 Halton 样本循环。
      for _ in 0..6 {
         renderer.render(&scene, &camera, clear).unwrap();
      }
      assert!(jitter_equals(
         renderer.current_taa_jitter(),
         0.0,
         -1.0 / 12.0
      ));
   }
}

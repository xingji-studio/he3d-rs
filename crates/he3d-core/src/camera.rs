use he3d_math::{Quat, Vec3};

/// Default distance to the near clipping plane.
///
/// It is small but not zero: a near plane at the eye would push the far end of the inverse-depth range to infinity and destroy depth precision up close.
/// 近裁剪平面的默认距离。它很小但不为零：若近平面落在眼睛处，逆深度范围远端的精度会被推到无穷，近处深度精度随之毁掉。
pub const DEFAULT_NEAR: f32 = 0.1;

/// How a camera projects its view volume onto the viewport.
/// 相机如何将视体积投影到视口。
///
/// The two variants carry different quantities: a perspective projection is an angle, an orthographic one is a world-space length.
/// Neither needs a second parameter, because the viewport's aspect ratio supplies the other axis.
/// 两个变体承载不同的量：透视投影是一个角度，正交投影是一个世界长度。
/// 两者都不需要第二个参数，因为视口的宽高比提供了另一根轴。
///
/// The enum is deliberately exhaustive: the renderer matches on it and has to produce an answer for every variant, so a new projection should be a compile error there rather than a silent fallback.
/// 该枚举刻意做成穷尽的：渲染器要对它进行匹配，且必须为每个变体给出结果，因此新增投影应当在那里成为编译错误，而不是静默的回退。
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Projection {
   /// A pinhole projection with the given vertical field of view in degrees.
   /// 给定垂直视野（以度为单位）的针孔投影。
   Perspective {
      /// Vertical field of view in degrees.
      /// 垂直视野，以度为单位。
      fov_y_degrees: f32,
   },
   /// A parallel projection whose viewport spans `height` world units vertically.
   /// 视口在垂直方向上跨越 `height` 个世界单位的平行投影。
   Orthographic {
      /// Vertical extent of the view volume in world units.
      /// 视体积的垂直跨度，以世界单位计。
      height: f32,
   },
}

/// A camera: where it is, which way it looks, and how it projects.
/// 相机：它在哪里、朝哪看、以及如何投影。
///
/// It is a plain value type and is never stored inside a renderer.
/// 它是一个普通的数值类型，从不被存放在渲染器内部。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Camera {
   /// Camera position in world space.
   /// 相机在世界空间中的位置。
   pub position: Vec3,
   /// Camera orientation in world space.
   /// 相机在世界空间中的朝向。
   pub orientation: Quat,
   /// How the view volume is projected onto the viewport.
   /// 视体积如何投影到视口。
   pub projection: Projection,
   /// Near clipping plane distance.
   /// 近裁剪平面距离。
   pub near: f32,
}

impl Camera {
   /// Construct a perspective camera with the default near-plane distance.
   /// 用默认的近平面距离构造一个透视相机。
   #[must_use]
   pub const fn perspective(position: Vec3, orientation: Quat, fov_y_degrees: f32) -> Self {
      Self {
         position,
         orientation,
         projection: Projection::Perspective { fov_y_degrees },
         near: DEFAULT_NEAR,
      }
   }

   /// Construct an orthographic camera with the default near-plane distance.
   /// 用默认的近平面距离构造一个正交相机。
   #[must_use]
   pub const fn orthographic(position: Vec3, orientation: Quat, height: f32) -> Self {
      Self {
         position,
         orientation,
         projection: Projection::Orthographic { height },
         near: DEFAULT_NEAR,
      }
   }
}

impl Default for Camera {
   fn default() -> Self {
      Self {
         position: Vec3::new(0.0, 0.0, 5.0),
         orientation: Quat::IDENTITY,
         projection: Projection::Perspective {
            fov_y_degrees: 90.0,
         },
         near: DEFAULT_NEAR,
      }
   }
}

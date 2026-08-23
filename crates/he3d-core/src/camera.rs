use he3d_math::{Quat, Vec3};

/// Default distance to the near clipping plane, matching the reference renderer's `HE3D_NEAR_Z`.
/// 近裁剪平面的默认距离，与参考渲染器的 `HE3D_NEAR_Z` 一致。
pub const DEFAULT_NEAR: f32 = 0.1;

/// A pinhole camera. A plain value type; it is never stored inside a renderer.
/// 针孔相机。一个普通的数值类型；它从不被存放在渲染器内部。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Camera {
   pub position: Vec3,
   pub orientation: Quat,
   /// Vertical field of view in degrees.
   /// 垂直视野，以度为单位。
   pub fov_y_degrees: f32,
   /// Near clipping plane distance.
   /// 近裁剪平面距离。
   pub near: f32,
}

impl Camera {
   pub const fn new(position: Vec3, orientation: Quat, fov_y_degrees: f32) -> Self {
      Self {
         position,
         orientation,
         fov_y_degrees,
         near: DEFAULT_NEAR,
      }
   }
}

impl Default for Camera {
   fn default() -> Self {
      Self {
         position: Vec3::new(0.0, 0.0, 5.0),
         orientation: Quat::IDENTITY,
         fov_y_degrees: 90.0,
         near: DEFAULT_NEAR,
      }
   }
}

use he3d_math::{Quat, Vec3};

/// Default distance to the near clipping plane.
///
/// It is small but not zero: a near plane at the eye would push the far end of the
/// inverse-depth range to infinity and destroy depth precision up close.
/// 近裁剪平面的默认距离。它很小但不为零：若近平面落在眼睛处，逆深度范围远端的精度会被
/// 推到无穷，近处深度精度随之毁掉。
pub const DEFAULT_NEAR: f32 = 0.1;

/// A pinhole camera. A plain value type; it is never stored inside a renderer.
/// 针孔相机。一个普通的数值类型；它从不被存放在渲染器内部。
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Camera {
   /// Camera position in world space.
   /// 相机在世界空间中的位置。
   pub position: Vec3,
   /// Camera orientation in world space.
   /// 相机在世界空间中的朝向。
   pub orientation: Quat,
   /// Vertical field of view in degrees.
   /// 垂直视野，以度为单位。
   pub fov_y_degrees: f32,
   /// Near clipping plane distance.
   /// 近裁剪平面距离。
   pub near: f32,
}

impl Camera {
   /// Construct a camera with the default near-plane distance.
   /// 用默认的近平面距离构造一个相机。
   #[must_use]
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

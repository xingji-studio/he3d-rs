//! HE3D physics — rigid-body dynamics for interactive applications.
//! HE3D 物理 —— 面向交互式应用的刚体动力学。
//!
//! The crate owns its Bodies through a [`BodyId`] and knows nothing about
//! rendering: it never sees an `ObjectId`, a `Scene`, or a `Transform`. A Body's
//! placement is a position and an orientation, and the application owns both the
//! link between a Body and whatever draws it and the per-frame copy of that
//! placement. See `docs/adr/0001`.
//! 本 crate 通过 [`BodyId`] 拥有自己的 Body，且对渲染一无所知：它永远看不到
//! `ObjectId`、`Scene` 或 `Transform`。Body 的摆放是位置与朝向；Body 与"画它的东西"
//! 之间的联系、以及每帧复制该摆放的动作，都由应用拥有。见 `docs/adr/0001`。
//!
//! Bodies come in three motion modes; the one that needs care is
//! [`MotionType::Kinematic`], driven by a velocity the application sets and
//! changeable by no Contact. See [`MotionType`].
//! Body 有三种运动模式；其中需要留意的是 [`MotionType::Kinematic`]：它由应用设定的速度
//! 驱动，且没有任何 Contact 能改变它。见 [`MotionType`]。
//!
//! # Example
//! # 示例
//! ```
//! use he3d_math::Vec3;
//! use he3d_physics::{BodyDesc, ContactPhase, Shape, World};
//!
//! let mut world = World::new();
//!
//! // Static scenery is just a Body that never moves.
//! // 静态景物就是一个永不移动的 Body。
//! world.add_body(
//!    BodyDesc::stationary(Shape::cuboid(Vec3::new(10.0, 0.5, 10.0))).at(Vec3::ZERO),
//! );
//! let ball = world.add_body(
//!    BodyDesc::dynamic(Shape::sphere(0.5), 1.0)
//!       .at(Vec3::new(0.0, 8.0, 0.0))
//!       .with_friction(0.45)
//!       .with_restitution(0.3),
//! );
//!
//! // A fixed step; the caller owns the accumulator and the per-frame step cap.
//! // 固定步长；累加器与每帧步数上限由调用方拥有。
//! //
//! // Each step publishes the Contacts it found. Asking that list is the *only* thing
//! // needed to know whether the ball has touched down: no geometry, no tolerances.
//! // 每一步都会发布它找到的 Contact。问这个列表就是判断球是否触地所需的**全部**手段：
//! // 无需几何，无需容差。
//! let mut touched_down = false;
//! for _ in 0..240 {
//!    world.step(1.0 / 120.0);
//!    touched_down |= world
//!       .contacts()
//!       .iter()
//!       .any(|contact| contact.involves(ball) && contact.phase != ContactPhase::End);
//! }
//! assert!(touched_down, "the ball must reach the floor on the way down");
//!
//! // Let it finish bouncing and settle.
//! // 让它弹完并稳定下来。
//! for _ in 0..600 {
//!    world.step(1.0 / 120.0);
//! }
//! let resting = world.position(ball).expect("the ball exists");
//! assert!((resting.y - 1.0).abs() < 0.05, "resting at {resting:?} instead");
//!
//! // One thing to know before relying on `contacts()`: a Body that has gone to sleep
//! // stops producing constraints, so its Contacts disappear with it. Ask while the
//! // Body is awake, or wake it first with [`World::wake`].
//! // 依赖 `contacts()` 之前要知道一件事：睡着的 Body 不再产生约束，因此它的 Contact 会随
//! // 之消失。请在 Body 清醒时询问，或先用 [`World::wake`] 唤醒它。
//! assert!(!world.is_empty());
//! ```

#![no_std]
#![forbid(unsafe_code)]

extern crate alloc;

#[cfg(feature = "std")]
extern crate std;

mod aabb;
mod body;
mod broadphase;
mod contact;
mod narrowphase;
mod shape;
mod solver;
mod static_geometry;
mod storage;
mod world;

pub use aabb::Aabb;
pub use body::{BodyDesc, BodyId, DEFAULT_FRICTION, MotionType};
pub use contact::{Contact, ContactPartner, ContactPhase};
pub use shape::{MassProperties, Shape};
pub use static_geometry::StaticMeshId;
pub use world::{
   DEFAULT_GRAVITY, SLEEP_ANGULAR_THRESHOLD, SLEEP_LINEAR_THRESHOLD, SLEEP_TIME, World,
};

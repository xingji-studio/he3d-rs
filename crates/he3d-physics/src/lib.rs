#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

//! HE3D physics.
//! HE3D 物理模块。
//!
//! A thin [`PhysicsWorld`] adapter over the standalone [`gp3d`] engine, mapping
//! render objects by stable [`ObjectId`] onto gp3d's generational body handles.
//! 一个建立在独立 [`gp3d`] 引擎之上的薄 [`PhysicsWorld`] 适配层，通过稳定的
//! [`ObjectId`] 将渲染对象映射到 gp3d 的代际刚体句柄上。
//! Bodies are owned by gp3d; after each [`PhysicsWorld::step`] the caller synchronizes
//! transforms back into the [`Scene`].
//! 刚体由 gp3d 持有；每次 [`PhysicsWorld::step`] 之后，调用方将变换同步回 [`Scene`]。

extern crate alloc;

mod world;

pub use gp3d::StepResult;
pub use world::PhysicsWorld;

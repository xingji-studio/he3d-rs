#![cfg_attr(not(feature = "std"), no_std)]

//! HE3D renderer.
//! HE3D 渲染器。
//!
//! Software rasterizer and post-processing. A [`Renderer`] owns its framebuffer and depth buffer; it receives `&Scene` and `&Camera` per frame and exposes the finished pixels as `&[ColorRgba8]`. It never touches a window or any backend type.
//! 软件光栅化器与后处理。每个 [`Renderer`] 拥有自己的帧缓冲与深度缓冲；它每帧接收 `&Scene` 与 `&Camera`，并通过 `&[ColorRgba8]` 暴露渲染完成的像素。它从不接触窗口或任何后端类型。

extern crate alloc;

mod config;
mod renderer;

pub use config::{AntiAliasing, RendererConfig};
pub use he3d_core::{Camera, ColorRgba8, Error, Rgb, Scene};
pub use renderer::Renderer;

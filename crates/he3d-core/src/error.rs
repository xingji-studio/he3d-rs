use core::fmt;

/// Errors produced by core construction and validation paths.
/// 核心构造与校验路径产生的错误。
///
/// Construction uses `checked_*` arithmetic and explicit limits, so oversized or malformed inputs return an error instead of overflowing or panicking.
/// 构造使用 `checked_*` 算术和显式限制，因此过大或畸形的输入会返回错误，而不是溢出或 panic。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[non_exhaustive]
pub enum Error {
   /// A dimension or size is zero, negative, or exceeds a configured limit.
   /// 维度或大小为 0、为负，或超过配置的限制。
   InvalidSize,
   /// An arithmetic operation (e.g. buffer sizing) would overflow.
   /// 算术运算（如缓冲区大小计算）会溢出。
   Overflow,
   /// A buffer length does not match its declared dimensions.
   /// 缓冲区长度与其声明的维度不匹配。
   BufferMismatch,
   /// Input data is structurally invalid (empty mesh, non-triangle count, etc.).
   /// 输入数据在结构上无效（空网格、非三角形的顶点数等）。
   InvalidData,
   /// The input is a recognized format that this build does not support (e.g. an image format compiled out by a feature).
   /// 输入是一种已识别、但当前构建不支持的格式（例如被某个特性编译掉的图像格式）。
   UnsupportedFormat,
}

impl fmt::Display for Error {
   fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
      let message = match self {
         Error::InvalidSize => "invalid size or dimension",
         Error::Overflow => "arithmetic overflow",
         Error::BufferMismatch => "buffer length does not match declared dimensions",
         Error::InvalidData => "malformed or invalid input data",
         Error::UnsupportedFormat => "recognized but unsupported input format",
      };
      f.write_str(message)
   }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

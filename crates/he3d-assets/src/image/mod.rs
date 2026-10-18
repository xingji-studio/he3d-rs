use alloc::vec::Vec;
use he3d_core::{ColorRgba8, Error, Texture};

#[cfg(feature = "bmp")]
mod bmp;
#[cfg(feature = "jpeg")]
mod jpeg;
#[cfg(feature = "png")]
mod png;

/// Maximum decoded image width or height.
///
/// A header can claim any size it likes, so decoding is refused beyond this rather than letting a small file ask for an enormous allocation.
/// 解码后图像的最大宽或高。
/// 文件头可以声称任意尺寸，因此超过此值即拒绝解码，而不是让一个小文件索要一笔巨大的分配。
#[cfg_attr(
   not(any(feature = "bmp", feature = "png", feature = "jpeg")),
   allow(dead_code)
)]
const MAX_IMAGE_DIMENSION: u32 = 8192;

/// Decode a BMP, PNG, or JPEG image from memory into RGBA8.
/// 从内存解码 BMP、PNG 或 JPEG 图像为 RGBA8。
///
/// Format detection is by magic bytes.
/// Decoders for formats compiled out (via the `bmp`/`png`/`jpeg` features) report [`Error::UnsupportedFormat`].
/// 通过魔数字节进行格式检测。
/// 未编译进来的格式解码器（通过 `bmp`/`png`/`jpeg` 特性）会返回 [`Error::UnsupportedFormat`]。
///
/// # Errors
/// Returns [`Error::UnsupportedFormat`] when the data matches no enabled decoder or its decoder was compiled out; format-specific parse errors are passed through.
/// 数据无法被任何已启用的解码器识别，或对应的解码器被编译掉时返回 [`Error::UnsupportedFormat`]；具体格式的解析错误由各解码器返回。
///
/// # Examples
/// 示例
/// ```
/// use he3d_assets::decode_image;
///
/// // Unknown magic bytes are rejected rather than mis-decoded.
/// // 无法识别的魔数字节会被拒绝，而不是被误解码。
/// assert!(decode_image(b"not-an-image").is_err());
/// ```
#[cfg_attr(
   not(any(feature = "bmp", feature = "png", feature = "jpeg")),
   allow(dead_code)
)]
pub fn decode_image(bytes: &[u8]) -> Result<Texture, Error> {
   #[cfg(feature = "bmp")]
   if is_bmp(bytes) {
      return bmp::decode(bytes);
   }
   #[cfg(feature = "png")]
   if is_png(bytes) {
      return png::decode(bytes);
   }
   #[cfg(feature = "jpeg")]
   if is_jpeg(bytes) {
      return jpeg::decode(bytes);
   }
   #[cfg(not(any(feature = "bmp", feature = "png", feature = "jpeg")))]
   {
      let _ = bytes;
   }
   Err(Error::UnsupportedFormat)
}

/// Expand channel-interleaved pixels to RGBA8.
/// 将通道交错排列的像素扩展为 RGBA8。
///
/// `sample(i)` returns the raw byte for absolute pixel-array index `i`.
/// Channels: 1 = gray, 2 = gray+alpha, 3 = RGB, 4 = RGBA.
/// `sample(i)` 返回绝对像素数组索引 `i` 处的原始字节。
/// 通道数：1 = 灰度，2 = 灰度+alpha，3 = RGB，4 = RGBA。
#[cfg_attr(not(any(feature = "png", feature = "jpeg")), allow(dead_code))]
// The channel-decode closure uses single-letter channel variables; idiomatic here.
// 通道解码闭包使用单字母通道变量；在此处是惯用法。
#[expect(clippy::many_single_char_names)]
pub(crate) fn expand<F>(
   out: &mut Vec<ColorRgba8>,
   channels: usize,
   total: usize,
   sample: F,
) -> Result<(), Error>
where
   F: Fn(usize) -> u8,
{
   if !(1..=4).contains(&channels) {
      return Err(Error::UnsupportedFormat);
   }
   for i in 0..total {
      let base = i * channels;
      let s = |k: usize| sample(base + k);
      let (r, g, b, a) = match channels {
         1 => (s(0), s(0), s(0), 255),
         2 => (s(0), s(0), s(0), s(1)),
         3 => (s(0), s(1), s(2), 255),
         _ => (s(0), s(1), s(2), s(3)),
      };
      out.push(ColorRgba8::from_rgba(r, g, b, a));
   }
   Ok(())
}

#[cfg(feature = "bmp")]
fn is_bmp(bytes: &[u8]) -> bool {
   bytes.len() >= 2 && bytes[0] == b'B' && bytes[1] == b'M'
}

#[cfg(feature = "png")]
fn is_png(bytes: &[u8]) -> bool {
   const MAGIC: [u8; 8] = [0x89, b'P', b'N', b'G', 0x0d, 0x0a, 0x1a, 0x0a];
   bytes.len() >= 8 && bytes[..8] == MAGIC
}

#[cfg(feature = "jpeg")]
fn is_jpeg(bytes: &[u8]) -> bool {
   bytes.len() >= 3 && bytes[0] == 0xff && bytes[1] == 0xd8 && bytes[2] == 0xff
}

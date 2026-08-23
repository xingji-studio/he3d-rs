use alloc::vec::Vec;
use he3d_core::{Error, Texture};
use zune_jpeg::JpegDecoder;
use zune_jpeg::zune_core::colorspace::ColorSpace;

use super::{MAX_IMAGE_DIMENSION, expand};

/// Decode a JPEG image from memory into RGBA8.
/// 从内存解码 JPEG 图像为 RGBA8。
///
/// The decoder is asked for RGB output (its default); RGBA output is also accepted.
/// 解码器要求 RGB 输出（其默认值）；RGBA 输出同样被接受。
pub fn decode(bytes: &[u8]) -> Result<Texture, Error> {
   let mut decoder = JpegDecoder::new(bytes);
   decoder.decode_headers().map_err(|_| Error::InvalidData)?;
   let info = decoder.info().ok_or(Error::InvalidData)?;

   let width = u32::from(info.width);
   let height = u32::from(info.height);
   if width == 0 || height == 0 || width > MAX_IMAGE_DIMENSION || height > MAX_IMAGE_DIMENSION {
      return Err(Error::InvalidSize);
   }
   let channels = match decoder.get_output_colorspace() {
      Some(ColorSpace::RGB) => 3,
      Some(ColorSpace::RGBA) => 4,
      _ => return Err(Error::UnsupportedFormat),
   };
   let total = (width as usize)
      .checked_mul(height as usize)
      .ok_or(Error::Overflow)?;

   let raw = decoder.decode().map_err(|_| Error::InvalidData)?;
   let mut pixels = Vec::with_capacity(total);
   expand(&mut pixels, channels, total, |i| {
      raw.get(i).copied().unwrap_or(0)
   })?;
   Texture::new(width, height, pixels)
}

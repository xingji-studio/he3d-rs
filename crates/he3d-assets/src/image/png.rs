use alloc::vec::Vec;
use he3d_core::{Error, Texture};
use zune_png::PngDecoder;
use zune_png::zune_core::bit_depth::BitDepth;
use zune_png::zune_core::bytestream::ZCursor;
use zune_png::zune_core::colorspace::ColorSpace;
use zune_png::zune_core::result::DecodingResult;

use super::{MAX_IMAGE_DIMENSION, expand};

/// Decode a PNG image from memory into RGBA8.
/// 从内存解码 PNG 图像为 RGBA8。
///
/// Supports 8-bit (after expansion from lower bit depths) and 16-bit sources in gray, gray+alpha, RGB, and RGBA color types.
/// 支持 8 位（从较低位深扩展后）与 16 位源，颜色类型为灰度、灰度+alpha、RGB 和 RGBA。
/// Sub-8-bit inputs are expanded to 8 bits by the decoder.
/// 低于 8 位的输入由解码器扩展为 8 位。
pub fn decode(bytes: &[u8]) -> Result<Texture, Error> {
   let mut decoder = PngDecoder::new(ZCursor::new(bytes));
   let result = decoder.decode().map_err(|_| Error::InvalidData)?;
   let info = decoder.info().ok_or(Error::InvalidData)?;

   // `info.width`/`info.height` are `usize`; validate before narrowing to `u32`
   // so a pathological dimension cannot silently truncate past the limit check.
   // `info.width`/`info.height` 是 `usize`；先校验再收窄为 `u32`，防止病态尺寸
   // 在通过上限检查前被静默截断。
   let width_usize = info.width;
   let height_usize = info.height;
   let max = MAX_IMAGE_DIMENSION as usize;
   if width_usize == 0 || height_usize == 0 || width_usize > max || height_usize > max {
      return Err(Error::InvalidSize);
   }
   // Bounded to `MAX_IMAGE_DIMENSION` (8192) by the check above, so the
   // `usize`→`u32` narrowing cannot truncate.
   // 上面的检查将尺寸限制在 `MAX_IMAGE_DIMENSION`（8192）内，因此 `usize`→`u32`
   // 的收窄不会截断。
   #[expect(clippy::cast_possible_truncation)]
   let width = width_usize as u32;
   #[expect(clippy::cast_possible_truncation)]
   let height = height_usize as u32;
   let channels = channels_for(decoder.colorspace().ok_or(Error::InvalidData)?)
      .ok_or(Error::UnsupportedFormat)?;
   let total = (width as usize)
      .checked_mul(height as usize)
      .ok_or(Error::Overflow)?;
   let expected = total.checked_mul(channels).ok_or(Error::Overflow)?;
   let mut pixels = Vec::with_capacity(total);

   match decoder.depth().ok_or(Error::InvalidData)? {
      BitDepth::Eight => match result {
         DecodingResult::U8(raw) => {
            // A short decode would otherwise silently pad black pixels.
            // 解码结果过短时，原先会静默补黑像素。
            if raw.len() < expected {
               return Err(Error::InvalidData);
            }
            expand(&mut pixels, channels, total, |i| raw[i])?;
         }
         _ => return Err(Error::UnsupportedFormat),
      },
      BitDepth::Sixteen => match result {
         DecodingResult::U16(raw) => {
            if raw.len() < expected {
               return Err(Error::InvalidData);
            }
            expand(&mut pixels, channels, total, |i| (raw[i] >> 8) as u8)?;
         }
         _ => return Err(Error::UnsupportedFormat),
      },
      _ => return Err(Error::UnsupportedFormat),
   }

   Texture::new(width, height, pixels)
}

fn channels_for(colorspace: ColorSpace) -> Option<usize> {
   match colorspace {
      ColorSpace::Luma => Some(1),
      ColorSpace::LumaA => Some(2),
      ColorSpace::RGB => Some(3),
      ColorSpace::RGBA => Some(4),
      _ => None,
   }
}

#[cfg(test)]
mod tests {
   use super::*;
   use he3d_core::ColorRgba8;

   /// A 2x2 RGBA PNG (8-bit, non-interlaced).
   /// 一个 2x2 RGBA PNG（8 位，非隔行扫描）。
   // Test fixtures are tiny, so the `usize`→`u32` narrowing below cannot truncate.
   // 测试夹具都很小，因此下面的 `usize`→`u32` 收窄不会截断。
   #[expect(clippy::cast_possible_truncation)]
   fn png_2x2_rgba() -> Vec<u8> {
      // Chunks: IHDR, IDAT, IEND. IDAT is a zlib stream of raw deflate data built with a tiny hand-rolled encoder (stored blocks).
      // 数据块：IHDR、IDAT、IEND。IDAT 是原始 deflate 数据的 zlib 流，由手写的小型编码器生成（存储块）。
      let ihdr = {
         let mut d = Vec::new();
         d.extend_from_slice(&13u32.to_be_bytes());
         d.extend_from_slice(b"IHDR");
         d.extend_from_slice(&2u32.to_be_bytes()); // width / 宽度
         d.extend_from_slice(&2u32.to_be_bytes()); // height / 高度
         d.push(8); // bit depth / 位深
         d.push(6); // color type RGBA / 颜色类型 RGBA
         d.extend_from_slice(&[0, 0, 0]); // compression/filter/interlace / 压缩/滤波/隔行扫描
         d
      };
      let idat = {
         // Filter byte 0 + RGBA scanlines: 4 pixels of known colors.
         // 滤波字节 0 + RGBA 扫描行：4 个已知颜色的像素。
         // Row 0: red, green; row 1: blue, white (9 bytes each).
         // 第 0 行：红、绿；第 1 行：蓝、白（每行 9 字节）。
         let raw = [
            0, 255, 0, 0, 255, 0, 255, 0, 255, // row0
            0, 0, 0, 255, 255, 255, 255, 255, 255, // row1
         ];
         let mut d = Vec::new();
         let zlib = zlib_stored(&raw);
         d.extend_from_slice(&(zlib.len() as u32).to_be_bytes());
         d.extend_from_slice(b"IDAT");
         d.extend_from_slice(&zlib);
         d
      };
      let iend = {
         let mut d = Vec::new();
         d.extend_from_slice(&0u32.to_be_bytes());
         d.extend_from_slice(b"IEND");
         d
      };

      let mut out = Vec::new();
      out.extend_from_slice(&[0x89, b'P', b'N', b'G', 0x0d, 0x0a, 0x1a, 0x0a]);
      for chunk in [ihdr, idat, iend] {
         let crc = crc32(&chunk[4..]);
         out.extend_from_slice(&chunk);
         out.extend_from_slice(&crc.to_be_bytes());
      }
      out
   }

   /// zlib wrapper around stored (uncompressed) deflate blocks.
   /// zlib 对存储（未压缩）deflate 块的封装。
   // Block lengths are clamped to 65535 (the stored-block max), so the
   // `usize`→`u16` narrowing cannot truncate.
   // 块长度被钳制在 65535（存储块上限）内，因此 `usize`→`u16` 不会截断。
   #[expect(clippy::cast_possible_truncation)]
   fn zlib_stored(data: &[u8]) -> Vec<u8> {
      let mut out = Vec::new();
      out.push(0x78);
      out.push(0x01);
      let mut pos = 0;
      while pos < data.len() {
         let block_len = (data.len() - pos).min(65535);
         let last = pos + block_len >= data.len();
         // Stored block header: BTYPE=00 in bits 1-2, BFINAL in bit 0.
         // 存储块头部：位 1-2 为 BTYPE=00，位 0 为 BFINAL。
         out.push(if last { 0x81 } else { 0x01 });
         out.extend_from_slice(&(block_len as u16).to_le_bytes());
         out.extend_from_slice(&(!(block_len as u16)).to_le_bytes());
         out.extend_from_slice(&data[pos..pos + block_len]);
         pos += block_len;
      }
      // adler32 over the raw data
      // 对原始数据计算 adler32
      out.extend_from_slice(&adler32(data).to_be_bytes());
      out
   }

   fn crc32(data: &[u8]) -> u32 {
      let mut crc = 0xffff_ffffu32;
      for &b in data {
         crc ^= u32::from(b);
         for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
         }
      }
      !crc
   }

   fn adler32(data: &[u8]) -> u32 {
      let mut a = 1u32;
      let mut b = 0u32;
      for &byte in data {
         a = (a + u32::from(byte)) % 65521;
         b = (b + a) % 65521;
      }
      (b << 16) | a
   }

   #[test]
   fn decodes_rgba_png() {
      let bytes = png_2x2_rgba();
      let tex = decode(&bytes).unwrap_or_else(|e| panic!("png decode failed: {e:?}"));
      assert_eq!((tex.width(), tex.height()), (2, 2));
      assert_eq!(tex.pixels()[0], ColorRgba8::from_rgb(255, 0, 0));
      assert_eq!(tex.pixels()[1], ColorRgba8::from_rgb(0, 255, 0));
      assert_eq!(tex.pixels()[2], ColorRgba8::from_rgb(0, 0, 255));
      assert_eq!(tex.pixels()[3], ColorRgba8::WHITE);
   }

   #[test]
   fn rejects_garbage() {
      assert_eq!(
         decode(&[0x89, b'P', b'N', b'G', 0, 0, 0, 0]),
         Err(Error::InvalidData)
      );
   }
}

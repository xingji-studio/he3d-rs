use alloc::vec::Vec;
use he3d_core::{ColorRgba8, Error, Texture};

use super::MAX_IMAGE_DIMENSION;

/// Decode a BMP image from memory into RGBA8.
/// 从内存解码 BMP 图像为 RGBA8。
///
/// Supports `24`/`32`-bit uncompressed (`BI_RGB`) bitmaps, bottom-up or top-down
/// (negative height), with rows padded to 4 bytes.
/// 支持 `24`/`32` 位未压缩（`BI_RGB`）位图，自底向上或自顶向下（负高度），行按 4 字节对齐。
///
/// # Errors
/// 数据不是合法的 BMP、尺寸非法或溢出，或使用不支持的压缩格式/位深时返回相应错误。
/// Returns an error when the data is not a valid BMP, when dimensions are invalid
/// or overflow, or when the format uses unsupported compression or bit depth.
// BMP 解码循环惯用单字母循环变量与 BGR 分量名；刻意保留。
// Single-letter loop and BGR component names are idiomatic in this decode loop.
#[expect(clippy::many_single_char_names)]
pub fn decode(bytes: &[u8]) -> Result<Texture, Error> {
   if bytes.len() < 54 || bytes[0] != b'B' || bytes[1] != b'M' {
      return Err(Error::InvalidData);
   }

   let pixel_offset = read_u32(bytes, 10)? as usize;
   let width = read_i32(bytes, 18)?;
   let height = read_i32(bytes, 22)?;
   let bpp = read_u16(bytes, 28)?;
   let compression = read_u32(bytes, 30)?;

   if pixel_offset >= bytes.len() {
      return Err(Error::InvalidData);
   }
   if width <= 0 || height == 0 {
      return Err(Error::InvalidData);
   }
   if compression != 0 {
      return Err(Error::UnsupportedFormat);
   }
   let bytes_per_pixel = match bpp {
      24 => 3,
      32 => 4,
      _ => return Err(Error::UnsupportedFormat),
   };

   let top_down = height < 0;
   let width = width.unsigned_abs();
   let height = height.unsigned_abs();

   if width == 0 || height == 0 || width > MAX_IMAGE_DIMENSION || height > MAX_IMAGE_DIMENSION {
      return Err(Error::InvalidSize);
   }

   let row_size = ((width as usize) * bytes_per_pixel + 3) & !3;
   let data_size = bytes.len() - pixel_offset;
   if row_size == 0 || data_size / row_size < height as usize {
      return Err(Error::InvalidData);
   }

   let total = (width as usize)
      .checked_mul(height as usize)
      .ok_or(Error::Overflow)?;
   let mut pixels = Vec::with_capacity(total);

   for y in 0..height {
      let src_y = if top_down { y } else { height - 1 - y };
      let row_start = pixel_offset + src_y as usize * row_size;
      let row = &bytes[row_start..row_start + row_size];
      for x in 0..width {
         let i = x as usize * bytes_per_pixel;
         // BMP stores BGR(A).
         // BMP 存储 BGR(A) 顺序。
         let (b, g, r) = (row[i], row[i + 1], row[i + 2]);
         let a = if bytes_per_pixel == 4 {
            row[i + 3]
         } else {
            255
         };
         pixels.push(ColorRgba8::from_rgba(r, g, b, a));
      }
   }

   Texture::new(width, height, pixels)
}

fn read_u16(bytes: &[u8], offset: usize) -> Result<u16, Error> {
   let b = bytes.get(offset..offset + 2).ok_or(Error::InvalidData)?;
   Ok(u16::from_le_bytes([b[0], b[1]]))
}

fn read_u32(bytes: &[u8], offset: usize) -> Result<u32, Error> {
   let b = bytes.get(offset..offset + 4).ok_or(Error::InvalidData)?;
   Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

fn read_i32(bytes: &[u8], offset: usize) -> Result<i32, Error> {
   let b = bytes.get(offset..offset + 4).ok_or(Error::InvalidData)?;
   Ok(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

#[cfg(test)]
mod tests {
   use alloc::vec;

   use super::*;

   /// Build a minimal 2x2 24-bit bottom-up BMP.
   /// 构建一个最小的 2x2 24 位自底向上 BMP。
   fn bmp_2x2() -> Vec<u8> {
      let w = 2u32;
      let h = 2u32;
      let row_size = 8; // 2*3 padded to 4 / 2*3 对齐到 4 字节
      let data_size = row_size * h as usize;
      let mut b = vec![0u8; 54 + data_size];
      b[0] = b'B';
      b[1] = b'M';
      let file_size = u32::try_from(54 + data_size).expect("test buffer fits u32");
      b[2..6].copy_from_slice(&file_size.to_le_bytes());
      b[10..14].copy_from_slice(&54u32.to_le_bytes());
      b[14..18].copy_from_slice(&40u32.to_le_bytes()); // BITMAPINFOHEADER structure / BITMAPINFOHEADER 结构体
      b[18..22].copy_from_slice(&w.to_le_bytes());
      b[22..26].copy_from_slice(&h.to_le_bytes());
      b[26..28].copy_from_slice(&1u16.to_le_bytes()); // planes / 平面数
      b[28..30].copy_from_slice(&24u16.to_le_bytes()); // bpp / 每像素位数
      // First file row (written first in bottom-up): blue (BGR 255,0,0), white (BGR 255,255,255).
      // 文件第一行（bottom-up 时最先写入）：蓝（BGR 255,0,0）、白（BGR 255,255,255）。
      b[54..60].copy_from_slice(&[255, 0, 0, 255, 255, 255]);
      // Second file row (top): red (BGR 0,0,255), green (BGR 0,255,0).
      // 文件第二行（top）：红（BGR 0,0,255）、绿（BGR 0,255,0）。
      let top = 54 + row_size;
      b[top..top + 6].copy_from_slice(&[0, 0, 255, 0, 255, 0]);
      b
   }

   #[test]
   fn decodes_bottom_up_bmp() {
      let tex = decode(&bmp_2x2()).unwrap();
      assert_eq!((tex.width(), tex.height()), (2, 2));
      // Output top row = last file row (top): red, green.
      // 输出顶行 = 文件最后一行（top）：红、绿。
      assert_eq!(tex.pixels()[0], ColorRgba8::from_rgb(255, 0, 0));
      assert_eq!(tex.pixels()[1], ColorRgba8::from_rgb(0, 255, 0));
      // Output bottom row = first file row (bottom): blue, white.
      // 输出底行 = 文件第一行（bottom）：蓝、白。
      assert_eq!(tex.pixels()[2], ColorRgba8::from_rgb(0, 0, 255));
      assert_eq!(tex.pixels()[3], ColorRgba8::WHITE);
   }

   #[test]
   fn rejects_truncated_header() {
      assert_eq!(decode(&b"BM\x00\x00"[..]), Err(Error::InvalidData));
      assert_eq!(decode(&[0; 10]), Err(Error::InvalidData));
   }

   #[test]
   fn rejects_compressed_bmp() {
      let mut b = bmp_2x2();
      b[30..34].copy_from_slice(&1u32.to_le_bytes()); // BI_RLE8 compression / BI_RLE8 压缩
      assert_eq!(decode(&b), Err(Error::UnsupportedFormat));
   }

   #[test]
   fn rejects_oversized_dimensions() {
      let mut b = bmp_2x2();
      b[18..22].copy_from_slice(&(MAX_IMAGE_DIMENSION + 1).to_le_bytes());
      assert_eq!(decode(&b), Err(Error::InvalidSize));
   }
}

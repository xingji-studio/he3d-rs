use std::io::{self, Write};
use std::time::{Duration, Instant};

use he3d_platform::{
   BackendConfig, ColorRgba8, Event, KeyCode, Platform, PlatformError, PlatformFactory,
   normalize_key,
};

/// Errors from terminal backend construction.
/// 终端后端构造错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TerminalError {
   InvalidSize,
   InputConfigureFailed,
}

impl core::fmt::Display for TerminalError {
   fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
      match self {
         TerminalError::InvalidSize => f.write_str("invalid terminal size"),
         TerminalError::InputConfigureFailed => f.write_str("failed to configure terminal input"),
      }
   }
}

/// The std terminal backend.
/// 标准库终端后端。
///
/// Presents an RGBA framebuffer to a Unix terminal using ANSI 24-bit color, two pixels per character cell (upper = foreground, lower = background, `U+2580`).
/// 使用 ANSI 24 位真彩色把 RGBA 帧缓冲呈现到 Unix 终端，每个字符单元显示两个像素（上半=前景色，下半=背景色，`U+2580`）。
/// Reads keys from stdin in raw non-blocking mode and synthesizes key-up events, since terminals do not report release events.
/// 以原始非阻塞模式从标准输入读取按键，并合成按键释放事件，因为终端不报告释放事件。
///
/// When stdin is not a TTY (e.g. CI pipelines), raw input configuration is skipped and `poll_events` returns nothing; presentation still works, so the backend is usable for headless smoke tests.
/// 当标准输入不是 TTY（例如 CI 流水线）时，跳过原始输入配置，`poll_events` 不返回任何内容；呈现仍然可用，因此该后端可用于无头冒烟测试。
pub struct TerminalBackend {
   columns: u32,
   rows: u32,
   close_requested: bool,
   key_down: [bool; 256],
   key_last_seen: [f64; 256],
   escape_pending: bool,
   escape_sequence_active: bool,
   escape_pending_time: f64,
   start: Instant,
   termios: Option<libc::termios>,
   stdin_flags: i32,
   configured: bool,
   first_present: bool,
}

impl PlatformFactory for TerminalBackend {
   type CreateError = TerminalError;

   /// Configure the terminal for `width` × `height` source pixels.
   /// 按 `width` × `height` 源像素配置终端。
   fn create(config: BackendConfig<'_>) -> Result<Self, TerminalError> {
      let mut backend = Self::create_raw(config.width, config.height)?;
      backend.configure_input()?;
      print!("\x1b[2J\x1b[H\x1b[?25l");
      let _ = io::stdout().flush();
      Ok(backend)
   }
}

impl TerminalBackend {
   fn create_raw(width: u32, height: u32) -> Result<Self, TerminalError> {
      if width == 0 || height == 0 {
         return Err(TerminalError::InvalidSize);
      }
      let (columns, rows) = compute_grid(width, height);
      Ok(Self {
         columns,
         rows,
         close_requested: false,
         key_down: [false; 256],
         key_last_seen: [0.0; 256],
         escape_pending: false,
         escape_sequence_active: false,
         escape_pending_time: 0.0,
         start: Instant::now(),
         termios: None,
         stdin_flags: -1,
         configured: false,
         first_present: true,
      })
   }

   fn configure_input(&mut self) -> Result<(), TerminalError> {
      if unsafe { libc::isatty(libc::STDIN_FILENO) } == 0 {
         return Ok(());
      }

      let mut original: libc::termios = unsafe { std::mem::zeroed() };
      if unsafe { libc::tcgetattr(libc::STDIN_FILENO, &mut original) } != 0 {
         return Err(TerminalError::InputConfigureFailed);
      }
      self.termios = Some(original);

      let mut raw = original;
      raw.c_lflag &= !(libc::ICANON | libc::ECHO);
      raw.c_iflag &= !(libc::IXON | libc::ICRNL);
      raw.c_cc[libc::VMIN] = 0;
      raw.c_cc[libc::VTIME] = 0;
      if unsafe { libc::tcsetattr(libc::STDIN_FILENO, libc::TCSANOW, &raw) } != 0 {
         return Err(TerminalError::InputConfigureFailed);
      }

      let flags = unsafe { libc::fcntl(libc::STDIN_FILENO, libc::F_GETFL) };
      if flags < 0 {
         self.restore_input();
         return Err(TerminalError::InputConfigureFailed);
      }
      self.stdin_flags = flags;
      unsafe {
         libc::fcntl(libc::STDIN_FILENO, libc::F_SETFL, flags | libc::O_NONBLOCK);
      }
      self.configured = true;
      Ok(())
   }

   fn restore_input(&mut self) {
      // Best-effort terminal restore: the destructor cannot report failures, and panicking here (e.g. broken stdout) would escalate to an abort.
      // 尽力恢复终端：析构函数无法报告失败，而在此处 panic（例如标准输出损坏）会升级为直接中止进程。
      if let Some(termios) = self.termios.take() {
         unsafe {
            let _ = libc::tcsetattr(libc::STDIN_FILENO, libc::TCSANOW, &termios);
         }
      }
      if self.stdin_flags >= 0 {
         unsafe {
            let _ = libc::fcntl(libc::STDIN_FILENO, libc::F_SETFL, self.stdin_flags);
         }
         self.stdin_flags = -1;
      }
      self.configured = false;
   }

   fn press_key(&mut self, key: KeyCode, now: f64, events: &mut Vec<Event>) {
      let key = if key == 3 { 27 } else { normalize_key(key) };
      if (0..256).contains(&key) {
         self.key_last_seen[key as usize] = now;
         if !self.key_down[key as usize] {
            self.key_down[key as usize] = true;
            events.push(Event::KeyDown(key));
         }
      } else {
         events.push(Event::KeyDown(key));
         events.push(Event::KeyUp(key));
      }
   }

   fn flush_pending_escape(&mut self, now: f64, events: &mut Vec<Event>) {
      self.escape_pending = false;
      self.close_requested = true;
      self.press_key(27, now, events);
   }
}

impl Drop for TerminalBackend {
   fn drop(&mut self) {
      self.restore_input();
      // Never panic in a destructor: a broken stdout while unwinding would escalate to an abort.
      // 绝不在析构函数中 panic：在栈展开期间标准输出损坏会升级为中止进程。
      let _ = writeln!(io::stdout(), "\x1b[0m\x1b[?25h");
      let _ = io::stdout().flush();
   }
}

impl Platform for TerminalBackend {
   fn present(
      &mut self,
      width: u32,
      height: u32,
      pixels: &[ColorRgba8],
   ) -> Result<(), PlatformError> {
      if width == 0 || height == 0 {
         return Err(PlatformError::InvalidFramebuffer);
      }
      let expected = (width as usize)
         .checked_mul(height as usize)
         .ok_or(PlatformError::InvalidFramebuffer)?;
      if pixels.len() != expected {
         return Err(PlatformError::InvalidFramebuffer);
      }

      if self.first_present {
         print!("\x1b[2J");
         self.first_present = false;
      }

      let columns = self.columns as usize;
      let rows = self.rows as usize;
      let sample_height = rows * 2;
      let width_u = width as usize;
      let height_u = height as usize;

      let mut out = String::with_capacity(columns * rows * 24);
      out.push_str("\x1b[H");

      let mut fg = ColorRgba8::BLACK;
      let mut bg = ColorRgba8::BLACK;
      let mut has_fg = false;
      let mut has_bg = false;

      for y in 0..rows {
         let src_top_y = ((y * 2) * height_u / sample_height).min(height_u - 1);
         let src_bottom_y = ((y * 2 + 1) * height_u / sample_height).min(height_u - 1);
         let top_row = &pixels[src_top_y * width_u..][..width_u];
         let bottom_row = &pixels[src_bottom_y * width_u..][..width_u];

         for x in 0..columns {
            let src_x = (x * width_u / columns).min(width_u - 1);
            let top = top_row[src_x];
            let bottom = bottom_row[src_x];

            if !has_fg || fg != top {
               out.push_str(&format!("\x1b[38;2;{};{};{}m", top.r, top.g, top.b));
               fg = top;
               has_fg = true;
            }
            if !has_bg || bg != bottom {
               out.push_str(&format!(
                  "\x1b[48;2;{};{};{}m",
                  bottom.r, bottom.g, bottom.b
               ));
               bg = bottom;
               has_bg = true;
            }
            out.push('\u{2580}');
         }
         out.push_str("\x1b[0m\n");
         has_fg = false;
         has_bg = false;
      }

      print!("{out}");
      io::stdout()
         .flush()
         .map_err(|_| PlatformError::BackendUnavailable)
   }

   fn poll_events(&mut self) -> Vec<Event> {
      let now = self.time_seconds();
      let mut events = Vec::new();
      if !self.configured {
         return events;
      }

      let mut buffer = [0u8; 64];
      loop {
         let n = unsafe {
            libc::read(
               libc::STDIN_FILENO,
               buffer.as_mut_ptr() as *mut libc::c_void,
               buffer.len(),
            )
         };
         if n <= 0 {
            break;
         }
         for &b in &buffer[..n as usize] {
            let raw = b as KeyCode;
            if self.escape_sequence_active {
               if is_escape_final(raw) {
                  self.escape_sequence_active = false;
               }
               continue;
            }
            if self.escape_pending {
               if is_escape_lead(raw) {
                  self.escape_pending = false;
                  self.escape_sequence_active = true;
                  continue;
               }
               self.flush_pending_escape(now, &mut events);
            }
            if raw == 27 {
               self.escape_pending = true;
               self.escape_pending_time = now;
               continue;
            }
            self.press_key(raw, now, &mut events);
         }
      }

      if self.escape_pending && now - self.escape_pending_time > 0.03 {
         self.flush_pending_escape(now, &mut events);
      }

      for key in 0..256 {
         if self.key_down[key] && now - self.key_last_seen[key] > 0.35 {
            self.key_down[key] = false;
            events.push(Event::KeyUp(key as KeyCode));
         }
      }
      events
   }

   fn should_close(&self) -> bool {
      self.close_requested
   }

   fn request_close(&mut self) {
      self.close_requested = true;
   }

   fn time_seconds(&self) -> f64 {
      self.start.elapsed().as_secs_f64()
   }

   fn sleep_ms(&mut self, milliseconds: u64) {
      std::thread::sleep(Duration::from_millis(milliseconds));
   }
}

fn is_escape_lead(key: KeyCode) -> bool {
   key == b'[' as KeyCode || key == b'O' as KeyCode
}

fn is_escape_final(key: KeyCode) -> bool {
   (0x40..=0x7e).contains(&key)
}

fn compute_grid(width: u32, height: u32) -> (u32, u32) {
   let w = u64::from(width);
   let h = u64::from(height);
   let mut columns = if w < 80 { w } else { 80 };
   let mut rows = (columns * h).div_ceil(w);
   rows = rows.div_ceil(2);
   if rows > 30 {
      rows = 30;
      columns = (30 * 2 * w) / h;
      if columns > 80 {
         columns = 80;
      }
   }
   (columns.clamp(1, 80) as u32, rows.clamp(1, 30) as u32)
}

#[cfg(test)]
mod tests {
   use super::*;

   #[test]
   fn compute_grid_matches_reference() {
      assert_eq!(compute_grid(640, 360), (80, 23));
      assert_eq!(compute_grid(160, 90), (80, 23));
      assert_eq!(compute_grid(64, 36), (64, 18));
      assert_eq!(compute_grid(80, 100), (48, 30));
   }

   #[test]
   fn terminal_smoke_non_tty() {
      let mut terminal = TerminalBackend::create(BackendConfig {
         title: "",
         width: 64,
         height: 36,
      })
      .expect("backend");
      assert!(!terminal.should_close());

      let events = terminal.poll_events();
      assert!(events.is_empty());

      let w = 64;
      let h = 36;
      let pixels = vec![ColorRgba8::from_rgb(255, 0, 0); (w * h) as usize];
      terminal.present(w, h, &pixels).expect("present one frame");

      assert!(terminal.time_seconds() >= 0.0);
      terminal.sleep_ms(1);

      terminal.request_close();
      assert!(terminal.should_close());
   }

   #[test]
   fn present_rejects_bad_framebuffer() {
      let mut terminal = TerminalBackend::create(BackendConfig {
         title: "",
         width: 64,
         height: 36,
      })
      .expect("backend");
      let pixels = vec![ColorRgba8::BLACK; 3];
      assert_eq!(
         terminal.present(2, 2, &pixels),
         Err(PlatformError::InvalidFramebuffer)
      );
      assert_eq!(
         terminal.present(0, 10, &[]),
         Err(PlatformError::InvalidFramebuffer)
      );
   }
}

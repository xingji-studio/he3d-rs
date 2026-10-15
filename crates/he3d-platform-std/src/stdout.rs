use std::fmt::Write;
use std::io::{self, Write as _};
use std::time::{Duration, Instant};

use he3d_platform::{
   BackendConfig, ColorRgba8, Event, KeyCode, Platform, PlatformError, normalize_key,
};

/// Errors from standard-output (stdout) backend construction.
/// 标准库标准输出（stdout）后端构造错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum StdBackendError {
   /// A zero or invalid framebuffer size was requested.
   /// 请求了为零或无效的帧缓冲尺寸。
   InvalidSize,
   /// Raw-mode input configuration on stdin failed.
   /// 标准输入原始模式配置失败。
   InputConfigureFailed,
}

impl core::fmt::Display for StdBackendError {
   fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
      match self {
         StdBackendError::InvalidSize => f.write_str("invalid size"),
         StdBackendError::InputConfigureFailed => f.write_str("failed to configure input"),
      }
   }
}

/// The Rust standard-library backend.
/// Rust 标准库后端。
///
/// Presents an RGBA framebuffer to the standard library's standard output (stdout)
/// using ANSI 24-bit color, two pixels per character cell (upper = foreground,
/// lower = background, `U+2580`).
/// 使用 ANSI 24 位真彩色把 RGBA 帧缓冲呈现到标准库标准输出（stdout），每个字符单元
/// 显示两个像素（上半=前景色，下半=背景色，`U+2580`）。
/// Reads keys from stdin in raw non-blocking mode and synthesizes key-up events,
/// since standard input does not report release events.
/// 以原始非阻塞模式从标准输入读取按键，并合成按键释放事件，因为标准输入不报告释放事件。
///
/// When stdin is not a TTY (e.g. CI pipelines), raw input configuration is skipped and `poll_events` returns nothing; presentation still works, so the backend is usable for headless smoke tests.
/// 当标准输入不是 TTY（例如 CI 流水线）时，跳过原始输入配置，`poll_events` 不返回任何内容；呈现仍然可用，因此该后端可用于无头冒烟测试。
// The stdin state machine legitimately tracks many boolean flags; grouping them
// would obscure the terminal-mode transitions they represent.
// 标准输入状态机合理地持有许多布尔标志；强行分组反而会掩盖它们所代表的终端模式切换。
#[expect(clippy::struct_excessive_bools)]
pub struct StdBackend {
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

impl core::fmt::Debug for StdBackend {
   fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
      // `termios`/`Instant` are not `Debug`; print the meaningful state only.
      // `termios`/`Instant` 未实现 `Debug`；只打印有意义的状态。
      f.debug_struct("StdBackend")
         .field("columns", &self.columns)
         .field("rows", &self.rows)
         .field("close_requested", &self.close_requested)
         .field("escape_pending", &self.escape_pending)
         .field("configured", &self.configured)
         .finish_non_exhaustive()
   }
}

impl StdBackend {
   /// Configure the standard library's input/output for `width` × `height` source pixels.
   /// 配置标准库的输入输出以呈现 `width` × `height` 源像素。
   ///
   /// Clears the terminal and hides the cursor, so the first frame replaces the
   /// shell prompt instead of scrolling under it.
   /// 清屏并隐藏光标，因此第一帧替换掉 shell 提示符，而不是在其下方滚动。
   ///
   /// # Errors
   /// 尺寸为零时返回 [`StdBackendError::InvalidSize`]；标准输入无法进入原始模式时返回
   /// [`StdBackendError::InputConfigureFailed`]。
   /// Returns [`StdBackendError::InvalidSize`] for a zero size, or
   /// [`StdBackendError::InputConfigureFailed`] when standard input cannot enter
   /// raw mode.
   pub fn new(config: BackendConfig<'_>) -> Result<Self, StdBackendError> {
      let mut backend = Self::create_raw(config.width, config.height)?;
      backend.configure_input()?;
      print!("\x1b[2J\x1b[H\x1b[?25l");
      let _ = io::stdout().flush();
      Ok(backend)
   }

   fn create_raw(width: u32, height: u32) -> Result<Self, StdBackendError> {
      if width == 0 || height == 0 {
         return Err(StdBackendError::InvalidSize);
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

   fn configure_input(&mut self) -> Result<(), StdBackendError> {
      // SAFETY: `isatty` takes a valid file descriptor (`STDIN_FILENO`) and returns an int.
      // 安全性：`isatty` 接收有效的文件描述符（`STDIN_FILENO`）并返回一个整数。
      if unsafe { libc::isatty(libc::STDIN_FILENO) } == 0 {
         return Ok(());
      }

      let mut original: libc::termios = unsafe { std::mem::zeroed() };
      // SAFETY: `original` points to a valid `termios` buffer for the whole call, and
      // `STDIN_FILENO` is a valid descriptor; `tcgetattr` initializes the struct on success.
      // 安全性：`original` 在整个调用期间指向有效的 `termios` 缓冲区，`STDIN_FILENO`
      // 是有效描述符；成功时 `tcgetattr` 会初始化该结构体。
      if unsafe { libc::tcgetattr(libc::STDIN_FILENO, std::ptr::from_mut(&mut original)) } != 0 {
         return Err(StdBackendError::InputConfigureFailed);
      }
      self.termios = Some(original);

      let mut raw = original;
      raw.c_lflag &= !(libc::ICANON | libc::ECHO);
      raw.c_iflag &= !(libc::IXON | libc::ICRNL);
      raw.c_cc[libc::VMIN] = 0;
      raw.c_cc[libc::VTIME] = 0;
      // SAFETY: `raw` is fully initialized (a copy of `original`), `STDIN_FILENO` is valid,
      // and `TCSANOW` applies the change synchronously.
      // 安全性：`raw` 已完全初始化（`original` 的副本），`STDIN_FILENO` 有效，
      // `TCSANOW` 同步应用该改动。
      if unsafe { libc::tcsetattr(libc::STDIN_FILENO, libc::TCSANOW, std::ptr::from_ref(&raw)) }
         != 0
      {
         return Err(StdBackendError::InputConfigureFailed);
      }

      // SAFETY: `fcntl` with `F_GETFL` on the valid `STDIN_FILENO` returns the current flags.
      // 安全性：在有效的 `STDIN_FILENO` 上以 `F_GETFL` 调用 `fcntl` 返回当前标志位。
      let flags = unsafe { libc::fcntl(libc::STDIN_FILENO, libc::F_GETFL) };
      if flags < 0 {
         self.restore_input();
         return Err(StdBackendError::InputConfigureFailed);
      }
      self.stdin_flags = flags;
      // SAFETY: `flags` came from `F_GETFL` above and is a valid bitmask for `F_SETFL`;
      // setting `O_NONBLOCK` on stdin cannot fail in a way we can recover from.
      // 安全性：`flags` 来自上面的 `F_GETFL`，是 `F_SETFL` 的有效位掩码；在标准输入上
      // 设置 `O_NONBLOCK` 不会以我们无法恢复的方式失败。
      unsafe {
         libc::fcntl(libc::STDIN_FILENO, libc::F_SETFL, flags | libc::O_NONBLOCK);
      }
      self.configured = true;
      Ok(())
   }

   fn restore_input(&mut self) {
      // Best-effort input restore: the destructor cannot report failures, and panicking here (e.g. broken stdout) would escalate to an abort.
      // 尽力恢复输入状态：析构函数无法报告失败，而在此处 panic（例如标准输出损坏）会升级为直接中止进程。
      if let Some(termios) = self.termios.take() {
         // SAFETY: `termios` is the previously saved, still-valid terminal state and
         // `STDIN_FILENO` is a valid descriptor; a failure only means the restore is skipped.
         // 安全性：`termios` 是先前保存且仍有效的终端状态，`STDIN_FILENO` 有效；
         // 失败只意味着恢复被跳过。
         unsafe {
            let _ = libc::tcsetattr(
               libc::STDIN_FILENO,
               libc::TCSANOW,
               std::ptr::from_ref(&termios),
            );
         }
      }
      if self.stdin_flags >= 0 {
         // SAFETY: `self.stdin_flags` was returned by `F_GETFL` earlier and is a valid mask.
         // 安全性：`self.stdin_flags` 是先前由 `F_GETFL` 返回的有效掩码。
         unsafe {
            let _ = libc::fcntl(libc::STDIN_FILENO, libc::F_SETFL, self.stdin_flags);
         }
         self.stdin_flags = -1;
      }
      self.configured = false;
   }

   fn press_key(&mut self, key: KeyCode, now: f64, events: &mut Vec<Event>) {
      let key = if key == 3 { 27 } else { normalize_key(key) };
      // The index is only used after the `(0..256)` bounds check, so the sign
      // loss cannot occur. 索引只在 `(0..256)` 边界检查之后使用，因此不会发生符号丢失。
      #[expect(clippy::cast_sign_loss)]
      {
         if (0..256).contains(&key) {
            let index = key as usize;
            self.key_last_seen[index] = now;
            if !self.key_down[index] {
               self.key_down[index] = true;
               events.push(Event::KeyDown(key));
            }
         } else {
            events.push(Event::KeyDown(key));
            events.push(Event::KeyUp(key));
         }
      }
   }

   fn flush_pending_escape(&mut self, now: f64, events: &mut Vec<Event>) {
      self.escape_pending = false;
      self.close_requested = true;
      self.press_key(27, now, events);
   }
}

impl Drop for StdBackend {
   fn drop(&mut self) {
      self.restore_input();
      // Never panic in a destructor: a broken stdout while unwinding would escalate to an abort.
      // 绝不在析构函数中 panic：在栈展开期间标准输出损坏会升级为中止进程。
      let _ = writeln!(io::stdout(), "\x1b[0m\x1b[?25h");
      let _ = io::stdout().flush();
   }
}

impl Platform for StdBackend {
   // `fg`/`bg` (and their `has_*` flags) are intentionally parallel names for the
   // foreground/background ANSI state tracking.
   // `fg`/`bg`（及其 `has_*` 标志）是前景/背景 ANSI 状态跟踪刻意使用的平行命名。
   #[expect(clippy::similar_names)]
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
               let _ = write!(out, "\x1b[38;2;{};{};{}m", top.r, top.g, top.b);
               fg = top;
               has_fg = true;
            }
            if !has_bg || bg != bottom {
               let _ = write!(out, "\x1b[48;2;{};{};{}m", bottom.r, bottom.g, bottom.b);
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
         // SAFETY: `buffer` is a valid `[u8; 64]`, so `as_mut_ptr` points to
         // writable memory of `buffer.len()` bytes; `STDIN_FILENO` is a valid
         // descriptor opened for reading. A short read or an error just exits the loop.
         // 安全性：`buffer` 是有效的 `[u8; 64]`，因此 `as_mut_ptr` 指向 `buffer.len()`
         // 字节的可写内存；`STDIN_FILENO` 是以读方式打开的有效描述符。短读或错误只会退出循环。
         let n = unsafe {
            libc::read(
               libc::STDIN_FILENO,
               buffer.as_mut_ptr().cast::<libc::c_void>(),
               buffer.len(),
            )
         };
         if n <= 0 {
            break;
         }
         // `n > 0` is checked above, so the isize→usize conversion cannot lose the sign.
         // 上面已检查 `n > 0`，因此 isize→usize 转换不会丢失符号。
         #[expect(clippy::cast_sign_loss)]
         let count = n as usize;
         for &b in &buffer[..count] {
            let raw = i32::from(b);
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

      for key in 0u8..=255u8 {
         let index = usize::from(key);
         if self.key_down[index] && now - self.key_last_seen[index] > 0.35 {
            self.key_down[index] = false;
            events.push(Event::KeyUp(i32::from(key)));
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
   key == i32::from(b'[') || key == i32::from(b'O')
}

fn is_escape_final(key: KeyCode) -> bool {
   (0x40..=0x7e).contains(&key)
}

/// Choose the character-cell grid this backend renders into.
/// 选择此后端渲染所用的字符单元网格。
///
/// The caps are presentation policy rather than a derived bound: 80 columns is the
/// conventional terminal width and 30 rows is about one screenful, so a larger
/// window is downsampled instead of spending more ANSI bytes per frame than a
/// terminal can usefully show. Both stay in step with the two-pixels-per-cell
/// packing described on `StdBackend`.
/// 这两个上限是呈现策略而非推导出的界：80 列是常规终端宽度，30 行约为一屏，因此更大的
/// 窗口会被降采样，而不是每帧输出终端根本无法有效显示的更多 ANSI 字节。两者与
/// `StdBackend` 上描述的每单元两像素打包保持一致。
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
   fn compute_grid_caps_to_a_screenful() {
      assert_eq!(compute_grid(640, 360), (80, 23));
      assert_eq!(compute_grid(160, 90), (80, 23));
      assert_eq!(compute_grid(64, 36), (64, 18));
      assert_eq!(compute_grid(80, 100), (48, 30));
   }

   #[test]
   fn stdout_smoke_non_tty() {
      let mut backend = StdBackend::new(BackendConfig::new("", 64, 36)).expect("backend");
      assert!(!backend.should_close());

      let events = backend.poll_events();
      assert_eq!(events, [] as [he3d_platform::Event; 0]);

      let w = 64;
      let h = 36;
      let pixels = vec![ColorRgba8::from_rgb(255, 0, 0); (w * h) as usize];
      backend.present(w, h, &pixels).expect("present one frame");

      assert!(backend.time_seconds() >= 0.0);
      backend.sleep_ms(1);

      backend.request_close();
      assert!(backend.should_close());
   }

   #[test]
   fn present_rejects_bad_framebuffer() {
      let mut backend = StdBackend::new(BackendConfig::new("", 64, 36)).expect("backend");
      let pixels = vec![ColorRgba8::BLACK; 3];
      assert_eq!(
         backend.present(2, 2, &pixels),
         Err(PlatformError::InvalidFramebuffer)
      );
      assert_eq!(
         backend.present(0, 10, &[]),
         Err(PlatformError::InvalidFramebuffer)
      );
   }
}

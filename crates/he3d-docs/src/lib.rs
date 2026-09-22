//! Documentation harness.
//! 文档编译宿主。
//!
//! This crate has no library of its own. `build.rs` reads the Markdown guides in
//! `docs/`, lifts every `rust` code block out of them, and generates the modules
//! below, so `cargo check` compiles the guides' code as part of this crate and a
//! guide that has drifted away from the API fails the build.
//! 本 crate 自身没有库。`build.rs` 读取 `docs/` 下的 Markdown 指南，把其中每个 `rust`
//! 代码块抽出来，生成下面这些模块，因此 `cargo check` 会把指南里的代码当作本 crate 的
//! 一部分编译；指南一旦与 API 脱节，构建就会失败。
//!
//! The guides stay ordinary Markdown. Nothing in them exists for this harness: no
//! hidden lines, no fence attributes, nothing that GitHub or an editor would show
//! differently from what the file says.
//! 指南仍是普通 Markdown。里面没有任何东西是为这个宿主而写的：没有隐藏行，没有围栏属性，
//! 没有任何 GitHub 或编辑器会显示得与文件内容不一致的东西。
//!
//! Blocks are compiled but never run. A guide's program opens a terminal and waits
//! for a keypress, which is not something a test run should do; running the engine
//! is what `examples/` and the per-crate test suites are for.
//! 代码块只编译，从不运行。指南里的程序会打开终端并等待按键，而测试运行不该做这件事；
//! 真正运行引擎是 `examples/` 与各 crate 测试套件的事。

mod compiled {
   #![allow(
      dead_code,
      unused_doc_comments,
      unused_imports,
      unused_mut,
      unused_variables
   )]
   #![allow(clippy::all, clippy::pedantic)]
   include!(concat!(env!("OUT_DIR"), "/guides.rs"));
}

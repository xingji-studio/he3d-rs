//! Compiles the `rust` code blocks of the Markdown guides.
//! 编译 Markdown 指南中的 `rust` 代码块。
//!
//! The guides are read exactly as written: no hidden lines, no `no_run`, nothing
//! that would show up when GitHub or an editor renders the same file. The setup a
//! fragment needs but does not show lives here instead, keyed by the fragment's
//! first line, and is prepended to it in the generated file.
//! 指南按其本来面目读取：没有隐藏行，没有 `no_run`,也没有任何在 GitHub 或编辑器渲染同一
//! 份文件时会露出来的东西。片段需要、但自身未显示的上下文改放在这里，以片段首行为键，在
//! 生成的文件里补在它前面。
//!
//! A fragment whose first line has no entry is compiled without setup; if it needed
//! any, the build fails on the generated file and the comment above it names the
//! fragment it came from.
//! 首行没有对应条目的片段会不带上下文编译；若它本该有，构建会在生成的文件上失败，其上方
//! 的注释指明了它来自哪个片段。
//!
//! Counterexamples run the other way: a fragment registered in `FAILING` must *not*
//! compile. It is kept out of the generated modules and written out as a crate of its
//! own, which `tests/compile_fail.rs` builds and expects to fail with the error code
//! the table names. Such a fragment is a complete program, `fn main` included, because
//! the test asserts on the error the compiler prints for it.
//! 反例则相反：登记在 `FAILING` 里的片段必须**编译不过**。它不进生成的模块，而是被写成
//! 一个独立的 crate，由 `tests/compile_fail.rs` 构建并期望它失败，且错误码与表中所列
//! 一致。这类片段是完整程序，含 `fn main`：测试断言的正是编译器为它打出的错误。

use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

/// The guides that must compile, relative to this crate.
/// 必须能编译的指南，相对于本 crate。
const GUIDES: &[&str] = &[
   "../../docs/getting-started.zh.md",
   // A chapter is compiled while it is still a draft, so its code is checked before
   // it joins the guide. Drop this line once the chapter moves over.
   // 章节在还是草稿时就参与编译，代码在并入指南之前已被检查。章节并入之后删掉这一行。
   "../../docs/plans/getting-started.ch05.zh.md",
];

/// Setup a fragment needs but does not show, keyed by its first line.
/// 片段需要但自身未显示的上下文，以首行为键。
const CONTEXTS: &[(&str, &str)] = &[
   (
      r"// 1. 后端：程序与宿主环境之间的接口，由二进制自己构造。",
      r"use he3d::BackendConfig;
use he3d_platform_std::StdBackend;",
   ),
   (
      r"// 2. 场景：两张资源表、一份对象列表与一盏平行光。",
      r"use he3d::{ColorRgba8, Mesh, Scene, Texture, Vec3};",
   ),
   // Two guides carry this first line today, one naming the handle `cube` and the
   // other `cube_mesh`, so the setup binds both names.
   // 目前有两份指南用这行作首行，一个把句柄命名为 `cube`，另一个叫 `cube_mesh`，因此
   // 上下文把两个名字都绑上。
   (
      r"// 3. 对象：引用网格与纹理，自带变换与材质。",
      r"use he3d::{ColorRgba8, Material, Mesh, RenderObject, Rgb, Scene, Texture, Transform, Vec3};
let mut scene = Scene::new();
let cube_mesh = scene.meshes.insert(Mesh::cube(1.0, 1.0, 1.0).unwrap());
let cube = cube_mesh;
let blue = scene.textures.insert(Texture::solid(ColorRgba8::from_rgb(90, 160, 255)));",
   ),
   (
      r"// 4. 相机：位置、朝向与垂直视野。",
      r"use he3d::{Camera, Quat, Vec3};",
   ),
   (
      r"// 5. 渲染器：持有帧缓冲，且跨帧保有状态，因此只创建一次。",
      r"use he3d::{Renderer, RendererConfig};",
   ),
   (
      r"// 6. 循环：取事件、更新场景、绘制、呈现。",
      r#"use he3d::{BackendConfig, Event, KEY_ESCAPE, Platform};
use he3d_platform_std::StdBackend;
let mut backend = StdBackend::new(BackendConfig::new("x", 64, 36)).unwrap();"#,
   ),
   (
      r"// 帧间隔上限 0.1 秒：拖动窗口或命中断点后不会一次转过好几圈。",
      r#"use he3d::{BackendConfig, Platform};
use he3d_platform_std::StdBackend;
let mut backend = StdBackend::new(BackendConfig::new("x", 64, 36)).unwrap();
let mut previous = backend.time_seconds();
let mut angle = 0.0_f32;"#,
   ),
   (
      r"// 每一帧都把新的朝向写进对象。",
      r"use he3d::{Material, Mesh, Quat, RenderObject, Rgb, Scene, Transform, Vec3};
let mut scene = Scene::new();
let mesh = scene.meshes.insert(Mesh::cube(1.0, 1.0, 1.0).unwrap());
let object = scene.spawn(RenderObject {
   mesh,
   texture: None,
   transform: Transform::from_position(Vec3::ZERO),
   material: Material::new(Rgb::WHITE),
   visible: true,
});
let mut angle = 0.0_f32;",
   ),
   (
      r"// 先画进帧缓冲，再交给后端。",
      r#"use he3d::{
   BackendConfig, Camera, Platform, Quat, Renderer, RendererConfig, Rgb, Scene, Vec3,
};
use he3d_platform_std::StdBackend;
let mut backend = StdBackend::new(BackendConfig::new("x", 64, 36)).unwrap();
let mut scene = Scene::new();
let mut renderer = Renderer::new(RendererConfig::new(64, 36)).unwrap();
let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 60.0);"#,
   ),
   (
      r"const FIXED_STEP: f64 = 1.0 / 120.0;",
      r"use he3d::World;
let mut world = World::new();
let frame_delta = 1.0 / 60.0_f64;",
   ),
   (
      r#"/// Body 与"画它的东西"之间的联系，由应用拥有。"#,
      r"use he3d::{BodyId, ObjectId};",
   ),
   (
      r"/// 把 Body 的摆放复制到绘制它的对象上。",
      r"use he3d::{BodyId, ObjectId, Scene, World};
struct Thing {
   body: BodyId,
   object: ObjectId,
}",
   ),
   (
      r"// 接触表里有涉及 cube、且阶段不为 End 的条目，即为触地。",
      r"use he3d::{BodyDesc, ContactPhase, Shape, Vec3, World};
let mut world = World::new();
let cube = world.add_body(BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0).at(Vec3::new(0.0, 4.0, 0.0)));",
   ),
   (
      r"// 三个箱子叠起来，最上面那个叫 top。",
      r"use he3d::{BodyDesc, Shape, Vec3, World};
let mut world = World::new();",
   ),
   (
      r"// 上面那个箱子此刻是否受托，同样只看接触表。",
      r"use he3d::{BodyDesc, ContactPhase, Shape, Vec3, World};
let mut world = World::new();
let top = world.add_body(BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0));",
   ),
   (
      r"world.wake(top); // 唤醒之后，接触表里才会重新出现它。",
      r"use he3d::{BodyDesc, Shape, Vec3, World};
let mut world = World::new();
let top = world.add_body(BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0));",
   ),
   (
      r"// 世界空间的三角形列表，每三个顶点构成一个三角形，",
      r"use he3d::{Vec3, World};
let mut world = World::new();
let positions = [Vec3::ZERO, Vec3::X, Vec3::Y];",
   ),
   // Chapter 5 in draft: fragments that use names its program binds earlier. The
   // imports come from the program itself, so only the values are listed here.
   // 第五章草稿：用到其程序先前绑定的名字的片段。import 由程序本身提供，这里只列取值。
   (
      r"// 6. 循环：取事件、算帧间隔、改场景、绘制、呈现。",
      r#"let mut backend = SdlBackend::new(BackendConfig::new("x", 64, 36)).unwrap();
let mut scene = Scene::new();
let cube_mesh = scene.meshes.insert(Mesh::cube(1.0, 1.0, 1.0).unwrap());
let blue = scene.textures.insert(Texture::solid(ColorRgba8::from_rgb(90, 160, 255)));
let cube = scene.spawn(RenderObject {
   mesh: cube_mesh,
   texture: Some(blue),
   transform: Transform::from_position(Vec3::new(0.0, 0.0, 4.0)),
   material: Material::new(Rgb::WHITE),
   visible: true,
});
let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 60.0);
let mut renderer = Renderer::new(RendererConfig::new(64, 36)).unwrap();"#,
   ),
];

/// Fragments that must not compile, with the rustc error code they have to produce.
/// 必须编译不过的片段，以及它必须产生的 rustc 错误码。
const FAILING: &[(&str, &str)] = &[
   // `SdlBackend` implements the method, but the trait is not in scope.
   // `SdlBackend` 实现了这个方法，但 trait 不在作用域内。
   (r"// 少了 use he3d::Platform", "E0599"),
];

fn main() {
   let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
   let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR"));
   let mut matched = vec![false; CONTEXTS.len()];
   let mut failing = vec![false; FAILING.len()];
   let mut generated = String::new();
   let mut cases = String::new();
   // One counter for every block of every guide: numbering per guide would make the
   // second guide reuse the module names of the first.
   // 所有指南共用一个块编号：若各编各的，第二份指南会重复第一份的模块名。
   let mut id = 0;
   for guide in GUIDES {
      let path = manifest.join(guide);
      println!("cargo:rerun-if-changed={}", path.display());
      let text =
         fs::read_to_string(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
      let blocks = rust_blocks(&text);
      let program = blocks
         .iter()
         .map(|(_, body)| body.as_str())
         .find(|body| body.lines().any(|line| line.starts_with("fn main")));
      // The program's own imports are in scope at every point of it, so a fragment
      // taken from the program starts with them too. That is what keeps a new
      // fragment from needing a hand-written `use` list here.
      // 程序自己的 import 在它的每一点上都在作用域内，因此取自程序的片段也带着它们。
      // 新写的片段正因此不必在这里手写 `use` 清单。
      let prelude = program.map_or("", imports);
      for (line, block) in &blocks {
         id += 1;
         let first = block
            .lines()
            .find(|text| !text.trim().is_empty())
            .map_or("", str::trim);
         if let Some(code) = counterexample(first, &mut failing) {
            write_case(&out_dir, &manifest, id, block, code, &mut cases);
            continue;
         }
         let is_program = Some(block.as_str()) == program;
         generated.push_str(&compile(
            guide,
            *line,
            id,
            block,
            if is_program { "" } else { prelude },
            &mut matched,
         ));
      }
   }
   for (entry, seen) in CONTEXTS.iter().zip(&matched) {
      if !*seen {
         println!(
            "cargo:warning=build.rs has setup for a fragment that is gone: {}",
            entry.0
         );
      }
   }
   for (entry, seen) in FAILING.iter().zip(&failing) {
      if !*seen {
         println!(
            "cargo:warning=build.rs calls this a counterexample, but no guide has it: {}",
            entry.0
         );
      }
   }
   let out = out_dir.join("guides.rs");
   fs::write(&out, generated).unwrap_or_else(|error| panic!("{}: {error}", out.display()));
   let cases_out = out_dir.join("compile_fail_cases.tsv");
   fs::write(&cases_out, cases).unwrap_or_else(|error| panic!("{}: {error}", cases_out.display()));
}

/// The error code when `key` names a registered counterexample, remembering the hit.
/// 若 `key` 是登记过的反例，返回它必须产生的错误码，并记下这次命中。
fn counterexample<'a>(key: &str, matched: &'a mut [bool]) -> Option<&'a str> {
   for (slot, (candidate, code)) in matched.iter_mut().zip(FAILING) {
      if *candidate == key {
         *slot = true;
         return Some(code);
      }
   }
   None
}

/// Write one counterexample out as a crate of its own, and remember it for the test.
/// 把一个反例写成一个独立的 crate，并记下它供测试使用。
fn write_case(
   out_dir: &Path,
   manifest: &Path,
   id: usize,
   block: &str,
   code: &str,
   cases: &mut String,
) {
   let root = manifest.join("../..");
   let dir = out_dir.join("compile_fail").join(format!("case_{id}"));
   let src = dir.join("src");
   fs::create_dir_all(&src).unwrap_or_else(|error| panic!("{}: {error}", src.display()));
   let he3d = root.join("crates/he3d");
   let sdl3 = root.join("crates/he3d-platform-sdl3");
   // Standalone: an empty `[workspace]` keeps the case out of this repository's
   // workspace, and the copied lockfile keeps resolution offline.
   // 独立存在：空的 `[workspace]` 把用例挡在本仓库工作区之外，复制过来的锁文件让依赖
   // 解析无需联网。
   let package = format!(
      "[package]\nname = \"guide-counterexample-{id}\"\nversion = \"0.0.0\"\nedition = \
       \"2024\"\npublish = false\n\n[workspace]\n\n[dependencies]\nhe3d = {{ path = \"{}\", \
       features = [\"std\"] }}\nhe3d-platform-sdl3 = {{ path = \"{}\" }}\n",
      he3d.display(),
      sdl3.display()
   );
   let cargo_toml = dir.join("Cargo.toml");
   fs::write(&cargo_toml, package)
      .unwrap_or_else(|error| panic!("{}: {error}", cargo_toml.display()));
   let main = src.join("main.rs");
   fs::write(&main, block).unwrap_or_else(|error| panic!("{}: {error}", main.display()));
   let lock = root.join("Cargo.lock");
   if lock.is_file() {
      let copied = dir.join("Cargo.lock");
      fs::copy(&lock, &copied).unwrap_or_else(|error| panic!("{}: {error}", copied.display()));
   }
   // Writing into a `String` cannot fail.
   // 往 `String` 里写不会失败。
   let _ = writeln!(cases, "{}\t{code}", cargo_toml.display());
}

/// Every fenced block whose info string starts with `rust`, in document order,
/// each with the line number of its opening fence.
/// 所有 info string 以 `rust` 开头的围栏代码块，按出现顺序，并带上其起始围栏的行号。
fn rust_blocks(text: &str) -> Vec<(usize, String)> {
   let mut blocks = Vec::new();
   let mut open: Option<usize> = None;
   let mut body = String::new();
   for (index, line) in text.lines().enumerate() {
      match open {
         Some(start) if line.trim() == "```" => {
            blocks.push((start + 1, std::mem::take(&mut body)));
            open = None;
         }
         Some(_) => {
            body.push_str(line);
            body.push('\n');
         }
         None if line.trim_start().starts_with("```rust") => open = Some(index),
         None => {}
      }
   }
   blocks
}

/// The imports a program block carries, i.e. everything before its `fn main`.
/// 程序块携带的 import，即它 `fn main` 之前的全部内容。
fn imports(program: &str) -> &str {
   let end = program
      .lines()
      .position(|line| line.starts_with("fn main"))
      .map_or(program.len(), |at| {
         program.lines().take(at).map(|line| line.len() + 1).sum()
      });
   program.get(..end).map_or("", str::trim_end)
}

/// One module holding one block, plus the setup it needs.
/// 一个模块装一个代码块，外加它需要的上下文。
fn compile(
   guide: &str,
   line: usize,
   index: usize,
   block: &str,
   prelude: &str,
   matched: &mut [bool],
) -> String {
   let first = block.lines().find(|text| !text.trim().is_empty());
   let context = first.map_or("", |text| lookup(text.trim(), matched));
   let wrapped = !block.lines().any(|text| text.starts_with("fn main"));
   // A module that brings its own imports keeps them: a fragment taken from outside
   // the program may need names the program never imports, and forcing the program's
   // imports in alongside its own would collide.
   // 自带 import 的模块保留自己的：不取自程序的片段可能需要程序从未 import 过的名字，而把
   // 程序那套硬塞进去会和它自己的撞名。
   let own_imports = context.lines().any(|text| text.starts_with("use "))
      || block.lines().any(|text| text.starts_with("use "));
   let prelude = if own_imports { "" } else { prelude };
   let mut out = format!("// {guide}:{line} (block {index})\nmod block_{index} {{\n");
   out.push_str("#![allow(unused)]\n");
   if wrapped {
      out.push_str("fn main() {\n");
   }
   if !prelude.is_empty() {
      out.push_str(prelude);
      out.push('\n');
   }
   if !context.is_empty() {
      out.push_str(context);
      out.push('\n');
   }
   out.push_str(block);
   if wrapped {
      out.push_str("}\n");
   }
   out.push_str("}\n\n");
   out
}

/// The setup for a fragment whose first line is `key`, remembering the hit.
/// 首行为 `key` 的片段所需的上下文，并记下这次命中。
fn lookup<'a>(key: &str, matched: &'a mut [bool]) -> &'a str {
   for (slot, (candidate, context)) in matched.iter_mut().zip(CONTEXTS) {
      if *candidate == key {
         *slot = true;
         return context;
      }
   }
   ""
}

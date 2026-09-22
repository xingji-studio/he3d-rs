//! Every counterexample registered in `build.rs` must fail to compile, with the error
//! code recorded beside it.
//! `build.rs` 里登记过的每个反例都必须编译不过，且错误码与登记的一致。

use std::path::{Path, PathBuf};
use std::process::Command;

#[test]
fn counterexamples_stay_broken() {
   let cases = include_str!(concat!(env!("OUT_DIR"), "/compile_fail_cases.tsv"));
   let out_dir = Path::new(env!("OUT_DIR"));
   // A target directory of its own: the outer build holds the workspace one, so a
   // nested cargo pointed at the same directory would wait on a lock nobody releases.
   // 单独一份 target 目录：外层构建占着工作区那一份，嵌套的 cargo 若指向同一个目录，等的
   // 是一把没人会放的锁。
   let target = out_dir.join("../../../../compile-fail-target");
   let cargo = option_env!("CARGO").unwrap_or("cargo");
   let mut checked = 0;
   for case in cases.lines() {
      let mut fields = case.split('\t');
      let (Some(manifest), Some(code)) = (fields.next(), fields.next()) else {
         continue;
      };
      let output = Command::new(cargo)
         .args(["check", "--offline", "--quiet", "--manifest-path"])
         .arg(manifest)
         .env("CARGO_TARGET_DIR", &target)
         .env_remove("CARGO_MAKEFLAGS")
         .env_remove("CARGO_ENCODED_RUSTFLAGS")
         .output()
         .expect("run cargo for a counterexample");
      let stderr = String::from_utf8_lossy(&output.stderr);
      let dir = PathBuf::from(manifest);
      let name = dir.parent().map_or_else(
         || manifest.to_string(),
         |parent| parent.display().to_string(),
      );
      assert!(
         !output.status.success(),
         "a counterexample compiled, but it must not: {name}"
      );
      assert!(
         stderr.contains(code),
         "a counterexample did not raise {code}: {name}\n{stderr}"
      );
      checked += 1;
   }
   assert!(checked > 0, "no counterexample was checked");
}

# Agent instructions — he3d-rs

Hard constraints for any human or agent working in this repository. These are
not preferences. Do not re-litigate them, do not "just this once" around them.

---

## 1. `he3d-cpp` is off-limits

**Never align with, port from, mirror, or reference `he3d-cpp`** — not in code,
not in comments, not in docs, not in commit messages, not now, not later.

`he3d-cpp` is a mess. It is not the specification for this project and its
behavior is not a reason for anything here. If a decision in this repo was
justified by "matches he3d-cpp", that justification is void: restate it on its
own terms, or re-derive the decision.

This also means: do not describe this project as "the Rust edition of he3d-cpp"
or as a "port". HE3D (Rust) stands on its own.

### The one exception — asset licensing

Attribution in `THIRD_PARTY_NOTICES.md`, `examples/assets/ATTRIBUTION.md`, and the
Attributions section of `README.md` is **license text, not a design reference**, and
it stays as long as the corresponding asset does. Do not strip it as part of a
de-referencing sweep; if the asset's provenance changes, rewrite the notice then.

---

## 2. `gp3d` must not come back

`gp3d` is gone: `vendor/gp3d/`, the `gp3d` dependency, and every reference to it
were deleted when `he3d-physics` was rewritten. Do not reintroduce it, and do not
copy, adapt, or take inspiration from it. It has deep defects that are not worth
fixing, and fixing it was already attempted and abandoned — it is not a task
waiting for a fresh attempt.

One consequence worth remembering: **never depend on a yanked crates.io version
again.** `gp3d` 0.1.0 and 0.2.0 were both yanked, which is what made the whole
physics crate unbuildable for anyone with a fresh `Cargo.lock`.

---

## 3. Physics was rewritten; its invariants are load-bearing

`crates/he3d-physics` was rebuilt from scratch against `docs/physics-spec.md`. The
old `ObjectId`-adapter shape is gone for good. The reasoning lives in `docs/adr/`
and the vocabulary in `CONTEXT.md`; read those before changing the crate.

What must not regress:

- Physics depends only on `he3d-math`. It never sees a `Scene`, an `ObjectId`, or a
  `Transform` (ADR 0001).
- Bodies are identified by `BodyId`, and the application owns both the
  Body-to-render-object association and the per-frame copy of the pose. Do not add
  a binding layer to the engine; that decision was deliberate.
- Static geometry stays out of the body broadphase and welds into one shared
  topology, so a Body cannot catch on a seam (ADR 0003).
- Contact phases, warm starting, and Island sleep are load-bearing, not
  decoration. Sleeping is per Island so that disturbing one member wakes all of
  them (ADR 0004).

---

## 4. The binary chooses the backend; no feature does

A backend is a type that implements `Platform` from `he3d-platform` — `SdlBackend`,
`StdBackend`, or one written outside this repository. The **binary** names that
type, depends on that backend's crate, and constructs it. See ADR 0005.

What must not regress:

- `he3d` depends on no backend crate, optional or otherwise, and has no feature
  that selects one. Do not bring back `create_backend`, `PlatformFactory`, or a
  "try `sdl3`, then `std`" fallback. The facade re-exports the `Platform` trait
  and the types around it; that is the whole of its knowledge about backends.
- Do not reintroduce a Cargo feature whose meaning is "which backend". Features
  are additive across the dependency graph, so such a feature is decided by
  whichever crate happens to enable it rather than by the application.
- `he3d-platform` states the contract and nothing else: no registry, no lookup by
  name, no constructor that hands back a trait object of the engine's choosing.
- The binary does not own the clock either, so `time_seconds` and `sleep_ms` stay
  on the trait: a `no_std` or wasm target has no portable clock or sleep, and only
  the backend knows what it has.

---

## 5. One comment line, one sentence

A sentence in a comment occupies exactly one line. Do not wrap a sentence onto a
second line, and do not put two sentences on one line.

Comments in this repository are bilingual, English first and Chinese second. The
Chinese line is a translation of the English one rather than an independent remark,
so the two blocks have the same sentences in the same order.

This rule governs comments: Rust `//` and `///`, `#` in manifests and scripts, and
the same in any other language this repository grows. Markdown documents
(`README.md`, `CHANGELOG.md`, `docs/`) are prose and keep their normal wrapping.

`rustfmt.toml` sets `wrap_comments = false`, so `cargo fmt` will not rewrap a comment
and undo this. A long sentence stays long on purpose.

The comments already in the tree predate this rule and are wrapped. Unwrap a file's
comments when you next touch that file; do not sweep the tree for it.

---

## Repository facts worth knowing

- **The build was once silently broken by a corrupted dependency cache.** A tool
  deleted `Vec3::ONE` / `Vec3::NEG_ONE` from the *unpacked* `glam` source under
  `~/.cargo/registry/src`. cargo verifies the archive it downloads, never the
  files it unpacks. If a build error points at a dependency's source, run
  `scripts/check-deps.sh` before debugging anything else.
- **Keep `~/.cargo/registry/src` read-only.** Editors and IDEs will rewrite
  dependency sources in place. See `scripts/check-deps.sh` for the repair path.
- **`cargo fmt --all` also formats local path dependencies**, so a crate that wants
  a different style needs its own `rustfmt.toml`.
- **Every library crate is publishable again.** Deleting the `gp3d` dependency
  removed the `publish = false` path dependency that used to block
  `he3d-physics`; publishing now only needs the usual dependency order from
  `README.md`.
- **A dependency on the SDL3 backend is a system requirement, not a code
  requirement.** `sdl3-sys` declares `links = "SDL3"`, so merely depending on
  `he3d-platform-sdl3` makes cargo look for SDL3 at build time. Anything expected
  to build headlessly must keep it out of its dependency graph.
- **Benchmarks live in `crates/he3d-bench`, and they are `publish = false`.** That
  crate is the only place criterion appears, and it depends on no backend crate, so
  it stays headless. Do not move benchmark targets into the library crates: that
  would drag criterion into their dependency graphs. CI runs the suite with
  `cargo bench -p he3d-bench -- --test` (smoke only), never a full timing run.

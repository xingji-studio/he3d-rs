//! HE3D benchmarks and heap accounting.
//!
//! This crate is internal tooling (`publish = false`); it is where the engine's performance claims are measured rather than asserted.
//! It holds three things:
//!
//! * [`workloads`] — builders for the physics scenes, render scenes, and asset bytes the benchmarks run against, shared by the criterion benches and the `memreport` binary.
//! * [`alloc`] — a counting global allocator that reports how many allocations a scenario makes and how much is live at its peak.
//! * six criterion targets under `benches/` — `physics`, `renderer`, `frame`, `assets`, `core_math`, and `query`.
//!
//! # Running
//!
//! ```sh
//! cargo bench -p he3d-bench              # full criterion run
//! cargo bench -p he3d-bench -- --test    # smoke: each case runs once
//! cargo run  -p he3d-bench --bin memreport   # heap + size_of tables
//! ```
//!
//! # Why a separate crate
//!
//! Benchmarks pull in criterion, and criterion must not appear anywhere in the library dependency graphs — the engine crates stay `no_std + alloc` and headless, and nothing here depends on `he3d-platform-sdl3`.
//! A dedicated crate keeps that wall intact.
//!
//! # Measuring active physics scenes
//!
//! A freshly built scene has no contacts yet: contact count climbs for several steps as Bodies settle into each other, so a step taken at step 0 is far cheaper than one taken in the developed, fully-awake state.
//! And criterion repeats a closure against the same value, so a long `b.iter(|| world.step(dt))` drifts toward sleep and reports a mix of awake and resting steps.
//! The physics benchmarks therefore use `iter_batched`: the build and some untimed warm-up steps are setup, and only the single following step is timed.
//! See the module comment in `benches/physics.rs`.

// Benchmark and measurement code: pedantic style lints (casts, punctuation in names, long format strings) cost more noise here than they buy, and none of this ships.
// Correctness lints (`all`) stay on.
#![allow(clippy::pedantic)]

pub mod alloc;
pub mod workloads;

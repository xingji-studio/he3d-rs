//! Cost as a function of step index, for scenes that take a long time to settle.
//!
//! Run with `cargo run -p he3d-bench --bin settle --release`.
//!
//! A criterion case answers "what does one step cost right now". For a scene that
//! ramps up and then decays — anything dropped into contact, where penetration
//! deepens before positional correction pushes it apart — that single number is
//! ambiguous, and it is the *peak* rather than what a running scene pays. This
//! binary prints the whole trajectory instead:
//!
//! * how much one step costs, every hundred steps;
//! * how many Contacts exist at that point; and
//! * how many Bodies have gone to sleep.
//!
//! The sleep column is the one that matters most. A settled pile is supposed to
//! fall asleep and cost almost nothing; a pile that never does pays for every
//! contact forever. `settle` is how "does this scene ever sleep" is answered rather
//! than assumed.
//!
//! ```text
//! cargo run -p he3d-bench --bin settle --release               # summary of the reference piles
//! cargo run -p he3d-bench --bin settle --release -- 500 1500 12 # one pile, every hundred steps
//! ```

#![allow(clippy::pedantic)]

use std::hint::black_box;
use std::time::Instant;

use he3d_bench::workloads;
use he3d_math::Vec3;

/// Pile sizes the summary reports: six, eight and twelve layers.
const REFERENCE_PILES: [usize; 3] = [200, 500, 2000];

/// Steps the summary runs each pile for, at 1/120 s.
const SUMMARY_STEPS: usize = 1500;

/// Steps between trajectory rows.
const REPORT_EVERY: usize = 100;

fn main() {
   let arguments: Vec<usize> = std::env::args()
      .skip(1)
      .filter_map(|value| value.parse().ok())
      .collect();

   match arguments.as_slice() {
      [] => summary(),
      [bodies, steps, iterations] => {
         trajectory(*bodies, *steps, u32::try_from(*iterations).unwrap_or(12));
      }
      _ => {
         eprintln!("usage: settle [bodies steps iterations]");
         std::process::exit(2);
      }
   }
}

/// One pile, every hundred steps: the shape of the cost curve.
fn trajectory(bodies: usize, steps: usize, iterations: u32) {
   let mut world = workloads::world_lattice_on_slab(bodies);
   world.set_velocity_iterations(iterations);

   println!(
      "lattice_on_slab/{bodies}: {layers} layers, {iterations} velocity iterations",
      layers = workloads::lattice_layers(bodies),
   );
   println!(
      "{:>6} {:>10} {:>10} {:>9}",
      "step", "ms", "contacts", "asleep"
   );
   println!("{}", "-".repeat(38));

   let mut report_at = 0;
   for step in 1..=steps {
      let millis = time_step(&mut world);
      if step > report_at || step == steps {
         println!(
            "{step:>6} {millis:>10.2} {:>10} {:>9}",
            world.contacts().len(),
            workloads::sleeping_count(&world),
         );
         report_at = step + REPORT_EVERY;
      }
   }

   let residual = residual_motion(&world);
   println!(
      "{:<24} max|v| {:.4} m/s   max|w| {:.4} rad/s   over threshold {}/{}",
      "residual motion",
      residual.0,
      residual.1,
      residual.2,
      world.len(),
   );
}

/// One line per reference pile: peak cost, settled cost, and whether it sleeps.
fn summary() {
   println!(
      "lattice_on_slab, 1/120 s step, {} steps per pile",
      SUMMARY_STEPS
   );
   println!(
      "{:>7} {:>7} {:>11} {:>11} {:>11} {:>9}",
      "bodies", "layers", "peak_ms", "settled_ms", "asleep", "max|v|"
   );
   println!("{}", "-".repeat(60));

   for &bodies in &REFERENCE_PILES {
      let mut world = workloads::world_lattice_on_slab(bodies);
      let mut peak: f64 = 0.0;
      let mut recent: Vec<f64> = Vec::with_capacity(REPORT_EVERY);
      for _ in 0..SUMMARY_STEPS {
         let millis = time_step(&mut world);
         peak = peak.max(millis);
         recent.push(millis);
         if recent.len() > REPORT_EVERY {
            recent.remove(0);
         }
      }
      recent.sort_by(f64::total_cmp);
      let settled = recent[recent.len() / 2];
      let asleep = workloads::sleeping_count(&world);
      let residual = residual_motion(&world);
      println!(
         "{bodies:>7} {:>7} {peak:>11.2} {settled:>11.2} {asleep:>11} {:>9.4}",
         workloads::lattice_layers(bodies),
         residual.0,
      );
   }
}

/// Seconds to run one step.
fn time_step(world: &mut he3d_physics::World) -> f64 {
   let start = Instant::now();
   world.step(black_box(workloads::STEP));
   start.elapsed().as_secs_f64() * 1e3
}

/// The largest linear and angular speed anywhere, and how many Bodies exceed the
/// sleep threshold on either. A pile that is over the threshold is a pile that
/// cannot sleep.
fn residual_motion(world: &he3d_physics::World) -> (f32, f32, usize) {
   let mut max_linear: f32 = 0.0;
   let mut max_angular: f32 = 0.0;
   let mut over = 0;

   for id in world.ids() {
      let linear = world.velocity(id).unwrap_or(Vec3::ZERO).length();
      let angular = world.angular_velocity(id).unwrap_or(Vec3::ZERO).length();
      max_linear = max_linear.max(linear);
      max_angular = max_angular.max(angular);
      if linear > he3d_physics::SLEEP_LINEAR_THRESHOLD
         || angular > he3d_physics::SLEEP_ANGULAR_THRESHOLD
      {
         over += 1;
      }
   }

   (max_linear, max_angular, over)
}

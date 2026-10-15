//! A counting global allocator for the `memreport` binary.
//!
//! [`CountingAlloc`] forwards every request to the system allocator and tallies
//! what passes through: how many calls, how many bytes were requested in total,
//! how many bytes are live, and the high-water mark of live bytes.
//!
//! Install it only where the numbers are wanted — the `memreport` binary does —
//! and leave the timing benchmarks on the plain allocator. The atomics are cheap
//! but not free, and a timing run should not pay for accounting it never reads.
//!
//! # What "bytes" means
//!
//! The tally counts the *requested* size from each [`Layout`], the same number the
//! allocator is asked for. It does not include the allocator's own bookkeeping
//! (size-class rounding, headers, alignment padding), so it is a lower bound on the
//! real resident cost and an exact count of what the engine asked for. That is the
//! useful number when the question is "how much does this type actually hold".

use core::sync::atomic::{AtomicUsize, Ordering};
use std::alloc::{GlobalAlloc, Layout, System};

static ALLOC_CALLS: AtomicUsize = AtomicUsize::new(0);
static REALLOC_CALLS: AtomicUsize = AtomicUsize::new(0);
static DEALLOC_CALLS: AtomicUsize = AtomicUsize::new(0);
static ALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);
static PEAK_BYTES: AtomicUsize = AtomicUsize::new(0);

/// A snapshot of the heap counters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AllocStats {
   /// Number of `alloc`/`alloc_zeroed` calls.
   pub alloc_calls: usize,
   /// Number of `realloc` calls that succeeded.
   pub realloc_calls: usize,
   /// Number of `dealloc` calls.
   pub dealloc_calls: usize,
   /// Cumulative bytes requested by `alloc` and by `realloc` growth.
   pub alloc_bytes: usize,
   /// Bytes currently live (requested, not resident).
   pub live_bytes: usize,
   /// High-water mark of `live_bytes` since the last [`reset_counters`].
   pub peak_bytes: usize,
}

/// Read the current counters.
#[must_use]
pub fn stats() -> AllocStats {
   AllocStats {
      alloc_calls: ALLOC_CALLS.load(Ordering::Relaxed),
      realloc_calls: REALLOC_CALLS.load(Ordering::Relaxed),
      dealloc_calls: DEALLOC_CALLS.load(Ordering::Relaxed),
      alloc_bytes: ALLOC_BYTES.load(Ordering::Relaxed),
      live_bytes: LIVE_BYTES.load(Ordering::Relaxed),
      peak_bytes: PEAK_BYTES.load(Ordering::Relaxed),
   }
}

/// Zero the call/byte counters and re-baseline the peak to the current live size.
///
/// `live_bytes` is left alone: it describes real memory in use, not a measurement
/// window. Call this immediately before the work being measured, then read
/// [`stats`] after it; `peak_bytes` then reports the high-water mark *during* the
/// window (because it was re-baselined to the live size at its start).
pub fn reset_counters() {
   ALLOC_CALLS.store(0, Ordering::Relaxed);
   REALLOC_CALLS.store(0, Ordering::Relaxed);
   DEALLOC_CALLS.store(0, Ordering::Relaxed);
   ALLOC_BYTES.store(0, Ordering::Relaxed);
   PEAK_BYTES.store(LIVE_BYTES.load(Ordering::Relaxed), Ordering::Relaxed);
}

/// Add `bytes` to the live total and lift the peak if it is a new high.
fn add_live(bytes: usize) {
   let live = LIVE_BYTES.fetch_add(bytes, Ordering::Relaxed) + bytes;
   PEAK_BYTES.fetch_max(live, Ordering::Relaxed);
}

/// A [`GlobalAlloc`] that tallies each request before delegating to [`System`].
pub struct CountingAlloc;

// SAFETY: every method forwards the caller's pointer and layout to the system
// allocator unchanged. The tallies use only atomic operations and never affect the
// returned pointer, so the allocation contract is exactly `System`'s.
unsafe impl GlobalAlloc for CountingAlloc {
   unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
      // SAFETY: `layout` is the caller's; forwarding it to the system allocator.
      let ptr = unsafe { System.alloc(layout) };
      if !ptr.is_null() {
         ALLOC_CALLS.fetch_add(1, Ordering::Relaxed);
         ALLOC_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
         add_live(layout.size());
      }
      ptr
   }

   unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
      // SAFETY: `layout` is the caller's; forwarding it to the system allocator.
      let ptr = unsafe { System.alloc_zeroed(layout) };
      if !ptr.is_null() {
         ALLOC_CALLS.fetch_add(1, Ordering::Relaxed);
         ALLOC_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
         add_live(layout.size());
      }
      ptr
   }

   unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
      DEALLOC_CALLS.fetch_add(1, Ordering::Relaxed);
      LIVE_BYTES.fetch_sub(layout.size(), Ordering::Relaxed);
      // SAFETY: `ptr` was returned by this allocator with this `layout`.
      unsafe { System.dealloc(ptr, layout) };
   }

   unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
      // SAFETY: `ptr`/`layout` describe a live allocation; `new_size` is the
      // requested new size. Forwarding to the system allocator.
      let new_ptr = unsafe { System.realloc(ptr, layout, new_size) };
      if !new_ptr.is_null() {
         REALLOC_CALLS.fetch_add(1, Ordering::Relaxed);
         let old = layout.size();
         if new_size >= old {
            ALLOC_BYTES.fetch_add(new_size - old, Ordering::Relaxed);
            add_live(new_size - old);
         } else {
            LIVE_BYTES.fetch_sub(old - new_size, Ordering::Relaxed);
         }
      }
      new_ptr
   }
}

use alloc::vec::Vec;

use he3d_math::Vec3;
use he3d_math::scalar::floor;

use crate::aabb::Aabb;

/// Default edge length of a broadphase cell, in world units.
/// 宽阶段单元格的默认边长（世界单位）。
///
/// It should be near the size of a typical Body. Too small and every Body spans
/// many cells; too large and every cell holds many Bodies.
/// 它应接近典型 Body 的尺寸。太小则每个 Body 跨越许多格子；太大则每个格子塞满 Body。
pub const DEFAULT_CELL_SIZE: f32 = 2.0;

/// A Body whose AABB spans more than this many cells leaves the grid for the
/// oversized list.
/// AABB 跨越超过这么多个格子的 Body 会离开网格、进入超大清单。
///
/// Inserting a Body costs one entry per covered cell, which grows with the cube of
/// its span, while the oversized list costs a linear scan of every AABB every step.
/// The cut sits where those two cross: 64 cells is a 4x4x4 span, cheap enough to
/// insert, and anything larger is wide enough that the scan is the better trade.
/// Bodies of ordinary size relative to the cell never reach it; a Body much larger
/// than the cell size falls back deliberately.
/// 插入一个 Body 的花费是每个覆盖格子一条记录，随其跨度立方增长；而超大清单每步都要线性
/// 扫描所有 AABB。这个切点取在两者相交处：64 个格子即 4x4x4 的跨度，插入足够便宜；更大的
/// 则宽到扫描是更划算的交易。相对于格子而言常规尺寸的 Body 永远不会碰到它；远大于格子尺寸
/// 的 Body 则是有意回退。
const MAX_CELLS_PER_BODY: u64 = 64;

/// Cell coordinates are clamped to this range so the float-to-int cast stays
/// exact and distinct positions cannot collide through the hash.
/// 格子坐标被钳制在此范围内，使浮点到整数的转换保持精确，且不同位置不会在哈希中相撞。
const CELL_LIMIT: f32 = 1_048_576.0;

/// One entry in a bucket's singly linked list.
/// 某个桶的单向链表中的一项。
#[derive(Clone, Copy, Debug)]
struct Entry {
   body: u32,
   next: i32,
}

/// A uniform spatial hash over Body bounding boxes.
/// 基于 Body 包围盒的均匀空间哈希。
///
/// Pairs are reported exactly once. Two AABBs that overlap in several cells share
/// a *canonical* cell — the one holding the overlap's minimum corner — and a pair
/// is emitted only while walking that cell, so no pair list needs to be
/// deduplicated or sorted.
/// 每对只上报一次。两个在多个格子中重叠的 AABB 共享一个**规范**格子——即包含重叠区域
/// 最小角的那个——而一对只在该格子被走到时上报，因此不需要对配对列表去重或排序。
#[derive(Clone, Debug, Default)]
pub(crate) struct SpatialGrid {
   inv_cell_size: f32,
   buckets: Vec<i32>,
   mask: usize,
   entries: Vec<Entry>,
   oversized: Vec<u32>,
   oversized_flag: Vec<bool>,
}

impl SpatialGrid {
   /// A grid with the given cell edge length.
   pub(crate) fn new(cell_size: f32) -> Self {
      let cell_size = if cell_size.is_finite() && cell_size > 0.0 {
         cell_size
      } else {
         DEFAULT_CELL_SIZE
      };
      Self {
         inv_cell_size: 1.0 / cell_size,
         buckets: Vec::new(),
         mask: 0,
         entries: Vec::new(),
         oversized: Vec::new(),
         oversized_flag: Vec::new(),
      }
   }

   /// Clear the grid and insert every AABB, in one pass.
   pub(crate) fn rebuild(&mut self, aabbs: &[Aabb]) {
      let wanted = aabbs.len().saturating_mul(2).next_power_of_two().max(64);
      if self.buckets.len() == wanted {
         self.buckets.fill(-1);
      } else {
         self.buckets = alloc::vec![-1; wanted];
         self.mask = wanted - 1;
      }
      self.entries.clear();
      self.oversized.clear();
      self.oversized_flag.clear();
      self.oversized_flag.resize(aabbs.len(), false);

      for (index, aabb) in aabbs.iter().enumerate() {
         let Ok(index) = u32::try_from(index) else {
            break;
         };
         let low = self.cell_of(aabb.min);
         let high = self.cell_of(aabb.max);
         let span = cell_span(low, high);
         if span > MAX_CELLS_PER_BODY {
            self.oversized.push(index);
            self.oversized_flag[index as usize] = true;
            continue;
         }
         for z in low.z..=high.z {
            for y in low.y..=high.y {
               for x in low.x..=high.x {
                  let bucket = bucket_index(x, y, z, self.mask);
                  self.entries.push(Entry {
                     body: index,
                     next: self.buckets[bucket],
                  });
                  self.buckets[bucket] = i32::try_from(self.entries.len() - 1).unwrap_or(-1);
               }
            }
         }
      }
   }

   /// The cell containing a world-space point.
   fn cell_of(&self, point: Vec3) -> Cell {
      Cell {
         x: cell_coord(point.x, self.inv_cell_size),
         y: cell_coord(point.y, self.inv_cell_size),
         z: cell_coord(point.z, self.inv_cell_size),
      }
   }

   /// Report every overlapping pair once, lower index first.
   pub(crate) fn for_each_candidate(&self, aabbs: &[Aabb], mut emit: impl FnMut(u32, u32)) {
      for (index, aabb) in aabbs.iter().enumerate() {
         if self.oversized_flag.get(index).copied().unwrap_or(false) {
            continue;
         }
         let Ok(index) = u32::try_from(index) else {
            break;
         };
         let low = self.cell_of(aabb.min);
         let high = self.cell_of(aabb.max);
         for z in low.z..=high.z {
            for y in low.y..=high.y {
               for x in low.x..=high.x {
                  let cell = Cell { x, y, z };
                  let mut cursor = self.buckets[bucket_index(x, y, z, self.mask)];
                  while cursor >= 0 {
                     let entry = self.entries[usize::try_from(cursor).unwrap_or(0)];
                     cursor = entry.next;
                     if entry.body <= index {
                        continue;
                     }
                     let other = &aabbs[entry.body as usize];
                     if !aabb.overlaps(other) {
                        continue;
                     }
                     // Emit only from the canonical cell, so a pair sharing
                     // several cells is still reported once.
                     // 只在规范格子中上报，因此共享多个格子的一对仍只上报一次。
                     let overlap_min = aabb.min.max(other.min);
                     if self.cell_of(overlap_min) != cell {
                        continue;
                     }
                     emit(index, entry.body);
                  }
               }
            }
         }
      }

      // Oversized AABBs never entered the grid, so test them against everything.
      // 超大 AABB 从未进入网格，因此逐一与所有对象比对。
      for &index in &self.oversized {
         for other_index in 0..aabbs.len() {
            let Ok(other_index) = u32::try_from(other_index) else {
               break;
            };
            if other_index == index {
               continue;
            }
            // Oversized-versus-oversized pairs are emitted once, by the lower
            // index, because this loop would otherwise visit both directions.
            // 超大与超大之间的一对只由较小索引上报，否则本循环会走两个方向。
            if self.oversized_flag[other_index as usize] && other_index < index {
               continue;
            }
            let (low, high) = if index < other_index {
               (index, other_index)
            } else {
               (other_index, index)
            };
            if aabbs[low as usize].overlaps(&aabbs[high as usize]) {
               emit(low, high);
            }
         }
      }
   }

   /// How many Bodies were kept out of the grid.
   #[cfg(test)]
   pub(crate) fn oversized_count(&self) -> usize {
      self.oversized.len()
   }
}

/// An integer cell coordinate.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Cell {
   x: i32,
   y: i32,
   z: i32,
}

/// How many cells a range covers.
fn cell_span(low: Cell, high: Cell) -> u64 {
   let span = |a: i32, b: i32| u64::try_from(i64::from(b) - i64::from(a) + 1).unwrap_or(0);
   span(low.x, high.x)
      .saturating_mul(span(low.y, high.y))
      .saturating_mul(span(low.z, high.z))
}

/// Floor a world coordinate to a cell index, clamped so the cast is exact.
#[expect(clippy::cast_possible_truncation)]
fn cell_coord(value: f32, inv_cell_size: f32) -> i32 {
   let scaled = value * inv_cell_size;
   if !scaled.is_finite() {
      return 0;
   }
   floor(scaled).clamp(-CELL_LIMIT, CELL_LIMIT) as i32
}

/// Mix three cell coordinates into a bucket index.
///
/// Each axis is multiplied by its own large odd salt and the three results are
/// combined with exclusive-or. The salts are what make this work, not any particular
/// pedigree:
/// multiplying one 32-bit word by an odd constant is a bijection on the low bits,
/// so it spreads an axis's low bits upward, and giving each axis a *different* salt
/// stops the three from cancelling when the coordinates are equal — which is the
/// common case for an axis-aligned lattice. Correctness never depends on the
/// distribution: bucket chains plus the canonical-cell check keep pair reporting
/// exact, so a poor mix costs chain walking and nothing else.
/// 把三个格子坐标混合成一个桶索引。
/// 每个轴乘以各自的大奇数盐，三个结果再做异或。真正起作用的是这些盐本身，而不是任何出处：
/// 把 32 位字乘以奇常数在低位上是双射，因此会把该轴的低位向上扩散；而给每个轴**不同**的
/// 盐，则避免三个坐标相等时互相抵消——对轴对齐的格子而言那正是常见情形。正确性从不依赖
/// 分布：桶链加上规范格子检查保证成对报告精确，因此混合得差只会多走链表，别的没有。
fn bucket_index(x: i32, y: i32, z: i32, mask: usize) -> usize {
   let mix = |value: i32, salt: u32| u32::from_ne_bytes(value.to_ne_bytes()).wrapping_mul(salt);
   let hashed = mix(x, 73_856_093) ^ mix(y, 19_349_663) ^ mix(z, 83_492_791);
   // `usize::from` has no `u32` impl for portability reasons; every target this
   // crate builds for has a usize of at least 32 bits.
   // `usize::from` 出于可移植性考虑没有 `u32` 实现；本 crate 构建的所有目标其 usize 都
   // 至少有 32 位。
   usize::try_from(hashed).unwrap_or(0) & mask
}

#[cfg(test)]
// Test fixtures turn small loop counters into world coordinates; the values sit
// far inside the range f32 represents exactly.
// 测试夹具把小整数循环计数转换为世界坐标；这些值远在 f32 能精确表示的范围内。
#[allow(clippy::cast_precision_loss)]
mod tests {
   use super::{DEFAULT_CELL_SIZE, SpatialGrid, cell_coord};
   use crate::aabb::Aabb;
   use alloc::vec::Vec;
   use he3d_math::Vec3;

   fn box_at(x: f32, half: f32) -> Aabb {
      Aabb::from_center_half_extents(Vec3::new(x, 0.0, 0.0), Vec3::splat(half))
   }

   fn pairs(aabbs: &[Aabb]) -> Vec<(u32, u32)> {
      let mut grid = SpatialGrid::new(DEFAULT_CELL_SIZE);
      grid.rebuild(aabbs);
      let mut found = Vec::new();
      grid.for_each_candidate(aabbs, |a, b| found.push((a, b)));
      found.sort_unstable();
      found
   }

   #[test]
   fn overlapping_boxes_are_reported_exactly_once() {
      let aabbs = [box_at(0.0, 1.0), box_at(1.0, 1.0)];
      assert_eq!(pairs(&aabbs), alloc::vec![(0, 1)]);
   }

   #[test]
   fn a_pair_sharing_many_cells_is_still_reported_once() {
      // Two large, deeply overlapping boxes span many cells together.
      let aabbs = [box_at(0.0, 5.0), box_at(0.0, 5.0)];
      assert_eq!(pairs(&aabbs), alloc::vec![(0, 1)]);
   }

   #[test]
   fn separated_boxes_are_not_reported() {
      let aabbs = [box_at(0.0, 0.5), box_at(100.0, 0.5)];
      assert!(pairs(&aabbs).is_empty());
   }

   #[test]
   fn touching_boxes_are_reported() {
      let aabbs = [box_at(0.0, 1.0), box_at(2.0, 1.0)];
      assert_eq!(pairs(&aabbs), alloc::vec![(0, 1)]);
   }

   #[test]
   fn three_overlapping_boxes_yield_every_pair_once() {
      let aabbs = [box_at(0.0, 1.0), box_at(0.5, 1.0), box_at(1.0, 1.0)];
      assert_eq!(pairs(&aabbs), alloc::vec![(0, 1), (0, 2), (1, 2)]);
   }

   #[test]
   fn a_huge_box_falls_back_to_the_oversized_list() {
      let aabbs = [box_at(0.0, 1000.0), box_at(10.0, 0.5), box_at(5000.0, 0.5)];
      let mut grid = SpatialGrid::new(DEFAULT_CELL_SIZE);
      grid.rebuild(&aabbs);
      assert_eq!(grid.oversized_count(), 1);
      // It still pairs with what it actually overlaps, and only that.
      assert_eq!(pairs(&aabbs), alloc::vec![(0, 1)]);
   }

   #[test]
   fn two_huge_boxes_pair_once() {
      let aabbs = [box_at(0.0, 1000.0), box_at(100.0, 1000.0)];
      assert_eq!(pairs(&aabbs), alloc::vec![(0, 1)]);
   }

   #[test]
   fn cell_coordinates_are_clamped_not_wrapped() {
      let far = cell_coord(1e30, 0.5);
      let opposite = cell_coord(-1e30, 0.5);
      assert_ne!(far, opposite);
      assert!(far > 0);
      assert!(opposite < 0);
      assert_eq!(cell_coord(f32::NAN, 0.5), 0);
   }

   #[test]
   fn an_empty_grid_reports_nothing() {
      assert!(pairs(&[]).is_empty());
   }

   #[test]
   fn many_scattered_boxes_report_only_real_overlaps() {
      let mut aabbs = Vec::new();
      for index in 0..200 {
         aabbs.push(box_at(index as f32 * 4.0, 0.5));
      }
      // Not touching: centres are 4 apart with half extent 0.5.
      assert!(pairs(&aabbs).is_empty());
   }
}

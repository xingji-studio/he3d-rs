use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use he3d_math::Vec3;

use crate::aabb::Aabb;

/// A stable handle to a piece of Static geometry registered in a
/// [`crate::World`].
/// 指向 [`crate::World`] 中某块 Static geometry 的稳定句柄。
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct StaticMeshId {
   index: u32,
   generation: u32,
}

impl StaticMeshId {
   /// Mint a handle. Only a World may do this.
   /// 铸造句柄。只有 World 可以这样做。
   pub(crate) const fn new(index: u32, generation: u32) -> Self {
      Self { index, generation }
   }

   /// The slot this handle addresses within its World.
   /// 此句柄在其 World 中寻址的槽位。
   #[must_use]
   pub const fn index(self) -> u32 {
      self.index
   }

   /// The slot generation this handle was minted against.
   /// 铸造此句柄时所对应的槽位代际。
   #[must_use]
   pub const fn generation(self) -> u32 {
      self.generation
   }
}

/// One registered triangle soup.
/// 一块已注册的三角形汤。
#[derive(Clone, Debug)]
pub(crate) struct StaticMesh {
   /// Flat vertex positions; every three consecutive entries form one triangle.
   /// 扁平顶点位置；每三个连续条目构成一个三角形。
   pub(crate) positions: Vec<Vec3>,
   /// A bounding box per triangle, so a Body only tests what it can reach.
   /// 每个三角形一个包围盒，使 Body 只测试它够得着的部分。
   pub(crate) triangle_bounds: Vec<Aabb>,
   /// Welded vertex ids, three per triangle.
   /// 焊接后的顶点 id，每个三角形三个。
   pub(crate) welded: Vec<[u32; 3]>,
   /// The mesh's own bounding box.
   /// 该网格自身的包围盒。
   pub(crate) bounds: Aabb,
}

impl StaticMesh {
   /// How many triangles the mesh holds.
   pub(crate) fn triangle_count(&self) -> usize {
      self.triangle_bounds.len()
   }

   /// The vertices of one triangle.
   pub(crate) fn triangle(&self, index: usize) -> [Vec3; 3] {
      let base = index * 3;
      [
         self.positions[base],
         self.positions[base + 1],
         self.positions[base + 2],
      ]
   }
}

/// The shared topology of every registered Static mesh in one World.
/// 一个 World 中所有已注册 Static mesh 的共享拓扑。
///
/// Vertices are welded by exact position across **all** meshes, so an edge lying
/// between two separate meshes is still recognised as an Internal edge. That is
/// what keeps a Body sliding across streamed terrain from catching on the seam
/// between two tiles — per-mesh welding would leave every tile boundary as a real
/// edge.
/// 顶点按精确位置在**所有** mesh 之间焊接，因此位于两个独立 mesh 之间的边仍被识别为
/// Internal edge。这正是让划过流式地形的 Body 不会卡在瓦片接缝上的原因——逐 mesh 焊接
/// 会让每条瓦片边界都成为真实的边。
#[derive(Clone, Debug, Default)]
pub(crate) struct StaticTopology {
   /// Exact position bits to welded vertex id.
   welded: BTreeMap<[u32; 3], u32>,
   /// How many live triangle corners use each welded vertex.
   vertex_use: BTreeMap<u32, u32>,
   /// How many triangles use each welded edge, keyed by the ordered vertex pair.
   edge_use: BTreeMap<(u32, u32), u32>,
   /// The next unused welded vertex id. Ids are never reused.
   next_vertex: u32,
}

impl StaticTopology {
   /// Weld one position and return its vertex id.
   pub(crate) fn weld(&mut self, position: Vec3) -> u32 {
      let key = position_key(position);
      if let Some(&id) = self.welded.get(&key) {
         return id;
      }
      let id = self.next_vertex;
      self.next_vertex = self.next_vertex.wrapping_add(1);
      self.welded.insert(key, id);
      id
   }

   /// Register one triangle's welded corners, counting its edges.
   pub(crate) fn add_triangle(&mut self, welded: [u32; 3]) {
      for vertex in welded {
         *self.vertex_use.entry(vertex).or_insert(0) += 1;
      }
      for edge in edges_of(welded) {
         *self.edge_use.entry(edge).or_insert(0) += 1;
      }
   }

   /// Release one triangle's welded corners, dropping what nothing else uses.
   pub(crate) fn remove_triangle(&mut self, welded: [u32; 3]) {
      for vertex in welded {
         if let Some(count) = self.vertex_use.get_mut(&vertex) {
            *count = count.saturating_sub(1);
            if *count == 0 {
               self.vertex_use.remove(&vertex);
            }
         }
      }
      for edge in edges_of(welded) {
         if let Some(count) = self.edge_use.get_mut(&edge) {
            *count = count.saturating_sub(1);
            if *count == 0 {
               self.edge_use.remove(&edge);
            }
         }
      }
   }

   /// Whether an edge between two welded vertices is shared by two triangles of
   /// the shared topology, and is therefore an Internal edge.
   ///
   /// A Contact whose normal comes from such an edge is spurious: the two
   /// triangles meet smoothly there, so there is no real edge for a Body to catch
   /// on.
   /// 两个焊接顶点之间的边是否被共享拓扑中的两个三角形共用，因而是一条 Internal edge。
   /// 法线来自这种边的 Contact 是伪接触：两个三角形在那里平滑相接，并不存在可供 Body 卡住
   /// 的真实棱。
   pub(crate) fn is_internal_edge(&self, first: u32, second: u32) -> bool {
      let key = if first <= second {
         (first, second)
      } else {
         (second, first)
      };
      self.edge_use.get(&key).copied().unwrap_or(0) >= 2
   }

   /// How many distinct welded vertices the topology holds.
   #[cfg(test)]
   pub(crate) fn vertex_count(&self) -> usize {
      self.welded.len()
   }
}

/// The three edges of a triangle, each ordered so the two directions agree.
fn edges_of(welded: [u32; 3]) -> [(u32, u32); 3] {
   let ordered = |first: u32, second: u32| {
      if first <= second {
         (first, second)
      } else {
         (second, first)
      }
   };
   [
      ordered(welded[0], welded[1]),
      ordered(welded[1], welded[2]),
      ordered(welded[2], welded[0]),
   ]
}

/// The exact bit pattern of a position, with negative zero folded onto zero so
/// that the two spellings of the same point weld together.
fn position_key(position: Vec3) -> [u32; 3] {
   let canonical = |value: f32| {
      if value == 0.0 {
         0.0_f32.to_bits()
      } else {
         value.to_bits()
      }
   };
   [
      canonical(position.x),
      canonical(position.y),
      canonical(position.z),
   ]
}

#[cfg(test)]
mod tests {
   use super::StaticTopology;
   use he3d_math::Vec3;

   #[test]
   fn identical_positions_weld_to_one_vertex() {
      let mut topology = StaticTopology::default();
      let first = topology.weld(Vec3::new(1.0, 2.0, 3.0));
      let second = topology.weld(Vec3::new(1.0, 2.0, 3.0));
      assert_eq!(first, second);
      assert_eq!(topology.vertex_count(), 1);
   }

   #[test]
   fn negative_zero_welds_with_positive_zero() {
      let mut topology = StaticTopology::default();
      assert_eq!(
         topology.weld(Vec3::new(0.0, 0.0, 0.0)),
         topology.weld(Vec3::new(-0.0, -0.0, -0.0))
      );
   }

   #[test]
   fn an_edge_shared_by_two_triangles_is_internal() {
      let mut topology = StaticTopology::default();
      // Two triangles of a quad sharing the diagonal (1, 2).
      topology.add_triangle([0, 1, 2]);
      assert!(
         !topology.is_internal_edge(1, 2),
         "one triangle is a boundary"
      );
      topology.add_triangle([1, 3, 2]);
      assert!(topology.is_internal_edge(1, 2));
      assert!(!topology.is_internal_edge(0, 1));
   }

   #[test]
   fn edges_weld_across_separate_meshes() {
      let mut topology = StaticTopology::default();
      // A mesh on the left of a seam.
      topology.add_triangle([0, 1, 2]);
      // A mesh on the right re-uses the same welded vertices along the seam.
      topology.add_triangle([1, 3, 2]);
      assert!(
         topology.is_internal_edge(2, 1),
         "the order of the pair must not matter"
      );
   }

   #[test]
   fn removing_a_triangle_releases_its_edges() {
      let mut topology = StaticTopology::default();
      topology.add_triangle([0, 1, 2]);
      topology.add_triangle([1, 3, 2]);
      assert!(topology.is_internal_edge(1, 2));
      topology.remove_triangle([1, 3, 2]);
      assert!(!topology.is_internal_edge(1, 2));
   }

   #[test]
   fn welding_is_stable_across_add_and_remove() {
      let mut topology = StaticTopology::default();
      let id = topology.weld(Vec3::X);
      topology.add_triangle([id, id, id]);
      topology.remove_triangle([id, id, id]);
      // The id survives, because welded ids are never reused or renumbered.
      assert_eq!(topology.weld(Vec3::X), id);
      assert_eq!(topology.vertex_count(), 1);
   }
}

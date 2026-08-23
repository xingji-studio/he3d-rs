use alloc::vec::Vec;
use core::marker::PhantomData;

/// A stable generational handle into an [`Arena<T>`].
/// 指向 [`Arena<T>`] 的稳定代际句柄。
///
/// An `Id` is two packed integers (slot index + generation) and never stores a reference, so it can outlive the arena without dangling.
/// `Id` 由两个打包的整数（槽位索引 + 代际）组成，且从不存储引用，因此可以比 arena 存活更久而不会悬垂。
/// Once a slot is removed, its generation advances, invalidating all previously issued `Id`s.
/// 一旦槽位被移除，其代际就会递增，从而使所有先前发出的 `Id` 失效。
///
/// The `T` parameter is phantom: it exists only to make IDs of different arenas distinct types at compile time.
/// `T` 参数是幻影的：它仅用于让不同 arena 的 ID 在编译期成为不同的类型。
/// Trait impls are written by hand so that `Id<T>` is always `Copy`/`Eq`/`Hash` regardless of `T`.
/// 特质实现均为手写，以便无论 `T` 为何，`Id<T>` 始终是 `Copy`/`Eq`/`Hash`。
pub struct Id<T> {
   index: u32,
   generation: u32,
   _marker: PhantomData<fn() -> T>,
}

impl<T> Id<T> {
   #[inline]
   pub fn index(self) -> u32 {
      self.index
   }

   #[inline]
   pub fn generation(self) -> u32 {
      self.generation
   }
}

impl<T> Clone for Id<T> {
   #[inline]
   fn clone(&self) -> Self {
      *self
   }
}

impl<T> Copy for Id<T> {}

impl<T> PartialEq for Id<T> {
   #[inline]
   fn eq(&self, other: &Self) -> bool {
      self.index == other.index && self.generation == other.generation
   }
}

impl<T> Eq for Id<T> {}

impl<T> PartialOrd for Id<T> {
   #[inline]
   fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
      Some(self.cmp(other))
   }
}

impl<T> Ord for Id<T> {
   #[inline]
   fn cmp(&self, other: &Self) -> core::cmp::Ordering {
      (self.index, self.generation).cmp(&(other.index, other.generation))
   }
}

impl<T> core::hash::Hash for Id<T> {
   #[inline]
   fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
      core::hash::Hash::hash(&self.index, state);
      core::hash::Hash::hash(&self.generation, state);
   }
}

impl<T> core::fmt::Debug for Id<T> {
   fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
      write!(f, "Id({}@g{})", self.index, self.generation)
   }
}

#[derive(Clone, Debug)]
struct Slot<T> {
   generation: u32,
   value: Option<T>,
}

/// A generational arena: dense storage with reuse of freed slots and stale-handle detection.
/// 代际 arena：密集存储，支持空闲槽位复用与过期句柄检测。
///
/// This is the ownership primitive behind meshes, textures, and render objects.
/// 这是网格、纹理和渲染对象背后的所有权原语。
/// It is intentionally a small self-contained type so it can be swapped for an external slot map without touching call sites.
/// 它刻意做成一个小而自包含的类型，以便可以在不改动调用方的情况下替换为外部的槽位映射。
#[derive(Clone, Debug)]
pub struct Arena<T> {
   slots: Vec<Slot<T>>,
   free: Vec<u32>,
   len: usize,
}

impl<T> Arena<T> {
   pub fn new() -> Self {
      Self {
         slots: Vec::new(),
         free: Vec::new(),
         len: 0,
      }
   }

   pub fn with_capacity(capacity: usize) -> Self {
      Self {
         slots: Vec::with_capacity(capacity),
         free: Vec::new(),
         len: 0,
      }
   }

   /// Insert a value and return its handle.
   /// 插入一个值并返回其句柄。
   pub fn insert(&mut self, value: T) -> Id<T> {
      self.len += 1;
      if let Some(index) = self.free.pop() {
         let slot = &mut self.slots[index as usize];
         slot.generation = slot.generation.wrapping_add(1);
         slot.value = Some(value);
         return Id {
            index,
            generation: slot.generation,
            _marker: PhantomData,
         };
      }

      let index = u32::try_from(self.slots.len()).expect("arena index space exhausted");
      self.slots.push(Slot {
         generation: 0,
         value: Some(value),
      });
      Id {
         index,
         generation: 0,
         _marker: PhantomData,
      }
   }

   pub fn get(&self, id: Id<T>) -> Option<&T> {
      let slot = self.slots.get(id.index as usize)?;
      if slot.generation != id.generation {
         return None;
      }
      slot.value.as_ref()
   }

   pub fn get_mut(&mut self, id: Id<T>) -> Option<&mut T> {
      let slot = self.slots.get_mut(id.index as usize)?;
      if slot.generation != id.generation {
         return None;
      }
      slot.value.as_mut()
   }

   pub fn contains(&self, id: Id<T>) -> bool {
      self.get(id).is_some()
   }

   /// Remove a value, invalidating the handle.
   /// 移除一个值，并使该句柄失效。
   pub fn remove(&mut self, id: Id<T>) -> Option<T> {
      let slot = self.slots.get_mut(id.index as usize)?;
      if slot.generation != id.generation {
         return None;
      }
      let value = slot.value.take()?;
      slot.generation = slot.generation.wrapping_add(1);
      self.free.push(id.index);
      self.len -= 1;
      Some(value)
   }

   pub fn len(&self) -> usize {
      self.len
   }

   pub fn is_empty(&self) -> bool {
      self.len == 0
   }

   /// Iterate over all live `(id, value)` pairs in slot order.
   /// 按槽位顺序遍历所有存活的 `(id, value)` 对。
   pub fn iter(&self) -> impl Iterator<Item = (Id<T>, &T)> {
      self.slots.iter().enumerate().filter_map(|(index, slot)| {
         let value = slot.value.as_ref()?;
         let index = u32::try_from(index).ok()?;
         Some((
            Id {
               index,
               generation: slot.generation,
               _marker: PhantomData,
            },
            value,
         ))
      })
   }
}

impl<T> Default for Arena<T> {
   fn default() -> Self {
      Self::new()
   }
}

#[cfg(test)]
mod tests {
   use alloc::vec::Vec;

   use super::Arena;

   #[test]
   fn insert_and_get() {
      let mut arena = Arena::new();
      let id = arena.insert(42u32);
      assert_eq!(arena.get(id), Some(&42));
      assert!(arena.contains(id));
      assert_eq!(arena.len(), 1);
   }

   #[test]
   fn removed_id_becomes_stale() {
      let mut arena = Arena::new();
      let id = arena.insert(7u32);
      assert_eq!(arena.remove(id), Some(7));
      assert!(!arena.contains(id));
      assert_eq!(arena.get(id), None);
      assert!(arena.is_empty());
   }

   #[test]
   fn reused_slot_invalidates_previous_handle() {
      let mut arena = Arena::new();
      let first = arena.insert(1u32);
      arena.remove(first);
      let second = arena.insert(2u32);
      // Same slot index, but the generation advanced.
      // 槽位索引相同，但代际已经递增。
      assert_eq!(first.index(), second.index());
      assert_ne!(first, second);
      assert_eq!(arena.get(first), None);
      assert_eq!(arena.get(second), Some(&2));
   }

   #[test]
   fn iter_yields_live_pairs() {
      let mut arena = Arena::new();
      let a = arena.insert("a");
      let b = arena.insert("b");
      let c = arena.insert("c");
      arena.remove(b);
      let mut seen: Vec<&str> = arena.iter().map(|(_, v)| *v).collect();
      seen.sort_unstable();
      assert_eq!(seen, ["a", "c"]);
      assert!(arena.contains(a) && arena.contains(c));
   }
}

use crate::body::Body;

/// The slot value that stands for "the other side is Static geometry".
/// 代表"另一侧是 Static geometry"的槽位取值。
///
/// Static geometry has no Body of its own, so a constraint against it names this
/// instead of a slot. The solver then treats that side as infinitely massive,
/// which is exactly what immovable geometry is.
/// Static geometry 本身没有 Body，因此与它相对的约束用这个值代替槽位。求解器随后把该侧
/// 当作无穷大质量处理——这正是不可移动几何的含义。
pub(crate) const STATIC_PARTNER: u32 = u32::MAX;

/// One slot in the World's body storage.
/// World 刚体存储中的一个槽位。
///
/// A slot keeps its generation across removals, so a [`crate::BodyId`] minted
/// before a removal can never address the Body that later reuses the slot.
/// 槽位在移除后保留其代际，因此在移除之前铸造的 [`crate::BodyId`] 永远无法指向后来复用
/// 该槽位的 Body。
#[derive(Clone, Copy, Debug)]
pub(crate) struct Slot {
   /// The generation currently occupying this slot.
   pub(crate) generation: u32,
   /// The Body, or `None` while the slot is free.
   pub(crate) body: Option<Body>,
}

impl Slot {
   /// The Body, if the slot is occupied.
   pub(crate) fn get(&self) -> Option<&Body> {
      self.body.as_ref()
   }

   /// The Body, if the slot is occupied.
   pub(crate) fn get_mut(&mut self) -> Option<&mut Body> {
      self.body.as_mut()
   }
}

/// Borrow the Body on `first` and, when there is one, the Body on `second`.
///
/// A constraint against Static geometry has no second Body; the second element of
/// the returned pair is then `None`.
/// 可变地借用 `first` 上的 Body，以及（若存在）`second` 上的 Body。与 Static geometry
/// 相对的约束没有第二个 Body，此时返回对的第二个元素为 `None`。
pub(crate) fn constraint_bodies(
   slots: &mut [Slot],
   first: usize,
   second: usize,
) -> Option<(&mut Body, Option<&mut Body>)> {
   if second == STATIC_PARTNER as usize {
      let body = slots.get_mut(first)?.get_mut()?;
      return Some((body, None));
   }
   if first == second {
      return None;
   }
   let (low, high) = if first < second {
      (first, second)
   } else {
      (second, first)
   };
   if high >= slots.len() {
      return None;
   }
   let (left, right) = slots.split_at_mut(high);
   let high_body = right.first_mut()?.get_mut()?;
   let low_body = left.get_mut(low)?.get_mut()?;
   if first < second {
      Some((low_body, Some(high_body)))
   } else {
      Some((high_body, Some(low_body)))
   }
}

/// The read-only counterpart of [`constraint_bodies`].
/// [`constraint_bodies`] 的只读对应物。
pub(crate) fn constraint_bodies_ref(
   slots: &[Slot],
   first: usize,
   second: usize,
) -> Option<(&Body, Option<&Body>)> {
   let first_body = slots.get(first)?.get()?;
   if second == STATIC_PARTNER as usize {
      return Some((first_body, None));
   }
   let second_body = slots.get(second)?.get()?;
   Some((first_body, Some(second_body)))
}

#[cfg(test)]
mod tests {
   use super::{STATIC_PARTNER, Slot, constraint_bodies};
   use crate::body::BodyDesc;
   use crate::shape::Shape;

   fn slots(count: usize) -> alloc::vec::Vec<Slot> {
      (0..count)
         .map(|_| Slot {
            generation: 0,
            body: Some(crate::body::Body::from_desc(&BodyDesc::dynamic(
               Shape::sphere(0.5),
               1.0,
            ))),
         })
         .collect()
   }

   #[test]
   fn constraint_bodies_returns_them_in_the_requested_order() {
      let mut storage = slots(3);
      let (first, second) = constraint_bodies(&mut storage, 0, 2).expect("both exist");
      first.linear_velocity.x = 1.0;
      second.expect("second exists").linear_velocity.x = 2.0;
      let (reversed_first, _) = constraint_bodies(&mut storage, 2, 0).expect("both exist");
      assert!((reversed_first.linear_velocity.x - 2.0).abs() < 1e-6);
   }

   #[test]
   fn a_static_partner_yields_a_single_body() {
      let mut storage = slots(1);
      let (first, second) =
         constraint_bodies(&mut storage, 0, STATIC_PARTNER as usize).expect("the body exists");
      assert!(second.is_none(), "immovable geometry has no Body");
      first.linear_velocity.x = 3.0;
      assert!((storage[0].get().expect("body").linear_velocity.x - 3.0).abs() < 1e-6);
   }

   #[test]
   fn constraint_bodies_rejects_self_and_missing_slots() {
      let mut storage = slots(2);
      assert!(constraint_bodies(&mut storage, 1, 1).is_none());
      assert!(constraint_bodies(&mut storage, 0, 9).is_none());
      storage[1].body = None;
      assert!(constraint_bodies(&mut storage, 0, 1).is_none());
   }
}

use he3d_math::scalar::sqrt;
use he3d_math::{Quat, Vec3};

use crate::contact::{MAX_MANIFOLD_POINTS, Manifold, ManifoldPoint};

/// The maximum number of vertices a clipped polygon can have.
/// 裁剪后的多边形最多拥有的顶点数。
///
/// Clipping a convex `n`-gon by one half-plane yields at most `n + 1` vertices, and
/// the largest thing ever clipped here is a four-corner box face against four side
/// planes, so eight is the exact bound rather than a guess. It matters because
/// [`Polygon::push`] silently drops anything past it: a bound that was too small
/// would cost contact points and let a Body sink, with nothing to report it.
/// 用一个半平面裁剪凸 `n` 边形最多得到 `n + 1` 个顶点，而这里被裁剪的最大对象是四角的盒
/// 面被四个侧面裁剪，因此 8 是精确上界而非估计。这一点重要，是因为 [`Polygon::push`] 会
/// 静默丢弃超出上限的部分：上界过小会丢失接触点、让 Body 下沉，而且没有任何地方会报告它。
const MAX_POLYGON_POINTS: usize = 8;

/// Handicap applied to a box's edge axis when both shapes are boxes.
///
/// The axis contest takes the smallest least translation, so an inflation is a
/// deliberate handicap: a resting face contact must not lose to a nearly-parallel
/// cross product whose least translation happens to be a hair smaller. Faces carry
/// no handicap here because a face axis is the trustworthy one.
/// 当两个形状都是盒时，施加于盒棱轴的劣势系数。
/// 轴选择取最小的最小平移量，因此放大是一种刻意的劣势：静止的面接触绝不能输给一个几乎
/// 平行的叉积——后者的最小平移量恰好小了一丁点。这里面对轴不加劣势，因为面轴才是可信的。
const BOX_BOX_EDGE_BIAS: f32 = 1.02;

/// Handicap applied to a box's face axis in the box-versus-triangle contest.
///
/// This contest is only reached when the box does *not* straddle the triangle's
/// plane, because the straddling case returns the triangle's face outright. What is
/// left is a boundary hit, where the box's own face is the more trusted axis — more
/// trusted than an edge, but still handicapped so it cannot outrank the triangle's
/// face if that ever re-enters the contest.
/// 盒对三角形时施加于盒面轴的劣势系数。
/// 只有在盒**没有**跨过三角形所在平面时才会走到这个选择，因为跨过的情况会直接返回三角形
/// 的面。剩下的是一次边界命中，此时盒自身的面是更可信的轴——比棱可信，但仍带劣势，以免在
/// 三角形面重新参与选择时压过它。
const BOX_TRIANGLE_FACE_BIAS: f32 = 1.02;

/// Handicap applied to a box's edge axis in the box-versus-triangle contest.
///
/// Larger than [`BOX_TRIANGLE_FACE_BIAS`] because an edge axis comes from crossing
/// two nearly-parallel directions and is the least trustworthy of the three.
/// 盒对三角形时施加于盒棱轴的劣势系数。
/// 比 [`BOX_TRIANGLE_FACE_BIAS`] 更大，因为棱轴来自两个几乎平行方向的叉积，是三者中最
/// 不可信的。
const BOX_TRIANGLE_EDGE_BIAS: f32 = 1.05;

/// Absolute slack added to every handicapped axis.
///
/// It is a world length added to a world length, so it is not scale-invariant: for a
/// Body much smaller than this the slack exceeds the overlaps being compared and the
/// axis choice becomes arbitrary. At ordinary scene scale it only breaks ties.
/// 加到每个带劣势轴上的绝对余量。
/// 它是长度加到长度上，因此不具尺度不变性：对远小于此值的 Body，该余量会超过被比较的重叠
/// 量，轴的选择就变成任意的。在常规场景尺度下它只用来打破平局。
const AXIS_BIAS_SLACK: f32 = 1e-4;

/// A clipped vertex this far outside the reference face still counts as touching.
///
/// Without the tolerance a point whose separation hovers around zero blinks in and
/// out of the manifold between steps, which throws away the warm-start impulse
/// every time it disappears and leaves a resting stack permanently jittering.
/// 超出参考面不超过此距离的裁剪顶点仍算作接触。若没有这个容差，分离量在零附近波动的点会
/// 在步与步之间反复出现与消失，每次消失都会丢掉 warm-start 冲量，使静止堆叠永远抖动。
const CONTACT_TOLERANCE: f32 = 1e-3;

/// How close to an endpoint a clipping crossing may land before it counts as one.
/// 裁剪交点距端点多近时就算作落在端点上。
const CLIP_ENDPOINT_MARGIN: f32 = 1e-4;

/// Two clipped vertices closer than this are the same contact point.
/// 两个裁剪后顶点距离小于此值时视为同一个接触点。
const VERTEX_WELD_DISTANCE: f32 = 1e-4;

/// Side planes are pushed out by this much so that two exactly aligned faces —
/// where the incident corners land precisely on the reference face's edges — are
/// kept rather than decided by floating-point noise.
/// 侧面外扩此距离，使两个完全对齐的面（入射角点恰好落在参考面的边上）被保留，而不是由
/// 浮点噪声决定去留。
const CLIP_PLANE_MARGIN: f32 = 1e-4;

/// A box's world-space frame: where it is and which way its local axes point.
/// 盒的世界空间标架：位置，以及局部各轴指向何处。
#[derive(Clone, Copy, Debug)]
pub(crate) struct BoxFrame {
   center: Vec3,
   axes: [Vec3; 3],
   half: Vec3,
}

impl BoxFrame {
   /// Build the frame of a box placed at `center` with `orientation`.
   pub(crate) fn new(center: Vec3, orientation: Quat, half: Vec3) -> Self {
      Self {
         center,
         axes: [
            orientation * Vec3::X,
            orientation * Vec3::Y,
            orientation * Vec3::Z,
         ],
         half,
      }
   }

   /// Half the box's extent when projected onto a unit `axis`.
   ///
   /// This is `sum_j |dot(axis, local_axis_j)| * half_j`, which is exactly the SAT
   /// projection radius of a box.
   /// 这是 `sum_j |dot(axis, local_axis_j)| * half_j`，正是长方体在 SAT 中的投影半径。
   fn radius(&self, axis: Vec3) -> f32 {
      self.axes[0].dot(axis).abs() * self.half.x
         + self.axes[1].dot(axis).abs() * self.half.y
         + self.axes[2].dot(axis).abs() * self.half.z
   }

   /// The four corners of the face whose outward normal is `sign * axes[axis]`,
   /// wound consistently around the face.
   /// 外法线为 `sign * axes[axis]` 的那个面的四个角点，按一致方向绕面排列。
   fn face(&self, axis: usize, sign: f32) -> [Vec3; 4] {
      let normal = axis_of(self.axes, axis) * sign;
      let face_center = self.center + normal * component(self.half, axis);
      let first = axis_of(self.axes, axis + 1) * component(self.half, axis + 1);
      let second = axis_of(self.axes, axis + 2) * component(self.half, axis + 2);
      [
         face_center + first + second,
         face_center - first + second,
         face_center - first - second,
         face_center + first - second,
      ]
   }

   /// The corner furthest along `direction`.
   fn support(&self, direction: Vec3) -> Vec3 {
      self.center
         + self.axes[0] * (self.half.x * sign_of(self.axes[0].dot(direction)))
         + self.axes[1] * (self.half.y * sign_of(self.axes[1].dot(direction)))
         + self.axes[2] * (self.half.z * sign_of(self.axes[2].dot(direction)))
   }

   /// The two endpoints of the edge through `corner` that runs along `axis`.
   fn edge_through(&self, corner: Vec3, axis: usize) -> [Vec3; 2] {
      let direction = axis_of(self.axes, axis);
      let signed_extent = (corner - self.center).dot(direction);
      [corner, corner - direction * (2.0 * signed_extent)]
   }
}

/// A small fixed-capacity polygon used while clipping.
/// 裁剪过程中使用的小型定容多边形。
#[derive(Clone, Copy, Debug)]
struct Polygon {
   points: [Vec3; MAX_POLYGON_POINTS],
   count: usize,
}

impl Polygon {
   fn new(points: &[Vec3]) -> Self {
      let mut polygon = Self {
         points: [Vec3::ZERO; MAX_POLYGON_POINTS],
         count: 0,
      };
      for &point in points {
         polygon.push(point);
      }
      polygon
   }

   fn push(&mut self, point: Vec3) {
      if self.count < MAX_POLYGON_POINTS {
         self.points[self.count] = point;
         self.count += 1;
      }
   }

   fn as_slice(&self) -> &[Vec3] {
      &self.points[..self.count]
   }
}

/// Clip `polygon` against the half-space `dot(normal, p) <= offset`.
///
/// This is Sutherland-Hodgman with a single plane: keep the vertices inside, and
/// insert a new vertex wherever an edge crosses. A crossing exactly at an endpoint
/// is not inserted, because a duplicated vertex becomes a duplicated contact point
/// — which doubles that point's effective mass and makes a resting stack drift.
/// 对单个平面做 Sutherland-Hodgman 裁剪：保留内侧顶点，并在边跨越平面处插入新顶点。
/// 恰好发生在端点上的跨越不会被插入，因为重复的顶点会变成重复的接触点——那会使该点的
/// 有效质量翻倍，让静止堆叠漂移。
fn clip_polygon(polygon: &Polygon, normal: Vec3, offset: f32) -> Polygon {
   let mut output = Polygon {
      points: [Vec3::ZERO; MAX_POLYGON_POINTS],
      count: 0,
   };
   let source = polygon.as_slice();
   if source.is_empty() {
      return output;
   }
   for index in 0..source.len() {
      let current = source[index];
      let next = source[(index + 1) % source.len()];
      let current_distance = normal.dot(current) - offset;
      let next_distance = normal.dot(next) - offset;
      let current_inside = current_distance <= 0.0;
      if current_inside {
         output.push(current);
      }
      if current_inside != (next_distance <= 0.0) {
         let span = current_distance - next_distance;
         if span.abs() > f32::EPSILON {
            let t = current_distance / span;
            // Skip crossings that land on an endpoint; those vertices are already
            // being emitted on one side or the other.
            // 跳过落在端点上的跨越；那些顶点已经在某一侧被输出。
            if t > CLIP_ENDPOINT_MARGIN && t < 1.0 - CLIP_ENDPOINT_MARGIN {
               output.push(current + (next - current) * t);
            }
         }
      }
   }
   output
}

/// Which SAT axis produced the minimum overlap.
/// 产生最小重叠的那个 SAT 轴。
#[derive(Clone, Copy, Debug)]
enum SatAxis {
   /// A face of the first box.
   FaceA(usize),
   /// A face of the second box.
   FaceB(usize),
   /// The cross product of one axis from each box.
   Edge(usize, usize),
}

/// The axis of least penetration found so far.
#[derive(Clone, Copy, Debug)]
struct Best {
   overlap: f32,
   axis: Option<SatAxis>,
}

/// The result of testing one separating axis.
///
/// A face axis wins ties against an edge axis, because face contact is the
/// trustworthy case and edge axes are the numerically fragile one.
/// 面轴在与棱轴打平时胜出，因为面接触是可信情形，而棱轴在数值上最脆弱。
enum AxisOutcome {
   Separated,
   Overlapping {
      overlap: f32,
      axis: SatAxis,
      /// Whether this axis beats what has been seen so far.
      preferred: bool,
   },
}

/// Test one candidate separating axis.
fn test_axis(
   a: &BoxFrame,
   b: &BoxFrame,
   delta: Vec3,
   axis: Vec3,
   candidate: SatAxis,
   is_edge: bool,
   best_overlap: f32,
) -> AxisOutcome {
   let overlap = a.radius(axis) + b.radius(axis) - delta.dot(axis).abs();
   if overlap <= 0.0 {
      return AxisOutcome::Separated;
   }
   // Edge axes are biased slightly upward so that a resting face contact is not
   // stolen by a nearly-parallel cross product.
   // 棱轴被略微上调，以免静止的面接触被一个几乎平行的叉积抢走。
   let biased = if is_edge {
      overlap * BOX_BOX_EDGE_BIAS + AXIS_BIAS_SLACK
   } else {
      overlap
   };
   AxisOutcome::Overlapping {
      overlap,
      axis: candidate,
      preferred: biased < best_overlap,
   }
}

/// Generate the contact manifold between two boxes.
fn box_box(a: &BoxFrame, b: &BoxFrame) -> Option<Manifold> {
   let delta = b.center - a.center;
   let mut best = Best {
      overlap: f32::MAX,
      axis: None,
   };

   for index in 0..3 {
      match test_axis(
         a,
         b,
         delta,
         a.axes[index],
         SatAxis::FaceA(index),
         false,
         best.overlap,
      ) {
         AxisOutcome::Separated => return None,
         AxisOutcome::Overlapping {
            overlap,
            axis,
            preferred,
         } => {
            if preferred {
               best = Best {
                  overlap,
                  axis: Some(axis),
               };
            }
         }
      }
   }
   for index in 0..3 {
      match test_axis(
         a,
         b,
         delta,
         b.axes[index],
         SatAxis::FaceB(index),
         false,
         best.overlap,
      ) {
         AxisOutcome::Separated => return None,
         AxisOutcome::Overlapping {
            overlap,
            axis,
            preferred,
         } => {
            if preferred {
               best = Best {
                  overlap,
                  axis: Some(axis),
               };
            }
         }
      }
   }
   for i in 0..3 {
      for j in 0..3 {
         let raw = a.axes[i].cross(b.axes[j]);
         let length = raw.length();
         if length < 1e-5 {
            continue;
         }
         match test_axis(
            a,
            b,
            delta,
            raw / length,
            SatAxis::Edge(i, j),
            true,
            best.overlap,
         ) {
            AxisOutcome::Separated => return None,
            AxisOutcome::Overlapping {
               overlap,
               axis,
               preferred,
            } => {
               if preferred {
                  best = Best {
                     overlap,
                     axis: Some(axis),
                  };
               }
            }
         }
      }
   }

   let axis = best.axis?;
   Some(build_manifold(a, b, delta, axis))
}

/// Turn the least-penetrating axis into contact points.
fn build_manifold(a: &BoxFrame, b: &BoxFrame, delta: Vec3, axis: SatAxis) -> Manifold {
   match axis {
      SatAxis::FaceA(index) => {
         let sign = sign_of(delta.dot(a.axes[index]));
         face_manifold(a, index, sign, b, false)
      }
      SatAxis::FaceB(index) => {
         // Reference the second box's face, then flip the normal so it still
         // points from A toward B.
         // 以第二个盒的面为参考面，随后翻转法线，使其仍由 A 指向 B。
         let sign = sign_of(delta.dot(b.axes[index]));
         face_manifold(b, index, -sign, a, true)
      }
      SatAxis::Edge(i, j) => edge_manifold(a, i, b, j, delta),
   }
}

/// Build a manifold by clipping the incident face against the reference face.
fn face_manifold(
   reference: &BoxFrame,
   reference_axis: usize,
   reference_sign: f32,
   incident: &BoxFrame,
   flip: bool,
) -> Manifold {
   let reference_normal = axis_of(reference.axes, reference_axis) * reference_sign;
   let reference_face = reference.face(reference_axis, reference_sign);

   // The incident face is the one whose outward normal is most anti-parallel to
   // the reference normal. That is the axis with the largest *absolute* alignment,
   // taken against the reference normal — searching the signed alignment instead
   // picks an orthogonal axis whenever two boxes are exactly axis-aligned, which
   // selects a side face and leaves only two contact points.
   // 入射面是其外法线与参考法线最反向的那个面。即 |对齐度| 最大的轴，方向与参考法线相反。
   // 若改用带符号的对齐度搜索，两个完全轴对齐的箱子会选到正交轴，从而选中一张侧面，
   // 只留下两个接触点。
   let mut incident_axis = 0;
   let mut largest = -1.0_f32;
   for index in 0..3 {
      let alignment = axis_of(incident.axes, index).dot(reference_normal).abs();
      if alignment > largest {
         largest = alignment;
         incident_axis = index;
      }
   }
   let incident_sign = -sign_of(axis_of(incident.axes, incident_axis).dot(reference_normal));
   let incident_face = incident.face(incident_axis, incident_sign);

   // Clip the incident face against the four side planes of the reference face.
   let mut polygon = Polygon::new(&incident_face);
   for step in 1..3 {
      let side_axis = (reference_axis + step) % 3;
      let direction = axis_of(reference.axes, side_axis);
      let extent = component(reference.half, side_axis);
      let centre_offset = direction.dot(reference.center);
      polygon = clip_polygon(
         &polygon,
         direction,
         centre_offset + extent + CLIP_PLANE_MARGIN,
      );
      polygon = clip_polygon(
         &polygon,
         -direction,
         -centre_offset + extent + CLIP_PLANE_MARGIN,
      );
   }

   let plane_offset = reference_normal.dot(reference_face[0]);
   let mut collected = [ManifoldPoint::default(); MAX_MANIFOLD_POINTS];
   let mut count = 0;
   for &point in polygon.as_slice() {
      let separation = reference_normal.dot(point) - plane_offset;
      if separation > CONTACT_TOLERANCE || count >= MAX_MANIFOLD_POINTS {
         continue;
      }
      // A duplicated point would carry twice the impulse and make the stack drift.
      // 重复的接触点会承载两倍冲量，使堆叠漂移。
      let duplicate = collected[..count].iter().any(|existing| {
         (existing.position - point).length_squared() < VERTEX_WELD_DISTANCE * VERTEX_WELD_DISTANCE
      });
      if duplicate {
         continue;
      }
      collected[count] = ManifoldPoint {
         position: point,
         depth: (-separation).max(0.0),
         feature: 0,
      };
      count += 1;
   }

   // Number the points by position rather than by clipping order. Which box
   // supplies the reference face flips with numerical noise between steps, and
   // clipping order flips with it — a positional order does not, so warm starting
   // keeps its impulses across the flip.
   // 按位置而非裁剪顺序给点编号。哪一方提供参考面会随数值噪声在步间翻转，裁剪顺序也随之
   // 翻转——位置顺序不会，因此 warm starting 能在翻转中保住自己的冲量。
   collected[..count].sort_unstable_by(|left, right| {
      left
         .position
         .x
         .total_cmp(&right.position.x)
         .then(left.position.z.total_cmp(&right.position.z))
         .then(left.position.y.total_cmp(&right.position.y))
   });

   let mut manifold = Manifold {
      normal: if flip {
         -reference_normal
      } else {
         reference_normal
      },
      ..Manifold::empty()
   };
   for (index, mut point) in collected[..count].iter().copied().enumerate() {
      point.feature = u32::try_from(index).unwrap_or(0);
      manifold.push(point);
   }
   manifold
}

/// Build a single-point manifold for the edge-versus-edge case.
fn edge_manifold(
   a: &BoxFrame,
   axis_a: usize,
   b: &BoxFrame,
   axis_b: usize,
   delta: Vec3,
) -> Manifold {
   let mut normal = a.axes[axis_a].cross(b.axes[axis_b]);
   if normal.length_squared() <= 0.0 {
      return Manifold::empty();
   }
   normal = normal.normalize();
   if normal.dot(delta) < 0.0 {
      normal = -normal;
   }

   let corner_a = a.support(normal);
   let corner_b = b.support(-normal);
   let edge_a = a.edge_through(corner_a, axis_a);
   let edge_b = b.edge_through(corner_b, axis_b);
   let (point_a, point_b) = closest_points(edge_a, edge_b);
   let overlap = a.radius(normal) + b.radius(normal) - delta.dot(normal).abs();

   let mut manifold = Manifold {
      normal,
      ..Manifold::empty()
   };
   if overlap > 0.0 {
      manifold.push(ManifoldPoint {
         position: (point_a + point_b) * 0.5,
         depth: overlap,
         feature: feature_id(1, axis_a, axis_b, 0),
      });
   }
   manifold
}

/// The closest pair of points on two segments.
///
/// The standard clamped-parameter solution; degenerate and parallel cases are
/// handled explicitly rather than by producing a `NaN`.
/// 标准的参数钳制解法；退化与平行情形被显式处理，而不是产生 `NaN`。
fn closest_points(first: [Vec3; 2], second: [Vec3; 2]) -> (Vec3, Vec3) {
   let first_delta = first[1] - first[0];
   let second_delta = second[1] - second[0];
   let between = first[0] - second[0];
   let first_length_squared = first_delta.dot(first_delta);
   let second_length_squared = second_delta.dot(second_delta);
   let second_projection = second_delta.dot(between);

   if first_length_squared <= f32::EPSILON && second_length_squared <= f32::EPSILON {
      return (first[0], second[0]);
   }
   if first_length_squared <= f32::EPSILON {
      let along_second = (second_projection / second_length_squared).clamp(0.0, 1.0);
      return (first[0], second[0] + second_delta * along_second);
   }
   let first_projection = first_delta.dot(between);
   let cross_projection = first_delta.dot(second_delta);
   let denominator =
      first_length_squared * second_length_squared - cross_projection * cross_projection;

   let mut along_first = if denominator.abs() > f32::EPSILON {
      ((cross_projection * second_projection - first_projection * second_length_squared)
         / denominator)
         .clamp(0.0, 1.0)
   } else {
      0.0
   };
   let mut along_second = if second_length_squared <= f32::EPSILON {
      0.0
   } else {
      (cross_projection * along_first + second_projection) / second_length_squared
   };

   // Clamp `along_second`, then re-solve `along_first` against the clamped value so
   // the result stays on both segments instead of only approximating.
   // 钳制 `along_second`，再针对钳制后的值重新求 `along_first`，使结果确实落在两段之上
   // 而非只是近似。
   if along_second < 0.0 {
      along_second = 0.0;
      along_first = (-first_projection / first_length_squared).clamp(0.0, 1.0);
   } else if along_second > 1.0 {
      along_second = 1.0;
      along_first = ((cross_projection - first_projection) / first_length_squared).clamp(0.0, 1.0);
   }
   (
      first[0] + first_delta * along_first,
      second[0] + second_delta * along_second,
   )
}

/// Generate the contact manifold between a sphere and a box.
///
/// `sphere_first` selects which shape the returned normal points away from.
/// `sphere_first` 决定返回的法线背离哪一个形状。
fn sphere_box(
   sphere_center: Vec3,
   sphere_radius: f32,
   box_frame: &BoxFrame,
   sphere_first: bool,
) -> Option<Manifold> {
   let orientation = quat_from_axes(box_frame.axes);
   let local = orientation.inverse() * (sphere_center - box_frame.center);
   let clamped = local.clamp(-box_frame.half, box_frame.half);
   let outside = local - clamped;
   let distance_squared = outside.length_squared();

   let (local_normal, depth) = if distance_squared > f32::EPSILON {
      let distance = sqrt(distance_squared);
      if distance > sphere_radius {
         return None;
      }
      (outside / distance, sphere_radius - distance)
   } else {
      // The sphere's center is inside the box: push out through the nearest face.
      // 球心在盒内部：沿最近的面推出。
      let mut axis = 0;
      let mut best = f32::MAX;
      for index in 0..3 {
         let distance = component(box_frame.half, index) - component(local, index).abs();
         if distance < best {
            best = distance;
            axis = index;
         }
      }
      let sign = sign_of(component(local, axis));
      let mut normal = Vec3::ZERO;
      set_component(&mut normal, axis, sign);
      (normal, sphere_radius + best)
   };

   // `local_normal` is the direction that leaves the box, i.e. from the box toward
   // the sphere, expressed in world space.
   // `local_normal` 是离开盒子的方向，即由盒指向球，已转换到世界空间。
   let outward = box_frame.axes[0] * local_normal.x
      + box_frame.axes[1] * local_normal.y
      + box_frame.axes[2] * local_normal.z;
   // The manifold normal must point from the first shape toward the second.
   // 流形法线必须由第一个形状指向第二个形状。
   let normal = if sphere_first { -outward } else { outward };
   // Halfway through the overlap, measured back from the sphere's surface.
   // 从球面沿重叠区回退一半深度。
   let position = sphere_center - outward * (sphere_radius - depth * 0.5);

   let mut manifold = Manifold {
      normal,
      ..Manifold::empty()
   };
   manifold.push(ManifoldPoint {
      position,
      depth,
      feature: feature_id(2, 0, 0, 0),
   });
   Some(manifold)
}

/// Generate the contact manifold between two spheres.
fn sphere_sphere(center_a: Vec3, radius_a: f32, center_b: Vec3, radius_b: f32) -> Option<Manifold> {
   let delta = center_b - center_a;
   let distance_squared = delta.length_squared();
   let reach = radius_a + radius_b;
   if distance_squared > reach * reach {
      return None;
   }
   let distance = sqrt(distance_squared);
   // Two concentric spheres have no meaningful normal; pick one deterministically.
   // 两个同心球没有有意义的法线；这里确定性地取一条。
   let normal = if distance > 1e-6 {
      delta / distance
   } else {
      Vec3::Y
   };
   let depth = reach - distance;
   let mut manifold = Manifold {
      normal,
      ..Manifold::empty()
   };
   manifold.push(ManifoldPoint {
      position: center_a + normal * (radius_a - depth * 0.5),
      depth,
      feature: feature_id(3, 0, 0, 0),
   });
   Some(manifold)
}

/// Generate the contact manifold between two shapes.
///
/// `shape_a` is the first shape and the returned normal points from it toward
/// `shape_b`. Shapes are taken by value because every one of them is `Copy`.
/// 生成两个形状之间的接触流形。`shape_a` 是第一个形状，返回的法线由它指向 `shape_b`。
/// 形状按值传入，因为它们全都是 `Copy`。
pub(crate) fn collide(
   shape_a: crate::shape::Shape,
   position_a: Vec3,
   orientation_a: Quat,
   shape_b: crate::shape::Shape,
   position_b: Vec3,
   orientation_b: Quat,
) -> Option<Manifold> {
   use crate::shape::Shape;
   match (shape_a, shape_b) {
      (Shape::Sphere { radius: ra }, Shape::Sphere { radius: rb }) => {
         sphere_sphere(position_a, ra, position_b, rb)
      }
      (Shape::Sphere { radius }, Shape::Box { half_extents }) => {
         let frame = BoxFrame::new(position_b, orientation_b, half_extents);
         sphere_box(position_a, radius, &frame, true)
      }
      (Shape::Box { half_extents }, Shape::Sphere { radius }) => {
         let frame = BoxFrame::new(position_a, orientation_a, half_extents);
         sphere_box(position_b, radius, &frame, false)
      }
      (
         Shape::Box {
            half_extents: half_a,
         },
         Shape::Box {
            half_extents: half_b,
         },
      ) => {
         let frame_a = BoxFrame::new(position_a, orientation_a, half_a);
         let frame_b = BoxFrame::new(position_b, orientation_b, half_b);
         box_box(&frame_a, &frame_b)
      }
   }
}

/// Rebuild a rotation from an orthonormal basis of world-space local axes.
fn quat_from_axes(axes: [Vec3; 3]) -> Quat {
   // The basis is orthonormal by construction, so the transpose is the inverse.
   // 该基由构造保证正交归一，因此转置即逆。
   Quat::from_mat3(&he3d_math::glam::Mat3::from_cols(axes[0], axes[1], axes[2]))
}

/// One component of a three-vector, indexed 0..=2.
fn component(vector: Vec3, index: usize) -> f32 {
   match index % 3 {
      0 => vector.x,
      1 => vector.y,
      _ => vector.z,
   }
}

/// A local axis by index, wrapping.
fn axis_of(axes: [Vec3; 3], index: usize) -> Vec3 {
   axes[index % 3]
}

/// Overwrite one component of a three-vector, indexed 0..=2.
fn set_component(vector: &mut Vec3, index: usize, value: f32) {
   match index % 3 {
      0 => vector.x = value,
      1 => vector.y = value,
      _ => vector.z = value,
   }
}

/// `1.0` for a non-negative input, `-1.0` otherwise.
fn sign_of(value: f32) -> f32 {
   if value >= 0.0 { 1.0 } else { -1.0 }
}

/// Pack the identifying indices of a manifold point into one value.
///
/// Warm starting matches points between steps by this value, so every point of a
/// manifold must get a distinct one.
/// Warm starting 通过该值在步与步之间匹配接触点，因此同一流形的每个点都必须得到不同的值。
fn feature_id(kind: u32, first: usize, second: usize, index: usize) -> u32 {
   let first = u32::try_from(first).unwrap_or(0) & 7;
   let second = u32::try_from(second).unwrap_or(0) & 7;
   let index = u32::try_from(index).unwrap_or(0) & 7;
   (kind << 9) | (first << 6) | (second << 3) | index
}

/// A box resting on or pressing into a triangle.
/// 压在三角形上的盒，或与之接触的盒。
#[derive(Clone, Copy, Debug)]
pub(crate) struct TriangleHit {
   /// The contact points, with the normal pointing from the box toward the
   /// triangle.
   /// 接触点，法线由盒指向三角形。
   pub manifold: Manifold,
   /// When the least-penetrating axis was a triangle edge, that edge's index.
   ///
   /// The caller discards the hit when that edge turns out to be an Internal edge.
   /// 当最小穿透轴是一条三角形棱时，该棱的索引。若该棱被判定为 Internal edge，调用方
   /// 会丢弃这次命中。
   pub source_edge: Option<usize>,
}

/// Which axis produced the box-versus-triangle contact.
/// 盒-三角接触由哪条轴产生。
#[derive(Clone, Copy, Debug)]
enum TriangleAxis {
   /// A face of the box.
   BoxFace(usize),
   /// A box axis crossed with a triangle edge, naming the edge and carrying the
   /// unit axis.
   Edge(usize, Vec3),
}

/// Generate the contact manifold between a box and one triangle.
///
/// The triangle is treated as a degenerate convex shape: a two-sided flat face
/// with three adjacent edges. The returned normal points from the box toward the
/// triangle, matching the convention that a Contact's normal runs from the first
/// side to the second.
/// 生成盒与单个三角形之间的接触流形。三角形被视为退化的凸形状：一张双面平面加三条相邻
/// 棱。返回的法线由盒指向三角形，与"Contact 法线由第一侧指向第二侧"的约定一致。
pub(crate) fn box_triangle(box_frame: &BoxFrame, triangle: [Vec3; 3]) -> Option<TriangleHit> {
   let edges = [
      triangle[1] - triangle[0],
      triangle[2] - triangle[1],
      triangle[0] - triangle[2],
   ];
   let raw_normal = edges[0].cross(edges[1]);
   let normal_length = raw_normal.length();
   if normal_length <= 1e-12 {
      // A zero-area triangle has no plane to rest on.
      // 零面积三角形没有可供支撑的平面。
      return None;
   }
   let triangle_normal = raw_normal / normal_length;

   // If the box straddles the triangle's plane, the triangle's own face is the
   // reference, and it wins outright rather than competing on least translation.
   //
   // Competing is wrong here: when a box slides across a triangulated surface its
   // leading face is exactly flush with the boundary of the triangle it is leaving,
   // so a box axis has a least translation of zero. That zero-depth contact would
   // win the axis contest and then cancel the box's sideways velocity, which is
   // precisely the "catching on a seam" a Body must not do.
   // 若盒跨过三角形所在平面，则以三角形自身的面为参考面，并直接取胜，而不参与最小平移量
   // 的竞争。
   // 竞争在这里是错的：当盒滑过三角化表面时，其前侧面恰好与将要离开的那个三角形的边界齐平，
   // 于是某个盒轴的最小平移量为零。这个零深度接触会赢得轴选择，随后抵消盒的侧向速度——而这
   // 正是 Body 绝不该发生的"卡在接缝上"。
   let face_overlap = triangle_overlap(box_frame, &triangle, triangle_normal);
   if face_overlap < 0.0 {
      // The triangle's own normal already separates them.
      // 三角形自身的法线已经将两者分离。
      return None;
   }
   let face_manifold = triangle_face_manifold(box_frame, &triangle, triangle_normal);
   if face_manifold.count > 0 {
      return Some(TriangleHit {
         manifold: face_manifold,
         source_edge: None,
      });
   }
   // The box reaches only the triangle's boundary, so it is an edge or vertex hit.
   // 盒只够得着三角形的边界，因此这是一次棱或顶点命中。

   let mut best_biased = f32::MAX;
   let mut best_overlap = 0.0_f32;
   let mut best = TriangleAxis::BoxFace(0);

   for index in 0..3 {
      let overlap = triangle_overlap(box_frame, &triangle, box_frame.axes[index]);
      if overlap < 0.0 {
         return None;
      }
      let biased = overlap * BOX_TRIANGLE_FACE_BIAS + AXIS_BIAS_SLACK;
      if biased < best_biased {
         best_biased = biased;
         best_overlap = overlap;
         best = TriangleAxis::BoxFace(index);
      }
   }
   for index in 0..3 {
      for (edge_index, edge) in edges.iter().enumerate() {
         let raw = box_frame.axes[index].cross(*edge);
         let length = raw.length();
         if length < 1e-5 {
            continue;
         }
         let axis = raw / length;
         let overlap = triangle_overlap(box_frame, &triangle, axis);
         if overlap < 0.0 {
            return None;
         }
         let biased = overlap * BOX_TRIANGLE_EDGE_BIAS + AXIS_BIAS_SLACK;
         if biased < best_biased {
            best_biased = biased;
            best_overlap = overlap;
            best = TriangleAxis::Edge(edge_index, axis);
         }
      }
   }

   let manifold = match best {
      TriangleAxis::BoxFace(index) => box_face_manifold(box_frame, &triangle, index),
      TriangleAxis::Edge(edge_index, axis) => {
         edge_triangle_manifold(box_frame, &triangle, edge_index, axis, best_overlap)
      }
   };
   Some(TriangleHit {
      manifold,
      source_edge: match best {
         TriangleAxis::Edge(edge_index, _) => Some(edge_index),
         TriangleAxis::BoxFace(_) => None,
      },
   })
}

/// How far apart a box and a triangle are when projected onto a unit `axis`,
/// negated: a positive result is the least translation that would separate them.
///
/// This is the minimum-translation form rather than the length of the interval
/// intersection. The two agree for solid shapes, but a triangle has no thickness:
/// its projection onto its own normal is a single point, where the intersection
/// length collapses to zero and would report every touch as a separation.
/// 盒与三角形投影到单位 `axis` 上时的分离量的相反数：正值即便可将两者分开的最小平移量。
/// 这里用最小平移形式而非区间交集长度。对实心形状两者一致，但三角形没有厚度：它在其自身
/// 法线上的投影是一个点，交集长度会塌缩为零，从而把每次接触都报成分离。
fn triangle_overlap(box_frame: &BoxFrame, triangle: &[Vec3; 3], axis: Vec3) -> f32 {
   let box_center = box_frame.center.dot(axis);
   let box_radius = box_frame.radius(axis);
   let mut triangle_min = f32::MAX;
   let mut triangle_max = f32::MIN;
   for vertex in triangle {
      let projected = vertex.dot(axis);
      triangle_min = triangle_min.min(projected);
      triangle_max = triangle_max.max(projected);
   }
   (box_center + box_radius - triangle_min).min(triangle_max - (box_center - box_radius))
}

/// The three inward half-spaces of a triangle's edges, as `(normal, offset)` pairs
/// keeping `dot(normal, p) <= offset`.
/// 三角形三条边的三个内侧半空间，表示为 `(法线, 偏移)` 对，保持
/// `dot(normal, p) <= offset`。
fn triangle_edge_planes(triangle: &[Vec3; 3], normal: Vec3) -> [(Vec3, f32); 3] {
   let centroid = (triangle[0] + triangle[1] + triangle[2]) / 3.0;
   let mut planes = [(Vec3::ZERO, 0.0); 3];
   for index in 0..3 {
      let first = triangle[index];
      let second = triangle[(index + 1) % 3];
      let raw = normal.cross(second - first);
      let length = raw.length();
      let mut plane_normal = if length > 1e-12 {
         raw / length
      } else {
         Vec3::ZERO
      };
      let mut offset = plane_normal.dot(first);
      // Keep whichever side the triangle's own centroid is on, so the result does
      // not depend on the winding order.
      // 保留三角形自身重心所在的那一侧，因此结果不依赖绕序。
      if plane_normal.dot(centroid) > offset {
         plane_normal = -plane_normal;
         offset = -offset;
      }
      planes[index] = (plane_normal, offset);
   }
   planes
}

/// Collect kept points into a manifold, welding duplicates and numbering the
/// survivors by position so warm-start identity survives a change of reference.
/// 把保留的点收集成流形，焊接重复点，并按位置给存活点编号，使 warm-start 身份在参考面
/// 改变时仍然保持。
fn manifold_from(
   mut points: [ManifoldPoint; MAX_MANIFOLD_POINTS],
   count: usize,
   normal: Vec3,
) -> Manifold {
   let live = &mut points[..count];
   live.sort_unstable_by(|left, right| {
      left
         .position
         .x
         .total_cmp(&right.position.x)
         .then(left.position.z.total_cmp(&right.position.z))
         .then(left.position.y.total_cmp(&right.position.y))
   });
   let mut manifold = Manifold {
      normal,
      ..Manifold::empty()
   };
   for mut point in live.iter().copied() {
      let duplicate = manifold.points[..manifold.count].iter().any(|existing| {
         (existing.position - point.position).length_squared()
            < VERTEX_WELD_DISTANCE * VERTEX_WELD_DISTANCE
      });
      if duplicate {
         continue;
      }
      point.feature = u32::try_from(manifold.count).unwrap_or(0);
      manifold.push(point);
   }
   manifold
}

/// The face contact: the triangle's plane supports the box.
/// 面接触：三角形的平面支撑着盒。
fn triangle_face_manifold(
   box_frame: &BoxFrame,
   triangle: &[Vec3; 3],
   triangle_normal: Vec3,
) -> Manifold {
   let triangle_center = (triangle[0] + triangle[1] + triangle[2]) / 3.0;
   // Point the plane normal from the triangle toward the box, so the manifold
   // normal ends up pointing the other way, from the box toward the triangle.
   // 让平面法线由三角形指向盒，于是流形法线指向相反方向，即由盒指向三角形。
   let up = if triangle_normal.dot(box_frame.center - triangle_center) < 0.0 {
      -triangle_normal
   } else {
      triangle_normal
   };

   // The box's contact face is the one facing the triangle.
   // 盒的接触面是朝向三角形的那一面。
   let mut face_axis = 0;
   let mut largest = -1.0_f32;
   for index in 0..3 {
      let alignment = box_frame.axes[index].dot(up).abs();
      if alignment > largest {
         largest = alignment;
         face_axis = index;
      }
   }
   let face_sign = -sign_of(box_frame.axes[face_axis].dot(up));
   let face = box_frame.face(face_axis, face_sign);

   let mut polygon = Polygon::new(&face);
   for (plane_normal, offset) in triangle_edge_planes(triangle, triangle_normal) {
      if plane_normal.length_squared() > 0.0 {
         polygon = clip_polygon(&polygon, plane_normal, offset);
      }
   }

   let plane_offset = up.dot(triangle[0]);
   let mut collected = [ManifoldPoint::default(); MAX_MANIFOLD_POINTS];
   let mut count = 0;
   for &point in polygon.as_slice() {
      let separation = up.dot(point) - plane_offset;
      if separation > CONTACT_TOLERANCE || count >= MAX_MANIFOLD_POINTS {
         continue;
      }
      collected[count] = ManifoldPoint {
         position: point,
         depth: (-separation).max(0.0),
         feature: 0,
      };
      count += 1;
   }
   manifold_from(collected, count, -up)
}

/// The box's own face is the reference: the triangle presses into it.
/// 以盒自身的面为参考面：三角形压进其中。
fn box_face_manifold(box_frame: &BoxFrame, triangle: &[Vec3; 3], face_axis: usize) -> Manifold {
   let triangle_center = (triangle[0] + triangle[1] + triangle[2]) / 3.0;
   let axis = box_frame.axes[face_axis];
   let sign = sign_of(axis.dot(triangle_center - box_frame.center));
   let reference_normal = axis * sign;
   let reference_face = box_frame.face(face_axis, sign);

   let mut polygon = Polygon::new(triangle);
   for step in 1..3 {
      let side_axis = (face_axis + step) % 3;
      let direction = box_frame.axes[side_axis];
      let extent = component(box_frame.half, side_axis);
      let centre_offset = direction.dot(box_frame.center);
      polygon = clip_polygon(
         &polygon,
         direction,
         centre_offset + extent + CLIP_PLANE_MARGIN,
      );
      polygon = clip_polygon(
         &polygon,
         -direction,
         -centre_offset + extent + CLIP_PLANE_MARGIN,
      );
   }

   let plane_offset = reference_normal.dot(reference_face[0]);
   let mut collected = [ManifoldPoint::default(); MAX_MANIFOLD_POINTS];
   let mut count = 0;
   for &point in polygon.as_slice() {
      let separation = reference_normal.dot(point) - plane_offset;
      if separation > CONTACT_TOLERANCE || count >= MAX_MANIFOLD_POINTS {
         continue;
      }
      collected[count] = ManifoldPoint {
         position: point,
         depth: (-separation).max(0.0),
         feature: 0,
      };
      count += 1;
   }
   manifold_from(collected, count, reference_normal)
}

/// The edge contact: a box edge meets a triangle edge, giving a single point.
/// 棱接触：盒的一条棱与三角形的一条棱相接，产生单个接触点。
fn edge_triangle_manifold(
   box_frame: &BoxFrame,
   triangle: &[Vec3; 3],
   edge_index: usize,
   axis: Vec3,
   overlap: f32,
) -> Manifold {
   let triangle_center = (triangle[0] + triangle[1] + triangle[2]) / 3.0;
   let normal = if axis.dot(triangle_center - box_frame.center) < 0.0 {
      -axis
   } else {
      axis
   };

   // The box edge that runs against the triangle edge is the one on its supporting
   // corner, along the box axis that is least parallel to the triangle edge.
   // 与三角形棱相对的盒棱位于盒的支撑角上，沿与三角形棱最不平行的盒轴延伸。
   let edge_direction = triangle[(edge_index + 1) % 3] - triangle[edge_index];
   let mut best_box_axis = 0;
   let mut best_alignment = f32::MAX;
   for index in 0..3 {
      let alignment = box_frame.axes[index].cross(edge_direction).length();
      if alignment < best_alignment {
         best_alignment = alignment;
         best_box_axis = index;
      }
   }

   let box_corner = box_frame.support(-normal);
   let box_edge = box_frame.edge_through(box_corner, best_box_axis);
   let triangle_edge = [triangle[edge_index], triangle[(edge_index + 1) % 3]];
   let (point_on_box, point_on_triangle) = closest_points(box_edge, triangle_edge);

   let mut manifold = Manifold {
      normal,
      ..Manifold::empty()
   };
   if overlap > 0.0 {
      manifold.push(ManifoldPoint {
         position: (point_on_box + point_on_triangle) * 0.5,
         depth: overlap,
         feature: 0,
      });
   }
   manifold
}

/// Generate the contact manifold between a sphere and one triangle.
///
/// The returned normal points from the sphere toward the triangle.
/// 生成球与单个三角形之间的接触流形。返回的法线由球指向三角形。
pub(crate) fn sphere_triangle(centre: Vec3, radius: f32, triangle: [Vec3; 3]) -> Option<Manifold> {
   let closest = closest_point_on_triangle(centre, triangle[0], triangle[1], triangle[2]);
   let offset = centre - closest;
   let distance_squared = offset.length_squared();
   if distance_squared > radius * radius {
      return None;
   }
   let distance = sqrt(distance_squared);
   // `toward_sphere` points from the triangle out to the sphere, so the manifold
   // normal points the other way.
   // `toward_sphere` 由三角形指向球，因此流形法线指向相反方向。
   let toward_sphere = if distance > 1e-6 {
      offset / distance
   } else {
      let mut normal = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]);
      let length = normal.length();
      if length <= 1e-12 {
         return None;
      }
      normal /= length;
      if normal.dot(centre - triangle[0]) < 0.0 {
         -normal
      } else {
         normal
      }
   };

   let mut manifold = Manifold {
      normal: -toward_sphere,
      ..Manifold::empty()
   };
   manifold.push(ManifoldPoint {
      position: closest,
      depth: radius - distance,
      feature: 0,
   });
   Some(manifold)
}

/// The point of a triangle closest to `point`.
///
/// The standard Voronoi-region walk: the three vertex regions, then the three edge
/// regions, then the interior.
/// 三角形上离 `point` 最近的点。标准的 Voronoi 区域遍历：三个顶点区域、三条棱区域，
/// 最后是内部。
#[expect(clippy::similar_names)]
fn closest_point_on_triangle(point: Vec3, first: Vec3, second: Vec3, third: Vec3) -> Vec3 {
   let edge_ab = second - first;
   let edge_ac = third - first;
   let to_point = point - first;
   let d1 = edge_ab.dot(to_point);
   let d2 = edge_ac.dot(to_point);
   if d1 <= 0.0 && d2 <= 0.0 {
      return first;
   }

   let to_point_b = point - second;
   let d3 = edge_ab.dot(to_point_b);
   let d4 = edge_ac.dot(to_point_b);
   if d3 >= 0.0 && d4 <= d3 {
      return second;
   }

   let vc = d1 * d4 - d3 * d2;
   if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
      let v = d1 / (d1 - d3);
      return first + edge_ab * v;
   }

   let to_point_c = point - third;
   let d5 = edge_ab.dot(to_point_c);
   let d6 = edge_ac.dot(to_point_c);
   if d6 >= 0.0 && d5 <= d6 {
      return third;
   }

   let vb = d5 * d2 - d1 * d6;
   if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
      let w = d2 / (d2 - d6);
      return first + edge_ac * w;
   }

   let va = d3 * d6 - d5 * d4;
   if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
      let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      return second + (third - second) * w;
   }

   let denominator = va + vb + vc;
   if denominator.abs() <= f32::EPSILON {
      return first;
   }
   let v = vb / denominator;
   let w = vc / denominator;
   first + edge_ab * v + edge_ac * w
}

#[cfg(test)]
mod tests {
   use super::{BoxFrame, collide};
   use crate::shape::Shape;
   use he3d_math::{Quat, Vec3};

   fn boxed(half: Vec3) -> Shape {
      Shape::cuboid(half)
   }

   #[test]
   fn spheres_that_are_apart_do_not_collide() {
      let result = collide(
         Shape::sphere(0.5),
         Vec3::ZERO,
         Quat::IDENTITY,
         Shape::sphere(0.5),
         Vec3::new(2.0, 0.0, 0.0),
         Quat::IDENTITY,
      );
      assert!(result.is_none());
   }

   #[test]
   fn overlapping_spheres_report_depth_and_normal() {
      let manifold = collide(
         Shape::sphere(0.5),
         Vec3::ZERO,
         Quat::IDENTITY,
         Shape::sphere(0.5),
         Vec3::new(0.8, 0.0, 0.0),
         Quat::IDENTITY,
      )
      .expect("spheres overlap");
      assert_eq!(manifold.count, 1);
      assert!((manifold.normal.x - 1.0).abs() < 1e-5);
      assert!((manifold.points[0].depth - 0.2).abs() < 1e-5);
   }

   #[test]
   fn touching_spheres_report_zero_depth() {
      let manifold = collide(
         Shape::sphere(0.5),
         Vec3::ZERO,
         Quat::IDENTITY,
         Shape::sphere(0.5),
         Vec3::new(1.0, 0.0, 0.0),
         Quat::IDENTITY,
      )
      .expect("spheres touch");
      assert!(manifold.points[0].depth.abs() < 1e-5);
   }

   #[test]
   fn a_sphere_resting_on_a_box_leans_upward() {
      let manifold = collide(
         Shape::sphere(0.5),
         Vec3::new(0.0, 1.2, 0.0),
         Quat::IDENTITY,
         boxed(Vec3::new(2.0, 1.0, 2.0)),
         Vec3::ZERO,
         Quat::IDENTITY,
      )
      .expect("sphere overlaps the box top");
      // The normal points from the sphere toward the box: downward.
      assert!(manifold.normal.y < 0.0, "got {:?}", manifold.normal);
      assert!((manifold.points[0].depth - 0.3).abs() < 1e-4);
   }

   #[test]
   fn a_sphere_inside_a_box_escapes_through_the_nearest_face() {
      let manifold = collide(
         Shape::sphere(0.1),
         Vec3::new(0.0, 0.9, 0.0),
         Quat::IDENTITY,
         boxed(Vec3::splat(1.0)),
         Vec3::ZERO,
         Quat::IDENTITY,
      )
      .expect("sphere is inside the box");
      assert!(manifold.normal.y < 0.0, "got {:?}", manifold.normal);
      assert!(manifold.points[0].depth > 0.0);
   }

   #[test]
   fn separated_boxes_do_not_collide() {
      let result = collide(
         boxed(Vec3::splat(0.5)),
         Vec3::ZERO,
         Quat::IDENTITY,
         boxed(Vec3::splat(0.5)),
         Vec3::new(2.0, 0.0, 0.0),
         Quat::IDENTITY,
      );
      assert!(result.is_none());
   }

   #[test]
   fn stacked_boxes_produce_a_four_point_face_manifold() {
      let manifold = collide(
         boxed(Vec3::splat(0.5)),
         Vec3::ZERO,
         Quat::IDENTITY,
         boxed(Vec3::splat(0.5)),
         Vec3::new(0.0, 0.9, 0.0),
         Quat::IDENTITY,
      )
      .expect("boxes overlap");
      assert_eq!(manifold.count, 4, "a face contact needs four points");
      assert!(manifold.normal.y > 0.0, "got {:?}", manifold.normal);
      for point in &manifold.points[..manifold.count] {
         assert!((point.depth - 0.1).abs() < 1e-4, "got {}", point.depth);
      }
   }

   #[test]
   fn a_rotated_box_pair_still_reports_contact() {
      // 45 degrees about Z; the corners interlock, so this is an edge case.
      let manifold = collide(
         boxed(Vec3::splat(0.5)),
         Vec3::ZERO,
         Quat::from_rotation_z(core::f32::consts::FRAC_PI_4),
         boxed(Vec3::splat(0.5)),
         Vec3::new(0.0, 1.1, 0.0),
         Quat::IDENTITY,
      );
      let manifold = manifold.expect("rotated boxes overlap");
      assert!(manifold.count >= 1);
      assert!(manifold.normal.y > 0.0, "got {:?}", manifold.normal);
   }

   #[test]
   fn a_deeply_overlapping_pair_is_resolved_along_one_axis() {
      let manifold = collide(
         boxed(Vec3::splat(0.5)),
         Vec3::ZERO,
         Quat::IDENTITY,
         boxed(Vec3::splat(0.5)),
         Vec3::new(0.9, 0.0, 0.0),
         Quat::IDENTITY,
      )
      .expect("boxes overlap");
      assert!(
         (manifold.normal.x.abs() - 1.0).abs() < 1e-4,
         "expected the X axis, got {:?}",
         manifold.normal
      );
   }

   #[test]
   fn a_box_resting_on_a_triangle_reports_a_face_contact() {
      // A triangle wide enough to contain the box's whole footprint; the right
      // triangle used elsewhere would clip one corner away.
      // 一个足够宽、能容下盒整个投影的三角形；别处用的直角三角形会裁掉一个角。
      let triangle = [
         Vec3::new(-10.0, 0.0, -10.0),
         Vec3::new(10.0, 0.0, -10.0),
         Vec3::new(0.0, 0.0, 10.0),
      ];
      let frame = BoxFrame::new(Vec3::new(0.0, 0.45, 0.0), Quat::IDENTITY, Vec3::splat(0.5));
      let hit = super::box_triangle(&frame, triangle).expect("the box overlaps the triangle");
      assert_eq!(hit.source_edge, None, "a flat rest is a face contact");
      assert_eq!(hit.manifold.count, 4, "all four box corners touch");
      assert!(
         hit.manifold.normal.y < 0.0,
         "the normal points from the box toward the triangle, got {:?}",
         hit.manifold.normal
      );
      for point in &hit.manifold.points[..hit.manifold.count] {
         assert!(
            (point.depth - 0.05).abs() < 1e-3,
            "got depth {}",
            point.depth
         );
      }
   }

   #[test]
   fn a_box_above_a_triangle_does_not_collide() {
      let triangle = [
         Vec3::new(-10.0, 0.0, -10.0),
         Vec3::new(-10.0, 0.0, 10.0),
         Vec3::new(10.0, 0.0, -10.0),
      ];
      let frame = BoxFrame::new(Vec3::new(0.0, 5.0, 0.0), Quat::IDENTITY, Vec3::splat(0.5));
      assert!(super::box_triangle(&frame, triangle).is_none());
   }

   #[test]
   fn a_box_beside_a_triangle_does_not_collide() {
      let triangle = [
         Vec3::new(-1.0, 0.0, -1.0),
         Vec3::new(-1.0, 0.0, 1.0),
         Vec3::new(1.0, 0.0, -1.0),
      ];
      let frame = BoxFrame::new(Vec3::new(20.0, 0.0, 0.0), Quat::IDENTITY, Vec3::splat(0.5));
      assert!(super::box_triangle(&frame, triangle).is_none());
   }

   #[test]
   fn a_box_baulks_on_a_small_triangle_beneath_it() {
      // The triangle is much smaller than the box face, so the contact comes from
      // the box's own face rather than the triangle's plane.
      let triangle = [
         Vec3::new(-0.1, 0.0, -0.1),
         Vec3::new(-0.1, 0.0, 0.1),
         Vec3::new(0.1, 0.0, -0.1),
      ];
      let frame = BoxFrame::new(Vec3::new(0.0, 0.4, 0.0), Quat::IDENTITY, Vec3::splat(0.5));
      let hit = super::box_triangle(&frame, triangle).expect("the triangle is inside the box");
      assert!(hit.manifold.count >= 1);
      assert!(hit.manifold.normal.y < 0.0, "got {:?}", hit.manifold.normal);
   }

   #[test]
   fn a_degenerate_triangle_never_collides() {
      let triangle = [
         Vec3::new(0.0, 0.0, 0.0),
         Vec3::new(1.0, 0.0, 0.0),
         Vec3::new(2.0, 0.0, 0.0),
      ];
      let frame = BoxFrame::new(Vec3::ZERO, Quat::IDENTITY, Vec3::splat(0.5));
      assert!(super::box_triangle(&frame, triangle).is_none());
   }

   #[test]
   fn box_frames_project_correctly_onto_their_own_axes() {
      let frame = BoxFrame::new(Vec3::ZERO, Quat::IDENTITY, Vec3::new(1.0, 2.0, 3.0));
      assert!((frame.radius(Vec3::X) - 1.0).abs() < 1e-6);
      assert!((frame.radius(Vec3::Y) - 2.0).abs() < 1e-6);
      assert!((frame.radius(Vec3::Z) - 3.0).abs() < 1e-6);
      // A diagonal axis projects to the sum of the |cos| weighted extents.
      let diagonal = Vec3::new(1.0, 1.0, 0.0).normalize();
      let expected = (1.0 + 2.0) / 2.0_f32.sqrt();
      assert!((frame.radius(diagonal) - expected).abs() < 1e-5);
   }

   #[test]
   fn a_face_polygon_is_wound_consistently() {
      let frame = BoxFrame::new(Vec3::ZERO, Quat::IDENTITY, Vec3::splat(1.0));
      let top = frame.face(1, 1.0);
      for corner in top {
         assert!((corner.y - 1.0).abs() < 1e-6);
      }
   }
}

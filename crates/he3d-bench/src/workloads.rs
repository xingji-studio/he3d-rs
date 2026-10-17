//! Scene builders shared by the criterion benchmarks and the `memreport` binary.
//!
//! Every builder is deterministic: the same arguments produce the same scene, so a number measured today can be compared with one measured tomorrow.
//! Where a scene mirrors a documented claim (`docs/physics-spec.md` §12) the shape of the scene is stated on the builder, because the claim is only as good as its reproduction.

use he3d_core::{Camera, ColorRgba8, Material, Mesh, RenderObject, Rgb, Scene, Texture, Transform};
use he3d_math::scalar::tan;
use he3d_math::{Quat, Vec3, conv};
use he3d_physics::{BodyDesc, Shape, World};

/// The fixed simulation step every physics workload uses: 1/120 s.
pub const STEP: f32 = 1.0 / 120.0;

/// Half extents of a unit cube, the Body shape every pile is built from.
pub const CUBE_HALF: Vec3 = Vec3::new(0.5, 0.5, 0.5);

/// Centre-to-centre spacing of the unit cubes in a lattice.
const SPACING: f32 = 1.0;

/// Warm-up steps taken before a lattice is considered settled.
const SETTLE_STEPS: usize = 1200;

// ---------------------------------------------------------------------------
// Physics
// ---------------------------------------------------------------------------

/// An empty [`World`].
/// Measures the fixed per-step overhead with nothing to do.
#[must_use]
pub fn world_empty() -> World {
   World::new()
}

/// `n` spheres on a sparse 3D grid, so none ever touches another.
///
/// This isolates integration and the broadphase: every Body is simulated every step, but the narrowphase and solver stay idle because no contact ever forms.
/// The grid spacing exceeds the default broadphase cell size, so each cell holds one Body and the cost is expected to scale linearly — this is the floor that a contact-loaded scene is measured against.
#[must_use]
pub fn world_freefall(n: usize) -> World {
   let mut world = World::new();
   let side = ((n.max(1) as f64).cbrt().ceil() as usize).max(1);
   let spacing = 3.0;
   let base = side as f32 * spacing * 0.5;
   for i in 0..n {
      let x = (i % side) as f32;
      let z = ((i / side) % side) as f32;
      let y = (i / (side * side)) as f32;
      let position = Vec3::new(x * spacing - base, y * spacing + 5.0, z * spacing - base);
      world.add_body(BodyDesc::dynamic(Shape::sphere(0.5), 1.0).at(position));
   }
   world
}

/// Return the near-cubic lattice dimensions that hold `n` cubes.
///
/// `(x, y, z)` with `x == z`; the caller places only the first `n` cells.
fn lattice_dims(n: usize) -> (usize, usize, usize) {
   let side = ((n.max(1) as f64).cbrt().ceil() as usize).max(1);
   let layers = n.max(1).div_ceil(side * side).max(1);
   (side, layers, side)
}

/// How many vertical layers [`world_lattice_on_slab`] stacks `n` cubes into.
///
/// A pile only sleeps if the solver converges it, and taller piles are harder to converge (see the tall-stack limitation in `docs/physics-spec.md`), so the layer count is the number to quote beside a pile's behaviour.
/// 堆叠越大越难收敛（见 `docs/physics-spec.md` 中的高塔限制），因此这个层数是与一堆物体的行为并列给出的数字。
#[must_use]
pub fn lattice_layers(n: usize) -> usize {
   lattice_dims(n).1
}

/// A lattice of `n` unit cubes resting on a slab, fully awake at step zero.
///
/// This is the `docs/physics-spec.md` §12 scenario: a block of touching unit cubes sitting on a slab, every one of them in contact, so the broadphase, narrowphase, and solver are all loaded.
/// The slab is sized to hold the lattice's footprint, so no cube falls off the edge.
#[must_use]
pub fn world_lattice_on_slab(n: usize) -> World {
   let (sx, sy, sz) = lattice_dims(n);
   let mut world = World::new();

   let half_x = sx as f32 * SPACING * 0.5 + 0.5;
   let half_z = sz as f32 * SPACING * 0.5 + 0.5;
   world.add_body(
      BodyDesc::stationary(Shape::cuboid(Vec3::new(half_x, 0.1, half_z)))
         .at(Vec3::new(0.0, -0.1, 0.0)),
   );

   let mut placed = 0;
   'fill: for layer in 0..sy {
      for z in 0..sz {
         for x in 0..sx {
            if placed == n {
               break 'fill;
            }
            let px = (x as f32 - (sx as f32 - 1.0) * 0.5) * SPACING;
            let pz = (z as f32 - (sz as f32 - 1.0) * 0.5) * SPACING;
            // Bottom layer's lower face sits exactly on the slab's top face.
            let py = layer as f32 * SPACING + CUBE_HALF.y;
            world.add_body(
               BodyDesc::dynamic(Shape::cuboid(CUBE_HALF), 1.0)
                  .at(Vec3::new(px, py, pz))
                  .with_friction(0.6)
                  .with_restitution(0.0)
                  .with_damping(0.1, 0.1),
            );
            placed += 1;
         }
      }
   }
   world
}

/// `n` unit cubes in a single layer on a slab, each dropped by a hair.
///
/// A single layer, not a tower: a tall packed stack is limited by the solver's iteration count (see `docs/physics-spec.md` §14 risk 7) and never settles, so it cannot answer "what do resting Bodies cost".
/// Spaced a full unit apart, these cubes touch only the slab, settle quickly, and go to sleep.
#[must_use]
pub fn world_resting_on_slab(n: usize) -> World {
   let side = ((n.max(1) as f64).sqrt().ceil() as usize).max(1);
   let spacing = 2.0;
   let half = side as f32 * spacing * 0.5 + 1.0;
   let mut world = World::new();
   world.add_body(
      BodyDesc::stationary(Shape::cuboid(Vec3::new(half, 0.1, half))).at(Vec3::new(0.0, -0.1, 0.0)),
   );

   let mut placed = 0;
   'fill: for z in 0..side {
      for x in 0..side {
         if placed == n {
            break 'fill;
         }
         let px = (x as f32 - (side as f32 - 1.0) * 0.5) * spacing;
         let pz = (z as f32 - (side as f32 - 1.0) * 0.5) * spacing;
         world.add_body(
            BodyDesc::dynamic(Shape::cuboid(CUBE_HALF), 1.0)
               .at(Vec3::new(px, CUBE_HALF.y + 0.02, pz))
               .with_friction(0.6)
               .with_restitution(0.0)
               .with_damping(0.2, 0.2),
         );
         placed += 1;
      }
   }
   world
}

/// [`world_resting_on_slab`], stepped until it has had time to settle and sleep.
///
/// The steady state is what "resting Bodies cost nothing" is about: if the solver and broadphase are doing their jobs, these Bodies are asleep and the per-step cost collapses toward [`world_empty`].
#[must_use]
pub fn world_settled_asleep(n: usize) -> World {
   let mut world = world_resting_on_slab(n);
   for _ in 0..SETTLE_STEPS {
      world.step(STEP);
   }
   world
}

/// The same lattice, with the broadphase cell size forced to `cell`.
#[must_use]
pub fn world_cell_size(n: usize, cell: f32) -> World {
   let mut world = world_lattice_on_slab(n);
   world.set_broadphase_cell_size(cell);
   world
}

/// Side length, in cells, of the square terrain grid that holds `tris` triangles.
fn terrain_side(tris: usize) -> usize {
   let quads = tris.div_ceil(2).max(1);
   ((quads as f64).sqrt().ceil() as usize).max(1)
}

/// A flat triangle grid on the XZ plane, `tris` triangles (rounded up to even).
///
/// Wound so every normal points up (+Y).
/// The layout is what [`World::add_static_mesh`] expects: three consecutive positions per triangle.
#[must_use]
pub fn static_terrain_positions(tris: usize) -> Vec<Vec3> {
   let quads = tris.div_ceil(2).max(1);
   let side = terrain_side(tris);
   let mut positions = Vec::with_capacity(quads * 6);
   for q in 0..quads {
      let i = (q % side) as f32;
      let j = (q / side) as f32;
      let a = Vec3::new(i, 0.0, j);
      let b = Vec3::new(i + 1.0, 0.0, j);
      let c = Vec3::new(i, 0.0, j + 1.0);
      let d = Vec3::new(i + 1.0, 0.0, j + 1.0);
      positions.extend_from_slice(&[a, c, b, b, c, d]);
   }
   positions
}

/// A static triangle terrain plus `bodies` spheres resting on it.
///
/// Isolates the cost a large *static* mesh adds: the terrain never moves, but every Body on it must be tested against the triangles beneath it.
#[must_use]
pub fn world_static_terrain(tris: usize, bodies: usize) -> World {
   let mut world = World::new();
   let positions = static_terrain_positions(tris);
   let _ = world.add_static_mesh(&positions);

   let side = terrain_side(tris).max(1);
   for i in 0..bodies {
      let x = ((i * 37) % side) as f32 + 0.5;
      let z = ((i * 17) % side) as f32 + 0.5;
      // Rest on the terrain (sphere radius 0.4) so the static narrowphase runs.
      world.add_body(
         BodyDesc::dynamic(Shape::sphere(0.4), 1.0)
            .at(Vec3::new(x, 0.42, z))
            .with_friction(0.8)
            .with_restitution(0.0),
      );
   }
   world
}

/// How many Bodies in `world` are currently asleep.
#[must_use]
pub fn sleeping_count(world: &World) -> usize {
   world
      .ids()
      .filter(|id| world.is_sleeping(*id).unwrap_or(false))
      .count()
}

// ---------------------------------------------------------------------------
// Renderer
// ---------------------------------------------------------------------------

/// The camera every render scene is viewed through: at the origin, looking +Z.
#[must_use]
pub fn default_camera() -> Camera {
   Camera::new(Vec3::ZERO, Quat::IDENTITY, 60.0)
}

/// A triangle-soup mesh of exactly `2 * ceil(tris/2)` triangles in the XY plane.
///
/// Faceted toward the camera (the renderer back-face culls), so the triangles the mesh contains are the triangles actually rasterized.
#[must_use]
pub fn triangle_soup(tris: usize) -> Mesh {
   let quads = tris.div_ceil(2).max(1);
   let side = ((quads as f64).sqrt().ceil() as usize).max(1);
   let cell = 0.2_f32;
   let half = side as f32 * cell * 0.5;
   let mut vertices = Vec::with_capacity(quads * 6);
   for q in 0..quads {
      let i = (q % side) as f32;
      let j = (q / side) as f32;
      let x0 = i * cell - half;
      let x1 = x0 + cell;
      let y0 = j * cell - half;
      let y1 = y0 + cell;
      let a = Vec3::new(x0, y0, 0.0);
      let b = Vec3::new(x1, y0, 0.0);
      let c = Vec3::new(x0, y1, 0.0);
      let d = Vec3::new(x1, y1, 0.0);
      // a-c-b / b-c-d keeps the normal pointing toward -Z, i.e. at the camera.
      vertices.extend_from_slice(&[a, c, b, b, c, d]);
   }
   Mesh::from_vertices(vertices).expect("triangle soup is well formed")
}

/// A scene with one object holding a `tris`-triangle mesh, framed by the camera.
#[must_use]
pub fn render_scene_tris(tris: usize) -> (Scene, Camera) {
   let mut scene = Scene::new();
   let mesh = scene.meshes.insert(triangle_soup(tris));
   scene.spawn(RenderObject {
      mesh,
      texture: None,
      transform: Transform::from_position(Vec3::new(0.0, 0.0, 3.0)),
      material: Material::new(Rgb::WHITE),
      visible: true,
   });
   (scene, default_camera())
}

/// A scene with `n` small cubes arranged in a grid, all sharing one mesh.
#[must_use]
pub fn render_scene_objects(n: usize) -> (Scene, Camera) {
   let mut scene = Scene::new();
   let mesh = scene
      .meshes
      .insert(Mesh::cube(0.4, 0.4, 0.4).expect("cube mesh is valid"));
   let count = n.max(1);
   let side = ((count as f64).sqrt().ceil() as usize).max(1);
   for i in 0..count {
      let col = (i % side) as f32 - side as f32 * 0.5;
      let row = (i / side) as f32 - side as f32 * 0.5;
      scene.spawn(RenderObject {
         mesh,
         texture: None,
         transform: Transform::from_position(Vec3::new(col * 0.5, row * 0.5, 3.0)),
         material: Material::new(Rgb::WHITE),
         visible: true,
      });
   }
   (scene, default_camera())
}

/// A scene of a handful of cubes, used where AA mode should dominate the cost.
#[must_use]
pub fn render_scene_small() -> (Scene, Camera) {
   render_scene_objects(8)
}

/// A single quad that fills the whole viewport exactly, seen from the [`default_camera`] distance of 3 units.
///
/// Fill-rate benchmarks use this to make every pixel pay the per-fragment cost once: two triangles cover the framebuffer with no overdraw and no bounding-box waste, so the frame cost is the fixed clear plus `pixels * per-fragment cost`, independent of triangle count.
#[must_use]
pub fn screen_fill_quad(width: u32, height: u32) -> Mesh {
   let distance = 3.0_f32;
   let half_h = distance * tan(conv::deg_to_rad(60.0) * 0.5);
   let half_w = half_h * (width as f32 / height as f32);
   // Clockwise winding leaves the face normal pointing at -Z, i.e. at the camera, so the renderer's back-face cull keeps it.
   Mesh::from_vertices(vec![
      Vec3::new(-half_w, -half_h, distance),
      Vec3::new(-half_w, half_h, distance),
      Vec3::new(half_w, -half_h, distance),
      Vec3::new(half_w, -half_h, distance),
      Vec3::new(-half_w, half_h, distance),
      Vec3::new(half_w, half_h, distance),
   ])
   .expect("screen-fill quad is well formed")
}

/// A scene with one screen-filling quad.
///
/// `textured` picks a 256x256 texture; otherwise the quad uses a 1x1 colour, which exercises the renderer's constant-texture fast path.
/// The pair separates the cost of sampling a real texture from the depth test and colour write.
#[must_use]
pub fn render_scene_screen_fill(width: u32, height: u32, textured: bool) -> (Scene, Camera) {
   let mut scene = Scene::new();
   let mesh = scene.meshes.insert(screen_fill_quad(width, height));
   let texture = if textured {
      let pixels = (0..256 * 256)
         .map(|i| ColorRgba8::from_rgb((i % 255) as u8, 40, 200))
         .collect();
      Some(
         scene
            .textures
            .insert(Texture::new(256, 256, pixels).expect("256x256 texture")),
      )
   } else {
      Some(
         scene
            .textures
            .insert(Texture::solid(ColorRgba8::from_rgb(180, 140, 90))),
      )
   };
   scene.spawn(RenderObject {
      mesh,
      texture,
      transform: Transform::default(),
      material: Material::new(Rgb::WHITE),
      visible: true,
   });
   (scene, default_camera())
}

// ---------------------------------------------------------------------------
// Assets
// ---------------------------------------------------------------------------

/// The real `examples/assets/biplane.obj` fixture (about 44 KB), embedded.
#[must_use]
pub fn biplane_obj_bytes() -> &'static [u8] {
   include_bytes!("../../../examples/assets/biplane.obj")
}

/// The real `examples/assets/biplane.bmp` fixture (about 262 KB), embedded.
#[must_use]
pub fn biplane_bmp_bytes() -> &'static [u8] {
   include_bytes!("../../../examples/assets/biplane.bmp")
}

/// A synthetic OBJ with `tris` detached triangles, as bytes.
///
/// Detached triangles exercise the parser's vertex/face handling without relying on any file on disk, which makes the scaling arm of the asset benchmarks stable.
#[must_use]
pub fn synthetic_obj(tris: usize) -> Vec<u8> {
   use core::fmt::Write as _;
   let mut text = String::new();
   for t in 0..tris {
      let x = (t % 64) as f32;
      let y = (t / 64) as f32;
      let _ = writeln!(text, "v {x} {y} 0");
      let _ = writeln!(text, "v {} {y} 0", x + 1.0);
      let _ = writeln!(text, "v {x} {} 0", y + 1.0);
   }
   for t in 0..tris {
      let base = t * 3;
      let _ = writeln!(text, "f {} {} {}", base + 1, base + 2, base + 3);
   }
   text.into_bytes()
}

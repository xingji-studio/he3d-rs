# HE3D (Rust)

A `no_std + alloc` software 3D engine: a software rasterizer, a scene graph, asset
decoders, a physics simulation, and platform backends.

This file fixes the words this project uses for its own concepts. It is a
glossary, not a spec, and it holds no implementation detail. The one exception is
**Open work** at the end, which records what is planned and what has already been
tried and ruled out; everything above it is vocabulary.

## Language

### Simulation

**Body**:
A single simulated physical entity: a shape, a motion mode, mass properties, and
a position and orientation in space. A Body has its own identity and is never
required to be rendered.
_Avoid_: RigidBody, rigid body, object, entity, collider

**BodyId**:
A stable generational handle to a Body within a World. It becomes permanently
invalid once that Body is removed.
_Avoid_: body handle, physics handle, body reference

**World**:
The container that owns Bodies and advances them through time. A World knows
nothing about rendering.
_Avoid_: PhysicsWorld, simulation, space, universe

**Body association**:
The app-owned link between a Body and a RenderObject. The engine neither stores
nor tracks it.
_Avoid_: binding, attachment, mapping, linkage

**Island**:
A maximal set of Bodies connected to one another through Contacts. An Island
sleeps and wakes as a unit, so a resting stack never freezes in mid-air when one
member is disturbed.
_Avoid_: cluster, group, batch, connected component

**Sleep**:
The state in which an Island stops being simulated because every member has come
to rest. Any disturbance to any member wakes the whole Island.
_Avoid_: inactive, dormant, frozen

### Motion modes

**Dynamic**:
A Body whose motion comes from gravity, forces, impulses, and Contacts. It has
finite mass and anything may move it.
_Avoid_: active, simulated, movable, rigid body

**Static**:
A Body that never moves and that no Contact can move.
_Avoid_: fixed, frozen, immovable, anchored

**Kinematic**:
A Body whose motion is driven by a velocity the app sets and that no Contact can
change. It pushes Dynamic Bodies as though infinitely massive.
_Avoid_: controller, animated, scripted, driven

### Collision

**Contact**:
A single point at which two Bodies touch, carrying a normal and a penetration
depth. One pair of Bodies may produce several Contacts.
_Avoid_: collision, hit, touch point

**Contact phase**:
Whether a Contact is appearing this step (Begin), has persisted (Persist), or is
disappearing (End).
_Avoid_: contact state, contact status

**Internal edge**:
An edge shared by two triangles of the same Static geometry. A Contact whose
normal comes from an Internal edge is spurious and must be discarded.
_Avoid_: seam, artificial edge, ghost edge

**Static geometry**:
The immovable triangle meshes of a World, welded into one shared topology so
that an edge lying between two separate meshes is still recognized as an
Internal edge.
_Avoid_: level geometry, world mesh, terrain

### Rendering

**RenderObject**:
A drawable entity in a Scene: a mesh, an optional texture, a material, a
transform, and a visibility flag. It is not a Body and carries no physical
meaning.
_Avoid_: GameObject, entity, node, actor

**ObjectId**:
A stable generational handle to a RenderObject within a Scene. It becomes
permanently invalid once that RenderObject is removed.
_Avoid_: object handle, scene id

**Scene**:
The render-side container: RenderObjects, meshes, textures, and a directional
light.
_Avoid_: world, level, stage

**Projection**:
How a Camera maps a view volume onto the viewport: a perspective projection
carries a vertical field of view in degrees, an orthographic one carries a
world-unit view height. A Camera holds exactly one, so neither mode's parameter
can exist while the other is in force.
_Avoid_: ortho flag, camera type, lens

### Platform

**Platform**:
The host services the engine depends on but does not provide: a surface that
accepts finished frames, a source of input events, a clock, and a request to
close. A Platform is the boundary between the engine and whatever machine it is
running on.
_Avoid_: host, window, environment, engine

**Backend**:
One implementation of Platform: `SdlBackend`, `StdBackend`, or a type written
outside this repository. Which backend an application runs on is decided by the
binary, not by the engine and not by a build feature.
_Avoid_: platform, driver, renderer, surface

## Open work

Everything above is vocabulary. This section is the exception: the work that is
planned, what it is expected to cost, and what has already been tried and ruled out.
A rejected attempt stays here so that it is not attempted a second time.

### Renderer performance

Frame cost is measured by the renderer target in `crates/he3d-bench`
(`render/empty`, `render/scene8`, `render/tris`, `render/n_objects`,
`render/fill/{solid,tex}`, `render/aa`). Fill rate is the driver: at 1280x720 a
full-screen solid-colour quad costs 1.93 ms and a full-screen 256x256-textured quad
costs 7.17 ms, so what is left to win is in the textured per-fragment path. Sampling
is about 4.0 ms of that 7.17 ms and shading about 1.8 ms, so the sampler itself is
no longer the lever.

Landed:

- A triangle is clipped into a fixed four-vertex stack buffer instead of a fresh
  `Vec`, which removes the only per-triangle allocation in a frame (about 49
  allocations per frame down to 1). 10 000 small triangles at 64x36: 611 µs → 530 µs.
- `draw_mesh` folds the object rotation and the world→view rotation into one
  quaternion and the two translations into one vector, so a vertex costs one rotation
  instead of two, and shading moves to view space.
- A 1x1 texture is constant, so its shaded colour is hoisted out of the per-pixel
  loop: a full-screen solid-colour quad at 1280x720 went 7.81 ms → 1.93 ms.
- `Texture::sample` drops a branch that `x - floor(x)` can never take and indexes with
  32-bit arithmetic, and the per-fragment path drops two clamp lower bounds that
  cannot be reached. The textured quad went 7.81 ms → 7.17 ms.
- `draw_mesh` computes the per-frame invariants once, in a `Frame`, instead of once
  per RenderObject: the camera's inverse rotation, the view-space light direction, the
  pixel scale (with its `tan`), the viewport half-extents, and the near plane. The
  measured effect is small — roughly 1-3% on the object- and triangle-heavy arms
  (`render/n_objects/1000`, `render/tris/{1000,10000}`) and within run-to-run noise on
  the fill-bound ones — so this was **not** the 10-20% the estimate below guessed. It
  is kept as a structural change, the way the timing-neutral resolve-once `ContactPair`
  was, because the "compute once per frame" invariant is now explicit rather than paid
  per mesh.

Ruled out:

- Replacing `floor` with a truncation-based wrap in `sample`. Measured about 25%
  slower end to end (9.70 ms), because the `f32 → i32 → f32` round trip costs more
  than the `floor` the compiler inlines.
- **Scanline rasterization.** The per-fragment sign test is the drain only on large,
  strongly slanted triangles. Skipping it entirely — which shades each triangle's whole
  bounding box, so it is a ceiling on what edge culling could ever recover, not a
  like-for-like comparison — costs about 19% of `render/fill/tex/1280x720` and 8% of
  `render/scene8/1280x720`, but only 1.7% of `render/tris/10000`, where the triangles
  are small and nearly fill their boxes. Iterating just the per-row x-interval while
  keeping every pixel identical would still advance the barycentric and
  depth/texture interpolants column by column to the interval's start, which costs
  about what the sign test it replaces costs. Re-seeding those interpolants at the
  interval's start is the cheap version, but it moves every later fragment by a
  rounding step, and a test pins the whole frame by hash (`fnv1a_pixels`). So the win
  is real only for the fill-bound arms, and buying it means adopting a different
  coverage rule and re-baselining that hash.

Candidates, in the order worth trying. Not yet measured:

1. **Interleave colour and depth into one buffer.** They are two `Vec`s today, so
   every fragment writes two cache lines; interleaving would halve that. This is not a
   local optimization: `Renderer::pixels` is public and returns the colour buffer, so
   the interleaved layout forces an API decision (a copy, or a strided view) and a
   minor version bump. Also estimated at 10-20%, unmeasured.

### Comment style migration

`AGENTS.md` §5 makes a comment sentence occupy one line. The files converted so far
are the ones a fix or a pass touched; the rest of the tree converts when a file is
next touched, which is what §5 asks for. Everything this cycle reached is clean: the
facade crate's docs, the flight-simulator and biplane sources, and the crates the
orthographic change or the one-line `# 示例` heading fix touched.

What is left is what nothing has opened since the rule landed — most of
`crates/he3d-physics` (`body.rs`, `broadphase.rs`, `shape.rs`, `static_geometry.rs`,
`storage.rs`), the `he3d-platform` crate and its two backends,
`crates/he3d-assets/src/gltf.rs`, `crates/he3d-renderer/src/config.rs`, and
`crates/he3d-math`. The `// ---- label` banners in test code are dividers rather than
sentences, so they stay as they are.


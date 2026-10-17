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

Ruled out:

- Replacing `floor` with a truncation-based wrap in `sample`. Measured about 25%
  slower end to end (9.70 ms), because the `f32 → i32 → f32` round trip costs more
  than the `floor` the compiler inlines.

Candidates, in the order worth trying. None of the three has been measured:

1. **Scanline rasterization.** The inner loop walks the triangle's bounding box and
   tests `w0 >= 0 && w1 >= 0 && w2 >= 0` per fragment, and the MSAA arm recomputes a
   per-pixel sample mask. Deriving the covered x range per scanline from the three
   edges would skip off-triangle fragments outright and take the sign test out of the
   inner loop. A first read of the code put this at 10-20%, which is an estimate and
   not a measurement.
2. **Interleave colour and depth into one buffer.** They are two `Vec`s today, so
   every fragment writes two cache lines; interleaving would halve that. This is not a
   local optimization: `Renderer::pixels` is public and returns the colour buffer, so
   the interleaved layout forces an API decision (a copy, or a strided view) and a
   minor version bump. Also estimated at 10-20%, unmeasured.
3. **Hoist what does not vary per object out of `draw_mesh`.** `cam_inv_rot` and
   `light_dir` depend only on the camera and the light, and are recomputed for every
   RenderObject in a frame.

### PhysicsTest demo controls

The demo applies a constant 6 N·m torque about z and a 14 N push. The torque is what
launches the cube off the platform once it rolls, because it never stops adding
angular velocity. A bounded torque, or a roll driven toward a target angular velocity,
would remove that. Not approved, so nothing has changed.

### Comment style migration

`AGENTS.md` §5 makes a comment sentence occupy one line. The files converted so far
are the ones the physics fix and the renderer pass touched; the rest of the tree
converts when a file is next touched, which is what §5 asks for. Two things are
unresolved:

- Five `# Errors` and `# Panics` sections put Chinese before English, which §5's
  bilingual sentence does not allow (`crates/he3d-core/src/texture.rs`,
  `crates/he3d-renderer/src/renderer.rs`).
- `crates/he3d-renderer/src/renderer.rs` has two comment runs that are Chinese-only.


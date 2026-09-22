# HE3D (Rust)

A `no_std + alloc` software 3D engine: a software rasterizer, a scene graph, asset
decoders, a physics simulation, and platform backends.

This file fixes the words this project uses for its own concepts. It is a
glossary, not a spec, and it holds no implementation detail.

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


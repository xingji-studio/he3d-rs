# Physics is decoupled from rendering

**Status:** accepted

A Body has its own `BodyId`; `he3d-physics` depends only on `he3d-math`; and the
application owns both the Body-to-RenderObject association and the per-frame copy
of a Body's placement into the RenderObject's transform. The engine provides no
binding layer. This replaces the current shape, where a Body is keyed by the
render `ObjectId` and transforms are written back through `synchronize(&mut
Scene)` — which forces a render object to exist before a Body can, limits a render
object to one Body, couples physics to the scene graph, and is the reason most of
the current public methods return `bool` ("that id is not a Body").

## Context

A Body's placement is deliberately a position and orientation, never a
`Transform`: scaling a collider means re-deriving its shape, and silently ignoring
a scale is worse than refusing it. Static triangle data is taken as a flat
`&[Vec3]` (every three consecutive vertices form one triangle) rather than
`&Mesh`, so the crate needs nothing from `he3d-core`.

## Consequences

- `he3d-physics` is testable on its own — no `Scene`, no render objects.
- Bodies may exist without being rendered (invisible walls, anchors, logic-only
  bodies), and one render object may later carry several Bodies.
- Fallible lookups become `Option`/`Result` instead of `bool`.
- **The application writes the association and the sync loop itself.** That
  boilerplate is the deliberate price; a convenience binding layer was considered
  and rejected to keep the engine free of a second source of truth.

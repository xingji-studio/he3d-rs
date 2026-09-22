# All static geometry shares one topology

**Status:** accepted

Static triangle meshes are welded into a single shared topology, and an edge
shared by two triangles *of that topology* — including an edge between two
separate meshes — is an Internal edge. Any Contact whose normal comes from an
Internal edge is spurious and is discarded.

## Context

`Mesh` stores a flat vertex list with no index buffer, so vertices on a shared edge
are duplicated and the engine sees every triangle seam as a real edge. A Body
sliding across flat ground then catches on each seam, which looks like excessive
friction and is extraordinarily hard to diagnose — every line of the physics code
is correct.

Terrain makes per-mesh welding insufficient: it is streamed as many separate tile
meshes, so every tile boundary would remain a real edge and a Body would catch once
per tile. Terrain is also not the only case — any static model loaded later would
have the same boundary problem.

## Consequences

A global static-geometry registry with a positional vertex index, maintained as
meshes are added and removed (streamed terrain tiles churn constantly). Static
geometry does **not** enter the Body broadphase: it is reached through its own
triangle BVH, so a whole terrain never lands in every grid cell. This is the
largest single work item in the rewrite.

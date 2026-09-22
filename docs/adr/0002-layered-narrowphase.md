# Layered narrowphase: analytic core plus general convex

**Status:** accepted

The three core shapes (box, sphere, capsule) collide with each other through
analytic routines — six pairs, SAT and face clipping for box-versus-box. Cylinder,
cone, convex hull, and every mixed pair go through a general convex path: GJK for
intersection and distance, EPA for penetration depth.

## Considered options

- **All pairs analytic.** Six shapes is twenty-one pairs; adding a shape means
  writing six more, and the curved shapes have the nastiest manifold cases. The
  combinatorial cost is what kills this.
- **All pairs through GJK/EPA.** One code path, new shapes are free. Rejected
  because box-versus-box is the workhorse of every stacking scene and gains the
  most from SAT plus clipping — routing it through EPA makes the most common pair
  both slower and worse, and EPA is least reliable at the shallow penetrations a
  resting stack produces.

## Consequences

Two code paths with different contact quality, which is a real inconsistency to
live with. Contact manifolds are produced per path, and a pair may yield several
Contacts rather than one. EPA's numerical stability at shallow penetration is the
single largest implementation risk in the plan; cylinder and cone are the hardest
curved cases and should be scheduled last.

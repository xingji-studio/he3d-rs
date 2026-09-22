# The binary chooses the backend

**Status:** accepted

A backend is a type that implements the `Platform` trait from `he3d-platform`.
The binary that runs the application names the concrete backend (`SdlBackend`,
`StdBackend`, or one written elsewhere), depends on that backend's crate, and
constructs it. `he3d` depends on no backend crate, optional or otherwise, and
provides no helper that picks one.

## Considered options

- **A Cargo feature on `he3d` selecting the backend, plus a `create_backend` that
  chooses among the enabled ones at runtime.** This is what the code did before,
  and it fails for two independent reasons. Cargo features are additive across the
  whole dependency graph, so the choice belongs to whichever crate happens to
  enable one — a library deep in the graph can decide that the application gets a
  window. And the runtime precedence ("try `sdl3`, then `std`") hands a windowed
  backend to a process that asked for none, as long as SDL3 is present. It also
  forced one feature to mean two unrelated things: `he3d/std` both compiled the
  libraries with `std` and compiled in the stdout backend.
- **A generic host type in the engine, `App<P: Platform>`.** Rejected because it
  takes over the main loop. The fixed-step accumulator, the per-frame step cap, the
  frame-rate policy, and pause handling are application decisions today, and the
  two existing demos do not agree on them.
- **An engine-side registry, or selecting a backend by name at runtime.**
  Rejected: backends have different construction errors and different system
  requirements, and a name lookup only works if every candidate is compiled in — so
  a headless application would carry the SDL3 build requirement for a window it
  never opens.

## Consequences

- Depending on `he3d-platform-sdl3` makes SDL3 a build requirement, because
  `sdl3-sys` declares `links = "SDL3"`. Nothing hides it any more: an application
  that must build headlessly has to keep that crate out of its dependency graph.
  The escape hatch is now a type name, not a feature flag.
- The four demos hardcode `SdlBackend`; the getting-started guide hardcodes
  `StdBackend`, which needs nothing from the system. Swapping backends is one line
  in `main` plus one line in the manifest.
- `he3d-platform` has no `PlatformFactory`: each backend has an inherent
  constructor and its own error type, so construction failures stay specific to the
  backend that produced them.
- The facade's headless smoke test lives in `he3d-platform-std`, since the facade
  no longer has a backend to drive.
- An application that wants to switch backends at runtime can box either one behind
  `Box<dyn Platform>` itself. The trait stays object-safe; the engine simply does
  not make that decision.

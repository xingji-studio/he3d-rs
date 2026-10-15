# HE3D (Rust) — physics specification

The feature set for `he3d-physics`. Sections 1–9 and 11–12 describe what the crate
**does**; whatever is still planned is called out where it applies.

`he3d-physics` 的功能规格。第 1–9 节与第 11–12 节描述该 crate **当前的行为**；仍属计划的
部分在各处单独标出。

Status: **implemented, with gaps recorded in §4, §5, §10 and §14**. Vocabulary is
fixed in [`CONTEXT.md`](../CONTEXT.md); the decisions that are hard to reverse have
their own [ADRs](./adr/).

状态：**已实现，缺口记录在 §4、§5、§10 与 §14**。术语固定在 [`CONTEXT.md`](../CONTEXT.md)；
难以反悔的决定各有自己的 [ADR](./adr/)。

### What is not built yet

| Spec section | Gap |
|---|---|
| §4 Shapes | Only `Box` and `Sphere`. Capsule, cylinder, cone and convex hull are unimplemented. |
| §5 Narrowphase | Only the analytic layer: box-box, sphere-box, sphere-sphere, and box/sphere against triangles. GJK and EPA do not exist, so the general-convex path does not either. |
| §10 Queries | No raycast at all. |
| §7 Internal edges | Filtered for box-versus-triangle only. A Sphere needs no filter — its normal is always the triangle's plane normal — and measurement confirms a seam between two separate meshes costs a rolling sphere exactly nothing (100.00% of its speed kept, no vertical dip). |

### 尚未构建的部分

| 规格章节 | 缺口 |
|---|---|
| §4 形状 | 只有 `Box` 与 `Sphere`。胶囊、圆柱、圆锥与凸包尚未实现。 |
| §5 窄相位 | 只有解析层：盒-盒、球-盒、球-球，以及盒/球对三角形。GJK 与 EPA 不存在，因此通用凸体路径也不存在。 |
| §10 查询 | 完全没有射线检测。 |
| §7 内部边 | 仅对盒-三角形过滤。球不需要过滤——其法线始终是三角形平面法线——实测确认两块独立网格之间的接缝对滚动的球毫无影响（速度保留 100.00%，无竖直下沉）。 |

---

## 1. Purpose

Interactive game rigid-body dynamics: stacking, pushing, throwing, and a character
standing on terrain, at interactive frame rates.

面向交互式游戏的刚体动力学：堆叠、推挤、抛掷、角色站在地形上，交互式帧率。

**Determinism is explicitly not a requirement.** Bit-exact reproducibility across
platforms, versions, or even runs was considered and deliberately dropped. Do not
re-introduce it as an implicit goal; if it is ever wanted, it must be re-decided,
because it constrains fixed stepping, solver ordering, threading, and the math
layer.

**确定性明确不是需求。** 跨平台、跨版本乃至跨运行的逐位可复现性经过讨论后被刻意
放弃。不要把它当作隐含目标重新引入；若将来需要，必须重新决策，因为它会约束固定
步长、求解顺序、线程化与数学层。

A fixed timestep is still expected of callers — not for reproducibility, but
because stacking is unstable and feel drifts with frame rate under a variable
step. The engine makes that the caller's call rather than enforcing it (§11).

固定步长仍是对调用方的期望——不是为了可复现，而是因为变步长下堆叠不稳、手感随帧率
漂移。引擎把它交给调用方决定，而非强制（§11）。

## 2. Identity and the rendering seam

A Body is a first-class entity with its own `BodyId`. Physics knows nothing about
`RenderObject`, `Scene`, or `ObjectId`.

Body 是一等实体，拥有自己的 `BodyId`。物理对 `RenderObject`、`Scene`、`ObjectId`
一无所知。

- **`he3d-physics` depends only on `he3d-math`.** No dependency on `he3d-core`.
- A Body's placement is a **position and orientation**, never a `Transform`. Scale
  is not accepted: scaling a collider means re-deriving its shape, and silently
  ignoring a scale is worse than refusing it.
- Static triangle data is taken as a **flat `&[Vec3]`** where every three
  consecutive vertices form one triangle — the same layout `Mesh` stores, without
  the UVs, normals, or the `he3d-core` dependency.
- **The application owns the association** between a Body and a RenderObject, and
  the per-frame copy of a Body's placement into the RenderObject's transform. The
  engine neither stores nor tracks this, and provides **no binding layer**.

- **`he3d-physics` 只依赖 `he3d-math`**，不依赖 `he3d-core`。
- Body 的摆放是**位置与朝向**，绝不是 `Transform`。不接受 scale：缩放碰撞体等于重新
  推导其形状，而静默忽略 scale 比拒绝它更糟。
- 静态三角数据以**扁平的 `&[Vec3]`** 传入，每三个连续顶点构成一个三角形——与 `Mesh`
  的存储布局相同，但不带 UV、法线，也不依赖 `he3d-core`。
- **关联由应用拥有**：Body 与 RenderObject 的对应关系，以及每帧把 Body 摆放复制进
  RenderObject 变换的动作，都由应用负责。引擎既不存储也不跟踪，且**不提供绑定层**。

See [ADR 0001](./adr/0001-physics-decoupled-from-rendering.md).

见 [ADR 0001](./adr/0001-physics-decoupled-from-rendering.md)。

## 3. Bodies and motion modes

Three motion modes: `Dynamic`, `Static`, `Kinematic`. Definitions are fixed in
`CONTEXT.md`; the load-bearing one is `Kinematic` — **velocity-driven and
unpushable**. `Kinematic` Bodies are placed by the velocity the app sets, are
unaffected by gravity and forces, and push `Dynamic` Bodies as though infinitely
massive. Position-driven kinematics are rejected: the solver needs a velocity to
compute relative velocity at a Contact, so a teleported Body transfers friction
and impulses incorrectly and produces platforms that behave inexplicably.

三种运动模式：`Dynamic`、`Static`、`Kinematic`。定义固定在 `CONTEXT.md`；其中真正
吃重的是 `Kinematic`——**速度驱动、不可被推动**。`Kinematic` 由应用设定的速度驱动，
不受重力与力影响，并以无穷大的质量推动 `Dynamic`。位置驱动的运动学被否决：求解器
需要速度来计算 Contact 处的相对速度，被瞬移的 Body 会错误地传递摩擦与冲量，产出行为
莫名其妙的平台。

**Mass is the input; the inertia tensor is derived** from the shape and the mass.
A hand-written inertia tensor that disagrees with the shape produces bodies that
"spin like plastic", and the resulting bug is close to unfindable. `Static` and
`Kinematic` Bodies have infinite mass.

**输入质量，惯量张量由形状与质量推导。** 手写一个与形状不一致的惯量张量，会让刚体
"转起来像塑料"，而由此产生的 bug 几乎无法排查。`Static` 与 `Kinematic` 的质量为无穷大。

**Gravity belongs to the World; a Body scales its share of it.** Every Body carries
a gravity scale that multiplies the World's gravity. `1.0` is the default, `2.0`
falls twice as fast, and `0.0` leaves the Body untouched by gravity while it still
collides and is still pushed — which is what a caller wants when something else
already owns the Body's motion, such as a flight model. Negative or non-finite
scales are clamped to zero. Changing the scale wakes the Body, so a Body that went
to sleep under one scale is not left asleep under another.

**重力属于 World，由每个 Body 按系数缩放。** 每个 Body 带一个重力系数，乘在 World 的
重力上。默认 `1.0`，`2.0` 以两倍速度下坠，`0.0` 让它不受重力、却仍然参与碰撞也仍然会被
推动——这正是当别的东西已经拥有该 Body 的运动时想要的，例如飞行模型。负值或非有限值被
钳制为零。改动系数会唤醒该 Body，因此不会出现"在一个系数下睡着、换到另一个系数下仍在睡"。

## 4. Shapes

`Box` · `Sphere` · `Capsule` · `Cylinder` · `Cone` · `ConvexHull`, plus static
triangle geometry as a separate concept.

`Box` · `Sphere` · `Capsule` · `Cylinder` · `Cone` · `ConvexHull`，外加作为独立概念的
静态三角几何。

Compound Bodies (one Body carrying several shapes) are **not** in this plan.

复合碰撞体（一个 Body 挂多个形状）**不在**本次计划内。

## 5. Narrowphase

Layered, not uniform: **analytic** for the three core shapes (box, sphere,
capsule — six pairs), **general convex** (GJK for intersection and distance, EPA
for penetration depth) for cylinder, cone, convex hull, and for every mixed pair.

分层，而非统一：三种核心形状（盒、球、胶囊——六对）走**解析解**，圆柱、圆锥、凸包
以及所有混合对走**通用凸体**（GJK 求交与距离，EPA 求穿透深度）。

Box-versus-box is the workhorse of every stacking scene and gets the most from SAT
plus face clipping; routing it through EPA would make the most common pair slower
*and* worse. Hand-writing all twenty-one pairs is combinatorial and the curved
shapes are the hardest to get right.

盒-盒是所有堆叠场景的主力，从 SAT + 面裁剪中获益最大；把它扔给 EPA 会让最常用的一对
更慢**且**更差。而手写全部二十一对是组合爆炸，且曲面形状最难写对。

A pair may produce **several** Contacts (a manifold), not one.

一对 Body 可以产出**多个** Contact（一个接触流形），而非单个。

See [ADR 0002](./adr/0002-layered-narrowphase.md).

见 [ADR 0002](./adr/0002-layered-narrowphase.md)。

## 6. Broadphase

A **uniform spatial hash grid** over Body AABBs. Oversized Bodies are kept in a
separate list checked against everything; they are expected to be few.

基于 Body AABB 的**均匀空间哈希网格**。超大 Body 单独列一份清单、与所有东西比对；
预期数量很少。

**Static geometry is not inserted into the grid.** It is a single global concept
(§7) reached through its own triangle BVH, so a whole terrain never lands in every
cell.

**静态几何不进入网格。** 它是单一全局概念（§7），通过自己的三角形 BVH 访问，因此
整块地形永远不会落进每一个格子。

## 7. Static geometry and internal edges

All static triangle meshes **weld into one shared topology**. An edge shared by two
triangles of that topology is an **Internal edge**, and any Contact whose normal
comes from one is spurious and must be discarded.

所有静态三角网格**焊接进同一套拓扑**。该拓扑中被两个三角形共享的边是 **Internal
edge**，任何法线来自它的 Contact 都是伪接触，必须丢弃。

Per-mesh welding is not enough: terrain is streamed as many separate tile meshes,
so every tile boundary would remain a real edge and a Body sliding across flat
ground would catch every tile. The shared topology makes cross-mesh edges count as
internal too, and gives the same protection to any static model added later.

逐 mesh 焊接不够：地形以许多独立瓦片网格流式加载，于是每条瓦片边界都仍是真实的边，
物体在平地上滑行会每块瓦片卡一次。共享拓扑让跨 mesh 的边也算内部边，并对以后加载的
任何静态模型提供同样的保护。

See [ADR 0003](./adr/0003-static-geometry-shared-topology.md).

见 [ADR 0003](./adr/0003-static-geometry-shared-topology.md)。

## 8. Solver, islands, and sleep

**Sequential impulse with warm starting.** Constraints are solved per Island.

**序列冲量 + Warm Starting。** 约束按 Island 求解。

Islands come from a union-find over the Contact graph. **Sleep is per Island, not
per Body**: a resting stack sleeps as one unit and wakes as one unit, so disturbing
the bottom box does not leave the ones above it frozen in mid-air.

Island 由 Contact 图上的并查集得出。**睡眠按 Island，而非按 Body**：一叠静止的箱子
作为整体睡去、作为整体醒来，因此推动最下面那个不会让上面的悬在空中不动。

Sleeping is not optional at this scale — a few hundred resting Bodies would
otherwise be re-solved every step forever.

在这个规模下睡眠不是可选项——否则几百个静止的 Body 会永远每步重解。

See [ADR 0004](./adr/0004-sequential-impulse-solver.md).

见 [ADR 0004](./adr/0004-sequential-impulse-solver.md)。

## 9. Contacts

Every `step` produces an iterable **`&[Contact]`** that the application walks. Each
Contact carries the two Bodies, the contact point, the normal, the penetration
depth, and a **phase**: `Begin`, `Persist`, or `End`.

每次 `step` 产出一段可迭代的 **`&[Contact]`**，由应用遍历。每个 Contact 携带两个
Body、接触点、法线、穿透深度，以及一个 **phase**：`Begin`、`Persist` 或 `End`。

This is polling, not callbacks: no closures, no lifetime or generic parameters
infecting the World, and it stays `no_std`-friendly.

这是轮询而非回调：没有闭包，没有污染 World 的生命周期或泛型参数，且保持对 `no_std`
友好。

Phase information is nearly free, because warm starting already requires
per-Contact persistent state across steps — the engine already knows whether a
Contact was present last step.

Phase 信息几乎免费，因为 warm starting 本就要求跨步保存每个 Contact 的持久状态——
引擎已经知道某个 Contact 上一帧在不在。

The practical consequence: an application must never need to hand-roll geometric
tests to answer "am I on the ground". Nothing like the current example's
tolerance-laden `is_grounded` — which hardcodes a floor's dimensions, ignores
rotation, and exists only because the engine could not answer the question.

实际后果：应用永远不该为了让"我落地了吗"有答案而手写几何判定。绝不能出现当前示例里
那种满是容差的 `is_grounded`——它写死了地板尺寸、忽略旋转，存在的原因仅仅是引擎答不了
这个问题。

## 10. Queries

A **raycast** against both Body shapes and static geometry, returning the hit
identity (a `BodyId`, or a reference to static geometry), the contact point, the
normal, and the distance.

**射线检测**同时支持 Body 形状与静态几何，返回命中身份（`BodyId`，或对静态几何的
引用）、命中点、法线与距离。

Hit filtering ("only hit enemies") is **not** in this plan; it depends on collision
filtering, which is also out. Ray-versus-convex-hull needs its own path (the slab
method for polytopes, or a GJK-based cast) and ray-versus-static-geometry needs ray
queries on the BVH — both are in scope here rather than deferred.

命中过滤（"只打敌人"）**不在**本次计划内；它依赖碰撞过滤，而碰撞过滤同样不做。
ray-vs-凸包需要自己的路径（多面体的 slab 法，或基于 GJK 的投射），ray-vs-静态几何
需要在 BVH 上做射线查询——这两项都在范围内，不推迟。

## 11. Time

**`step(fixed_dt)` only.** The engine exposes a raw fixed-step entry point and
holds no time state. The application accumulates wall time and caps how many steps
it runs per frame, so a slow frame cannot trigger an unbounded catch-up spiral.

**只有 `step(fixed_dt)`。** 引擎暴露裸的固定步长入口，不持有任何时间状态。应用自己
累积墙钟时间，并限制每帧最多跑几步，使一次慢帧不会触发无界的追赶螺旋。

An internal accumulator was rejected: it hardcodes the per-frame step cap, makes
"which time span does this Contact list describe" ambiguous when a frame runs zero,
one, or many steps, and puts time state inside the World.

内部的累加器被否决：它写死了每帧步数上限，让"这段 Contact 列表对应哪一段时间"在
一帧跑 0 步 / 1 步 / 多步时变得含糊，并把时间状态塞进了 World。

## 12. Scale target

**Hundreds to two thousand** simultaneously simulating Bodies, single-threaded.

**几百到两千**个同时模拟的 Body，单线程。

This is what forces §6 (a real broadphase) and §8 (island sleep). Ten thousand is
out of reach without threading, and there is no threading substrate in a
`no_std + alloc` crate.

这是 §6（真正的宽阶段）与 §8（岛屿睡眠）被强制引入的原因。不上万——没有线程化就
到不了，而 `no_std + alloc` 的 crate 里没有线程基础设施。

### Measured

Per `step()` call at a 1/120 s step, single-threaded, measured by
`crates/he3d-bench` on an AMD Ryzen 7 5800X with rustc 1.99.0. The scenes are
defined in that crate; `cargo bench -p he3d-bench --bench physics` reproduces the
peaks below, `cargo run -p he3d-bench --bin settle` the settled ones, and
`cargo run -p he3d-bench --bin memreport` the heap figures.

单次 `step()` 调用的耗时，1/120 秒步长，单线程，由 `crates/he3d-bench` 在
AMD Ryzen 7 5800X、rustc 1.99.0 上测得。场景定义在该 crate 中；下表的峰值可用
`cargo bench -p he3d-bench --bench physics` 复现，稳态值用
`cargo run -p he3d-bench --bin settle`，堆数据用
`cargo run -p he3d-bench --bin memreport`。

A scene dropped into contact has two costs, not one. Penetration deepens for the
first few steps before positional correction pushes the pile apart, so the cost
climbs to a **peak**; a pile the solver converges then falls asleep and costs almost
nothing, while a pile it cannot converge **never sleeps** and pays for every Contact
forever. Both numbers are stated because they answer different questions: the peak is
what can overspend a frame, the settled cost is what a running scene actually pays.

落入接触的场景有两个成本而非一个。最初的若干步里穿透会加深，随后位置修正把堆叠推开，
因此成本爬升到一个**峰值**；求解器能收敛的堆叠随后入睡、几乎不花代价，而收敛不了的堆叠
**永不入睡**，永远为每个 Contact 付费。两个数字都要给，因为它们回答不同的问题：峰值是可能
让某一帧超支的成本，稳态成本才是运行中的场景实际支付的。

| Scene | Bodies | Layers | Peak `step()` | Settled `step()` | Sleeps |
|---|---|---|---|---|---|
| Freefall grid, nothing ever touches | 2000 | — | 0.23 ms | 0.23 ms | nothing to settle |
| Single layer on a slab | 2000 | 1 | 0.41 ms | 0.41 ms | yes, all 2000 |
| Lattice on a slab | 200 | 6 | 2.5 ms | 0.05 ms | yes, after ~800 steps |
| Lattice on a slab | 500 | 8 | 6.6 ms | 4.4 ms | **never** |
| Lattice on a slab | 2000 | 12 | 32 ms | 8.8 ms | **never**; sheds edge Bodies |

The peak is the criterion bench: one step, taken sixteen steps into a freshly built
scene. The settled value is `settle`: the median of the last hundred of 1500 steps.

峰值取自 criterion 基准：新建场景后第 16 步处测量的单步耗时。稳态值取自 `settle`：
1500 步中最后一百步的中位数。

**A pile the solver cannot converge never sleeps.** Six layers settle, fall asleep,
and then cost 0.05 ms for the whole pile. Eight and twelve layers never do: their
residual speed settles around 0.16 m/s — three times the 0.05 m/s sleep threshold —
so every Body stays awake and the scene pays for its Contacts forever. More
iterations do not rescue it: at 48 velocity iterations an eight-layer pile is still
awake, at close to four times the cost. This is the tall-stack limitation of §14
risk 7, now measured instead of assumed. The twelve-layer pile additionally creeps
and sheds edge Bodies, which then free-fall to the end of the run — that is why its
settled cost (8.8 ms) sits well below its peak (32 ms) rather than at zero.

**求解器收敛不了的堆叠永不入睡。** 六层会稳定、入睡，之后整堆只要 0.05 ms。八层与十二层
则永远不会：残余速度稳定在约 0.16 m/s——是 0.05 m/s 睡眠阈值的 3 倍——因此每个 Body 都保持
清醒，场景永远为它的 Contact 付费。增加迭代次数救不了它：48 次速度迭代下八层堆叠仍然清醒，
代价却接近四倍。这就是 §14 风险 7 的高塔限制，现在是实测而非假设。十二层堆叠还会蠕动并
甩掉边缘的 Body，后者一路自由落体到运行结束——这正是它的稳态成本（8.8 ms）远低于峰值
（32 ms）、而不是降到零的原因。

**Cost tracks contacts, not Bodies.** Two thousand Bodies that step without ever
touching cost 0.23 ms; the same two thousand packed so that a fifth of them are in
contact cost 4.4 ms at eight layers — twenty times more, at an identical Body count,
and 8.8 ms at twelve. Within a regime the cost *is* linear in Body count, exactly as
§6 and §8 intend: the awake lattice runs at roughly 12 µs per Body, while sleep
removes a Body's cost entirely.

**开销跟的是接触，不是 Body。** 两千个互不接触的 Body 每步 0.23 ms；同样两千个、其中约
五分之一彼此相触的堆叠，八层时要 4.4 ms——Body 数完全相同，代价高二十倍——十二层时要
8.8 ms。在同一状态内，开销对 Body 数量**确实是线性的**，正是 §6 与 §8 想要的结果：清醒格架
约 12 µs/Body，而睡眠会完全抹掉一个 Body 的开销。

**The frame budget.** Two thousand resting Bodies cost 0.41 ms and leave the frame
untouched. Two thousand packed Bodies peak at 32 ms — they do **not** fit a 60 Hz
frame (16.7 ms), and settle only to 8.8 ms, about half of one. A scene of two
thousand Bodies that are moving but largely out of contact — freefall, spreading
debris — costs 0.23 ms and is comfortably real-time. Contact density, not Body count,
decides which regime a scene is in, and the tall-stack limitation decides whether the
dense regime ever ends.

**帧预算。** 两千个静止 Body 只花 0.41 ms，几乎不占一帧。两千个塞满接触的 Body 峰值 32 ms
——**塞不进** 60 Hz 的一帧（16.7 ms）——稳态也只降到 8.8 ms，约占半帧。而两千个虽有运动但
基本不接触的场景（自由落体、散开的碎块）只要 0.23 ms，实时绰绰有余。决定场景落在哪个区间
的是接触密度而非 Body 数量；而密集区间能否结束，取决于高塔限制。

**Broadphase cell size** at 2000 awake Bodies: 0.5 → 38 ms, 2.0 → 32 ms, 8.0 →
32 ms. The default 2.0 is near-optimal; halving the cell is the only setting that
hurts.

**宽阶段网格尺寸**（2000 个清醒 Body）：0.5 → 38 ms，2.0 → 32 ms，8.0 → 32 ms。默认的
2.0 接近最优；把格子减半是唯一会拖慢的取值。

**Static geometry** under 16 resting Bodies: 100 triangles → 31 µs, 1000 →
57 µs, 10 000 → 238 µs. It is not free, but 10 000 triangles are 1.4% of a 60 Hz
frame, and the mesh welds into one shared topology (§7) rather than being paid for
per Body.

**静态几何**（16 个静止 Body 之下）：100 个三角形 → 31 µs，1000 个 → 57 µs，
10000 个 → 238 µs。它不是免费的，但 10000 个三角形也只占 60 Hz 一帧的 1.4%；而且网格
焊接进同一套共享拓扑（§7），而不是按 Body 逐个付费。

**Heap.** Steady-state `World::step` allocates nothing: the World reuses its
buffers, so a step makes zero allocation calls whether there are 200 Bodies or 2000.
A Body costs about 196 requested bytes of storage, and a settled scene holds no
Contact data at all.

**堆。** 稳态的 `World::step` 不分配任何内存：World 复用其缓冲，因此无论有 200 个还是
2000 个 Body，一步的分配调用都是零。每个 Body 约占 196 请求字节；已稳定的场景不持有任何
Contact 数据。

## 13. Explicitly out of scope

Each entry records what would bring it back.

| Not doing | Revisit when |
|---|---|
| Joints and constraints (distance, hinge, ball, spring) | Something must stay attached — doors, suspension, ragdolls |
| Compound Bodies | One visual object needs several shapes — a vehicle, a humanoid |
| Collision filtering (layers/masks) | Triggers arrive, or anything must not hit itself |
| Triggers and sensors | Anything must react to overlap without being pushed |
| Overlap queries | Explosions, AI perception, or area effects |
| Shapecast / swept queries | Movement must be predicted before committing |
| Character controller | A player must climb steps and slopes |
| Continuous collision detection | Fast Bodies tunnel — see the risks below |
| Multiple Worlds | Independent regions must simulate separately |
| Render interpolation | The render rate drops below the step rate |
| Determinism | Replay, netcode, or batch simulation becomes a real requirement |

| 不做 | 何时重启 |
|---|---|
| 关节与约束（位移、铰链、球窝、弹簧） | 有东西必须连在一起——门、悬挂、布娃娃 |
| 复合碰撞体 | 一个视觉对象需要多个形状——载具、人形 |
| 碰撞过滤（层/掩码） | 触发器出现，或有东西必须不撞自己 |
| 触发器与传感器 | 需要在重叠时反应但不被推动 |
| 重叠查询 | 爆炸、AI 感知、范围效果 |
| 形状投射 / 扫掠查询 | 移动需要在提交前预测 |
| 角色控制器 | 玩家需要爬台阶、走斜坡 |
| 连续碰撞检测 | 快速 Body 发生隧穿——见下方风险 |
| 多 World | 不同区域需要独立模拟 |
| 渲染插值 | 渲染帧率低于步进频率 |
| 确定性 | 回放、联机或批处理成为真实需求 |

## 14. Risks

1. **EPA numerics.** The single real trap in the layered narrowphase. EPA is
   unstable at shallow penetration; budget debugging time for it specifically.
2. **Internal-edge filtering plus shared topology** is the largest single work
   item: the engine must rebuild adjacency from a flat vertex list with no
   topology, index it by world position, and maintain it as static meshes are
   added and removed.
3. **Six shapes across a hybrid narrowphase** is the widest surface area. The
   curved shapes (cylinder, cone) are the hardest contact-manifold cases and
   should be scheduled last within §5.
4. **The two-thousand-Body budget is now a measurement, with one gap left.** §12
   records it: 0.41 ms asleep, 32 ms at the peak of a fully packed pile that settles
   to 8.8 ms. The gap is the *combination* — physics and the software rasterizer
   share one CPU and have been profiled only separately, never in the same frame. A
   scene that both simulates and draws two thousand Bodies needs the two costs added
   before any frame rate is promised.
5. **No CCD means tunnelling.** The flight simulator is the concrete case: direct
   `set_velocity`, heavy gravity, and terrain cells twelve metres apart. Either
   add CCD, thicken the terrain, or cap speed — decide with a measurement.
6. **No collision filtering.** Correct for this feature set, but it becomes
   mandatory the moment triggers or self-collision appear.
7. **Tall stacks are limited by the solver, not by the manifold — measured.** §12
   records it: a six-layer pile settles and sleeps (0.05 ms), while eight- and
   twelve-layer piles never sleep at all, their residual speed holding near
   0.16 m/s against a 0.05 m/s threshold. Raising the iteration count does not
   rescue them: at 48 velocity iterations an eight-layer pile is still awake, at
   close to four times the cost. Plain sequential impulses converge by Gauss-Seidel,
   so the iteration count would have to scale with the height of what is stacked, and
   past a handful of layers no practical count is enough. The identified fix is a
   block solver over the four points of a box manifold. Separately, the profile says
   most of one velocity iteration goes into `apply_inv_inertia`, which recomputes two
   quaternion rotations on every impulse; hoisting `inv_inertia(arm × axis)` per
   point and axis would cut that, but it buys speed, not convergence, and would not
   make these piles sleep. Until the convergence work lands, do not promise that a
   dense pile settles.

1. **EPA 数值稳定性。** 分层窄相位唯一的真坑。EPA 在浅穿透下不稳定，要专门为它留
   调试时间。
2. **内部边过滤 + 共享拓扑**是最大的单块工作量：引擎必须从没有拓扑的扁平顶点表重建
   邻接关系、按世界位置建立索引，并在静态网格增删时维护它。
3. **六种形状 × 混合窄相位**是最宽的工作面。曲面形状（圆柱、圆锥）是最难的接触流形
   情形，应排在 §5 的最后。
4. **两千个 Body 的预算现在是实测，但还差一块。** §12 已记录：睡眠 0.41 ms，塞满接触的堆叠
   峰值 32 ms、稳态 8.8 ms。缺的是**两者叠加**——物理与软件光栅化器抢同一批 CPU，目前只
   分别测过，从未在同一帧里一起量。既模拟又绘制两千个 Body 的场景，需要先把两份开销相加，
   再谈帧率。
5. **没有 CCD 意味着隧穿。** flight simulator 就是具体案例：直接 `set_velocity`、
   大重力、地形格子间距十二米。要么加 CCD，要么加厚地形，要么限制速度——用实测决定。
6. **没有碰撞过滤。** 对当前功能集是正确的，但一旦触发器或自碰撞出现就变成刚需。

7. **高塔的限制来自求解器，而非流形——已实测。** §12 已记录：六层堆叠会稳定并入睡
   （0.05 ms），而八层与十二层堆叠**永不入睡**，残余速度稳定在约 0.16 m/s，远超 0.05 m/s
   的阈值。调高迭代次数救不了它们：48 次速度迭代下八层堆叠仍然清醒，代价却接近四倍。纯
   序列冲量靠 Gauss-Seidel 收敛，迭代次数必须随堆叠高度增长，而超过若干层后没有一个实用的
   次数够用。已识别的修法是针对盒流形四个点做块求解器。另外，profile 显示一次速度迭代的
   大部分时间花在 `apply_inv_inertia` 上——它对每个冲量重算两次四元数旋转；把
   `inv_inertia(arm × axis)` 按点与轴预先算好可以削减它，但那买到的是速度而非收敛，不会让
   这些堆叠入睡。在收敛那一项落地之前，不要承诺密集堆叠会稳定。

## 15. Acceptance criteria

- `no_std + alloc` preserved: the bare-metal target check and the
  `--no-default-features` check both pass.
- `cargo fmt --all --check`, `cargo clippy --workspace --all-targets -- -D warnings`,
  and `cargo test --workspace` are green.
- A stack of Bodies settles and sleeps: no jitter, no sinking, no drift, and the
  whole Island wakes when one member is disturbed — within the height the solver can
  converge (six layers at the default iterations, §12).
- A Body sliding across flat static geometry does not catch — neither on a
  triangle's internal edge nor on a boundary between two static meshes.
- Answering "is this Body resting on something" requires only the Contact list.
  No hand-rolled geometry tests with tuned tolerances.
- Two thousand sleeping Bodies cost approximately nothing (0.41 ms, §12), and the
  cost of two thousand active Bodies is measured and recorded with the frame budget
  stated as a number (0.23 ms out of contact; 32 ms at the packed peak, settling to
  8.8 ms). What the solver cannot settle is recorded beside it: piles eight layers
  high and taller never sleep.
- Determinism is **not** tested, because it is not a goal.

- 保持 `no_std + alloc`：裸机 target 检查与 `--no-default-features` 检查均通过。
- `cargo fmt --all --check`、`cargo clippy --workspace --all-targets -- -D warnings`
  与 `cargo test --workspace` 全绿。
- 一叠 Body 静止后进入睡眠：不抖、不下沉、不漂移；扰动其中任一个时整个 Island 醒来——在
  求解器还能收敛的高度之内（默认迭代次数下是六层，见 §12）。
- Body 在平坦静态几何上滑行不会卡——既不卡在三角形的内部边上，也不卡在两个静态网格
  的边界上。
- 回答"这个 Body 是不是搁在什么东西上"只需要 Contact 列表，不需要任何带调参容差的
  手写几何判定。
- 两千个睡眠 Body 的开销约等于零（0.41 ms，§12）；两千个活跃 Body 的开销经过实测并记录，
  帧预算以一个具体数字写明（不接触 0.23 ms；塞满接触时峰值 32 ms、稳态 8.8 ms）。求解器
  无法稳定的部分也一并记录：八层及更高的堆叠永不入睡。
- **不**测试确定性，因为它不是目标。

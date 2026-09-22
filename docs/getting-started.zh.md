# HE3D 入门

这套引擎按像素计费，不按三角形。本文从零开始：先画出一个受光的立方体，再让一个箱子
落到地面上，最后把箱子换成从文件读取的模型。

---

## 一、这套引擎是什么

HE3D 是一套软件 3D 引擎：在 CPU 上把场景光栅化为一块 RGBA 像素缓冲，然后交由呈现
后端送出。它不依赖 GPU，不含着色器语言，也不包装任何图形 API。由此得到的是可移植性
与 `no_std` 支持，代价是填充率会成为帧预算的主要支出。

以下约定贯穿全部接口：

| 项目 | 约定 |
|---|---|
| 手性 | 右手系 |
| 上方向 | `+Y` |
| 前方向 | `+Z`，因此朝向为单位四元数的相机沿 `+Z` 观察 |
| 深度 | 逆深度，即 `1/w`；设有近裁剪面，未设远裁剪面 |
| 纹理原点 | 左上角，即 `v = 0` 对应首行像素 |

三个坐标轴连同手性一起确定了「前」是哪一侧。右手系中 `x` 向右、`y` 向上、`z` 向前，
因此相机看向前方即看向 `+Z`；一个位于相机前方的物体，其 `z` 分量大于相机自身的 `z`
分量。

深度缓冲保存的是距离的倒数而非距离本身。这样做的效果是近处的精度远高于远处，代价是
远处两个物体的前后关系可能无法分辨；由于不存在远裁剪面，远处的物体不会被裁掉，只是
排序可能出错。

纹理原点在左上角，与 BMP、PNG、JPEG 的存储顺序一致，读取时无须翻转 `v`。glTF 同样
以左上角为原点。

---

## 二、先把画面跑起来

代码库里有四个可运行的例子，覆盖的能力由少到多：

| 命令 | 内容 |
|---|---|
| `cargo run -p he3d-triangle` | 最小的程序，一个旋转的三角形 |
| `cargo run -p he3d-physics-test` | 物理、键盘输入与位姿复制，WASD 与空格操作 |
| `cargo run -p he3d-biplane` | 读取 OBJ 模型与 BMP 纹理 |
| `cargo run -p he3d-flight-simulator` | 全部功能合用，另含流式加载的地形 |

四条命令之后均可追加 `-- --frames 60`，程序在运行固定帧数后退出。四个例子都依赖 SDL3
后端，因此需要系统中已安装 SDL3；把依赖与 `main` 里的后端类型一起换掉，同一份程序就能
把画面绘制为半块字符写入标准输出，不需要显示设备。

---

## 三、一个最小程序

先加入依赖：

```toml
[dependencies]
he3d = { version = "0.2", features = ["std"] }
he3d-platform-std = "0.2"
```

后端是一个独立 crate，由二进制自己依赖并构造。这里选标准输出后端：它不需要系统中的
任何东西，因此任何机器上都能运行，包括 SSH 会话与无头 CI。想要窗口，把第二行换成
`he3d-platform-sdl3 = "0.2"`，再把下面的 `StdBackend` 换成 `SdlBackend` 即可。

`he3d` 自己不依赖任何后端，`features = ["std"]` 只表示给各库 crate 开 `std`，与选哪个
后端无关。选后端这件事没有 Cargo 特性参与：特性在依赖图里是叠加的，那样的特性会由碰巧
启用它的 crate 决定，而不是由应用决定。

下面的程序在终端里绘制一个受光照的旋转立方体，按下 Esc 键即退出。

```rust
use he3d::{
   BackendConfig, Camera, ColorRgba8, Event, KEY_ESCAPE, Material, Mesh, Platform, Quat,
   RenderObject, Renderer, RendererConfig, Rgb, Scene, Texture, Transform, Vec3,
};
use he3d_platform_std::StdBackend;

fn main() {
   // 1. 后端：程序与宿主环境之间的接口，由二进制自己构造。
   let mut backend = StdBackend::new(BackendConfig::new("hello he3d", 640, 360))
      .expect("std backend");

   // 2. 场景：两张资源表、一份对象列表与一盏平行光。
   let mut scene = Scene::new();
   scene.light.direction = Vec3::new(1.0, 1.0, -1.0).normalize();
   // 网格与纹理是彼此独立的资源，由句柄指代。
   let cube = scene.meshes.insert(Mesh::cube(1.0, 1.0, 1.0).expect("cube"));
   let blue = scene
      .textures
      .insert(Texture::solid(ColorRgba8::from_rgb(90, 160, 255)));

   // 3. 对象：引用网格与纹理，自带变换与材质。
   let object = scene.spawn(RenderObject {
      mesh: cube,
      texture: Some(blue),
      transform: Transform::from_position(Vec3::new(0.0, 0.0, 4.0)),
      material: Material::new(Rgb::WHITE),
      visible: true,
   });

   // 4. 相机：位置、朝向与垂直视野。
   let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 60.0);

   // 5. 渲染器：持有帧缓冲，且跨帧保有状态，因此只创建一次。
   let mut renderer = Renderer::new(RendererConfig::new(640, 360)).expect("renderer");

   // 6. 循环：取事件、更新场景、绘制、呈现。
   let mut angle = 0.0_f32;
   let mut previous = backend.time_seconds();
   while !backend.should_close() {
      // 取回本帧累积的事件；Esc 只置起关闭标志，本帧照常画完。
      for event in backend.poll_events() {
         if let Event::KeyDown(KEY_ESCAPE) = event {
            backend.request_close();
         }
      }

      // 帧间隔上限 0.1 秒：拖动窗口或命中断点后不会一次转过好几圈。
      let now = backend.time_seconds();
      let delta = ((now - previous) as f32).clamp(0.0, 0.1);
      previous = now;
      angle += delta;

      // 每一帧都把新的朝向写进对象。
      if let Some(cube) = scene.objects.get_mut(object) {
         cube.transform.orientation = Quat::from_rotation_y(angle);
      }

      // 先画进帧缓冲，再交给后端。
      renderer
         .render(&scene, &camera, Rgb::new(0.08, 0.10, 0.14))
         .expect("render");
      backend
         .present(renderer.width(), renderer.height(), renderer.pixels())
         .expect("present");
   }
}
```

程序由六个部分构成，以下逐段说明。

### 创建后端

```rust
// 1. 后端：程序与宿主环境之间的接口，由二进制自己构造。
let mut backend = StdBackend::new(BackendConfig::new("hello he3d", 640, 360))
   .expect("std backend");
```

后端是程序与宿主环境之间唯一的接口：取回事件，送出像素，外加一小组生命周期与时钟方法。
它是一个实现了 `Platform` 的具体类型，由这段代码自己构造。随仓库发布的有两个实现，
别处也可以提供，而引擎对此不作规定，也不参与选择。

`BackendConfig::new` 接收标题、渲染宽度与渲染高度，后两者以像素计。标准输出后端没有
标题栏，因此忽略第一个参数。

这一行之后，程序只需通过七个方法驱动它：`present`、`poll_events`、`should_close`、
`request_close`、`time_seconds`、`sleep_ms` 与 `set_title`。时钟与休眠也在其中：`no_std`
与 WebAssembly 目标既没有可移植的时钟，也没有可移植的休眠，只有后端知道手边有什么。

`expect` 在失败时打印消息并终止程序。标准输出后端的构造只在两种情况下失败：尺寸为零，
或标准输入无法进入原始模式。桌面上这一层很少出错，出错时也确实没有别的可做，因此直接
终止是合理的。

`Platform` 也必须引入。后端是具体类型，而它那几个方法来自 trait；trait 不在作用域内时，
调用会报"没有找到名为 `present` 的方法"，只有在提示的下一行才会提到它。

绑定声明为 `mut`，因为取事件与呈现都要求可变借用。

### 构造场景

```rust
// 2. 场景：两张资源表、一份对象列表与一盏平行光。
let mut scene = Scene::new();
scene.light.direction = Vec3::new(1.0, 1.0, -1.0).normalize();
// 网格与纹理是彼此独立的资源，由句柄指代。
let cube = scene.meshes.insert(Mesh::cube(1.0, 1.0, 1.0).expect("cube"));
let blue = scene
   .textures
   .insert(Texture::solid(ColorRgba8::from_rgb(90, 160, 255)));
```

场景是纯数据，不含渲染器状态，也不含任何回调。它由两张资源表、一份对象列表与一盏
平行光（directional light）构成。

光源方向是**从物体指向光源**的方向，着色公式取表面法线与该方向的点积。`(1, 1, -1)`
指向右上方并偏向相机一侧，因此受光的是朝上、朝右与朝向相机的三个面。渲染器会自行
归一化这个向量，此处的调用只是把意图写明：只有方向有意义，长度无关。

`meshes.insert` 把一份网格放进资源表，返回一枚**代际句柄**（generational handle）。
句柄是 `Copy` 语义的普通值，而不是指针；它所指向的那一项被移除之后，用旧句柄读取会
得到 `None`，不会形成悬垂引用。

`Mesh::cube` 的三个参数是宽、高、深，以单位计，立方体以原点为中心。它只在某个尺寸为
零、为负或非有限时才失败，因此在示例里可以直接 `expect`。

`Texture::solid` 生成一张 1×1 的纹理，每个纹素都是给定颜色，效果相当于给物体刷一层
纯色，比准备一张图片省事。`ColorRgba8::from_rgb` 的三个参数是 0 至 255 的整数，这些
分量按线性光使用，不做 sRGB 转换。

### 放置对象

```rust
// 3. 对象：引用网格与纹理，自带变换与材质。
let object = scene.spawn(RenderObject {
   mesh: cube,
   texture: Some(blue),
   transform: Transform::from_position(Vec3::new(0.0, 0.0, 4.0)),
   material: Material::new(Rgb::WHITE),
   visible: true,
});
```

一个渲染对象就是一份网格、一张纹理、一个变换、一个材质，外加一个可见性开关。纹理是
可选项，不给出时物体显示为内置的 2×2 品红与黑相间的棋盘，用来一眼看出某个对象漏了
贴图。可见性设为 `false` 的对象仍然留在场景里，只是不被绘制。

变换由位置、朝向与缩放三部分组成。`Transform::from_position` 把位置设为给定值，朝向
取单位四元数，缩放取 1。相机位于原点并沿 `+Z` 观察，因此把物体放在 `z = 4` 处，就是
放在相机正前方四米。

材质只有一个字段，即色调（tint），它乘以采样到的纹素。`Rgb::WHITE` 三个分量都是 1，
因此纹理原样呈现；改这个字段即可在不换纹理的情况下改变物体的颜色。

`spawn` 返回对象编号（`ObjectId`），此后修改这个对象都要用到它。

### 设定相机

```rust
// 4. 相机：位置、朝向与垂直视野。
let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 60.0);
```

相机是针孔相机，位置、朝向与视野三者各自独立，且只是一个普通值类型，从不由渲染器
持有。朝向取单位四元数时沿 `+Z` 观察；`Vec3::ZERO` 表示位于世界原点。第三个参数是
**垂直**视野，以度为单位：60 度在 16:9 的窗口上对应约 91 度的水平视野。近裁剪面默认
位于 0.1 处，没有远裁剪面。

### 创建渲染器

```rust
// 5. 渲染器：持有帧缓冲，且跨帧保有状态，因此只创建一次。
let mut renderer = Renderer::new(RendererConfig::new(640, 360)).expect("renderer");
```

渲染器持有帧缓冲，即一整块 640×360 的 RGBA 像素。尺寸为零或总量超过内存上限时构造
失败，因此返回 `Result`。帧缓冲的宽高应与后端的宽高一致，后者会用这两个数来校验收到的
缓冲区。

渲染器跨帧保有内部状态，例如时间性抗锯齿的历史缓冲，因此**只应创建一次**，不要每帧
重建。重建会让这些状态每帧归零，画面会持续闪烁。

### 驱动循环

```rust
// 6. 循环：取事件、更新场景、绘制、呈现。
while !backend.should_close() {
   // 取回本帧累积的事件；Esc 只置起关闭标志，本帧照常画完。
   for event in backend.poll_events() {
      if let Event::KeyDown(KEY_ESCAPE) = event {
         backend.request_close();
      }
   }
   // ...
}
```

循环条件取自后端的关闭标志：按下 Esc（标准输出后端也会把 Ctrl+C 当作 Esc）或关闭窗口，
该标志变为真，循环随之结束。

`poll_events` 非阻塞地取回当前累积的事件，返回一个可遍历的集合，取空即返回。按下
Esc 时调用 `request_close` 而不是直接跳出循环：它只置起标志，本帧照常画完，直到循环
顶部才退出。终端的原始模式与光标由后端在析构时恢复，本帧画完再退出，屏幕不会停在半帧。

```rust
// 帧间隔上限 0.1 秒：拖动窗口或命中断点后不会一次转过好几圈。
let now = backend.time_seconds();
let delta = ((now - previous) as f32).clamp(0.0, 0.1);
previous = now;
angle += delta;
```

`time_seconds` 返回一个单调递增的秒数，类型是 `f64`，相邻两帧之差即帧间隔。上限 0.1
秒是为长时间停顿准备的：拖动窗口或命中断点时，两帧之间可能隔了好几秒，不设上限会让
立方体突然转过好几圈。下限 0 用于应付时钟回退一类的意外。

`angle` 以弧度累加，每秒钟增加 1，因此立方体每秒自转约 57 度。

```rust
// 每一帧都把新的朝向写进对象。
if let Some(cube) = scene.objects.get_mut(object) {
   cube.transform.orientation = Quat::from_rotation_y(angle);
}
```

对象的读写都经过编号，查询返回 `Option`。这里每一帧都重写朝向，而位置保持不变。
场景的修改在调用 `render` 之前不会产生任何可见效果。

```rust
// 先画进帧缓冲，再交给后端。
renderer
   .render(&scene, &camera, Rgb::new(0.08, 0.10, 0.14))
   .expect("render");
backend
   .present(renderer.width(), renderer.height(), renderer.pixels())
   .expect("present");
```

`render` 把场景画进帧缓冲，第三个参数是清屏色，此处是一种接近黑色的深蓝。`present`
把缓冲区交给后端。`pixels()` 给出的是整块缓冲，`width()` 与 `height()` 单独传入，是
为了让后端校验缓冲区长度与声明尺寸是否相符。

两个调用都返回 `Result`：渲染只在相机参数不合法时失败（近裁剪面或视野不是有限正数），
呈现则可能因为后端无法完成而失败（窗口被系统关闭，或标准输出写不进去）。示例用
`expect` 处理，正式程序可以改为按错误类型区别对待。

---

## 四、几条贯穿全局的约定

**句柄而非引用。** 公开接口不交出指向场景内部的引用，只交出句柄。持有句柄，以它查询，
处理 `None`。移除资源是安全的，且经过检查。

**数据与绘制分离。** 场景负责描述，渲染器负责绘制。修改场景在调用 `render` 之前不会
产生任何可见效果。因此接口中不需要回调，也不需要订阅机制。

**时间归应用所有。** 引擎不提供 `run`，也不锁定帧率。应用取得 `time_seconds`，自行
决定步长。物理需要恒定的步长（`world` 是下一章要建的物理世界），因此两个演示循环都采用
固定步长累加器：

```rust
const FIXED_STEP: f64 = 1.0 / 120.0;
const MAX_STEPS_PER_FRAME: u32 = 8;

let mut accumulator = 0.0_f64;
// 每帧：frame_delta 是上一帧到这一帧的时长，以秒计。
accumulator += frame_delta.clamp(0.0, 0.25);
let mut steps = 0;
while accumulator >= FIXED_STEP && steps < MAX_STEPS_PER_FRAME {
   world.step(FIXED_STEP as f32);
   accumulator -= FIXED_STEP;
   steps += 1;
}
```

时间以 `f64` 累加、以 `f32` 交给物理，这样长时间的累加不会积累成可见的漂移。两个
上限用于应对长时间停顿：单帧时间上限防止积压过多，每帧步数上限防止追帧过程自身失控。
一旦步数到顶而时间仍未追平，剩余的时间应当丢弃，否则下一帧会继续追赶，帧率再也回不
到正常水平。

**单位由应用决定。** 引擎不规定长度单位，但物理的默认值只在一种选择下才有意义：`1`
单位等于 `1` 米。此时默认重力为 `-9.81`，睡眠阈值为每秒 `0.05` 单位。采用其他单位时，
重力、阻尼与宽阶段格子尺寸都需要重新调整。

**物理不了解渲染。** `he3d-physics` 只依赖 `he3d-math`，其中不存在 `Scene`、`ObjectId`
或 `Transform`。这一取舍的代价由下一节说明。

---

## 五、物理

### 建立世界与物体

物理世界（`World`）是一份独立的数据。它与场景之间没有任何自动联系，两者各自持有一份
摆放，由应用负责在每帧把物理的结果抄给渲染。

先建世界，再往里放物体：

```rust
use he3d::{BodyDesc, Shape, Vec3, World};

let mut world = World::new();

// 地面：永不移动，因此用 stationary，且不需要质量。
let floor = world.add_body(
   BodyDesc::stationary(Shape::cuboid(Vec3::new(4.0, 0.1, 4.0)))
      .at(Vec3::new(0.0, -0.1, 0.0)),
);
// 箱子：1 千克，从 4 米高处静止落下。
let cube = world.add_body(
   BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0)
      .at(Vec3::new(0.0, 4.0, 0.0))
      .with_friction(0.45)
      .with_restitution(0.4),
);

world.step(1.0 / 120.0);
```

`BodyDesc` 描述一个物体的全部参数。`Shape::cuboid` 接收**半长**而非全长，
`Vec3::splat(0.5)` 因此得到一个边长 1 米的立方体；同名的 `Shape::sphere` 接收半径。
形状的合法性在构造时就已处理（负的半长被钳制为零），因此插入世界不会失败，`add_body`
也就不需要返回 `Result`。

三种运动模式互相排斥，由构造方法选定：

| 模式 | 行为 |
|---|---|
| `dynamic` | 受重力、力、冲量与接触影响；需要给定质量 |
| `stationary` | 永不移动，也没有任何接触能移动它；不需要质量 |
| `kinematic` | 每步按应用设定的速度移动，没有任何接触能改变这个速度 |

`dynamic` 的参数是形状与质量，单位是千克；惯量张量由形状与质量推出，因此两者不可能
互相矛盾。同样的力矩之下，质量越集中在外围，角加速度越小。

其余的构造参数均为链式调用：

| 方法 | 作用 |
|---|---|
| `at` | 设定初始位置 |
| `oriented` | 设定初始朝向 |
| `with_velocity` | 设定初速度 |
| `with_angular_velocity` | 设定初角速度 |
| `with_friction` | 设定库仑摩擦系数 |
| `with_restitution` | 设定弹性，0 为不反弹 |
| `with_damping` | 设定每秒的线速度与角速度衰减 |
| `with_gravity_scale` | 缩放重力对该物体的作用 |

重力系数取 0 即完全不受重力，飞行器一类的物体靠它保持高度。

`add_body` 返回物体编号（`BodyId`），其性质与对象编号相同：可以自由复制，移除之后旧
编号读作 `None`。

`step` 把模拟推进一个**固定**的时间段，参数即该时间段，以秒计。传入零、负数或非有限
值时调用被忽略，World 的状态不受影响。每一步之内，引擎依次积分速度、找出接触、求解
约束、积分位置，最后发布接触表。

力与力矩只作用一步：`apply_force` 与 `apply_torque` 累积的量在下一次 `step` 中生效，
随后被清除，因此持续施力需要每帧调用。`apply_impulse` 与 `apply_angular_impulse` 是
一次性版本，其引起的速度变化等于冲量除以质量。

读取状态的查询都返回 `Option`，包括位置、朝向、线速度、角速度、摩擦、弹性与是否正在
睡眠。写入则返回 `bool`，表示该编号是否仍然有效。`set_pose` 一次设定位置与朝向，
`set_velocity` 与 `set_angular_velocity` 直接覆盖速度，不是力，因此不受「只作用一步」
的限制。把物体瞬移之后，引擎会顺带唤醒它。

### 关联由应用拥有

物体编号与对象编号之间没有关联。维持它们的对应关系，并在每帧把物体的位置与朝向复制
到渲染对象之上，是应用的责任。

承担这件事只需要一个结构体：

```rust
/// Body 与"画它的东西"之间的联系，由应用拥有。
struct Thing {
   body: BodyId,
   object: ObjectId,
}
```

以及一个函数：

```rust
/// 把 Body 的摆放复制到绘制它的对象上。
fn sync_pose(world: &World, scene: &mut Scene, thing: &Thing) {
   let (Some(position), Some(orientation)) =
      (world.position(thing.body), world.orientation(thing.body))
   else {
      return;
   };
   if let Some(object) = scene.objects.get_mut(thing.object) {
      object.transform.position = position;
      object.transform.orientation = orientation;
      // 缩放只属于渲染器；物理没有缩放。
   }
}
```

两个查询都返回 `Option`，因为编号可能已经失效；对象一侧同理。缩放刻意不复制：物理
的碰撞形状没有缩放这个概念，需要更大的碰撞体就应当指定更大的形状。

整个适配器只有十五行。代价是这十五行需要维护，收益是物理 crate 永远不必知道 `ObjectId`
是什么。

### 判断是否触地

不要自行编写几何判定：刚刚跑完的那一步已经给出了答案。

```rust
// 接触表里有涉及 cube、且阶段不为 End 的条目，即为触地。
let grounded = world
   .contacts()
   .iter()
   .any(|contact| contact.involves(cube) && contact.phase != ContactPhase::End);
```

每调用一次 `step`，物理世界都会发布一份**接触表**。表中每一条给出接触的双方（可能是
一个物体，也可能是一块静态几何）、接触点、法线、深度，以及一个**阶段**：`Begin`、
`Persist` 或 `End`。遍历这份表比手写的任何重叠判定都更便宜，也更准确，因为引擎已经
算过一遍了。

### 睡着的物体不再产生约束

三个箱子叠在桌面上。一秒半之后，它们全睡着了。这不是反应变慢，而是引擎不再计算它们：
既不施加重力，也不解接触，仿佛这一叠东西已从世界上消失。这一步是必要的，否则几百个
静止的箱子会让每一帧都在重解同一组方程。

现在推最底下那个。

它醒了。值得注意的是，上面两个也醒了。这不是巧合：睡眠以**岛**（Island）为单位，而岛
由接触图上的并查集得出。三个箱子彼此接触，因此属于同一个岛，要么一起睡，要么一起醒。
按单个物体分别睡眠时，最底下那个被抽走之后，上面两个会悬在半空，直到有人碰它们才
掉下来。

判断上面那个箱子是否被托住，只需查一次接触表：

```rust
// 三个箱子叠起来，最上面那个叫 top。
let bottom = world.add_body(BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0));
let middle = world.add_body(BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0).at(Vec3::new(0.0, 1.0, 0.0)));
let top = world.add_body(BodyDesc::dynamic(Shape::cuboid(Vec3::splat(0.5)), 1.0).at(Vec3::new(0.0, 2.0, 0.0)));
```

```rust
// 上面那个箱子此刻是否受托，同样只看接触表。
let supported = world
   .contacts()
   .iter()
   .any(|contact| contact.involves(top) && contact.phase != ContactPhase::End);
```

须在它仍清醒时发问。睡着的物体不产生约束，因此不出现在接触表里，此时 `supported` 会是
`false`，而它明明好好地叠在那里。这不是缺陷，只是睡眠的定义。需要它回答，先行唤醒即可：

```rust
world.wake(top); // 唤醒之后，接触表里才会重新出现它。
```

### 静态几何

静止的东西可以是盒或球，也可以是任意形状的三角形网格。地形与关卡几何应改用三角形：

```rust
// 世界空间的三角形列表，每三个顶点构成一个三角形，
// 与 Mesh::vertices() 的布局相同。
let terrain = world.add_static_mesh(&positions);
```

顶点列表为空、长度不是三的倍数，或存在非有限坐标时，这个调用返回 `None`。

静态网格没有变换，其顶点按定义即位于世界空间。它们还会焊接进同一套共享拓扑，因此两块
瓦片之间的接缝不再被算作可供物体卡住的棱边。

---

## 六、加载模型

网格与纹理都可以从文件读入。两个解析函数接收 `&[u8]`，因此不关心字节来自文件、网络
还是内嵌数组：

```rust
use he3d::{
   Material, ObjectId, RenderObject, Rgb, Scene, Transform, Vec3, decode_image, parse_obj,
};

fn load_model(scene: &mut Scene) -> ObjectId {
   // 先读字节，再解析；两处都写明失败原因，出错时能看出是哪一步。
   let obj_bytes = std::fs::read("model.obj").expect("model.obj is missing");
   let bmp_bytes = std::fs::read("model.bmp").expect("model.bmp is missing");
   let mesh = parse_obj(&obj_bytes).expect("model.obj is not a valid OBJ");
   let texture = decode_image(&bmp_bytes).expect("model.bmp is not a valid image");

   // 解析结果进入资源表，最后由对象引用。
   let mesh_id = scene.meshes.insert(mesh);
   let texture_id = scene.textures.insert(texture);
   scene.spawn(RenderObject {
      mesh: mesh_id,
      texture: Some(texture_id),
      transform: Transform::from_position(Vec3::ZERO),
      material: Material::new(Rgb::WHITE),
      visible: true,
   })
}
```

文件读取返回的是标准库的错误类型，解析返回的是 `he3d::Error`，两者之间没有转换，因此
无法用同一个 `?` 串起来。示例统一用 `expect`，并把失败原因写进消息，这样出错时能直接
看出是哪一步。

`decode_image` 按内容而非按扩展名判断图片格式。BMP 始终可用；PNG 与 JPEG 各需对应的
特性。glTF 需要 `gltf` 特性，并有两种入口：`parse_gltf(json, buffers)` 与
`parse_glb(bytes)`。

完整的写法见 `examples/biplane`，其中包含文件缺失时的处理。

---

## 七、后端

后端由二进制选，方式是在依赖里写上它，并在 `main` 里点名它的类型。仓库里有两个实现：

| crate | 类型 | 呈现位置 | 额外要求 |
|---|---|---|---|
| `he3d-platform-std` | `StdBackend` | 标准输出，字符半块 | 无 |
| `he3d-platform-sdl3` | `SdlBackend` | 真实窗口 | 系统已安装 SDL3 |

两者都实现 `he3d-platform` 里的 `Platform`，因此循环里的代码对二者完全相同：换后端只需
换依赖与那一行类型名。第三方后端同理，只要能实现这个契约。

有一条硬性差别值得注意：依赖 SDL3 后端就让 SDL3 成为**构建**前提，因为 `sdl3-sys` 声明了
`links = "SDL3"`，cargo 在编译阶段就要找到这个库。因此需要构建于无头机器上的程序，应当
让 `he3d-platform-sdl3` 彻底不出现在依赖图里。这里没有可以把它藏起来的特性，逃生通道
就是类型名本身。

引擎不提供代选后端的函数，也不提供按名字查找的登记表。要在运行期切换，由应用自己把两者
装进 `Box<dyn Platform>`。

资源格式特性转发至 `he3d-assets`：`png`、`jpeg`、`gltf`。

---

## 八、易犯之处

**物理动了，画面不动。** 引擎不会自动把物体的位置抄给渲染对象。忘记每帧复制位姿时，
物体会在物理世界里照常运动，而画面上停在原地。

**睡着的物体没有接触。** 物体稳定之后进入睡眠，而睡眠中的物体不再产生约束，因此从
`contacts()` 中消失。依赖这份表判断某物是否受到支撑时，须在它仍清醒时查询，或先行
`world.wake(id)`。

**物理没有缩放，`Transform` 有。** 位姿复制刻意不修改 `transform.scale`。缩放碰撞体
不在本引擎的能力之内；需要更大的盒子，应为物体指定更大的形状。

**力只作用于一步。** `apply_force` 与 `apply_torque` 作用于下一次 `step`，随后被清除。
需要持续施力，应每帧调用。`apply_impulse` 为一次性版本，其引起的速度变化等于冲量除以
质量。

**质量为零或非有限会使物体无法被移动。** `BodyDesc::dynamic(shape, 0.0)` 不报错，它只是
给出一个重力推不动的物体。物体不下落时，先检查此处。

**渲染器只创建一次。** 每帧重建渲染器会丢掉时间性抗锯齿的历史，画面持续抖动。

**`remove_body` 使句柄失效。** 物体编号带代际，移除之后旧编号读作 `None`，而不会指向
复用该槽位的新物体。

**失效句柄不引起 panic。** `world.position(id)` 返回 `Option`。

**填充率是帧预算的本体。** 光栅化器按像素计费，而非按三角形计费。需要追回帧率时，先
考虑下调分辨率，削减场景规模通常收效甚微。

---

## 九、接下来读什么

| 位置 | 内容 |
|---|---|
| `cargo doc --workspace --no-deps --open` | 完整接口，多数类型附示例 |
| `examples/triangle` | 最小的完整程序 |
| `examples/physics-test` | 物理、输入与位姿复制 |
| `examples/biplane` | 读取 OBJ 与 BMP |
| `examples/flight-simulator` | 全部功能合用，另含流式地形 |
| `CONTEXT.md` | 术语表：Body、Island、Contact、Static geometry |
| `docs/physics-spec.md` | 物理 crate 的能力范围与已知缺口 |
| `docs/adr/` | 各项架构决定及其理由 |
| `docs/writing-style.zh.md` | 本文档所遵循的写作规范 |

---

## 十、为什么这样设计

**物理与渲染之间没有绑定层。** 二者的关联是一份由应用维护的对应表，以及每帧一次位姿
复制。另一种做法是在引擎内部保留关联，让 `Body` 直接携带一个渲染对象编号。那样写起来
更省事，但会让物理 crate 依赖渲染 crate，从而使物理无法在无渲染的场合使用，也无法
独立测试。当前做法的代价是每个应用都要写那十五行。

**以句柄而非引用指代资源。** 句柄是普通值，可以自由复制与存放，移除资源之后旧句柄读作
`None`。以引用指代会迫使场景的借用跨越帧边界，进而把可变性约束扩散到整个应用。

**时间由应用驱动。** 引擎不提供 `run`，因为不同应用对步长、追帧策略与暂停的处理各不
相同。物理需要恒定步长，而渲染需要尽可能快的帧，把两者分开是应用的责任。

**不设远裁剪面。** 深度采用逆深度存储，其精度在近处最高、向远处递减，因此远处无需
硬性截断。

**静态几何共享一套拓扑。** `Mesh` 不保存索引缓冲，共享边上的顶点是重复的，引擎因而会
把每条三角形接缝视为真实的棱边，物体滑过平地时便卡在缝上。把所有静态网格焊接进同一套
拓扑之后，两块独立网格之间的边也能被识别为内部边。另一种做法是为每个网格单独焊接，
但地形以瓦片流式加载，逐网格焊接会让每条瓦片边界依然成为可供卡住的棱边。

**睡眠以岛为单位。** 一叠箱子在物理上本就是一个整体：抽走最底层，上面几层的支撑同时
消失。按物体分别睡眠，等于假设每一层都独立地悬在空中，而这个假设只在它们睡着的时候
看起来成立。

**后端由应用选择。** 引擎里若有一个"构造可用后端"的函数，它就必须先决定哪些后端算
可用，而这件事只能用 Cargo 特性表达；特性在整个依赖图中叠加，于是真正做决定的是碰巧
启用它的那个 crate，而不是应用。曾经还有一个运行期兜底：先试窗口后端，失败再退标准
输出。它会在一台装着 SDL3 的无头机器上安静地开出一个窗口，因为"能构造"被当成了
"想要"。另一种做法是把主循环也收进引擎，让宿主类型泛型于后端。那样省掉应用的一层
代码，但固定步长、追帧上限与暂停策略本来就是各应用自己定的，收进去只会把它们变成
引擎的配置项。当前的代价是每个二进制都要写一行类型名。

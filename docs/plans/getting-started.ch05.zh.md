# 第五章：一个最小程序（草稿）

内部草稿。定稿之后并入 `docs/getting-started.zh.md`，本文件删除。

---

## 五、一个最小程序

本章写出第一份完整程序：一个受光照的立方体在窗口里缓慢旋转，按下 Esc 退出。程序由六个
部分组成，每一部分在下面单独说明。

```rust
use he3d::{
   BackendConfig, Camera, ColorRgba8, Event, KEY_ESCAPE, Material, Mesh, Platform, Quat,
   RenderObject, Renderer, RendererConfig, Rgb, Scene, Texture, Transform, Vec3,
};
use he3d_platform_sdl3::SdlBackend;

fn main() {
   // 1. 后端：程序与宿主环境之间的接口，由二进制构造。
   let mut backend = SdlBackend::new(BackendConfig::new("hello he3d", 640, 360))
      .expect("sdl backend");

   // 2. 场景：三张资源表与一盏平行光。
   let mut scene = Scene::new();
   scene.light.direction = Vec3::new(1.0, 1.0, -1.0).normalize();
   // 网格与纹理是彼此独立的资源，都由句柄指代。
   let cube_mesh = scene.meshes.insert(Mesh::cube(1.0, 1.0, 1.0).expect("cube mesh"));
   let blue = scene
      .textures
      .insert(Texture::solid(ColorRgba8::from_rgb(90, 160, 255)));

   // 3. 对象：引用网格与纹理，自带变换与材质。
   let cube = scene.spawn(RenderObject {
      mesh: cube_mesh,
      texture: Some(blue),
      transform: Transform::from_position(Vec3::new(0.0, 0.0, 4.0)),
      material: Material::new(Rgb::WHITE),
      visible: true,
   });

   // 4. 相机：位置、朝向与垂直视野，单位朝向表示沿 +Z 观察。
   let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 60.0);

   // 5. 渲染器：持有帧缓冲，跨帧保有状态，因此只创建一次。
   let mut renderer = Renderer::new(RendererConfig::new(640, 360)).expect("renderer");

   // 6. 循环：取事件、算帧间隔、改场景、绘制、呈现。
   let mut angle = 0.0_f32;
   let mut previous = backend.time_seconds();
   while !backend.should_close() {
      // 取回本帧累积的事件；Esc 只置起关闭标志，本帧照常画完。
      for event in backend.poll_events() {
         if let Event::KeyDown(KEY_ESCAPE) = event {
            backend.request_close();
         }
      }

      // 帧间隔取上限，免得拖动窗口或命中断点之后一次转过几十圈。
      let now = backend.time_seconds();
      let delta = ((now - previous) as f32).clamp(0.0, 0.1);
      previous = now;
      angle += delta;

      // 每帧把新的朝向写进对象。
      if let Some(cube) = scene.objects.get_mut(cube) {
         cube.transform.orientation = Quat::from_rotation_y(angle);
      }

      // 先画进帧缓冲，再交给后端。
      renderer
         .render(&scene, &camera, Rgb::new(0.08, 0.10, 0.14))
         .expect("render frame");
      backend
         .present(renderer.width(), renderer.height(), renderer.pixels())
         .expect("present frame");
   }
}
```

### 创建后端

```rust
// 1. 后端：程序与宿主环境之间的接口，由二进制构造。
let mut backend = SdlBackend::new(BackendConfig::new("hello he3d", 640, 360))
   .expect("sdl backend");
```

后端是程序与宿主环境之间唯一的接口：取回事件，送出像素，外加一小组生命周期与时钟方法。
`SdlBackend::new` 接收一份后端配置（`BackendConfig`），其中依次是窗口标题、渲染宽度与
渲染高度，后两者以像素计。

构造可能失败：SDL3 未能初始化，或者系统没有可用的显示设备，都会在构造时返回错误。这段代码用
`expect` 处理，失败时打印消息并终止程序；需要自行处理错误时改用 `match`。

绑定声明为 `mut`，因为取事件与呈现都要求可变借用。

`Platform` 必须在作用域里：具体后端的那些方法来自 `trait`，`trait` 不在时，调用会报
「没有找到方法」。

```rust
// 少了 use he3d::Platform
use he3d::BackendConfig;
use he3d_platform_sdl3::SdlBackend;

fn main() {
   let mut backend = SdlBackend::new(BackendConfig::new("no trait", 64, 36))
      .expect("sdl backend");
   backend.request_close();
}
```

```text
error[E0599]: no method named `request_close` found for struct `SdlBackend` in the current scope
   --> src/main.rs:8:12
    |
  8 |    backend.request_close();
    |            ^^^^^^^^^^^^^ method not found in `SdlBackend`
    |
   ::: .../crates/he3d-platform/src/lib.rs:175:7
    |
175 |    fn request_close(&mut self);
    |       ------------- the method is available for `SdlBackend` here
    |
    = help: items from traits can only be used if the trait is in scope
help: trait `Platform` which provides `request_close` is implemented but not in scope; perhaps you want to import it
    |
  2 + use he3d::Platform;
    |
```

把 `Platform` 加进 `use` 即可。上面指向引擎源码的那一帧，路径随安装位置变化，这里用 `...`
省去前缀。

### 构造场景

```rust
// 2. 场景：三张资源表与一盏平行光。
let mut scene = Scene::new();
scene.light.direction = Vec3::new(1.0, 1.0, -1.0).normalize();
// 网格与纹理是彼此独立的资源，都由句柄指代。
let cube_mesh = scene.meshes.insert(Mesh::cube(1.0, 1.0, 1.0).expect("cube mesh"));
let blue = scene
   .textures
   .insert(Texture::solid(ColorRgba8::from_rgb(90, 160, 255)));
```

场景（`Scene`）由三张资源表与一盏平行光（directional light）构成，三张表分别存放网格、
纹理与渲染对象。它不持有渲染器状态，也不含回调。

光源方向是从物体指向光源的方向，着色公式取表面法线与它的点积。`(1.0, 1.0, -1.0)` 指向
右上方并偏向相机一侧，因此受光的是朝上、朝右与朝向相机的三个面。渲染器会自行归一化这个
方向，代码里的 `normalize` 只是把意图写明。

`scene.meshes.insert` 放入一份网格并返回一枚代际句柄（generational handle）。句柄是
`Copy` 语义的普通值；它指向的那一项被移除之后，用旧句柄读取得到 `None`，不会留下悬垂
引用。

`Mesh::cube` 的三个参数是宽、高、深，以单位计，立方体以原点为中心。`Texture::solid` 生成
一张单色纹理，效果相当于给物体刷一层底色。

### 放置对象

```rust
// 3. 对象：引用网格与纹理，自带变换与材质。
let cube = scene.spawn(RenderObject {
   mesh: cube_mesh,
   texture: Some(blue),
   transform: Transform::from_position(Vec3::new(0.0, 0.0, 4.0)),
   material: Material::new(Rgb::WHITE),
   visible: true,
});
```

渲染对象（`RenderObject`）把网格、纹理、变换与材质绑在一起。网格与纹理都以句柄引用资源表
中的项，因此多份对象可以共用同一份网格或同一张纹理。

变换（`Transform`）由位置、朝向与缩放组成，这里只给位置。相机位于原点并朝向 `+Z`，所以
`z = 4` 的立方体落在相机正前方四单位处。

材质（`Material`）目前只有一项：色调（tint），它是乘在采样结果上的颜色，`Rgb::WHITE`
表示按纹理原样着色。

`scene.spawn` 把对象加入场景，返回指向它的代际句柄。

### 设定相机

```rust
// 4. 相机：位置、朝向与垂直视野，单位朝向表示沿 +Z 观察。
let camera = Camera::new(Vec3::ZERO, Quat::IDENTITY, 60.0);
```

相机（`Camera`）由位置、朝向与垂直视野构成，另有一个近裁剪面，此处取默认值 `0.1`。
`Quat::IDENTITY` 表示不旋转，于是相机沿 `+Z` 观察。

视野是垂直方向的张角；水平方向的张角由它与画面宽高比共同决定，因此宽屏窗口的横向视野
比纵向更宽。

相机不交给渲染器保管，绘制时按帧传入引用，随时可以修改。

### 创建渲染器

```rust
// 5. 渲染器：持有帧缓冲，跨帧保有状态，因此只创建一次。
let mut renderer = Renderer::new(RendererConfig::new(640, 360)).expect("renderer");
```

渲染器（`Renderer`）持有帧缓冲，并且跨帧保有状态，因此建在循环之外，只创建一次。

**画面抖** → 渲染器建在循环里面，跨帧状态每一帧都被重置 → 把渲染器建在循环之外。

帧缓冲的尺寸由渲染器配置给出，与后端配置的尺寸各自独立。绘制之后，渲染器把自己的宽高与
像素切片一起交给后端，后端据此校验缓冲长度与声明尺寸是否相符。

绑定同样需要 `mut`，因为绘制要求可变借用。

### 驱动循环

```rust
// 6. 循环：取事件、算帧间隔、改场景、绘制、呈现。
let mut angle = 0.0_f32;
let mut previous = backend.time_seconds();
while !backend.should_close() {
   // 取回本帧累积的事件；Esc 只置起关闭标志，本帧照常画完。
   for event in backend.poll_events() {
      if let Event::KeyDown(KEY_ESCAPE) = event {
         backend.request_close();
      }
   }

   // 帧间隔取上限，免得拖动窗口或命中断点之后一次转过几十圈。
   let now = backend.time_seconds();
   let delta = ((now - previous) as f32).clamp(0.0, 0.1);
   previous = now;
   angle += delta;

   // 每帧把新的朝向写进对象。
   if let Some(cube) = scene.objects.get_mut(cube) {
      cube.transform.orientation = Quat::from_rotation_y(angle);
   }

   // 先画进帧缓冲，再交给后端。
   renderer
      .render(&scene, &camera, Rgb::new(0.08, 0.10, 0.14))
      .expect("render frame");
   backend
      .present(renderer.width(), renderer.height(), renderer.pixels())
      .expect("present frame");
}
```

循环每一轮做五件事：取事件、算帧间隔、改场景、绘制、呈现。

取事件时一次取回自上一轮以来的全部输入。Esc 的处理只是置起关闭标志，本帧照常画完，下一轮
条件为假时退出。事件枚举目前只有按下与松开两个变体，而且是 `non_exhaustive` 的，因此用
`if let` 只挑出关心的一项。

帧间隔是两个时刻之差，并被钳制在一个上限之内：窗口被拖动、机器休眠或者命中断点之后，这个
差可能是若干秒，不钳制会让立方体一次转过几十圈。上限只防止跳变，正常帧的间隔不会被它改变。

`angle` 按帧间隔累加，随后写进对象的朝向；`Quat::from_rotation_y` 构造绕 `y` 轴的旋转。取
对象用的是可能返回 `None` 的接口，因此写成 `if let`。

**改了场景，画面没有变化** → 改动发生在绘制之后 → 先改场景，再绘制。

最后两步是绘制与呈现：`render` 接收场景与相机，把画面写进帧缓冲，第三个参数是背景色；
`present` 把帧缓冲交给后端显示。退出循环无须额外清理，窗口与帧缓冲各自随所属的值释放。

### 易犯之处

- 画面抖：见「创建渲染器」。
- 改了场景，画面没变：见「驱动循环」。
- 方法找不到（`E0599`）：见「创建后端」。

### 问与答

- **`expect` 可以在程序里用吗？** → **可以，但失败时程序立刻终止（恐慌）。** 它打印错误
  消息然后结束进程；希望自行提示或恢复时改用 `match`。
- **引擎为什么不自己跑循环？** → **引擎不拥有主循环**：绘制与呈现都由应用调用。帧率、
  事件处理与物理步长因此全部归应用决定，引擎只负责一次绘制与一次呈现。
- **`present` 为什么单独接收宽高？** → **为了让后端校验像素缓冲长度与声明的尺寸是否
  相符。** 对不上时后端返回错误，不会按错误的长度读取像素。
- **相机朝向怎么算？** → **由应用自行计算**：单位四元数沿 `+Z` 观察，看向某一点可用
  `Quat::from_rotation_arc`，第九章给出具体写法。（→ 第九章）

### 改一改

- 立方体尺寸由 `1.0` 改成 `2.0`。
- 物体的 `z` 由 `4.0` 改成 `8.0`，观察它如何变小。
- 视野由 `60.0` 改成 `90.0`。

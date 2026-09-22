# 第二章：准备环境（草稿）

内部草稿。定稿之后并入 `docs/getting-started.zh.md`，本文件删除。

内部备注：

- Linux 从源码构建 SDL3 的那段直接取自 `.github/workflows/ci.yml`，改 CI 时须同步本节。
- 标准输出后端目前只在 Unix 上可用（`he3d-platform-std` 的 `StdBackend` 以 `#[cfg(unix)]` 导出）；本节照此写明。

---

## 二、准备环境

本章把编译与运行所需的东西装齐：一份足够新的工具链、SDL3，以及两条依赖。

### 工具链

工具链的版本要求写在各 crate 的清单里（`rust-version`），当前是 1.86。低于它的工具链会在构建开始时被 cargo 拒绝，并直接给出要求的版本，不需要自行比对。

### SDL3

窗口后端依赖系统中的 SDL3，每个平台一条命令：

```sh
# Linux：发行版仓库里有这个包时
sudo apt install libsdl3-dev
# macOS
brew install sdl3
# Windows
vcpkg install sdl3
```

发行版的仓库里没有这个包时（Ubuntu 24.04 就没有），从源码构建：CI 走的正是这条路，下面的命令与它相同。

```sh
sudo apt-get update
sudo apt-get install -y build-essential cmake libx11-dev libxext-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libwayland-dev libxkbcommon-dev
git clone --depth 1 --branch release-3.2.0 https://github.com/libsdl-org/SDL.git /tmp/SDL
cmake -S /tmp/SDL -B /tmp/SDL/build -DSDL_SHARED=ON -DSDL_STATIC=OFF -DSDL_TEST_LIBRARY=OFF
cmake --build /tmp/SDL/build -j2
sudo cmake --install /tmp/SDL/build
sudo ldconfig
```

**装完 SDL3 仍然提示找不到这个库** → 动态库缓存没有刷新，构建脚本还看不到刚装进去的库 → 执行 `sudo ldconfig`。

官方文档按平台分篇：Linux（<https://wiki.libsdl.org/SDL3/README-linux>）、macOS（<https://wiki.libsdl.org/SDL3/README-macos>）、Windows（<https://wiki.libsdl.org/SDL3/README-windows>），构建配置另见 CMake 一篇（<https://wiki.libsdl.org/SDL3/README-cmake>）。源码仓库为 <https://github.com/libsdl-org/SDL>。

### 两条依赖

```toml
[dependencies]
he3d = { version = "0.2", features = ["std"] }
he3d-platform-sdl3 = "0.2"
```

`he3d` 是唯一入口，转发其余各库 crate，自身不依赖任何后端。后端是独立 crate，由二进制依赖并构造：换后端就是换这一行依赖，再加上 `main` 里的一行类型名。

`features = ["std"]` 只表示给各库 crate 打开 `std`（错误类型由此实现 `std::error::Error` 一类），与选哪个后端无关。

**构建时找不到后端类型** → 依赖里只有 `he3d`，而它不转发任何后端 crate → 把所选后端的 crate 加进依赖，此处是 `he3d-platform-sdl3`。

### 没有显示设备时

标准输出后端（`he3d-platform-std`）不需要系统中的任何东西：同一份像素被写成半块字符送到标准输出，因此无头机器与远程会话都能运行。它目前只在 Unix 上可用。换它只需替换依赖与类型名，其余代码不变。

### 易犯之处

- 构建时找不到后端类型：见「两条依赖」。
- 装完 SDL3 仍然提示找不到这个库：见「SDL3」。

### 问与答

- **为什么编译时就要有 SDL3？** → **因为 SDL3 的绑定 crate 在编译阶段就要在系统里找到这个共享库，不是运行阶段才需要。** 它把 SDL3 声明为链接目标，因此只要依赖窗口后端，SDL3 就成了构建前提；需要构建于无头机器上的程序，应让 `he3d-platform-sdl3` 不出现在依赖图里。（→ 第四章）
- **只用标准输出后端可以吗？** → **可以，在 Unix 上可以**：依赖换成 `he3d-platform-std`，类型换成 `StdBackend`，画面变成半块字符，系统中不需要任何东西。（→ 第十三章）

### 改一改

- 把后端依赖换成 `he3d-platform-std`，类型名一并换掉，在同一个终端里比较两次输出。
- 把任一示例里的窗口尺寸常量改成别的值，重新运行，观察画面与每帧开销的变化。

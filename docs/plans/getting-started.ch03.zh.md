# 第三章：起步（草稿）

内部草稿。定稿之后并入 `docs/getting-started.zh.md`，本文件删除。

内部备注：`examples/playground` 尚不存在，本节暂列四份示例；它写出之后补进第五行命令与表格一行（第 8、9、11 章都指向它）。

---

## 三、起步

本章的用法是先看到画面，再回到代码：几条命令，各自对应一份可以运行的示例。

### 现成的示例

```sh
cargo run -p he3d-triangle
cargo run -p he3d-physics-test
cargo run -p he3d-biplane
cargo run -p he3d-flight-simulator
```

| 命令 | 内容 |
|---|---|
| `cargo run -p he3d-triangle` | 一个旋转的带纹理三角形，最小的完整程序 |
| `cargo run -p he3d-physics-test` | 静态地板上一个可操控的方块：WASD 施力、空格起跳、R 复位 |
| `cargo run -p he3d-biplane` | 读取 OBJ 模型与 BMP 纹理 |
| `cargo run -p he3d-flight-simulator` | 全部功能合用：流式地形、飞行模型、追逐相机 |

每个示例都按下 Esc 或关闭窗口退出。方块着地之后颜色会变化：接触中的方块偏橙，腾空的方块偏蓝。这一条由物理给出的接触表决定，第六章会给出这张表。

### 固定帧数与退出

```sh
cargo run -p he3d-triangle -- --frames 60
```

`--frames 60` 让程序跑满 60 帧后退出。它是程序接受的参数，不是 cargo 的参数，因此写在 `--` 之后；不带它时程序一直运行，直到按下 Esc 或关闭窗口。

**`--frames` 没有生效** → 它属于程序，cargo 不认识这个参数 → 写在 `--` 之后。

### 易犯之处

- `--frames` 放在 `--` 之前：见「固定帧数与退出」。

### 问与答

- **为什么示例都写死 `SdlBackend`？** → **因为后端由二进制指定，示例也是二进制。** 引擎里没有代选后端的函数，`he3d` 也不依赖任何后端 crate。（→ 第四章）
- **标准输出后端画出来的是什么？** → **半块字符**：一个字符单元对应两个像素，上半用前景色、下半用背景色，颜色是 ANSI 24 位真彩色。（→ 第十三章）

### 改一改

- 给任意一条命令加上 `-- --frames 5`，观察它是怎样退出的。
- 按住 D 把方块推下地板，观察它落到一定高度之后被送回起点。

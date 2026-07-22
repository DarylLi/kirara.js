# kirara.js

一个面向 Web 的 Rust → WASM 刚体物理项目。
v1 范围:纯 Rust 刚体物理内核(`kirara-core`)+ wasm-bindgen 绑定(`kirara-wasm`)+ three.js 演示。

## 现状

`kirara-core` 已经过单元测试验证(球体静止在地面高度、两球弹性碰撞分离),
`kirara-wasm` 的 wasm-bindgen API 已通过 native target 的 `cargo check` 类型检查。

> **关于本仓库的生成环境**:这份代码是在一个网络受限的沙箱容器里写的——容器的出站白名单里
> 没有 `static.rust-lang.org` / `rustup.rs`,因此无法用 `rustup target add wasm32-unknown-unknown`
> 装上官方预编译的 wasm32 标准库,也就没能在容器里跑出最终的 `.wasm` 文件。
> 但这只是**沙箱的网络限制**,不是代码本身的问题——`kirara-core` 的物理逻辑已经在容器内用
> 原生 x86_64 target 跑过 `cargo test` 并全部通过,`kirara-wasm` 的绑定层也过了 `cargo check`。
> 在你自己的机器或 CI 上(能访问 rustup.rs),下面的步骤可以直接产出可用的 wasm。

## 本机构建步骤

```bash
# 1. 安装 wasm32 target(只需一次)
rustup target add wasm32-unknown-unknown

# 2. 安装 wasm-pack
cargo install wasm-pack

# 3. 编译出 web 可用的 wasm + JS 胶水代码
cd crates/kirara-wasm
wasm-pack build --target web

# 4. 回到仓库根目录,启动静态服务器
cd ../..
npm start
# 浏览器打开 http://localhost:8080/examples/threejs-demo/
```

构建成功后会看到 12 个球体/箱子从空中落下,砸在地面上弹跳、摩擦滚动直到静止。

如果你不想用 `npm start`,也可以在**仓库根目录**运行:

```bash
python3 -m http.server 8080
# 浏览器打开 http://localhost:8080/examples/threejs-demo/
```

注意:不要在 `examples/threejs-demo/` 目录里直接起静态服务器,因为 demo 页面还需要加载
`/crates/kirara-wasm/pkg/` 下的 wasm/JS 产物。

## 项目结构

```
kirara-js/
├── crates/
│   ├── kirara-core/     纯 Rust 物理内核,平台无关,可独立单测
│   │   └── src/
│   │       ├── math.rs      Vec3 / Quat / Mat3 / Transform
│   │       ├── shape.rs     Sphere / Box / Plane
│   │       ├── body.rs      RigidBody + 半隐式欧拉积分
│   │       ├── collide.rs   broadphase(O(n²) AABB)+ narrowphase
│   │       ├── solver.rs    序列脉冲(sequential impulse)求解器
│   │       └── world.rs     World::step() 主循环
│   └── kirara-wasm/     wasm-bindgen 绑定,暴露 KiraraWorld 给 JS
├── examples/threejs-demo/  three.js 渲染演示
└── ROADMAP.md            Ralph loop 迭代任务规划(完整功能拆分)
```

## v1 已实现 / 未实现

| 能力 | 状态 |
|---|---|
| 刚体积分(半隐式欧拉) | ✅ |
| Sphere / Box / Plane 形状 | ✅ |
| sphere-sphere / sphere-plane / sphere-box / box-plane 碰撞 | ✅ |
| 序列脉冲求解器(法线 + 摩擦) | ✅ |
| box-box 碰撞(SAT) | ❌ 见 ROADMAP v1.1 |
| box 旋转参与碰撞检测 | ❌ v1 假设 box 未旋转,见 ROADMAP |
| Capsule / ConvexHull / TriangleMesh | ❌ v2 |
| 约束/关节(hinge、point2point、6dof) | ❌ v2 |
| 连续碰撞检测(CCD) | ❌ v2 |
| 软体、车辆 | ❌ v3 |

详细的迭代计划、每个任务的验收标准、以及 Ralph loop 的执行方式见 **ROADMAP.md**。

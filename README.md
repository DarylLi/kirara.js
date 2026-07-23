# kirara.js

<img width="1010" height="611" alt="企业微信截图_99aca811-8055-4bc7-a80a-967582de1a72" src="https://github.com/user-attachments/assets/724a0c82-7deb-4347-9eff-f06764e6da0d" />

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

# 任务清单

## v1 — 已完成(可运行的最小闭环)

- [x] v1-01 math: Vec3 基础运算(dot/cross/normalize)
      验收:`cargo test -p kirara-core` 无需新增测试,由后续任务间接覆盖
- [x] v1-02 math: Quat + 旋转矩阵 + 角速度积分
- [x] v1-03 math: Mat3(含逆矩阵,用于世界系逆惯性张量)
- [x] v1-04 shape: Sphere/Box/Plane 定义 + 惯性张量公式
- [x] v1-05 body: RigidBody + 半隐式欧拉积分(integrate）
- [x] v1-06 collide: sphere-sphere / sphere-plane / sphere-box / box-plane narrowphase
- [x] v1-07 collide: O(n²) AABB broadphase
- [x] v1-08 solver: 序列脉冲求解器(法线 restitution + Baumgarte + 库仑摩擦近似)
- [x] v1-09 world: World::step() 整合积分/粗筛/窄相/求解
      验收:`sphere_falls_and_rests_on_ground` 测试通过(球体停在半径高度 ±0.05)
- [x] v1-10 world: 多体碰撞验证
      验收:`two_spheres_collide_and_separate` 测试通过
- [x] v1-11 kirara-wasm: wasm-bindgen 绑定(KiraraWorld:add_sphere/add_box/add_ground_plane/step/get_transforms)
      验收:`cargo check`(native target)通过,API 签名冻结
- [x] v1-12 examples/threejs-demo: three.js 渲染 + 主循环
      验收:人工验证——本机跑 `wasm-pack build` 后浏览器打开能看到物体下落静止

## v1.1 — 提升可用性和性能(建议下一批 loop 优先做)

- [x] v1.1-01 box 支持旋转:collide.rs 里 box-plane / sphere-box 改用
      `transform.rotation` 把局部顶点变换到世界系,而不是假设未旋转
      验收:新增测试,让一个初始带旋转角度的 box 落到地面,断言最终
      `angular_velocity` 接近 0(box 一个面完全贴地静止,不再抖动或穿透)
      完成:旋转 box-plane / sphere-box 已接入姿态变换,并补了 `rotated_box_falls_and_settles_on_ground` 测试
- [x] v1.1-02 box-box 碰撞(SAT):15 个分离轴(3+3+9)测试 + 最小穿透轴求接触流形
      验收:两个相向运动的 box 应该弹开而不是穿透,新增
      `two_boxes_collide_and_separate` 测试
      完成:已接入 OBB SAT(15 轴) box-box 窄相检测,并补了 `two_boxes_collide_and_separate` 测试
- [x] v1.1-03 broadphase 换成 Sweep-and-Prune(按包围盒 x 轴排序 + 扫描)
      验收:新增 100 个随机刚体的 benchmark 测试,确认产生的碰撞对集合
      与 v1 的 O(n²) 版本完全一致(用暴力法做 ground truth 对比)
      完成:已用 x 轴 Sweep-and-Prune 替换 O(n²) broadphase,并补了 `broadphase_pair_count_reduced` 一致性测试
- [x] v1.1-04 静止刚体休眠(sleeping):线速度/角速度低于阈值超过 N 帧后跳过积分与求解
      验收:新增测试,确认休眠中的刚体 `step()` 不消耗额外计算(用调用计数
      mock 或简单的 flag 断言),且被外力碰到时能正确唤醒
      完成:已为动态刚体接入 sleeping/唤醒逻辑,并补了 `sleeping_body_skips_integration_and_wakes_on_collision` 测试
- [x] v1.1-05 raycast 查询:`World::raycast(origin, dir, max_dist) -> Option<RaycastHit>`
      先支持 sphere/box/plane 三种形状
      验收:对已知位置的球体做射线测试,命中点误差 < 1e-4
      完成:已实现 `World::raycast` 与 `RaycastHit`,支持 sphere/box/plane,并补了 `raycast_hits_known_sphere_with_small_error` 测试

## v2 — 完整 Bullet 特性对齐

- [x] v2-01 Capsule 形状 + capsule-sphere/capsule-plane/capsule-box/capsule-capsule
      完成:已新增 `Capsule` 形状与基础惯性/AABB,接入 capsule 四类窄相碰撞,并补了 `capsule_supports_core_collision_pairs` 与 `capsule_falls_and_rests_on_ground` 测试
- [x] v2-02 ConvexHull 形状 + GJK 最近点算法
      验收:新增 `convex_hull_support_returns_extreme_point` / `gjk_reports_distance_for_separated_convex_hulls` / `gjk_detects_overlapping_convex_hulls`
      完成:已新增 `Shape::ConvexHull`、support mapping 与独立 `gjk_closest_points()` 工具,可返回分离最近点并检测凸包重叠
- [~] v2-03 EPA 深度穿透算法(配合 GJK,给凸包对提供穿透深度和法线)
  卡住:已尝试在当前 `GJK` simplex 基础上接 `EPA`,但简单重叠凸包的独立验收仍无法稳定得到 tetrahedron 种子与穿透结果,暂时回退到全量测试全绿状态,后续需要先增强 `GJK` 命中 simplex 的稳定性
- [x] v2-04 CompoundShape(多个子形状 + 局部偏移组成一个刚体)
      验收:新增 `compound_shape_support_reaches_offset_child` /
      `compound_shape_collides_via_offset_child` /
      `raycast_hits_offset_child_inside_compound`
      完成:已新增 `Shape::Compound` / `CompoundChild`,接入 compound 的 AABB、惯性近似、support mapping、递归窄相和 raycast,并补了 3 条验收测试
- [x] v2-05 TriangleMesh 静态形状(用于地形/复杂场景,只需支持静态刚体)
      验收:新增 `triangle_mesh_support_and_aabb_cover_vertices` /
      `triangle_mesh_supports_core_collision_pairs` /
      `box_falls_and_rests_on_triangle_mesh` /
      `raycast_hits_triangle_mesh_surface`
      完成:已新增 `Shape::TriangleMesh` / `MeshTriangle`,接入静态 mesh 的 AABB、support mapping、sphere/capsule/box 对 mesh 窄相与 raycast,并补了 4 条验收测试
- [x] v2-06 动态 BVH broadphase(替换 v1.1 的 Sweep-and-Prune,支撑更大规模场景)
      验收:沿用 `broadphase_pair_count_reduced`,并新增
      `dynamic_bvh_matches_ground_truth_on_large_mixed_scene`
      完成:已用每帧重建的 BVH 宽相替换 SAP,保持 pair 集合与暴力真值完全一致,并补了大规模混合场景一致性测试
- [~] v2-07 接触流形持久化(btPersistentManifold 等价物),减少帧间抖动
  卡住:已尝试先做“单点持久流形”最小闭环(缓存接触脉冲/接触点并做 warm start),但在减少部分堆叠抖动的同时会回归 `rotated_box_falls_and_settles_on_ground` 等现有稳定性验收,说明当前窄相单接触点输出对 box-box/box-plane 仍不够稳定,后续需要先增强稳定接触点选择或升级到真正的多点 manifold,本轮已回退到全量测试全绿状态
- [x] v2-08 point2point 约束(球窝关节)
      验收:新增 `point2point_constraint_keeps_dynamic_pair_close` /
      `point2point_constraint_supports_static_anchor`
      完成:已新增 `constraint.rs` 与 `Constraint::Point2Point`,接入 `World::constraints` 和求解循环,并补了双动态体/静态锚点两条验收测试
- [x] v2-09 hinge 约束(铰链)
      验收:新增 `hinge_constraint_restricts_off_axis_rotation` /
      `hinge_constraint_allows_rotation_around_hinge_axis`
      完成:已在 `constraint.rs` 中新增 `Constraint::Hinge` / `HingeConstraint`,通过“共点锚点 + 轴向对齐”实现最小 hinge 闭环,并补了抑制非铰链轴旋转与保留铰链轴自由转动两条验收测试
- [x] v2-10 generic 6-DOF 约束
      验收:新增 `generic_6dof_can_lock_selected_linear_axes` /
      `generic_6dof_can_lock_selected_angular_axes`
      完成:已在 `constraint.rs` 中新增 `Constraint::Generic6Dof` / `Generic6DofConstraint` / `AxisLock`,支持按轴锁定线性与角向自由度,并补了线性锁定与角向锁定两条验收测试
- [x] v2-11 slider 约束
      验收:新增 `slider_constraint_allows_motion_along_slider_axis` /
      `slider_constraint_blocks_off_axis_motion_and_rotation`
      完成:已在 `constraint.rs` 中新增 `Constraint::Slider` / `SliderConstraint`,通过复用 `Generic6Dof` 的按轴锁定能力实现最小 slider 闭环,并补了沿滑轨轴移动与阻止离轴运动/旋转两条验收测试
- [x] v2-12 连续碰撞检测(CCD,conservative advancement),解决高速穿透
      验收:新增 `ccd_prevents_fast_sphere_from_tunneling_through_plane` /
      `ccd_prevents_fast_box_from_tunneling_through_static_box`
      完成:已在 `world.rs` 中新增最小 CCD 预处理,对高速动态 `Sphere` / `Box` 在积分后、宽相前执行到静态 `Plane` / `Box` / `TriangleMesh` 的保守截停,并补了高速穿地/穿墙两条验收测试
- [ ] v2-13 sweep test(形状扫掠测试,配合 CCD 和武器/角色控制器场景)

## v3 — 扩展特性(按需,优先级低于 v2)

- [ ] v3-01 软体基础:质点-弹簧系统 + Verlet/XPBD 积分
- [ ] v3-02 软体-刚体耦合碰撞
- [ ] v3-03 布料专用约束(结构/剪切/弯曲弹簧 + 风力)
- [ ] v3-04 raycast vehicle(基于悬挂 raycast 的车辆模型)
- [ ] v3-05 角色控制器(kinematic character controller,类似 btKinematicCharacterController)

## 长期维护型任务(不属于某个里程碑,随时可插入 loop)

- [ ] maint-01 用 criterion 建立性能基准套件,防止后续改动引入性能回退
- [ ] maint-02 补充文档注释里的公式来源引用(方便审计物理正确性)
- [ ] maint-03 评估是否要把 math.rs 换成 glam(性能 + SIMD),做迁移影响评估报告


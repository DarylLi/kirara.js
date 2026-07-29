//! 软体基础:质点-弹簧系统 + XPBD (Extended Position Based Dynamics) 积分。
//!
//! XPBD 相比传统 Verlet 的优点:
//! - 通过 compliance(柔度,即 1/stiffness)统一处理约束力度
//! - 对大步长稳定,不需子步
//! - 布料/软体/体积约束用同一个框架
//!
//! 刚体耦合:
//! - `pin_to_body`: 将软体质点固定到刚体局部坐标
//! - `collide_with_world`: 粒子-形状穿透解决(Sphere/Box/Plane 精确,其余包围球近似)
//!
//! 布料专用:
//! - `add_cloth_grid`: 构建 rows×cols 网格,自动添加结构/剪切/弯曲弹簧
//! - `apply_wind`: 基于三角形法线的气动力(垂直于法线的分量产生阻力)

use crate::math::{Transform, Vec3};
use crate::world::World;
use crate::shape::Shape;

/// 弹簧约束类型(用于区分结构/剪切/弯曲,调试用)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpringKind {
    Structural,
    Shear,
    Bend,
}

/// 一个弹簧约束:连接粒子 a,b,静止长度 rest_length,柔度 compliance (0 = 无限刚度)
#[derive(Clone, Copy, Debug)]
pub struct Spring {
    pub a: usize,
    pub b: usize,
    pub rest_length: f32,
    /// compliance = 1 / stiffness,越小越硬
    pub compliance: f32,
    pub kind: SpringKind,
}

impl Spring {
    pub fn new(a: usize, b: usize, rest_length: f32, stiffness: f32) -> Self {
        Spring {
            a,
            b,
            rest_length,
            compliance: if stiffness > 0.0 { 1.0 / stiffness } else { 0.0 },
            kind: SpringKind::Structural,
        }
    }

    pub fn with_kind(a: usize, b: usize, rest_length: f32, stiffness: f32, kind: SpringKind) -> Self {
        Spring {
            a,
            b,
            rest_length,
            compliance: if stiffness > 0.0 { 1.0 / stiffness } else { 0.0 },
            kind,
        }
    }
}

/// 三角形面(用于风力/渲染)
#[derive(Clone, Copy, Debug)]
pub struct Triangle {
    pub a: usize,
    pub b: usize,
    pub c: usize,
}

/// 软体质点到刚体的固定约束(one-way)
struct Attachment {
    particle: usize,
    body: usize,
    local_offset: Vec3,
}

pub struct SoftBody {
    /// 质点当前坐标
    pub positions: Vec<Vec3>,
    /// 质点上一帧坐标(Verlet 用)
    pub prev_positions: Vec<Vec3>,
    /// 质点速度(由位置差分估算)
    pub velocities: Vec<Vec3>,
    /// 倒数质量(0 = 固定/无限质量)
    pub inv_masses: Vec<f32>,

    pub springs: Vec<Spring>,
    /// 外力累积(每帧清零)
    forces: Vec<Vec3>,

    pub particle_count: usize,

    /// 刚体附着列表
    attachments: Vec<Attachment>,

    /// 三角形面(布料风力用)
    pub triangles: Vec<Triangle>,
}

impl SoftBody {
    pub fn new() -> Self {
        SoftBody {
            positions: Vec::new(),
            prev_positions: Vec::new(),
            velocities: Vec::new(),
            inv_masses: Vec::new(),
            springs: Vec::new(),
            forces: Vec::new(),
            particle_count: 0,
            attachments: Vec::new(),
            triangles: Vec::new(),
        }
    }

    /// 添加一个质点
    pub fn add_particle(&mut self, position: Vec3, mass: f32) -> usize {
        let idx = self.particle_count;
        self.positions.push(position);
        self.prev_positions.push(position);
        self.velocities.push(Vec3::ZERO);
        self.inv_masses.push(if mass > 0.0 { 1.0 / mass } else { 0.0 });
        self.forces.push(Vec3::ZERO);
        self.particle_count += 1;
        idx
    }

    /// 添加一条弹簧约束
    pub fn add_spring(&mut self, a: usize, b: usize, stiffness: f32) -> usize {
        let rest = (self.positions[a] - self.positions[b]).length();
        let idx = self.springs.len();
        self.springs.push(Spring::new(a, b, rest, stiffness));
        idx
    }

    /// 构建 rows×cols 的布料网格(四边均布,起点在左上角 origin),
    /// 自动添加结构/剪切/弯曲弹簧和三角形面。
    /// 竖挂模式(vertical=true):在 xz 平面,法线沿 y;横向模式:在 xy 平面,法线沿 z。
    /// 返回所有粒子的索引列表(行优先)。
    pub fn add_cloth_grid(
        &mut self,
        rows: usize,
        cols: usize,
        origin: Vec3,
        spacing: f32,
        mass: f32,
        structural_stiffness: f32,
        shear_stiffness: f32,
        bend_stiffness: f32,
    ) -> Vec<usize> {
        let total = rows * cols;
        let per_particle_mass = mass / total as f32;
        let mut indices = Vec::with_capacity(total);

        // 1) 创建粒子(默认在 xz 平面,即水平布料)
        for r in 0..rows {
            for c in 0..cols {
                let pos = Vec3::new(
                    origin.x + c as f32 * spacing,
                    origin.y,
                    origin.z + r as f32 * spacing,
                );
                indices.push(self.add_particle(pos, per_particle_mass));
            }
        }

        // 2) 结构与剪切弹簧
        for r in 0..rows {
            for c in 0..cols {
                let idx = r * cols + c;
                // 结构:右 + 下
                if c + 1 < cols {
                    self.add_spring_kind(indices[idx], indices[idx + 1], structural_stiffness, SpringKind::Structural);
                }
                if r + 1 < rows {
                    self.add_spring_kind(indices[idx], indices[idx + cols], structural_stiffness, SpringKind::Structural);
                }
                // 剪切:右下 + 左下
                if r + 1 < rows && c + 1 < cols {
                    self.add_spring_kind(
                        indices[idx],
                        indices[idx + cols + 1],
                        shear_stiffness,
                        SpringKind::Shear,
                    );
                }
                if r + 1 < rows && c >= 1 {
                    self.add_spring_kind(
                        indices[idx],
                        indices[idx + cols - 1],
                        shear_stiffness,
                        SpringKind::Shear,
                    );
                }
            }
        }

        // 3) 弯曲弹簧(跳过一格)
        for r in 0..rows {
            for c in 0..cols {
                let idx = r * cols + c;
                if c + 2 < cols {
                    self.add_spring_kind(indices[idx], indices[idx + 2], bend_stiffness, SpringKind::Bend);
                }
                if r + 2 < rows {
                    self.add_spring_kind(indices[idx], indices[idx + 2 * cols], bend_stiffness, SpringKind::Bend);
                }
            }
        }

        // 4) 三角形面(两个三角形 per quad)
        for r in 0..rows - 1 {
            for c in 0..cols - 1 {
                let a = indices[r * cols + c];
                let b = indices[r * cols + c + 1];
                let d = indices[(r + 1) * cols + c];
                let e = indices[(r + 1) * cols + c + 1];
                self.triangles.push(Triangle { a, b, c: d });
                self.triangles.push(Triangle { a: b, b: e, c: d });
            }
        }

        indices
    }

    /// 内部方法:加指定类型的弹簧
    fn add_spring_kind(&mut self, a: usize, b: usize, stiffness: f32, kind: SpringKind) -> usize {
        let rest = (self.positions[a] - self.positions[b]).length();
        let idx = self.springs.len();
        self.springs.push(Spring::with_kind(a, b, rest, stiffness, kind));
        idx
    }

    /// 施加风力:风速向量(世界系),仅对有三角形面的布料有效。
    /// 每个三角形计算法线,气动力 = drag_coeff * 0.5 * air_density * |v_rel·n| * area * wind
    /// 其中 v_rel = wind - triangle_velocity
    pub fn apply_wind(&mut self, wind: Vec3, air_density: f32, drag_coeff: f32) {
        for tri in &self.triangles {
            let p_a = self.positions[tri.a];
            let p_b = self.positions[tri.b];
            let p_c = self.positions[tri.c];

            let v_a = self.velocities[tri.a];
            let v_b = self.velocities[tri.b];
            let v_c = self.velocities[tri.c];

            // 三角形中心
            let center = (p_a + p_b + p_c).scale(1.0 / 3.0);
            let tri_vel = (v_a + v_b + v_c).scale(1.0 / 3.0);

            // 法线(未归一化,其长度的 2 倍 = 平行四边形面积)
            let ab = p_b - p_a;
            let ac = p_c - p_a;
            let cross = Vec3::new(
                ab.y * ac.z - ab.z * ac.y,
                ab.z * ac.x - ab.x * ac.z,
                ab.x * ac.y - ab.y * ac.x,
            );
            let area_double = cross.length();
            if area_double < 1e-12 {
                continue;
            }
            let area = 0.5 * area_double;
            let normal = cross.scale(1.0 / area_double);

            // 相对风速
            let rel_vel = wind - tri_vel;
            let vn = rel_vel.dot(normal);
            let vn_abs = vn.abs();

            // 气动力大小:F = 0.5 * ρ * C_d * A * v²,方向沿法线
            let force_mag = 0.5 * air_density * drag_coeff * area * vn_abs * vn.abs();
            let force_dir = if vn > 0.0 {
                normal // 风从法线方向吹来,沿法线推
            } else {
                normal.scale(-1.0) // 风从法线反方向吹来
            };

            let force = force_dir.scale(force_mag);
            let per_particle = force.scale(1.0 / 3.0);
            self.forces[tri.a] = self.forces[tri.a] + per_particle;
            self.forces[tri.b] = self.forces[tri.b] + per_particle;
            self.forces[tri.c] = self.forces[tri.c] + per_particle;
        }
    }

    /// 对质点施加外力(如重力/风力),下一帧 reset 前累积
    pub fn apply_force(&mut self, particle: usize, force: Vec3) {
        self.forces[particle] = self.forces[particle] + force;
    }

    /// 将软体质点固定到刚体的局部坐标(质点跟随刚体运动)
    pub fn pin_to_body(&mut self, particle: usize, body: usize, local_offset: Vec3) {
        self.attachments.push(Attachment {
            particle,
            body,
            local_offset,
        });
    }

    /// 解除某个质点对刚体的所有附着
    pub fn unpin(&mut self, particle: usize) {
        self.attachments.retain(|a| a.particle != particle);
    }

    /// 粒子-场景碰撞检测:将每个软体质点视为半径为 `radius` 的球,
    /// 检测其与 `world` 中所有刚体的穿透,将穿透粒子推出形状。
    ///
    /// 注意:这是 one-way 修正——只推软体,不反馈到刚体。
    pub fn collide_with_world(&mut self, world: &World, radius: f32) {
        for i in 0..self.particle_count {
            if self.inv_masses[i] <= 0.0 {
                continue;
            }
            let p = self.positions[i];
            for (body_idx, rigid_body) in world.bodies.iter().enumerate() {
                // 跳过该质点附着的刚体(避免自穿透)
                if self.attachments.iter().any(|a| a.particle == i && a.body == body_idx) {
                    continue;
                }
                let (penetration, normal) = point_shape_penetration(p, radius, &rigid_body.shape, rigid_body.transform);
                if penetration > 0.0 {
                    self.positions[i] = self.positions[i] + normal.scale(penetration);
                }
            }
        }
    }

    /// XPBD 时间步进
    pub fn step(&mut self, gravity: Vec3, dt: f32, iterations: u32) {
        let sub_dt = dt / iterations.max(1) as f32;
        let alpha = compliance_alpha(dt);

        for _ in 0..iterations {
            // 0) 附着更新:将附着粒子同步到刚体
            // (在子步内不解耦,放在外面)
            // XXX: 在 step_coupled 中处理

            // 1) 施加重力 + 外力 → 预测位置
            for i in 0..self.particle_count {
                if self.inv_masses[i] <= 0.0 {
                    continue;
                }
                let force = gravity + self.forces[i];
                self.velocities[i] = self.velocities[i] + force.scale(sub_dt);
                self.prev_positions[i] = self.positions[i];
                self.positions[i] = self.positions[i] + self.velocities[i].scale(sub_dt);
                self.forces[i] = Vec3::ZERO;
            }

            // 2) 求解弹簧约束
            solve_springs(&mut self.positions, &self.inv_masses, &self.springs, alpha);
        }

        // 3) 更新速度(由位置差分估算)
        let inv_dt = 1.0 / dt.max(1e-12);
        for i in 0..self.particle_count {
            if self.inv_masses[i] <= 0.0 {
                continue;
            }
            self.velocities[i] = (self.positions[i] - self.prev_positions[i]).scale(inv_dt);
        }
    }

    /// 带刚体耦合的一个完整时间步:
    /// 1) 同步 pin 到刚体
    /// 2) XPBD(重力 + 弹簧)
    /// 3) 粒子-世界碰撞解决
    pub fn step_coupled(
        &mut self,
        world: &World,
        gravity: Vec3,
        dt: f32,
        iterations: u32,
        particle_radius: f32,
    ) {
        // 0) 附着同步
        self.resolve_attachments(world);

        // 1+2) XPBD
        self.step(gravity, dt, iterations);

        // 3) 世界碰撞解决
        self.collide_with_world(world, particle_radius);
    }

    /// 将所有 pin_to_body 的粒子同步到对应刚体的当前世界位置
    fn resolve_attachments(&mut self, world: &World) {
        for att in &self.attachments {
            let transform = world.bodies[att.body].transform;
            let world_pos = transform.transform_point(att.local_offset);
            self.positions[att.particle] = world_pos;
            // 重置速度为 0,避免附着粒子有残余运动
            self.velocities[att.particle] = Vec3::ZERO;
        }
    }
}

impl Default for SoftBody {
    fn default() -> Self {
        Self::new()
    }
}

fn compliance_alpha(dt: f32) -> f32 {
    if dt > 1e-12 {
        1.0 / (dt * dt)
    } else {
        0.0
    }
}

/// 求解弹簧约束的内部函数(拆出来复用于子步)
fn solve_springs(positions: &mut [Vec3], inv_masses: &[f32], springs: &[Spring], alpha: f32) {
    for spring in springs {
        let (a, b) = (spring.a, spring.b);
        let w_a = inv_masses[a];
        let w_b = inv_masses[b];
        let w_sum = w_a + w_b;
        if w_sum <= 1e-12 {
            continue;
        }

        let delta = positions[a] - positions[b];
        let dist = delta.length();
        if dist <= 1e-12 {
            continue;
        }
        let n = delta.scale(1.0 / dist);
        let c = dist - spring.rest_length;
        let lambda = -c / (w_sum + spring.compliance * alpha);

        let correction = n.scale(lambda);
        positions[a] = positions[a] + correction.scale(w_a);
        positions[b] = positions[b] - correction.scale(w_b);
    }
}

/// 计算点(半径为 radius 的球心位置)与形状的穿透深度和推出方向。
/// 返回 (penetration, normal),其中 normal 指向推离形状的方向。
/// 支持: Sphere / Box / Plane,其他形状退化为包围球近似。
pub fn point_shape_penetration(
    point: Vec3,
    radius: f32,
    shape: &Shape,
    transform: Transform,
) -> (f32, Vec3) {
    match *shape {
        Shape::Sphere { radius: r } => {
            let center = transform.position;
            let delta = point - center;
            let dist = delta.length();
            let pen = r + radius - dist;
            if pen > 0.0 {
                let n = if dist > 1e-8 { delta.scale(1.0 / dist) } else { Vec3::new(0.0, 1.0, 0.0) };
                (pen, n)
            } else {
                (0.0, Vec3::ZERO)
            }
        }
        Shape::Box { half_extents } => {
            point_box_penetration_radius(point, radius, transform, half_extents)
        }
        Shape::Plane { normal, offset } => {
            let inv_rot = transform.rotation.to_mat3().inverse();
            let local = inv_rot.mul_vec3(point - transform.position);
            let d = local.dot(normal) - offset;
            let pen = radius - d;
            if pen > 0.0 {
                let world_normal = transform.rotation.to_mat3().mul_vec3(normal).normalized();
                (pen, world_normal)
            } else {
                (0.0, Vec3::ZERO)
            }
        }
        _ => {
            // Capsule / TriangleMesh / ConvexHull / Compound → 包围球近似
            let half = shape.local_aabb_half_extents();
            let app_radius = half.length();
            let center = transform.position;
            let delta = point - center;
            let dist = delta.length();
            let pen = app_radius + radius - dist;
            if pen > 0.0 {
                let n = if dist > 1e-8 { delta.scale(1.0 / dist) } else { Vec3::new(0.0, 1.0, 0.0) };
                (pen, n)
            } else {
                (0.0, Vec3::ZERO)
            }
        }
    }
}

/// 点(带半径)对 OBB 盒子的穿透计算。
fn point_box_penetration_radius(
    point: Vec3,
    radius: f32,
    transform: Transform,
    half_extents: Vec3,
) -> (f32, Vec3) {
    let inv_rot = transform.rotation.to_mat3().inverse();
    let local = inv_rot.mul_vec3(point - transform.position);
    let closest = Vec3::new(
        local.x.clamp(-half_extents.x, half_extents.x),
        local.y.clamp(-half_extents.y, half_extents.y),
        local.z.clamp(-half_extents.z, half_extents.z),
    );
    let delta = local - closest;
    let dist_sq = delta.length_sq();

    if dist_sq < radius * radius {
        let dist = dist_sq.sqrt();
        let pen = radius - dist;
        let local_n = if dist > 1e-8 { delta.scale(1.0 / dist) } else {
            // 点在盒子内部,推到最近的面上
            let dx = (half_extents.x - local.x.abs()).abs();
            let dy = (half_extents.y - local.y.abs()).abs();
            let dz = (half_extents.z - local.z.abs()).abs();
            if dx <= dy && dx <= dz {
                Vec3::new(if local.x > 0.0 { 1.0 } else { -1.0 }, 0.0, 0.0)
            } else if dy <= dz {
                Vec3::new(0.0, if local.y > 0.0 { 1.0 } else { -1.0 }, 0.0)
            } else {
                Vec3::new(0.0, 0.0, if local.z > 0.0 { 1.0 } else { -1.0 })
            }
        };
        let world_n = transform.rotation.to_mat3().mul_vec3(local_n).normalized();
        (pen, world_n)
    } else {
        (0.0, Vec3::ZERO)
    }
}

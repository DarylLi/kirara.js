//! v1 支持的碰撞形状。
//! 当前先实现最常用的几种:
//! - Sphere
//! - Box
//! - Plane(通常用作地面,只能是静态刚体)
//! - Capsule
//!
//! ConvexHull / Cylinder 留给后续版本(见 ROADMAP.md)。

use crate::math::{Mat3, Transform, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct CompoundChild {
    pub shape: Shape,
    pub transform: Transform,
}

#[derive(Clone, Copy, Debug)]
pub struct MeshTriangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub enum Shape {
    Sphere { radius: f32 },
    Box { half_extents: Vec3 },
    /// 无限大静态平面,normal 指向刚体一侧,offset 为原点到平面的有符号距离
    Plane { normal: Vec3, offset: f32 },
    /// 沿局部 Y 轴摆放的胶囊体:中间线段半长 + 半径
    Capsule { half_height: f32, radius: f32 },
    /// 任意凸包,点集位于局部坐标系
    ConvexHull { points: &'static [Vec3] },
    /// 静态三角网格,用于地形/复杂静态场景
    TriangleMesh { triangles: &'static [MeshTriangle] },
    /// 多个局部子形状组成一个刚体
    Compound { children: &'static [CompoundChild] },
}

impl Shape {
    /// 轴对齐包围盒的半宽高深(局部坐标系下,未旋转),用于粗筛(broadphase)
    pub fn local_aabb_half_extents(&self) -> Vec3 {
        match *self {
            Shape::Sphere { radius } => Vec3::new(radius, radius, radius),
            Shape::Box { half_extents } => half_extents,
            Shape::Capsule { half_height, radius } => Vec3::new(radius, half_height + radius, radius),
            Shape::ConvexHull { points } => hull_half_extents(points),
            Shape::TriangleMesh { triangles } => triangle_mesh_half_extents(triangles),
            Shape::Compound { children } => compound_half_extents(children),
            // 平面在粗筛阶段特殊处理,不参与常规 AABB 求交
            Shape::Plane { .. } => Vec3::new(1e6, 1e6, 1e6),
        }
    }

    /// 计算单位密度下的局部惯性张量(实际使用时按质量缩放)
    /// 公式来自标准刚体力学中的长方体/球体惯性张量
    pub fn unit_inertia(&self) -> Mat3 {
        match *self {
            Shape::Sphere { radius } => {
                let i = 0.4 * radius * radius; // (2/5) r^2
                Mat3::diagonal(i, i, i)
            }
            Shape::Box { half_extents } => {
                let Vec3 { x, y, z } = half_extents;
                let (x2, y2, z2) = (4.0 * x * x, 4.0 * y * y, 4.0 * z * z); // 全宽的平方
                Mat3::diagonal(
                    (y2 + z2) / 12.0,
                    (x2 + z2) / 12.0,
                    (x2 + y2) / 12.0,
                )
            }
            Shape::Capsule { half_height, radius } => {
                let x = radius;
                let y = half_height + radius;
                let z = radius;
                let (x2, y2, z2) = (4.0 * x * x, 4.0 * y * y, 4.0 * z * z);
                Mat3::diagonal(
                    (y2 + z2) / 12.0,
                    (x2 + z2) / 12.0,
                    (x2 + y2) / 12.0,
                )
            }
            Shape::ConvexHull { points } => {
                let e = hull_half_extents(points);
                let (x2, y2, z2) = (4.0 * e.x * e.x, 4.0 * e.y * e.y, 4.0 * e.z * e.z);
                Mat3::diagonal(
                    (y2 + z2) / 12.0,
                    (x2 + z2) / 12.0,
                    (x2 + y2) / 12.0,
                )
            }
            Shape::TriangleMesh { .. } => Mat3::diagonal(0.0, 0.0, 0.0),
            Shape::Compound { children } => {
                let e = compound_half_extents(children);
                let (x2, y2, z2) = (4.0 * e.x * e.x, 4.0 * e.y * e.y, 4.0 * e.z * e.z);
                Mat3::diagonal(
                    (y2 + z2) / 12.0,
                    (x2 + z2) / 12.0,
                    (x2 + y2) / 12.0,
                )
            }
            Shape::Plane { .. } => Mat3::diagonal(0.0, 0.0, 0.0), // 静态,不参与旋转积分
        }
    }

    pub fn support_point_local(&self, dir: Vec3) -> Option<Vec3> {
        match *self {
            Shape::Sphere { radius } => {
                let axis = if dir.length_sq() > 1e-12 { dir.normalized() } else { Vec3::new(1.0, 0.0, 0.0) };
                Some(axis.scale(radius))
            }
            Shape::Box { half_extents } => Some(Vec3::new(
                if dir.x >= 0.0 { half_extents.x } else { -half_extents.x },
                if dir.y >= 0.0 { half_extents.y } else { -half_extents.y },
                if dir.z >= 0.0 { half_extents.z } else { -half_extents.z },
            )),
            Shape::Capsule { half_height, radius } => {
                let axis = if dir.length_sq() > 1e-12 { dir.normalized() } else { Vec3::new(1.0, 0.0, 0.0) };
                let center = if axis.y >= 0.0 {
                    Vec3::new(0.0, half_height, 0.0)
                } else {
                    Vec3::new(0.0, -half_height, 0.0)
                };
                Some(center + axis.scale(radius))
            }
            Shape::ConvexHull { points } => {
                if points.is_empty() {
                    return Some(Vec3::ZERO);
                }
                let mut best = points[0];
                let mut best_dot = best.dot(dir);
                for &p in &points[1..] {
                    let d = p.dot(dir);
                    if d > best_dot {
                        best_dot = d;
                        best = p;
                    }
                }
                Some(best)
            }
            Shape::TriangleMesh { triangles } => {
                let mut best: Option<Vec3> = None;
                let mut best_dot = f32::NEG_INFINITY;
                for tri in triangles {
                    for p in [tri.a, tri.b, tri.c] {
                        let dot = p.dot(dir);
                        if dot > best_dot {
                            best_dot = dot;
                            best = Some(p);
                        }
                    }
                }
                Some(best.unwrap_or(Vec3::ZERO))
            }
            Shape::Compound { children } => {
                let mut best: Option<Vec3> = None;
                let mut best_dot = f32::NEG_INFINITY;
                for child in children {
                    let rot = child.transform.rotation.to_mat3();
                    let local_dir = rot.transposed().mul_vec3(dir);
                    let Some(local_support) = child.shape.support_point_local(local_dir) else {
                        continue;
                    };
                    let support = child.transform.transform_point(local_support);
                    let dot = support.dot(dir);
                    if dot > best_dot {
                        best_dot = dot;
                        best = Some(support);
                    }
                }
                Some(best.unwrap_or(Vec3::ZERO))
            }
            Shape::Plane { .. } => None,
        }
    }
}

fn hull_half_extents(points: &'static [Vec3]) -> Vec3 {
    if points.is_empty() {
        return Vec3::ZERO;
    }
    let mut min = points[0];
    let mut max = points[0];
    for &p in &points[1..] {
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        min.z = min.z.min(p.z);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
        max.z = max.z.max(p.z);
    }
    (max - min).scale(0.5)
}

fn triangle_mesh_half_extents(triangles: &'static [MeshTriangle]) -> Vec3 {
    let mut min = Vec3::ZERO;
    let mut max = Vec3::ZERO;
    let mut initialized = false;
    for tri in triangles {
        for p in [tri.a, tri.b, tri.c] {
            if !initialized {
                min = p;
                max = p;
                initialized = true;
            } else {
                min.x = min.x.min(p.x);
                min.y = min.y.min(p.y);
                min.z = min.z.min(p.z);
                max.x = max.x.max(p.x);
                max.y = max.y.max(p.y);
                max.z = max.z.max(p.z);
            }
        }
    }
    if initialized { (max - min).scale(0.5) } else { Vec3::ZERO }
}

fn compound_half_extents(children: &'static [CompoundChild]) -> Vec3 {
    let mut extents = Vec3::ZERO;
    for child in children {
        let child_half = child.shape.local_aabb_half_extents();
        let rotated = rotated_half_extents(child.transform.rotation.to_mat3(), child_half);
        extents.x = extents.x.max(child.transform.position.x.abs() + rotated.x);
        extents.y = extents.y.max(child.transform.position.y.abs() + rotated.y);
        extents.z = extents.z.max(child.transform.position.z.abs() + rotated.z);
    }
    extents
}

fn rotated_half_extents(rotation: Mat3, half_extents: Vec3) -> Vec3 {
    Vec3::new(
        rotation.m[0][0].abs() * half_extents.x
            + rotation.m[0][1].abs() * half_extents.y
            + rotation.m[0][2].abs() * half_extents.z,
        rotation.m[1][0].abs() * half_extents.x
            + rotation.m[1][1].abs() * half_extents.y
            + rotation.m[1][2].abs() * half_extents.z,
        rotation.m[2][0].abs() * half_extents.x
            + rotation.m[2][1].abs() * half_extents.y
            + rotation.m[2][2].abs() * half_extents.z,
    )
}

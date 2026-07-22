//! v1 支持的碰撞形状。
//! 当前先实现最常用的三种:
//! - Sphere
//! - Box
//! - Plane(通常用作地面,只能是静态刚体)
//!
//! ConvexHull / TriangleMesh / Capsule / Cylinder / CompoundShape 留给 v2(见 ROADMAP.md)。

use crate::math::{Vec3, Mat3};

#[derive(Clone, Copy, Debug)]
pub enum Shape {
    Sphere { radius: f32 },
    Box { half_extents: Vec3 },
    /// 无限大静态平面,normal 指向刚体一侧,offset 为原点到平面的有符号距离
    Plane { normal: Vec3, offset: f32 },
}

impl Shape {
    /// 轴对齐包围盒的半宽高深(局部坐标系下,未旋转),用于粗筛(broadphase)
    pub fn local_aabb_half_extents(&self) -> Vec3 {
        match *self {
            Shape::Sphere { radius } => Vec3::new(radius, radius, radius),
            Shape::Box { half_extents } => half_extents,
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
            Shape::Plane { .. } => Mat3::diagonal(0.0, 0.0, 0.0), // 静态,不参与旋转积分
        }
    }
}

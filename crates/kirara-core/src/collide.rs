//! 窄相碰撞检测(narrowphase)。
//! v1 只实现最常见、视觉效果最显著的几种形状对:
//!   sphere-sphere / sphere-plane / box-plane / box-sphere
//! box-box(需要 SAT/GJK)、capsule、mesh 等留给 v2 —— 见 ROADMAP.md 中的
//! "collide::narrowphase 扩展任务"。

use crate::body::RigidBody;
use crate::math::{Transform, Vec3};
use crate::shape::Shape;

/// 一个接触点。法线方向定义为:从物体 A 指向物体 B。
#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub a: usize,
    pub b: usize,
    pub point: Vec3,
    pub normal: Vec3,
    pub penetration: f32,
}

pub fn narrowphase(bodies: &[RigidBody], a: usize, b: usize) -> Option<Contact> {
    let (ra, rb) = (&bodies[a], &bodies[b]);
    match (ra.shape, rb.shape) {
        (Shape::Sphere { radius: rra }, Shape::Sphere { radius: rrb }) => {
            sphere_sphere(a, b, ra.transform.position, rra, rb.transform.position, rrb)
        }
        (Shape::Sphere { radius }, Shape::Plane { normal, offset }) => {
            sphere_plane(a, b, ra.transform.position, radius, normal, offset, false)
        }
        (Shape::Plane { normal, offset }, Shape::Sphere { radius }) => {
            sphere_plane(b, a, rb.transform.position, radius, normal, offset, true)
        }
        (Shape::Box { half_extents }, Shape::Plane { normal, offset }) => {
            box_plane(a, b, ra.transform, half_extents, normal, offset, false)
        }
        (Shape::Plane { normal, offset }, Shape::Box { half_extents }) => {
            box_plane(b, a, rb.transform, half_extents, normal, offset, true)
        }
        (Shape::Sphere { radius }, Shape::Box { half_extents }) => {
            sphere_box(a, b, ra.transform.position, radius, rb.transform, half_extents, false)
        }
        (Shape::Box { half_extents }, Shape::Sphere { radius }) => {
            sphere_box(b, a, rb.transform.position, radius, ra.transform, half_extents, true)
        }
        // box-box 等未实现的组合在 v1 中直接跳过(不产生接触),
        // 对应现象:两个 box 会互相穿透 —— 这是 ROADMAP 里明确标出的已知限制。
        _ => None,
    }
}

fn sphere_sphere(a: usize, b: usize, pa: Vec3, ra: f32, pb: Vec3, rb: f32) -> Option<Contact> {
    let delta = pb - pa;
    let dist_sq = delta.length_sq();
    let sum_r = ra + rb;
    if dist_sq >= sum_r * sum_r || dist_sq < 1e-12 {
        return None;
    }
    let dist = dist_sq.sqrt();
    let normal = delta.scale(1.0 / dist);
    let penetration = sum_r - dist;
    let point = pa + normal.scale(ra - penetration * 0.5);
    Some(Contact { a, b, point, normal, penetration })
}

fn sphere_plane(sphere_idx: usize, plane_idx: usize, center: Vec3, radius: f32, normal: Vec3, offset: f32, flip: bool) -> Option<Contact> {
    let dist = center.dot(normal) - offset;
    if dist >= radius {
        return None;
    }
    let penetration = radius - dist;
    let point = center - normal.scale(dist);
    if flip {
        // 此时 a = plane_idx, b = sphere_idx,法线需要指向 sphere 一侧保持“A->B”约定
        Some(Contact { a: plane_idx, b: sphere_idx, point, normal, penetration })
    } else {
        Some(Contact { a: sphere_idx, b: plane_idx, point, normal: -normal, penetration })
    }
}

fn box_plane(box_idx: usize, plane_idx: usize, transform: Transform, half_extents: Vec3, normal: Vec3, offset: f32, flip: bool) -> Option<Contact> {
    // 把穿透平面的顶点聚合成一个近似接触点。
    // 当一个面几乎完全贴地时,比“只取最深单点”更不容易制造持续抖动/扭矩。
    let corners = [
        Vec3::new(half_extents.x, half_extents.y, half_extents.z),
        Vec3::new(-half_extents.x, half_extents.y, half_extents.z),
        Vec3::new(half_extents.x, -half_extents.y, half_extents.z),
        Vec3::new(half_extents.x, half_extents.y, -half_extents.z),
        Vec3::new(-half_extents.x, -half_extents.y, half_extents.z),
        Vec3::new(-half_extents.x, half_extents.y, -half_extents.z),
        Vec3::new(half_extents.x, -half_extents.y, -half_extents.z),
        Vec3::new(-half_extents.x, -half_extents.y, -half_extents.z),
    ];
    let mut deepest_dist = f32::INFINITY;
    let mut point_sum = Vec3::ZERO;
    let mut count = 0usize;
    for c in corners {
        let world_p = transform.transform_point(c);
        let dist = world_p.dot(normal) - offset;
        if dist < 0.0 {
            let projected = world_p - normal.scale(dist);
            point_sum = point_sum + projected;
            count += 1;
            if dist < deepest_dist {
                deepest_dist = dist;
            }
        }
    }
    if count == 0 {
        return None;
    }
    let point = point_sum.scale(1.0 / count as f32);
    let penetration = -deepest_dist;
    if flip {
        Some(Contact { a: plane_idx, b: box_idx, point, normal, penetration })
    } else {
        Some(Contact { a: box_idx, b: plane_idx, point, normal: -normal, penetration })
    }
}

/// 用最近点近似 box-sphere:先把球心变换到 box 的局部坐标,clamp 后再转回世界坐标。
fn sphere_box(sphere_idx: usize, box_idx: usize, sphere_c: Vec3, radius: f32, box_transform: Transform, half_extents: Vec3, flip: bool) -> Option<Contact> {
    let rot = box_transform.rotation.to_mat3();
    let inv_rot = rot.transposed();
    let local = inv_rot.mul_vec3(sphere_c - box_transform.position);
    let clamped = Vec3::new(
        local.x.clamp(-half_extents.x, half_extents.x),
        local.y.clamp(-half_extents.y, half_extents.y),
        local.z.clamp(-half_extents.z, half_extents.z),
    );
    let closest = box_transform.position + rot.mul_vec3(clamped);
    let delta = sphere_c - closest;
    let dist_sq = delta.length_sq();
    if dist_sq >= radius * radius || dist_sq < 1e-12 {
        return None;
    }
    let dist = dist_sq.sqrt();
    let normal = delta.scale(1.0 / dist); // box -> sphere
    let penetration = radius - dist;
    if flip {
        Some(Contact { a: box_idx, b: sphere_idx, point: closest, normal, penetration })
    } else {
        Some(Contact { a: sphere_idx, b: box_idx, point: closest, normal: -normal, penetration })
    }
}

/// 粗筛(broadphase):v1 用最朴素的 O(n^2) AABB 求交 + 静态平面特判。
/// n 较大时应换成 Sweep-and-Prune 或 BVH(见 ROADMAP)。
pub fn broadphase_pairs(bodies: &[RigidBody]) -> Vec<(usize, usize)> {
    let n = bodies.len();
    let mut pairs = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            if bodies[i].is_static && bodies[j].is_static {
                continue;
            }
            if matches!(bodies[i].shape, Shape::Plane { .. }) || matches!(bodies[j].shape, Shape::Plane { .. }) {
                pairs.push((i, j));
                continue;
            }
            let ea = bodies[i].shape.local_aabb_half_extents();
            let eb = bodies[j].shape.local_aabb_half_extents();
            let delta = bodies[j].transform.position - bodies[i].transform.position;
            if delta.x.abs() <= ea.x + eb.x && delta.y.abs() <= ea.y + eb.y && delta.z.abs() <= ea.z + eb.z {
                pairs.push((i, j));
            }
        }
    }
    pairs
}

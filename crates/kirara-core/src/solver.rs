//! 序列脉冲(Sequential Impulse)接触求解器。
//! 当前实现是一个面向 v1 的简化版本:
//! - 法线方向:带弹性恢复系数(restitution)的脉冲
//! - 切向方向:库仑摩擦近似(clamp 到 mu * normal_impulse)
//! - 用 Baumgarte 稳定项做浅层的穿透修正(v1 暂不做完整的 split-impulse)

use crate::body::RigidBody;
use crate::collide::Contact;
use crate::math::Vec3;

const BAUMGARTE: f32 = 0.2;
const SLOP: f32 = 0.005; // 允许的微小穿透,避免抖动
const ITERATIONS: usize = 8;

pub fn solve_contacts(bodies: &mut [RigidBody], contacts: &[Contact], dt: f32) {
    for _ in 0..ITERATIONS {
        for c in contacts {
            solve_one(bodies, c, dt);
        }
    }
}

fn solve_one(bodies: &mut [RigidBody], c: &Contact, dt: f32) {
    let (inv_mass_a, inv_mass_b) = (bodies[c.a].inv_mass, bodies[c.b].inv_mass);
    let inv_mass_sum = inv_mass_a + inv_mass_b;
    if inv_mass_sum <= 0.0 {
        return; // 两个都是静态/运动学物体
    }

    let ra = c.point - bodies[c.a].transform.position;
    let rb = c.point - bodies[c.b].transform.position;

    let vel_a = bodies[c.a].linear_velocity + bodies[c.a].angular_velocity.cross(ra);
    let vel_b = bodies[c.b].linear_velocity + bodies[c.b].angular_velocity.cross(rb);
    let rel_vel = vel_b - vel_a;
    let vn = rel_vel.dot(c.normal);

    // 已经在分离或分离中,不需要脉冲
    let restitution = (bodies[c.a].restitution + bodies[c.b].restitution) * 0.5;
    let bias = (BAUMGARTE / dt.max(1e-6)) * (c.penetration - SLOP).max(0.0);

    if vn > 0.0 && c.penetration <= SLOP {
        return;
    }

    let normal_mass = effective_mass_along(bodies, c.a, c.b, ra, rb, c.normal).max(1e-6);
    let jn = (-(1.0 + restitution) * vn + bias) / normal_mass;
    let jn = jn.max(0.0);
    let impulse = c.normal.scale(jn);

    apply_impulse(bodies, c.a, c.b, ra, rb, impulse);

    // --- 摩擦(切向) ---
    let vel_a = bodies[c.a].linear_velocity + bodies[c.a].angular_velocity.cross(ra);
    let vel_b = bodies[c.b].linear_velocity + bodies[c.b].angular_velocity.cross(rb);
    let rel_vel = vel_b - vel_a;
    let tangent_vel = rel_vel - c.normal.scale(rel_vel.dot(c.normal));
    let t_len = tangent_vel.length();
    if t_len > 1e-6 {
        let tangent = tangent_vel.scale(1.0 / t_len);
        let vt = rel_vel.dot(tangent);
        let tangent_mass = effective_mass_along(bodies, c.a, c.b, ra, rb, tangent).max(1e-6);
        let jt = (-vt) / tangent_mass;
        let friction = (bodies[c.a].friction + bodies[c.b].friction) * 0.5;
        let max_jt = friction * jn;
        let jt = jt.clamp(-max_jt, max_jt);
        let f_impulse = tangent.scale(jt);
        apply_impulse(bodies, c.a, c.b, ra, rb, f_impulse);
    }
}

fn effective_mass_along(bodies: &[RigidBody], a: usize, b: usize, ra: Vec3, rb: Vec3, dir: Vec3) -> f32 {
    let mut k = bodies[a].inv_mass + bodies[b].inv_mass;
    if !bodies[a].is_static {
        let inv_inertia_a = bodies[a].inv_inertia_world();
        let ang_a = inv_inertia_a.mul_vec3(ra.cross(dir)).cross(ra);
        k += ang_a.dot(dir);
    }
    if !bodies[b].is_static {
        let inv_inertia_b = bodies[b].inv_inertia_world();
        let ang_b = inv_inertia_b.mul_vec3(rb.cross(dir)).cross(rb);
        k += ang_b.dot(dir);
    }
    k
}

/// 对 body a 施加 -impulse、对 body b 施加 +impulse(线动量与角动量都更新)。
/// a、b 是分开两次索引 bodies(而非同时可变借用),因此不需要 split_at_mut。
fn apply_impulse(bodies: &mut [RigidBody], a: usize, b: usize, ra: Vec3, rb: Vec3, impulse: Vec3) {
    if !bodies[a].is_static {
        let inv_mass_a = bodies[a].inv_mass;
        let inv_inertia_a = bodies[a].inv_inertia_world();
        bodies[a].linear_velocity = bodies[a].linear_velocity - impulse.scale(inv_mass_a);
        bodies[a].angular_velocity = bodies[a].angular_velocity - inv_inertia_a.mul_vec3(ra.cross(impulse));
    }
    if !bodies[b].is_static {
        let inv_mass_b = bodies[b].inv_mass;
        let inv_inertia_b = bodies[b].inv_inertia_world();
        bodies[b].linear_velocity = bodies[b].linear_velocity + impulse.scale(inv_mass_b);
        bodies[b].angular_velocity = bodies[b].angular_velocity + inv_inertia_b.mul_vec3(rb.cross(impulse));
    }
}

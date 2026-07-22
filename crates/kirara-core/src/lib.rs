//! kirara-core: kirara.js 的纯 Rust 物理内核(不依赖 wasm-bindgen,可独立测试/复用)。
//! v1 范围:刚体 + Sphere/Box/Plane 形状 + 序列脉冲求解器。
//! 完整功能拆分与迭代计划见仓库根目录 ROADMAP.md。

pub mod math;
pub mod shape;
pub mod body;
pub mod collide;
pub mod solver;
pub mod world;

pub use body::RigidBody;
pub use math::{Vec3, Quat, Transform};
pub use shape::Shape;
pub use world::World;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_falls_and_rests_on_ground() {
        let mut world = World::new();
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));
        let sphere = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 5.0, 0.0),
            1.0,
        ));

        for _ in 0..600 {
            world.step(1.0 / 60.0);
        }

        let y = world.bodies[sphere].transform.position.y;
        // 球应该稳定停在半径高度附近(允许 SLOP 误差)
        assert!((y - 0.5).abs() < 0.05, "sphere resting height = {y}, expected ~0.5");
    }

    #[test]
    fn two_spheres_collide_and_separate() {
        let mut world = World::new();
        let a = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(-1.0, 0.0, 0.0),
            1.0,
        ));
        let b = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        ));
        world.gravity = Vec3::ZERO;
        world.bodies[a].linear_velocity = Vec3::new(2.0, 0.0, 0.0);
        world.bodies[b].linear_velocity = Vec3::new(-2.0, 0.0, 0.0);

        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }

        let dist = (world.bodies[b].transform.position - world.bodies[a].transform.position).length();
        assert!(dist >= 0.95, "spheres should have bounced apart, dist = {dist}");
    }

    #[test]
    fn rotated_box_falls_and_settles_on_ground() {
        let mut world = World::new();
        let ground = world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));
        world.bodies[ground].restitution = 0.0;
        world.bodies[ground].friction = 1.0;
        let box_idx = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.6, 0.3, 0.4) },
            Vec3::new(0.0, 4.0, 0.0),
            1.0,
        ));
        world.bodies[box_idx].transform.rotation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.6);
        world.bodies[box_idx].restitution = 0.0;
        world.bodies[box_idx].friction = 1.0;

        for _ in 0..6000 {
            world.step(1.0 / 60.0);
        }

        let angular_speed = world.bodies[box_idx].angular_velocity.length();
        let y = world.bodies[box_idx].transform.position.y;
        assert!(angular_speed < 0.12, "box angular_velocity should settle near zero, got {angular_speed}");
        assert!(y > 0.2, "box should remain above the plane without obvious penetration, y = {y}");
    }
}

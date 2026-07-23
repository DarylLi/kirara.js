//! kirara-core: kirara.js 的纯 Rust 物理内核(不依赖 wasm-bindgen,可独立测试/复用)。
//! v1 范围:刚体 + Sphere/Box/Plane 形状 + 序列脉冲求解器。
//! 完整功能拆分与迭代计划见仓库根目录 ROADMAP.md。

pub mod math;
pub mod shape;
pub mod body;
pub mod collide;
pub mod constraint;
pub mod gjk;
pub mod solver;
pub mod world;

pub use body::RigidBody;
pub use constraint::{AxisLock, Constraint, Generic6DofConstraint, HingeConstraint, Point2PointConstraint, SliderConstraint};
pub use gjk::{gjk_closest_points, GjkResult};
pub use math::{Vec3, Quat, Transform};
pub use shape::{CompoundChild, MeshTriangle, Shape};
pub use world::{RaycastHit, World};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collide::{broadphase_pairs, broadphase_pairs_ground_truth, narrowphase};
    use crate::gjk::gjk_closest_points;

    static FLAT_MESH: [MeshTriangle; 2] = [
        MeshTriangle {
            a: Vec3 { x: -2.0, y: 0.0, z: -2.0 },
            b: Vec3 { x: 2.0, y: 0.0, z: -2.0 },
            c: Vec3 { x: 2.0, y: 0.0, z: 2.0 },
        },
        MeshTriangle {
            a: Vec3 { x: -2.0, y: 0.0, z: -2.0 },
            b: Vec3 { x: 2.0, y: 0.0, z: 2.0 },
            c: Vec3 { x: -2.0, y: 0.0, z: 2.0 },
        },
    ];

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

    #[test]
    fn two_boxes_collide_and_separate() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let a = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.5, 0.5, 0.5) },
            Vec3::new(-1.5, 0.0, 0.0),
            1.0,
        ));
        let b = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.5, 0.5, 0.5) },
            Vec3::new(1.5, 0.0, 0.0),
            1.0,
        ));
        world.bodies[a].restitution = 0.0;
        world.bodies[b].restitution = 0.0;
        world.bodies[a].linear_velocity = Vec3::new(2.0, 0.0, 0.0);
        world.bodies[b].linear_velocity = Vec3::new(-2.0, 0.0, 0.0);

        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }

        let delta = world.bodies[b].transform.position - world.bodies[a].transform.position;
        assert!(delta.x > 0.0, "boxes should keep ordering instead of tunneling through each other, delta = {:?}", delta);
        assert!(delta.length() >= 0.95, "boxes should separate instead of overlap, dist = {}", delta.length());
    }

    #[test]
    fn broadphase_pair_count_reduced() {
        fn next_f32(seed: &mut u32, min: f32, max: f32) -> f32 {
            *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let unit = (*seed as f32) / (u32::MAX as f32);
            min + (max - min) * unit
        }

        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let mut seed = 0x1234_5678u32;

        for _ in 0..100 {
            let use_box = next_f32(&mut seed, 0.0, 1.0) > 0.4;
            let shape = if use_box {
                Shape::Box {
                    half_extents: Vec3::new(
                        next_f32(&mut seed, 0.2, 0.8),
                        next_f32(&mut seed, 0.2, 0.8),
                        next_f32(&mut seed, 0.2, 0.8),
                    ),
                }
            } else {
                Shape::Sphere { radius: next_f32(&mut seed, 0.2, 0.7) }
            };
            let mass = if next_f32(&mut seed, 0.0, 1.0) > 0.8 { 0.0 } else { next_f32(&mut seed, 0.5, 3.0) };
            let pos = Vec3::new(
                next_f32(&mut seed, -15.0, 15.0),
                next_f32(&mut seed, -8.0, 8.0),
                next_f32(&mut seed, -8.0, 8.0),
            );
            let idx = if mass == 0.0 {
                world.add_body(RigidBody::new_static(shape, pos))
            } else {
                world.add_body(RigidBody::new_dynamic(shape, pos, mass))
            };

            if let Shape::Box { .. } = world.bodies[idx].shape {
                world.bodies[idx].transform.rotation = Quat::from_axis_angle(
                    Vec3::new(
                        next_f32(&mut seed, -1.0, 1.0),
                        next_f32(&mut seed, -1.0, 1.0),
                        next_f32(&mut seed, -1.0, 1.0),
                    ),
                    next_f32(&mut seed, -1.5, 1.5),
                );
            }
        }

        let truth = broadphase_pairs_ground_truth(&world.bodies);
        let sap = broadphase_pairs(&world.bodies);
        assert_eq!(sap, truth, "BVH broadphase pair set should exactly match bruteforce ground truth");
    }

    #[test]
    fn point2point_constraint_keeps_dynamic_pair_close() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let a = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.3 },
            Vec3::new(-1.0, 0.0, 0.0),
            1.0,
        ));
        let b = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.3 },
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        ));
        world.bodies[a].linear_velocity = Vec3::new(-1.5, 0.0, 0.0);
        world.bodies[b].linear_velocity = Vec3::new(1.5, 0.0, 0.0);
        world.add_constraint(Constraint::Point2Point(Point2PointConstraint::new(
            a,
            b,
            Vec3::ZERO,
            Vec3::ZERO,
        )));

        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }

        let dist = (world.bodies[a].transform.position - world.bodies[b].transform.position).length();
        assert!(dist < 0.2, "point2point should keep the pivots close, dist = {dist}");
    }

    #[test]
    fn point2point_constraint_supports_static_anchor() {
        let mut world = World::new();
        let anchor = world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.1 },
            Vec3::new(0.0, 2.0, 0.0),
        ));
        let bob = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.3 },
            Vec3::new(0.8, 0.4, 0.0),
            1.0,
        ));
        world.bodies[bob].restitution = 0.0;
        world.add_constraint(Constraint::Point2Point(Point2PointConstraint::new(
            anchor,
            bob,
            Vec3::ZERO,
            Vec3::new(0.0, 0.8, 0.0),
        )));

        for _ in 0..240 {
            world.step(1.0 / 60.0);
        }

        let anchor_world = world.bodies[anchor].transform.position;
        let bob_pivot = world.bodies[bob].transform.transform_point(Vec3::new(0.0, 0.8, 0.0));
        let error = (anchor_world - bob_pivot).length();
        assert!(error < 0.12, "static-anchor point2point should keep pivot error small, error = {error}");
    }

    #[test]
    fn hinge_constraint_restricts_off_axis_rotation() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let anchor = world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.1 },
            Vec3::ZERO,
        ));
        let bar = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.8, 0.1, 0.1) },
            Vec3::new(0.8, 0.0, 0.0),
            1.0,
        ));
        world.bodies[bar].angular_velocity = Vec3::new(6.0, 0.0, 0.0);
        world.add_constraint(Constraint::Hinge(HingeConstraint::new(
            anchor,
            bar,
            Vec3::ZERO,
            Vec3::new(-0.8, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )));

        for _ in 0..240 {
            world.step(1.0 / 60.0);
        }

        let axis_world = world.bodies[bar]
            .transform
            .rotation
            .to_mat3()
            .mul_vec3(Vec3::new(0.0, 1.0, 0.0))
            .normalized();
        assert!(axis_world.dot(Vec3::new(0.0, 1.0, 0.0)) > 0.97, "hinge axis should stay aligned, axis = {:?}", axis_world);
        assert!(world.bodies[bar].angular_velocity.x.abs() < 0.35, "off-axis angular velocity should be damped, wx = {}", world.bodies[bar].angular_velocity.x);
    }

    #[test]
    fn hinge_constraint_allows_rotation_around_hinge_axis() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let anchor = world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.1 },
            Vec3::ZERO,
        ));
        let bar = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.8, 0.1, 0.1) },
            Vec3::new(0.8, 0.0, 0.0),
            1.0,
        ));
        world.bodies[bar].angular_velocity = Vec3::new(0.0, 5.0, 0.0);
        world.add_constraint(Constraint::Hinge(HingeConstraint::new(
            anchor,
            bar,
            Vec3::ZERO,
            Vec3::new(-0.8, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )));

        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }

        let axis_world = world.bodies[bar]
            .transform
            .rotation
            .to_mat3()
            .mul_vec3(Vec3::new(0.0, 1.0, 0.0))
            .normalized();
        assert!(axis_world.dot(Vec3::new(0.0, 1.0, 0.0)) > 0.98, "hinge axis should remain aligned, axis = {:?}", axis_world);
        assert!(world.bodies[bar].angular_velocity.y.abs() > 1.0, "hinge should preserve free spin around hinge axis, wy = {}", world.bodies[bar].angular_velocity.y);
    }

    #[test]
    fn generic_6dof_can_lock_selected_linear_axes() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let anchor = world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.1 },
            Vec3::ZERO,
        ));
        let body = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.3 },
            Vec3::new(0.0, 0.0, 0.0),
            1.0,
        ));
        world.bodies[body].linear_velocity = Vec3::new(2.0, 3.0, 0.0);
        world.add_constraint(Constraint::Generic6Dof(Generic6DofConstraint::new(
            anchor,
            body,
            Vec3::ZERO,
            Vec3::ZERO,
            AxisLock::from_bools(true, false, true),
            AxisLock::from_bools(false, false, false),
        )));

        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }

        let pos = world.bodies[body].transform.position;
        assert!(pos.x.abs() < 0.08, "x axis should remain locked, x = {}", pos.x);
        assert!(pos.z.abs() < 0.08, "z axis should remain locked, z = {}", pos.z);
        assert!(pos.y.abs() > 0.2, "unlocked y axis should still move, y = {}", pos.y);
    }

    #[test]
    fn generic_6dof_can_lock_selected_angular_axes() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let anchor = world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.1 },
            Vec3::ZERO,
        ));
        let body = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.6, 0.2, 0.2) },
            Vec3::new(1.0, 0.0, 0.0),
            1.0,
        ));
        world.bodies[body].angular_velocity = Vec3::new(4.0, 0.0, 4.0);
        world.add_constraint(Constraint::Generic6Dof(Generic6DofConstraint::new(
            anchor,
            body,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::ZERO,
            AxisLock::all_locked(),
            AxisLock::from_bools(true, false, true),
        )));

        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }

        let rot = world.bodies[body].transform.rotation.to_mat3();
        let x_axis = rot.mul_vec3(Vec3::new(1.0, 0.0, 0.0)).normalized();
        let z_axis = rot.mul_vec3(Vec3::new(0.0, 0.0, 1.0)).normalized();
        assert!(x_axis.dot(Vec3::new(1.0, 0.0, 0.0)) > 0.96, "locked x axis should stay aligned, axis = {:?}", x_axis);
        assert!(z_axis.dot(Vec3::new(0.0, 0.0, 1.0)) > 0.96, "locked z axis should stay aligned, axis = {:?}", z_axis);
        assert!(world.bodies[body].angular_velocity.y.abs() < 0.6, "free y axis should not be heavily constrained when no y spin is injected, wy = {}", world.bodies[body].angular_velocity.y);
        assert!(world.bodies[body].angular_velocity.x.abs() < 0.5, "locked x angular velocity should be damped, wx = {}", world.bodies[body].angular_velocity.x);
        assert!(world.bodies[body].angular_velocity.z.abs() < 0.5, "locked z angular velocity should be damped, wz = {}", world.bodies[body].angular_velocity.z);
    }

    #[test]
    fn slider_constraint_allows_motion_along_slider_axis() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let rail = world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.1 },
            Vec3::ZERO,
        ));
        let cart = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.3, 0.2, 0.2) },
            Vec3::ZERO,
            1.0,
        ));
        world.bodies[cart].linear_velocity = Vec3::new(3.0, 0.0, 0.0);
        world.add_constraint(Constraint::Slider(SliderConstraint::new(
            rail,
            cart,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )));

        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }

        let pos = world.bodies[cart].transform.position;
        assert!(pos.x.abs() > 0.5, "slider should allow visible travel along x axis, x = {}", pos.x);
        assert!(pos.y.abs() < 0.08, "slider should keep y axis constrained, y = {}", pos.y);
        assert!(pos.z.abs() < 0.08, "slider should keep z axis constrained, z = {}", pos.z);
    }

    #[test]
    fn slider_constraint_blocks_off_axis_motion_and_rotation() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let rail = world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.1 },
            Vec3::ZERO,
        ));
        let cart = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.4, 0.2, 0.2) },
            Vec3::ZERO,
            1.0,
        ));
        world.bodies[cart].linear_velocity = Vec3::new(0.0, 2.5, 2.5);
        world.bodies[cart].angular_velocity = Vec3::new(0.0, 3.0, 2.5);
        world.add_constraint(Constraint::Slider(SliderConstraint::new(
            rail,
            cart,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )));

        for _ in 0..180 {
            world.step(1.0 / 60.0);
        }

        let pos = world.bodies[cart].transform.position;
        assert!(pos.y.abs() < 0.08, "slider should block y motion, y = {}", pos.y);
        assert!(pos.z.abs() < 0.08, "slider should block z motion, z = {}", pos.z);
        assert!(world.bodies[cart].angular_velocity.y.abs() < 0.4, "slider should damp y rotation, wy = {}", world.bodies[cart].angular_velocity.y);
        assert!(world.bodies[cart].angular_velocity.z.abs() < 0.4, "slider should damp z rotation, wz = {}", world.bodies[cart].angular_velocity.z);
    }

    #[test]
    fn ccd_prevents_fast_sphere_from_tunneling_through_plane() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));
        let sphere = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.25 },
            Vec3::new(0.0, 3.0, 0.0),
            1.0,
        ));
        world.bodies[sphere].restitution = 0.0;
        world.bodies[sphere].linear_velocity = Vec3::new(0.0, -240.0, 0.0);

        world.step(1.0 / 60.0);

        let y = world.bodies[sphere].transform.position.y;
        assert!(y >= 0.2, "CCD sphere should not tunnel below plane, y = {y}");
        assert!(world.bodies[sphere].linear_velocity.y >= -1e-4, "CCD should cancel inward velocity after impact, vy = {}", world.bodies[sphere].linear_velocity.y);
    }

    #[test]
    fn ccd_prevents_fast_box_from_tunneling_through_static_box() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        world.add_body(RigidBody::new_static(
            Shape::Box { half_extents: Vec3::new(0.2, 1.0, 1.0) },
            Vec3::new(0.0, 0.0, 0.0),
        ));
        let moving = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.2, 0.2, 0.2) },
            Vec3::new(-4.0, 0.0, 0.0),
            1.0,
        ));
        world.bodies[moving].restitution = 0.0;
        world.bodies[moving].linear_velocity = Vec3::new(300.0, 0.0, 0.0);

        world.step(1.0 / 60.0);

        let x = world.bodies[moving].transform.position.x;
        assert!(x <= -0.3, "CCD box should stop before crossing the static wall, x = {x}");
        assert!(world.bodies[moving].linear_velocity.x <= 1e-4, "CCD should cancel inward x velocity after impact, vx = {}", world.bodies[moving].linear_velocity.x);
    }

    #[test]
    fn dynamic_bvh_matches_ground_truth_on_large_mixed_scene() {
        fn next_f32(seed: &mut u32, min: f32, max: f32) -> f32 {
            *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let unit = (*seed as f32) / (u32::MAX as f32);
            min + (max - min) * unit
        }

        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let mut seed = 0xCAFEBABEu32;

        static FLAT_MESH: [MeshTriangle; 2] = [
            MeshTriangle {
                a: Vec3 { x: -3.0, y: 0.0, z: -3.0 },
                b: Vec3 { x: 3.0, y: 0.0, z: -3.0 },
                c: Vec3 { x: 3.0, y: 0.0, z: 3.0 },
            },
            MeshTriangle {
                a: Vec3 { x: -3.0, y: 0.0, z: -3.0 },
                b: Vec3 { x: 3.0, y: 0.0, z: 3.0 },
                c: Vec3 { x: -3.0, y: 0.0, z: 3.0 },
            },
        ];

        for i in 0..220 {
            let pick = (next_f32(&mut seed, 0.0, 1.0) * 4.0) as i32;
            let shape = match pick {
                0 => Shape::Sphere { radius: next_f32(&mut seed, 0.2, 0.7) },
                1 => Shape::Box {
                    half_extents: Vec3::new(
                        next_f32(&mut seed, 0.2, 0.8),
                        next_f32(&mut seed, 0.2, 0.8),
                        next_f32(&mut seed, 0.2, 0.8),
                    ),
                },
                2 => Shape::Capsule {
                    half_height: next_f32(&mut seed, 0.2, 0.9),
                    radius: next_f32(&mut seed, 0.15, 0.45),
                },
                _ => Shape::TriangleMesh { triangles: &FLAT_MESH },
            };

            let force_static = matches!(shape, Shape::TriangleMesh { .. });
            let mass = if force_static || next_f32(&mut seed, 0.0, 1.0) > 0.82 {
                0.0
            } else {
                next_f32(&mut seed, 0.5, 3.0)
            };
            let pos = Vec3::new(
                next_f32(&mut seed, -25.0, 25.0),
                next_f32(&mut seed, -12.0, 12.0),
                next_f32(&mut seed, -12.0, 12.0),
            );
            let idx = if mass == 0.0 {
                world.add_body(RigidBody::new_static(shape, pos))
            } else {
                world.add_body(RigidBody::new_dynamic(shape, pos, mass))
            };

            match world.bodies[idx].shape {
                Shape::Box { .. } | Shape::Capsule { .. } => {
                    world.bodies[idx].transform.rotation = Quat::from_axis_angle(
                        Vec3::new(
                            next_f32(&mut seed, -1.0, 1.0),
                            next_f32(&mut seed, -1.0, 1.0),
                            next_f32(&mut seed, -1.0, 1.0),
                        ),
                        next_f32(&mut seed, -1.5, 1.5),
                    );
                }
                _ => {}
            }

            if i % 40 == 0 {
                world.add_body(RigidBody::new_static(
                    Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: next_f32(&mut seed, -2.0, 2.0) },
                    Vec3::ZERO,
                ));
            }
        }

        let truth = broadphase_pairs_ground_truth(&world.bodies);
        let bvh = broadphase_pairs(&world.bodies);
        assert_eq!(bvh, truth, "dynamic BVH broadphase should exactly match bruteforce ground truth");
    }

    #[test]
    fn sleeping_body_skips_integration_and_wakes_on_collision() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        let target = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::ZERO,
            1.0,
        ));

        for _ in 0..80 {
            world.step(1.0 / 60.0);
        }

        assert!(world.bodies[target].is_sleeping, "body should enter sleeping state after staying still");
        world.step(1.0 / 60.0);
        assert!(!world.bodies[target].integrated_last_step, "sleeping body should skip integration");

        let striker = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(-3.0, 0.0, 0.0),
            1.0,
        ));
        world.bodies[striker].restitution = 0.0;
        world.bodies[target].restitution = 0.0;
        world.bodies[striker].linear_velocity = Vec3::new(6.0, 0.0, 0.0);

        let mut woke = false;
        for _ in 0..60 {
            world.step(1.0 / 60.0);
            if !world.bodies[target].is_sleeping {
                woke = true;
                break;
            }
        }

        assert!(woke, "sleeping body should wake when hit by another body");
        world.step(1.0 / 60.0);
        assert!(world.bodies[target].integrated_last_step, "woken body should resume integration on the next step");
    }

    #[test]
    fn raycast_hits_known_sphere_with_small_error() {
        let mut world = World::new();
        let sphere = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(2.0, 0.0, 0.0),
            1.0,
        ));

        let hit = world
            .raycast(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 10.0)
            .expect("ray should hit the sphere");

        assert_eq!(hit.body, sphere);
        assert!((hit.distance - 1.5).abs() < 1e-4, "hit distance = {}", hit.distance);
        assert!((hit.point.x - 1.5).abs() < 1e-4, "hit point.x = {}", hit.point.x);
        assert!(hit.point.y.abs() < 1e-4, "hit point.y = {}", hit.point.y);
        assert!(hit.point.z.abs() < 1e-4, "hit point.z = {}", hit.point.z);
        assert!((hit.normal.x + 1.0).abs() < 1e-4, "hit normal.x = {}", hit.normal.x);
        assert!(hit.normal.y.abs() < 1e-4, "hit normal.y = {}", hit.normal.y);
        assert!(hit.normal.z.abs() < 1e-4, "hit normal.z = {}", hit.normal.z);
    }

    #[test]
    fn capsule_supports_core_collision_pairs() {
        fn capsule_body(position: Vec3) -> RigidBody {
            RigidBody::new_dynamic(
                Shape::Capsule { half_height: 0.6, radius: 0.3 },
                position,
                1.0,
            )
        }

        let sphere = RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.3 },
            Vec3::new(0.0, 0.9, 0.0),
            1.0,
        );
        let plane = RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: -0.2 },
            Vec3::ZERO,
        );
        let box_body = RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.4, 0.4, 0.4) },
            Vec3::new(0.45, 0.0, 0.0),
            1.0,
        );
        let other_capsule = RigidBody::new_dynamic(
            Shape::Capsule { half_height: 0.6, radius: 0.3 },
            Vec3::new(0.45, 0.0, 0.0),
            1.0,
        );

        assert!(narrowphase(&[capsule_body(Vec3::ZERO), sphere], 0, 1).is_some(), "capsule-sphere should collide");
        assert!(narrowphase(&[capsule_body(Vec3::ZERO), plane], 0, 1).is_some(), "capsule-plane should collide");
        assert!(narrowphase(&[capsule_body(Vec3::ZERO), box_body], 0, 1).is_some(), "capsule-box should collide");
        assert!(narrowphase(&[capsule_body(Vec3::ZERO), other_capsule], 0, 1).is_some(), "capsule-capsule should collide");
    }

    #[test]
    fn capsule_falls_and_rests_on_ground() {
        let mut world = World::new();
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));
        let capsule = world.add_body(RigidBody::new_dynamic(
            Shape::Capsule { half_height: 0.6, radius: 0.4 },
            Vec3::new(0.0, 4.0, 0.0),
            1.0,
        ));
        world.bodies[capsule].restitution = 0.0;

        for _ in 0..600 {
            world.step(1.0 / 60.0);
        }

        let y = world.bodies[capsule].transform.position.y;
        assert!(y > 0.9 && y < 1.2, "capsule should settle near half_height + radius, y = {y}");
    }

    #[test]
    fn convex_hull_support_returns_extreme_point() {
        static HULL: [Vec3; 4] = [
            Vec3 { x: -0.5, y: 0.0, z: 0.0 },
            Vec3 { x: 0.25, y: 0.5, z: 0.0 },
            Vec3 { x: 1.25, y: 0.2, z: 0.0 },
            Vec3 { x: 0.0, y: -0.4, z: 0.0 },
        ];
        let shape = Shape::ConvexHull { points: &HULL };
        let support = shape
            .support_point_local(Vec3::new(1.0, 0.1, 0.0))
            .expect("convex hull should provide support point");
        assert!((support.x - 1.25).abs() < 1e-6, "support = {:?}", support);
    }

    #[test]
    fn gjk_reports_distance_for_separated_convex_hulls() {
        static CUBE: [Vec3; 8] = [
            Vec3 { x: -0.5, y: -0.5, z: -0.5 },
            Vec3 { x: 0.5, y: -0.5, z: -0.5 },
            Vec3 { x: -0.5, y: 0.5, z: -0.5 },
            Vec3 { x: 0.5, y: 0.5, z: -0.5 },
            Vec3 { x: -0.5, y: -0.5, z: 0.5 },
            Vec3 { x: 0.5, y: -0.5, z: 0.5 },
            Vec3 { x: -0.5, y: 0.5, z: 0.5 },
            Vec3 { x: 0.5, y: 0.5, z: 0.5 },
        ];
        let a = Shape::ConvexHull { points: &CUBE };
        let b = Shape::ConvexHull { points: &CUBE };
        let result = gjk_closest_points(
            &a,
            Transform { position: Vec3::ZERO, rotation: Quat::IDENTITY },
            &b,
            Transform { position: Vec3::new(3.0, 0.0, 0.0), rotation: Quat::IDENTITY },
        )
        .expect("gjk should support convex hulls");

        assert!(!result.intersect, "separated hulls should not intersect");
        assert!((result.distance - 2.0).abs() < 1e-4, "distance = {}", result.distance);
        assert!((result.closest_a.x - 0.5).abs() < 1e-4, "closest_a = {:?}", result.closest_a);
        assert!((result.closest_b.x - 2.5).abs() < 1e-4, "closest_b = {:?}", result.closest_b);
    }

    #[test]
    fn gjk_detects_overlapping_convex_hulls() {
        static TETRA: [Vec3; 4] = [
            Vec3 { x: -0.5, y: -0.5, z: -0.5 },
            Vec3 { x: 0.5, y: -0.5, z: 0.5 },
            Vec3 { x: -0.5, y: 0.5, z: 0.5 },
            Vec3 { x: 0.5, y: 0.5, z: -0.5 },
        ];
        let a = Shape::ConvexHull { points: &TETRA };
        let b = Shape::ConvexHull { points: &TETRA };
        let result = gjk_closest_points(
            &a,
            Transform { position: Vec3::ZERO, rotation: Quat::IDENTITY },
            &b,
            Transform { position: Vec3::new(0.2, 0.0, 0.0), rotation: Quat::IDENTITY },
        )
        .expect("gjk should return overlap result");

        assert!(result.intersect, "overlapping hulls should intersect");
        assert!(result.distance.abs() < 1e-5, "distance = {}", result.distance);
    }

    #[test]
    fn compound_shape_support_reaches_offset_child() {
        static CHILDREN: [CompoundChild; 2] = [
            CompoundChild {
                shape: Shape::Sphere { radius: 0.5 },
                transform: Transform {
                    position: Vec3 { x: -1.0, y: 0.0, z: 0.0 },
                    rotation: Quat::IDENTITY,
                },
            },
            CompoundChild {
                shape: Shape::Box { half_extents: Vec3 { x: 0.5, y: 0.25, z: 0.25 } },
                transform: Transform {
                    position: Vec3 { x: 1.5, y: 0.0, z: 0.0 },
                    rotation: Quat::IDENTITY,
                },
            },
        ];
        let shape = Shape::Compound { children: &CHILDREN };

        let support = shape
            .support_point_local(Vec3::new(1.0, 0.0, 0.0))
            .expect("compound should provide support point");
        let aabb = shape.local_aabb_half_extents();

        assert!((support.x - 2.0).abs() < 1e-6, "support = {:?}", support);
        assert!((aabb.x - 2.0).abs() < 1e-6, "compound aabb.x = {}", aabb.x);
    }

    #[test]
    fn compound_shape_collides_via_offset_child() {
        static CHILDREN: [CompoundChild; 1] = [CompoundChild {
            shape: Shape::Sphere { radius: 0.6 },
            transform: Transform {
                position: Vec3 { x: 1.2, y: 0.0, z: 0.0 },
                rotation: Quat::IDENTITY,
            },
        }];

        let bodies = [
            RigidBody::new_dynamic(Shape::Compound { children: &CHILDREN }, Vec3::ZERO, 1.0),
            RigidBody::new_dynamic(Shape::Sphere { radius: 0.5 }, Vec3::new(2.1, 0.0, 0.0), 1.0),
        ];
        let contact = narrowphase(&bodies, 0, 1).expect("offset child should generate a contact");

        assert!(contact.penetration > 0.15, "penetration = {}", contact.penetration);
        assert!(contact.normal.x > 0.9, "normal = {:?}", contact.normal);
    }

    #[test]
    fn raycast_hits_offset_child_inside_compound() {
        static CHILDREN: [CompoundChild; 1] = [CompoundChild {
            shape: Shape::Box { half_extents: Vec3 { x: 0.5, y: 0.5, z: 0.5 } },
            transform: Transform {
                position: Vec3 { x: 2.0, y: 0.0, z: 0.0 },
                rotation: Quat::IDENTITY,
            },
        }];

        let mut world = World::new();
        let body = world.add_body(RigidBody::new_static(
            Shape::Compound { children: &CHILDREN },
            Vec3::ZERO,
        ));

        let hit = world
            .raycast(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 10.0)
            .expect("ray should hit the compound child");

        assert_eq!(hit.body, body);
        assert!((hit.distance - 1.5).abs() < 1e-4, "hit distance = {}", hit.distance);
        assert!((hit.point.x - 1.5).abs() < 1e-4, "hit point = {:?}", hit.point);
    }

    #[test]
    fn triangle_mesh_support_and_aabb_cover_vertices() {
        let shape = Shape::TriangleMesh { triangles: &FLAT_MESH };
        let support = shape
            .support_point_local(Vec3::new(1.0, 0.2, 0.0))
            .expect("triangle mesh should provide support point");
        let aabb = shape.local_aabb_half_extents();

        assert!((support.x - 2.0).abs() < 1e-6, "support = {:?}", support);
        assert!((aabb.x - 2.0).abs() < 1e-6, "aabb.x = {}", aabb.x);
        assert!(aabb.y.abs() < 1e-6, "aabb.y = {}", aabb.y);
        assert!((aabb.z - 2.0).abs() < 1e-6, "aabb.z = {}", aabb.z);
    }

    #[test]
    fn triangle_mesh_supports_core_collision_pairs() {
        let sphere = RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 0.4, 0.0),
            1.0,
        );
        let capsule = RigidBody::new_dynamic(
            Shape::Capsule { half_height: 0.4, radius: 0.35 },
            Vec3::new(0.0, 0.7, 0.0),
            1.0,
        );
        let box_body = RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.4, 0.4, 0.4) },
            Vec3::new(0.0, 0.3, 0.0),
            1.0,
        );

        assert!(
            narrowphase(
                &[
                    sphere,
                    RigidBody::new_static(Shape::TriangleMesh { triangles: &FLAT_MESH }, Vec3::ZERO),
                ],
                0,
                1,
            )
            .is_some(),
            "sphere-triangle-mesh should collide"
        );
        assert!(
            narrowphase(
                &[
                    capsule,
                    RigidBody::new_static(Shape::TriangleMesh { triangles: &FLAT_MESH }, Vec3::ZERO),
                ],
                0,
                1,
            )
            .is_some(),
            "capsule-triangle-mesh should collide"
        );
        assert!(
            narrowphase(
                &[
                    box_body,
                    RigidBody::new_static(Shape::TriangleMesh { triangles: &FLAT_MESH }, Vec3::ZERO),
                ],
                0,
                1,
            )
            .is_some(),
            "box-triangle-mesh should collide"
        );
    }

    #[test]
    fn box_falls_and_rests_on_triangle_mesh() {
        let mut world = World::new();
        world.add_body(RigidBody::new_static(
            Shape::TriangleMesh { triangles: &FLAT_MESH },
            Vec3::ZERO,
        ));
        let box_idx = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.5, 0.5, 0.5) },
            Vec3::new(0.0, 3.0, 0.0),
            1.0,
        ));
        world.bodies[box_idx].restitution = 0.0;
        world.bodies[box_idx].friction = 1.0;

        for _ in 0..600 {
            world.step(1.0 / 60.0);
        }

        let y = world.bodies[box_idx].transform.position.y;
        assert!(y > 0.35 && y < 0.7, "box should settle on triangle mesh near half extent height, y = {y}");
    }

    #[test]
    fn raycast_hits_triangle_mesh_surface() {
        let mut world = World::new();
        let mesh = world.add_body(RigidBody::new_static(
            Shape::TriangleMesh { triangles: &FLAT_MESH },
            Vec3::ZERO,
        ));

        let hit = world
            .raycast(Vec3::new(0.0, 3.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 10.0)
            .expect("ray should hit the triangle mesh");

        assert_eq!(hit.body, mesh);
        assert!((hit.distance - 3.0).abs() < 1e-4, "hit distance = {}", hit.distance);
        assert!(hit.point.y.abs() < 1e-4, "hit point = {:?}", hit.point);
        assert!(hit.normal.y > 0.9, "hit normal = {:?}", hit.normal);
    }

}

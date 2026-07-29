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
pub mod character;
pub mod vehicle;
pub mod softbody;

pub use body::RigidBody;
pub use character::{CharacterController, CharacterState};
pub use constraint::{AxisLock, Constraint, Generic6DofConstraint, HingeConstraint, Point2PointConstraint, SliderConstraint};
pub use vehicle::{RaycastVehicle, Wheel};

pub use softbody::{SoftBody, SpringKind, Triangle};
pub use gjk::{gjk_closest_points, GjkResult};
pub use math::{Vec3, Quat, Transform};
pub use shape::{CompoundChild, MeshTriangle, Shape};
pub use world::{RaycastHit, SweepHit, World};

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
            .raycast(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 10.0, None)
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
            .raycast(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 10.0, None)
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
            .raycast(Vec3::new(0.0, 3.0, 0.0), Vec3::new(0.0, -1.0, 0.0), 10.0, None)
            .expect("ray should hit the triangle mesh");

        assert_eq!(hit.body, mesh);
        assert!((hit.distance - 3.0).abs() < 1e-4, "hit distance = {}", hit.distance);
        assert!(hit.point.y.abs() < 1e-4, "hit point = {:?}", hit.point);
        assert!(hit.normal.y > 0.9, "hit normal = {:?}", hit.normal);
    }

    #[test]
    fn sweep_test_hits_sphere_along_path() {
        let mut world = World::new();
        let target = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(5.0, 0.0, 0.0),
            1.0,
        ));

        let hit = world
            .sweep_test(
                &Shape::Sphere { radius: 0.25 },
                Vec3::ZERO,
                Vec3::new(1.0, 0.0, 0.0),
                10.0,
            )
            .expect("swept sphere should hit the target sphere");

        // 两球表面接触时,中心距离 = 0.25 + 0.5,起点在 origin,故 distance = 5 - 0.75
        assert_eq!(hit.body, target);
        assert!((hit.distance - 4.25).abs() < 1e-4, "hit distance = {}", hit.distance);
        assert!((hit.fraction - 0.425).abs() < 1e-4, "hit fraction = {}", hit.fraction);
        assert!((hit.point.x - 4.5).abs() < 1e-4, "hit point = {:?}", hit.point);
        assert!((hit.normal.x + 1.0).abs() < 1e-4, "hit normal = {:?}", hit.normal);
    }

    #[test]
    fn sweep_test_returns_none_when_path_clear() {
        let mut world = World::new();
        world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 5.0, 0.0),
            1.0,
        ));
        world.add_body(RigidBody::new_static(
            Shape::Box { half_extents: Vec3::new(0.5, 0.5, 0.5) },
            Vec3::new(0.0, -5.0, 0.0),
        ));

        let hit = world.sweep_test(
            &Shape::Sphere { radius: 0.25 },
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            3.0,
        );
        assert!(hit.is_none(), "sweep along clear path should miss, got {hit:?}");
    }

    #[test]
    fn sweep_test_against_box_and_plane() {
        let mut world = World::new();
        let plane = world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));
        let wall = world.add_body(RigidBody::new_static(
            Shape::Box { half_extents: Vec3::new(0.2, 1.0, 1.0) },
            Vec3::new(0.0, 5.0, 0.0),
        ));

        // 竖直向下扫掠,命中平面:distance = 3 - 0.25
        let down = world
            .sweep_test(
                &Shape::Sphere { radius: 0.25 },
                Vec3::new(5.0, 3.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                10.0,
            )
            .expect("downward sweep should hit the plane");
        assert_eq!(down.body, plane);
        assert!((down.distance - 2.75).abs() < 1e-4, "down distance = {}", down.distance);
        assert!(down.normal.y > 0.99, "down normal = {:?}", down.normal);

        // 水平扫掠,命中 box 墙面:distance = 4 - 0.2 - 0.25
        let sideways = world
            .sweep_test(
                &Shape::Sphere { radius: 0.25 },
                Vec3::new(-4.0, 5.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                10.0,
            )
            .expect("sideways sweep should hit the static box");
        assert_eq!(sideways.body, wall);
        assert!((sideways.distance - 3.55).abs() < 1e-4, "sideways distance = {}", sideways.distance);
        assert!((sideways.normal.x + 1.0).abs() < 1e-4, "sideways normal = {:?}", sideways.normal);
    }

    #[test]
    fn sweep_test_picks_earliest_hit_among_multiple_bodies() {
        let mut world = World::new();
        world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(8.0, 0.0, 0.0),
        ));
        let near = world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(3.0, 0.0, 0.0),
        ));

        let hit = world
            .sweep_test(
                &Shape::Sphere { radius: 0.5 },
                Vec3::ZERO,
                Vec3::new(1.0, 0.0, 0.0),
                20.0,
            )
            .expect("sweep should hit something");
        assert_eq!(hit.body, near, "should report the earliest hit along the path");
        assert!((hit.distance - 2.0).abs() < 1e-4, "hit distance = {}", hit.distance);
    }

    #[test]
    fn character_walks_and_stops_at_wall() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));
        world.add_body(RigidBody::new_static(
            Shape::Box { half_extents: Vec3::new(0.2, 2.0, 2.0) },
            Vec3::new(3.0, 2.0, 0.0),
        ));

        let mut controller = CharacterController::new(0.6, 0.3, Vec3::new(0.0, 1.0, 0.0));
        controller.gravity = 0.0; // 只关心水平运动

        let dt = 1.0 / 60.0;
        for _ in 0..240 {
            controller.update(&world, dt, Vec3::new(3.0 * dt, 0.0, 0.0));
        }

        // 墙的内侧面在 x = 3 - 0.2 = 2.8,胶囊半径 0.3 + skin 0.02,中心最多到 ~2.48
        assert!(controller.position.x < 2.55, "character should stop before the wall, x = {}", controller.position.x);
        assert!(controller.position.x > 1.5, "character should have walked a visible distance, x = {}", controller.position.x);
    }

    #[test]
    fn character_falls_and_lands_on_ground() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));

        let mut controller = CharacterController::new(0.6, 0.3, Vec3::new(0.0, 3.0, 0.0));
        let dt = 1.0 / 60.0;
        let mut state = controller.update(&world, dt, Vec3::ZERO);
        for _ in 0..300 {
            state = controller.update(&world, dt, Vec3::ZERO);
        }

        // 胶囊中心应停在 half_height + radius + skin ≈ 0.9 附近
        assert!(state.on_ground, "character should end up grounded");
        assert!((state.position.y - 0.92).abs() < 0.1, "resting y = {}", state.position.y);
        assert!(controller.vertical_velocity.abs() < 1e-3, "vertical velocity should be zeroed on landing");
    }

    #[test]
    fn character_steps_up_onto_low_ledge() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));
        // 矮台阶:高 0.2(低于 step_height 0.3),顶面 y=0.2
        world.add_body(RigidBody::new_static(
            Shape::Box { half_extents: Vec3::new(1.0, 0.1, 1.0) },
            Vec3::new(3.0, 0.1, 0.0),
        ));

        let mut controller = CharacterController::new(0.6, 0.3, Vec3::new(0.0, 0.92, 0.0));
        controller.gravity = 0.0;
        controller.step_height = 0.3;

        let dt = 1.0 / 60.0;
        let mut stepped = false;
        for _ in 0..240 {
            let state = controller.update(&world, dt, Vec3::new(2.0 * dt, 0.0, 0.0));
            if state.stepped_up {
                stepped = true;
            }
        }

        assert!(stepped, "controller should have stepped up onto the low ledge at least once");
        assert!(controller.position.x > 2.5, "character should have crossed onto the ledge, x = {}", controller.position.x);
    }

    #[test]
    fn character_slides_down_too_steep_slope() {
        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        // 60° 斜坡,超过默认 50° 上限
        let slope_normal = Vec3::new(0.0, 60.0_f32.to_radians().cos(), 60.0_f32.to_radians().sin()).normalized();
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: slope_normal, offset: 0.0 },
            Vec3::ZERO,
        ));

        let mut controller = CharacterController::new(0.6, 0.3, Vec3::new(0.0, 1.2, 0.0));
        controller.max_slope_angle = 50.0_f32.to_radians();

        let dt = 1.0 / 60.0;
        for _ in 0..120 {
            controller.update(&world, dt, Vec3::ZERO);
        }

        assert!(controller.position.z.abs() > 0.05 || !controller.on_ground,
            "character should slide down the too-steep slope instead of resting, z = {}, on_ground = {}",
            controller.position.z, controller.on_ground);
    }

    #[test]
    fn raycast_vehicle_holds_chassis_at_suspension_height() {
        let mut world = World::new();
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));

        let chassis = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.8, 0.2, 0.4) },
            Vec3::new(0.0, 0.85, 0.0),
            800.0,
        ));
        world.bodies[chassis].restitution = 0.0;

        let mut vehicle = RaycastVehicle::new(chassis);
        let wheel_dir = Vec3::new(0.0, -1.0, 0.0);
        let wheel_axle = Vec3::new(0.0, 0.0, 1.0);
        for (x, z) in [(-0.7, -0.35), (-0.7, 0.35), (0.7, -0.35), (0.7, 0.35)] {
            vehicle.add_wheel(Wheel::new(
                Vec3::new(x, 0.0, z),
                wheel_dir,
                wheel_axle,
                0.3,
                0.6,
            ));
        }

        let dt = 1.0 / 60.0;
        for _ in 0..600 {
            vehicle.update(&mut world, dt);
            world.step(dt);
        }

        let y = world.bodies[chassis].transform.position.y;
        // 悬挂静止长度 0.6 + 轮半径 0.3 => 底盘中心应停在 ~0.9 附近(允许悬挂振荡余量)
        assert!(y > 0.6 && y < 1.3, "chassis should settle at suspension height, y = {y}");
        let vy = world.bodies[chassis].linear_velocity.y;
        assert!(vy.abs() < 0.5, "chassis vertical velocity should be damped, vy = {vy}");
        assert!(vehicle.wheels.iter().all(|w| w.on_ground), "all wheels should be on the ground");
    }

    #[test]
    fn raycast_vehicle_engine_force_accelerates_chassis_forward() {
        let mut world = World::new();
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));

        // 底盘初始 y=0.85,在射线 max_len=0.9 范围内,能检测到地面
        let chassis = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.8, 0.2, 0.4) },
            Vec3::new(0.0, 0.85, 0.0),
            800.0,
        ));
        world.bodies[chassis].restitution = 0.0;

        let mut vehicle = RaycastVehicle::new(chassis);
        let wheel_dir = Vec3::new(0.0, -1.0, 0.0);
        let wheel_axle = Vec3::new(0.0, 0.0, 1.0);
        let mut wheel_indices = Vec::new();
        for (x, z) in [(-0.7, -0.35), (-0.7, 0.35), (0.7, -0.35), (0.7, 0.35)] {
            wheel_indices.push(vehicle.add_wheel(Wheel::new(
                Vec3::new(x, 0.0, z),
                wheel_dir,
                wheel_axle,
                0.3,
                0.6,
            )));
        }

        let dt = 1.0 / 60.0;
        for idx in &wheel_indices {
            vehicle.wheels[*idx].engine_force = 4000.0;
        }
        for _ in 0..180 {
            vehicle.update(&mut world, dt);
            world.step(dt);
        }

        let vx = world.bodies[chassis].linear_velocity.x;
        let x = world.bodies[chassis].transform.position.x;
        assert!(vx > 2.0, "engine force should accelerate the chassis forward, vx = {vx}");
        assert!(x > 1.0, "chassis should have visibly moved forward, x = {x}");
    }

    #[test]
    fn raycast_vehicle_lateral_grip_resists_sideways_slide() {
        let mut world = World::new();
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));

        // 底盘初始 y=0.85,在射线 max_len=0.9 范围内,确保车轮接地
        let chassis = world.add_body(RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(0.8, 0.2, 0.4) },
            Vec3::new(0.0, 0.85, 0.0),
            800.0,
        ));
        world.bodies[chassis].restitution = 0.0;

        let mut vehicle = RaycastVehicle::new(chassis);
        let wheel_dir = Vec3::new(0.0, -1.0, 0.0);
        let wheel_axle = Vec3::new(0.0, 0.0, 1.0);
        for (x, z) in [(-0.7, -0.35), (-0.7, 0.35), (0.7, -0.35), (0.7, 0.35)] {
            vehicle.add_wheel(Wheel::new(
                Vec3::new(x, 0.0, z),
                wheel_dir,
                wheel_axle,
                0.3,
                0.6,
            ));
        }

        let dt = 1.0 / 60.0;
        // 先跑 20 帧让悬挂稳定,再给侧向速度让 grip 生效
        for _ in 0..20 {
            vehicle.update(&mut world, dt);
            world.step(dt);
        }
        world.bodies[chassis].linear_velocity = Vec3::new(0.0, 0.0, 3.0);
        for _ in 0..120 {
            vehicle.update(&mut world, dt);
            world.step(dt);
        }

        let vz = world.bodies[chassis].linear_velocity.z;
        assert!(vz.abs() < 0.5, "lateral grip should damp most of the sideways velocity, vz = {vz}");
    }

    #[test]
    fn softbody_chain_falls_under_gravity() {
        use crate::softbody::SoftBody;

        let mut body = SoftBody::new();
        // 5 质点链条,垂直排列,间距 0.5
        let p0 = body.add_particle(Vec3::new(0.0, 2.0, 0.0), 0.0); // 固定
        let p1 = body.add_particle(Vec3::new(0.0, 1.5, 0.0), 1.0);
        let p2 = body.add_particle(Vec3::new(0.0, 1.0, 0.0), 1.0);
        let p3 = body.add_particle(Vec3::new(0.0, 0.5, 0.0), 1.0);
        let p4 = body.add_particle(Vec3::new(0.0, 0.0, 0.0), 1.0);
        body.add_spring(p0, p1, 100.0);
        body.add_spring(p1, p2, 100.0);
        body.add_spring(p2, p3, 100.0);
        body.add_spring(p3, p4, 100.0);

        let dt = 1.0 / 60.0;
        for _ in 0..120 {
            body.step(Vec3::new(0.0, -9.81, 0.0), dt, 4);
        }

        // 仅验证质点未爆炸(NaN/Inf)且运动合理
        for i in 0..body.particle_count {
            let p = body.positions[i];
            assert!(p.x.is_finite() && p.y.is_finite(), "particle {i} NaN/Inf");
        }
        // 最底端质点应低于起点
        assert!(body.positions[p4].y < 0.3, "bottom particle should have fallen, y={}", body.positions[p4].y);
        // 固定点应不动
        assert!((body.positions[p0].y - 2.0).abs() < 1e-4, "fixed particle moved");
    }

    #[test]
    fn softbody_spring_restores_rest_length() {
        use crate::softbody::SoftBody;

        let mut body = SoftBody::new();
        let a = body.add_particle(Vec3::new(0.0, 0.0, 0.0), 1.0);
        let b = body.add_particle(Vec3::new(1.5, 0.0, 0.0), 1.0);
        body.add_spring(a, b, 50.0); // rest_length = 1.5 (当前距离)

        let dt = 1.0 / 60.0;
        // 手动拉长后,弹簧应回缩
        body.positions[b] = Vec3::new(3.0, 0.0, 0.0);
        for _ in 0..60 {
            body.step(Vec3::ZERO, dt, 8);
        }

        let dist = (body.positions[a] - body.positions[b]).length();
        assert!((dist - 1.5).abs() < 0.15, "spring should return near rest length, dist={dist}");
    }

    #[test]
    fn softbody_fixed_particle_unmoved_by_springs() {
        use crate::softbody::SoftBody;

        let mut body = SoftBody::new();
        let anchor = body.add_particle(Vec3::new(0.0, 1.0, 0.0), 0.0); // 固定
        let hang = body.add_particle(Vec3::new(0.0, 0.0, 0.0), 1.0);
        body.add_spring(anchor, hang, 100.0);

        let dt = 1.0 / 60.0;
        let original = body.positions[anchor];
        for _ in 0..30 {
            body.step(Vec3::new(0.0, -9.81, 0.0), dt, 4);
        }
        assert!((body.positions[anchor].y - original.y).abs() < 1e-6,
            "fixed particle should never move");
    }

    #[test]
    fn softbody_no_nan_with_zero_mass_particles() {
        use crate::softbody::SoftBody;

        let mut body = SoftBody::new();
        let a = body.add_particle(Vec3::ZERO, 0.0);
        let b = body.add_particle(Vec3::new(1.0, 0.0, 0.0), 0.0);
        body.add_spring(a, b, 100.0);

        let dt = 1.0 / 60.0;
        for _ in 0..10 {
            body.step(Vec3::new(0.0, -9.81, 0.0), dt, 4);
        }
        for i in 0..body.particle_count {
            let p = body.positions[i];
            assert!(p.x.is_finite() && p.y.is_finite(), "particle {i} NaN with zero mass");
        }
    }

    #[test]
    fn softbody_pinned_follows_rigid_body() {
        use crate::softbody::SoftBody;

        let mut world = World::new();
        // 一个会移动的刚体
        world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 1.0 },
            Vec3::new(2.0, 2.0, 0.0),
        ));
        let mover = world.add_body(RigidBody::new_dynamic(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 0.0, 0.0),
            1.0,
        ));
        world.bodies[mover].linear_velocity = Vec3::new(5.0, 0.0, 0.0);

        let mut body = SoftBody::new();
        let pin = body.add_particle(Vec3::new(0.0, 0.5, 0.0), 1.0);
        let free = body.add_particle(Vec3::new(0.0, -0.5, 0.0), 1.0);
        body.add_spring(pin, free, 100.0);
        body.pin_to_body(pin, mover, Vec3::new(0.0, 0.5, 0.0));

        let dt = 1.0 / 60.0;
        for _ in 0..30 {
            body.step_coupled(&world, Vec3::ZERO, dt, 4, 0.05);
            world.step(dt);
        }

        // 附着质点跟随刚体运动
        assert!(body.positions[pin].x > 1.0, "pinned particle should follow body, x={}", body.positions[pin].x);
        // 自由质点也被弹簧拉走
        assert!(body.positions[free].x > 0.5, "free particle should be pulled, x={}", body.positions[free].x);
    }

    #[test]
    fn softbody_particles_bounce_off_static_box() {
        use crate::softbody::SoftBody;

        let mut world = World::new();
        // 地面
        world.add_body(RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(0.0, 1.0, 0.0), offset: 0.0 },
            Vec3::ZERO,
        ));

        let mut body = SoftBody::new();
        // 仅一个自由质点从高处掉落
        let p = body.add_particle(Vec3::new(0.0, 2.0, 0.0), 1.0);

        let dt = 1.0 / 60.0;
        for _ in 0..120 {
            body.step_coupled(&world, Vec3::new(0.0, -9.81, 0.0), dt, 4, 0.2);
        }

        // 半径 0.2,不应穿透 y=0 平面
        assert!(body.positions[p].y >= 0.15, "particle should rest above plane, y={}", body.positions[p].y);
    }

    #[test]
    fn softbody_particles_bounce_off_static_sphere() {
        use crate::softbody::SoftBody;

        let mut world = World::new();
        world.gravity = Vec3::ZERO;
        world.add_body(RigidBody::new_static(
            Shape::Sphere { radius: 0.5 },
            Vec3::new(0.0, 1.0, 0.0),
        ));

        let mut body = SoftBody::new();
        let p = body.add_particle(Vec3::new(0.0, 2.5, 0.0), 1.0);

        let dt = 1.0 / 60.0;
        for _ in 0..60 {
            body.step_coupled(&world, Vec3::new(0.0, -9.81, 0.0), dt, 4, 0.2);
        }

        // 粒子半径 0.2 + 球半径 0.5 = 0.7, 球心 y=1.0, 粒子最低 y=1.7
        assert!(body.positions[p].y > 1.65, "particle should rest on sphere, y={}", body.positions[p].y);
    }

    #[test]
    fn softbody_pinned_unpin_frees_particle() {
        use crate::softbody::SoftBody;

        let mut world = World::new();
        let box_body = world.add_body(RigidBody::new_static(
            Shape::Box { half_extents: Vec3::new(0.5, 0.5, 0.5) },
            Vec3::new(0.0, 0.5, 0.0),
        ));

        let mut body = SoftBody::new();
        let pin = body.add_particle(Vec3::new(0.0, 1.8, 0.0), 1.0);  // 盒子顶上方
        let hang = body.add_particle(Vec3::new(0.0, 1.2, 0.0), 1.0);
        body.add_spring(pin, hang, 100.0);
        // 附着到 box 上方 offset: (0, 0.8, 0) → world (0, 0.5+0.8, 0) = (0, 1.3, 0)
        body.pin_to_body(pin, box_body, Vec3::new(0.0, 0.8, 0.0));

        let dt = 1.0 / 60.0;
        for _ in 0..10 {
            body.step_coupled(&world, Vec3::ZERO, dt, 4, 0.05);
            world.step(dt);
        }
        let pinned_y = body.positions[pin].y;
        assert!((pinned_y - 1.3).abs() < 0.1, "pin should be at box top+offset, y={pinned_y}");

        // 解除附着
        body.unpin(pin);
        // 给重力,看 pin 是否不再固定在原位置
        for _ in 0..30 {
            body.step_coupled(&world, Vec3::new(0.0, -9.81, 0.0), dt, 4, 0.05);
            world.step(dt);
        }
        // 解绑后 pin 应因重力/弹簧移动,不再是原来的固定位置
        assert!((body.positions[pin].y - pinned_y).abs() > 0.05,
            "unpinned particle should move from pinned position, y={}", body.positions[pin].y);
        // 但不应穿透盒子(粒子半径 0.05 + 盒顶 y=1.0,最低 ~1.05)
        assert!(body.positions[pin].y > 1.0, "unpinned particle should stay above box, y={}", body.positions[pin].y);
    }

    #[test]
    fn softbody_cloth_grid_hangs_under_gravity() {
        use crate::softbody::SoftBody;

        let mut world = World::new();
        let mut body = SoftBody::new();
        // 5x5 网格
        let indices = body.add_cloth_grid(
            5, 5, Vec3::new(0.0, 2.0, 0.0), 0.3,
            2.0,
            200.0, 50.0, 20.0,
        );
        // 固定左上两角
        body.inv_masses[indices[0]] = 0.0;
        body.inv_masses[indices[4]] = 0.0;

        let dt = 1.0 / 60.0;
        for _ in 0..180 {
            body.step_coupled(&world, Vec3::new(0.0, -9.81, 0.0), dt, 4, 0.02);
            world.step(dt);
        }

        for p in &body.positions {
            assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite());
        }
        // 底部粒子应下垂
        let bottom = indices[4 * 5 + 0];
        assert!(body.positions[bottom].y < 1.6, "cloth should drape, y={}", body.positions[bottom].y);
        // 验证三种弹簧都存在
        let structural = body.springs.iter().filter(|s| s.kind == crate::softbody::SpringKind::Structural).count();
        let shear = body.springs.iter().filter(|s| s.kind == crate::softbody::SpringKind::Shear).count();
        let bend = body.springs.iter().filter(|s| s.kind == crate::softbody::SpringKind::Bend).count();
        assert!(structural > 10, "structural springs missing");
        assert!(shear > 10, "shear springs missing");
        assert!(bend > 10, "bend springs missing");
    }

    #[test]
    fn softbody_wind_pushes_cloth() {
        use crate::softbody::SoftBody;

        let mut world = World::new();
        let mut body = SoftBody::new();
        // 手动构建竖直(在 xy 平面)3x3 网格
        let mut indices = Vec::new();
        for row in 0..3 {
            for col in 0..3 {
                let pos = Vec3::new(col as f32 * 0.5, 2.0 - row as f32 * 0.5, 0.0);
                indices.push(body.add_particle(pos, 1.0 / 9.0));
            }
        }
        for row in 0..3 {
            for col in 0..3 {
                let idx = row * 3 + col;
                if col + 1 < 3 { body.add_spring(indices[idx], indices[idx+1], 200.0); }
                if row + 1 < 3 { body.add_spring(indices[idx], indices[idx+3], 200.0); }
                if row + 1 < 3 && col + 1 < 3 { body.add_spring(indices[idx], indices[idx+4], 50.0); }
                if row + 1 < 3 && col >= 1 { body.add_spring(indices[idx], indices[idx+2], 50.0); }
                if col + 2 < 3 { body.add_spring(indices[idx], indices[idx+2], 20.0); }
                if row + 2 < 3 { body.add_spring(indices[idx], indices[idx+6], 20.0); }
            }
        }
        for r in 0..2 {
            for c in 0..2 {
                let a = indices[r * 3 + c];
                let b = indices[r * 3 + c + 1];
                let d = indices[(r+1) * 3 + c];
                let e = indices[(r+1) * 3 + c + 1];
                body.triangles.push(crate::softbody::Triangle { a, b, c: d });
                body.triangles.push(crate::softbody::Triangle { a: b, b: e, c: d });
            }
        }
        // 固定四角
        body.inv_masses[indices[0]] = 0.0;
        body.inv_masses[indices[2]] = 0.0;
        body.inv_masses[indices[6]] = 0.0;
        body.inv_masses[indices[8]] = 0.0;

        let dt = 1.0 / 60.0;
        for _ in 0..60 {
            body.step_coupled(&world, Vec3::new(0.0, -9.81, 0.0), dt, 4, 0.02);
            world.step(dt);
        }
        let z_before = body.positions[indices[4]].z;

        // 吹 z+ 方向的风,法线有 z 分量
        let wind = Vec3::new(0.0, 0.0, 20.0);
        body.apply_wind(wind, 1.2, 1.0);
        for _ in 0..60 {
            body.step_coupled(&world, Vec3::new(0.0, -9.81, 0.0), dt, 4, 0.02);
            body.apply_wind(wind, 1.2, 1.0);
            world.step(dt);
        }

        let z_after = body.positions[indices[4]].z;
        assert!(z_after > z_before + 0.05,
            "wind should push cloth, z_before={z_before}, z_after={z_after}");
    }

}

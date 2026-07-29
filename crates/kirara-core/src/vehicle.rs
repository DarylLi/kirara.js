//! 基于悬挂 raycast 的车辆模型(类似 btRaycastVehicle)。
//!
//! 设计要点:
//! - 底盘是一个普通动态刚体(通常 Box),加入 `World` 参与正常碰撞与求解。
//! - 每个轮子不建独立刚体,而是每帧从 `connection_point`(世界系)沿
//!   `wheel_direction`(世界系,通常向下)发射 `World::raycast`,
//!   命中距离决定悬挂压缩量。
//! - 悬挂力 = 刚度 × 压缩量 - 阻尼 × 压缩速度,沿 `wheel_direction` 反方向
//!   施加在底盘上(简化为质心力,不产生扭矩,足够稳定)。
//! - 引擎/刹车力沿 `wheel_axle` 的切向(前进方向)施加;侧向摩擦用简单的
//!   速度投影消除来模拟抓地力。

use crate::math::Vec3;
use crate::world::World;

#[derive(Clone, Copy, Debug)]
pub struct Wheel {
    /// 悬挂发射起点(底盘局部系)
    pub connection_point: Vec3,
    /// 悬挂方向(底盘局部系,通常 (0,-1,0))
    pub wheel_direction: Vec3,
    /// 轮轴方向(底盘局部系,通常 (1,0,0) 或 (0,0,1))
    pub wheel_axle: Vec3,
    pub radius: f32,
    /// 悬挂静止长度(未压缩时,从 connection_point 到轮心的距离)
    pub rest_length: f32,
    /// 悬挂刚度(N/m)
    pub stiffness: f32,
    /// 悬挂阻尼(N·s/m)
    pub damping: f32,
    /// 引擎/制动力,由上层每帧设置
    pub engine_force: f32,
    pub brake_force: f32,
    pub on_ground: bool,
}

impl Wheel {
    pub fn new(
        connection_point: Vec3,
        wheel_direction: Vec3,
        wheel_axle: Vec3,
        radius: f32,
        rest_length: f32,
    ) -> Self {
        Wheel {
            connection_point,
            wheel_direction,
            wheel_axle,
            radius,
            rest_length,
            stiffness: 20000.0,
            damping: 3000.0,
            engine_force: 0.0,
            brake_force: 0.0,
            on_ground: false,
        }
    }
}

pub struct RaycastVehicle {
    /// 底盘刚体在 `World::bodies` 中的索引
    pub chassis: usize,
    pub wheels: Vec<Wheel>,
    /// 侧向抓地强度(0~1),越大越不容易侧滑
    pub lateral_grip: f32,
}

impl RaycastVehicle {
    pub fn new(chassis: usize) -> Self {
        RaycastVehicle {
            chassis,
            wheels: Vec::new(),
            lateral_grip: 0.9,
        }
    }

    pub fn add_wheel(&mut self, wheel: Wheel) -> usize {
        self.wheels.push(wheel);
        self.wheels.len() - 1
    }

    /// 在 `World::step()` 之前调用,把悬挂/驱动/刹车力施加到底盘上。
    pub fn update(&mut self, world: &mut World, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        let chassis_index = self.chassis;
        let chassis_transform = world.bodies[chassis_index].transform;
        let chassis_mass = if world.bodies[chassis_index].inv_mass > 0.0 {
            1.0 / world.bodies[chassis_index].inv_mass
        } else {
            return;
        };

        // 先把上一帧的底盘速度拿出来,用于计算悬挂压缩速度与侧滑
        let chassis_velocity = world.bodies[chassis_index].linear_velocity;

        for wheel in self.wheels.iter_mut() {
            let ray_origin = chassis_transform.transform_point(wheel.connection_point);
            let ray_dir = chassis_transform.rotation.to_mat3().mul_vec3(wheel.wheel_direction).normalized();
            let max_len = wheel.rest_length + wheel.radius;

            let hit = world.raycast(ray_origin, ray_dir, max_len, Some(chassis_index));
            wheel.on_ground = false;
            if let Some(hit) = hit {
                let compression = wheel.rest_length + wheel.radius - hit.distance;
                if compression <= 0.0 {
                    continue;
                }
                wheel.on_ground = true;

                // 悬挂力:F = max(0, k·compression + d·compression_velocity),悬架不能拉
                let compression_velocity = chassis_velocity.dot(ray_dir);
                let force_mag = (wheel.stiffness * compression + wheel.damping * compression_velocity).max(0.0);
                let suspension_force = ray_dir.scale(-force_mag);
                world.bodies[chassis_index].apply_force(suspension_force);

                // 前进方向 = wheel_axle × wheel_direction(在引擎/驱动坐标系)
                let axle_world = chassis_transform.rotation.to_mat3().mul_vec3(wheel.wheel_axle).normalized();
                let forward = axle_world.cross(ray_dir).normalized();

                // 引擎/刹车力
                let drive = wheel.engine_force - wheel.brake_force.copysign(chassis_velocity.dot(forward));
                if drive.abs() > 1e-6 {
                    world.bodies[chassis_index].apply_force(forward.scale(drive));
                }

                // 侧向抓地:按比例逐步消除横向速度(避免用 /dt 产生奇异值)
                let lateral_speed = chassis_velocity.dot(axle_world);
                if lateral_speed.abs() > 1e-6 {
                    // grip_strength 控制每帧衰减比例:dv = -v * grip * grip_strength / 60
                    let grip_strength = 10.0;
                    let grip_force = -axle_world.scale(lateral_speed * self.lateral_grip * grip_strength * chassis_mass);
                    world.bodies[chassis_index].apply_force(grip_force);
                }
            }
        }
    }
}

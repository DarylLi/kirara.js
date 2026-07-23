use crate::math::{Vec3, Quat, Mat3, Transform};
use crate::shape::Shape;

const SLEEP_LINEAR_THRESHOLD: f32 = 0.05;
const SLEEP_ANGULAR_THRESHOLD: f32 = 0.05;
const SLEEP_FRAMES: u32 = 60;

/// 单个刚体。
/// v1 先把常用字段全部展平、公开,方便直接读写与测试。
pub struct RigidBody {
    pub shape: Shape,
    pub transform: Transform,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,

    pub inv_mass: f32,
    pub inv_inertia_local: Mat3,

    pub restitution: f32,
    pub friction: f32,

    /// 静态/运动学物体:inv_mass = 0,不参与积分,但参与碰撞
    pub is_static: bool,
    pub is_sleeping: bool,
    pub sleep_counter: u32,
    pub integrated_last_step: bool,

    force_accum: Vec3,
}

impl RigidBody {
    pub fn new_dynamic(shape: Shape, position: Vec3, mass: f32) -> Self {
        let inertia = shape.unit_inertia();
        let inv_inertia_local = Mat3::diagonal(
            if inertia.m[0][0] > 0.0 { 1.0 / (mass * inertia.m[0][0]) } else { 0.0 },
            if inertia.m[1][1] > 0.0 { 1.0 / (mass * inertia.m[1][1]) } else { 0.0 },
            if inertia.m[2][2] > 0.0 { 1.0 / (mass * inertia.m[2][2]) } else { 0.0 },
        );
        RigidBody {
            shape,
            transform: Transform { position, rotation: Quat::IDENTITY },
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: if mass > 0.0 { 1.0 / mass } else { 0.0 },
            inv_inertia_local,
            restitution: 0.3,
            friction: 0.5,
            is_static: false,
            is_sleeping: false,
            sleep_counter: 0,
            integrated_last_step: false,
            force_accum: Vec3::ZERO,
        }
    }

    pub fn new_static(shape: Shape, position: Vec3) -> Self {
        RigidBody {
            shape,
            transform: Transform { position, rotation: Quat::IDENTITY },
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            inv_inertia_local: Mat3::diagonal(0.0, 0.0, 0.0),
            restitution: 0.3,
            friction: 0.5,
            is_static: true,
            is_sleeping: false,
            sleep_counter: 0,
            integrated_last_step: false,
            force_accum: Vec3::ZERO,
        }
    }

    pub fn apply_force(&mut self, f: Vec3) {
        if f.length_sq() > 0.0 {
            self.wake_up();
        }
        self.force_accum = self.force_accum + f;
    }

    pub fn wake_up(&mut self) {
        if self.is_static {
            return;
        }
        self.is_sleeping = false;
        self.sleep_counter = 0;
    }

    pub fn begin_step(&mut self) {
        self.integrated_last_step = false;
    }

    /// 世界坐标系下的逆惯性张量: R * I_local^-1 * R^T
    pub fn inv_inertia_world(&self) -> Mat3 {
        if self.is_static {
            return Mat3::diagonal(0.0, 0.0, 0.0);
        }
        let r = self.transform.rotation.to_mat3();
        r.mul_mat3(&self.inv_inertia_local).mul_mat3(&r.transposed())
    }

    /// 半隐式欧拉积分(symplectic Euler):先更新速度,再更新位置。
    /// 这种积分方式数值稳定性较好,实现也足够直接。
    pub fn integrate(&mut self, gravity: Vec3, dt: f32) {
        if self.is_static {
            self.force_accum = Vec3::ZERO;
            return;
        }
        if self.is_sleeping {
            self.force_accum = Vec3::ZERO;
            return;
        }
        let accel = gravity + self.force_accum.scale(self.inv_mass);
        self.linear_velocity = self.linear_velocity + accel.scale(dt);
        self.transform.position = self.transform.position + self.linear_velocity.scale(dt);
        self.transform.rotation = self.transform.rotation.integrate(self.angular_velocity, dt);
        self.integrated_last_step = true;
        self.force_accum = Vec3::ZERO;
    }

    pub fn update_sleep_state(&mut self) {
        if self.is_static {
            return;
        }
        let linear_slow = self.linear_velocity.length() < SLEEP_LINEAR_THRESHOLD;
        let angular_slow = self.angular_velocity.length() < SLEEP_ANGULAR_THRESHOLD;
        if linear_slow && angular_slow {
            self.sleep_counter += 1;
            if self.sleep_counter >= SLEEP_FRAMES {
                self.is_sleeping = true;
                self.linear_velocity = Vec3::ZERO;
                self.angular_velocity = Vec3::ZERO;
            }
        } else {
            self.wake_up();
        }
    }
}

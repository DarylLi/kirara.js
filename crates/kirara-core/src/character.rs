//! 运动学角色控制器(kinematic character controller,类似 btKinematicCharacterController)。
//!
//! 设计要点:
//! - 角色用竖直摆放的 Capsule 表示,不加入 World 的动力学求解,而是每帧由
//!   上层给出期望位移,控制器内部用 `World::sweep_test` + `World::raycast`
//!   做防穿透与落地检测。
//! - 水平移动支持沿墙面滑动(slide):命中后把剩余位移投影到切平面再走一段。
//! - 支持台阶(step up):水平受阻时尝试把胶囊抬高 step_height 后重试。
//! - 支持最大可走坡度(max_slope):落地法线与竖直方向夹角超过阈值视为不可站立,
//!   重力阶段会沿斜面下滑而不是稳稳站住。

use crate::math::Vec3;
use crate::shape::Shape;
use crate::world::World;

const SKIN_WIDTH: f32 = 0.02;
const MAX_SLIDE_ITERATIONS: usize = 3;
const VERTICAL_RAYCAST_PADDING: f32 = 0.5;

#[derive(Clone, Copy, Debug)]
pub struct CharacterState {
    pub position: Vec3,
    pub on_ground: bool,
    pub ground_normal: Vec3,
    /// 本帧是否因为台阶逻辑而抬升过
    pub stepped_up: bool,
}

pub struct CharacterController {
    pub capsule: Shape,
    pub position: Vec3,
    pub vertical_velocity: f32,
    pub gravity: f32,
    pub step_height: f32,
    /// 最大可站立坡度(与竖直方向的夹角,弧度)
    pub max_slope_angle: f32,
    pub on_ground: bool,
}

impl CharacterController {
    pub fn new(half_height: f32, radius: f32, position: Vec3) -> Self {
        CharacterController {
            capsule: Shape::Capsule { half_height, radius },
            position,
            vertical_velocity: 0.0,
            gravity: -9.81,
            step_height: 0.3,
            max_slope_angle: 50.0_f32.to_radians(),
            on_ground: false,
        }
    }

    /// 推进一帧:`desired_horizontal` 是本帧想要的水平位移(已乘 dt),
    /// 竖直方向由内部重力积分 + 落地检测处理。
    pub fn update(&mut self, world: &World, dt: f32, desired_horizontal: Vec3) -> CharacterState {
        let mut stepped_up = false;
        let mut remaining = Vec3::new(desired_horizontal.x, 0.0, desired_horizontal.z);

        // 1. 水平移动:sweep + slide,必要时 step up
        for _ in 0..MAX_SLIDE_ITERATIONS {
            let dist = remaining.length();
            if dist <= 1e-6 {
                break;
            }
            let dir = remaining.normalized();
            match world.sweep_test(&self.capsule, self.position, dir, dist + SKIN_WIDTH) {
                None => {
                    self.position = self.position + remaining;
                    break;
                }
                Some(hit) if hit.distance > dist => {
                    // 在 skin width 范围内还没碰到,直接走完
                    self.position = self.position + remaining;
                    break;
                }
                Some(hit) => {
                    // 先走到接触点(留 skin width)
                    let safe = (hit.distance - SKIN_WIDTH).max(0.0);
                    self.position = self.position + dir.scale(safe);

                    let leftover = remaining - dir.scale(safe);

                    // 尝试台阶:抬高 step_height 后重走 leftover
                    if self.step_height > 0.0 && leftover.length() > 1e-6 {
                        let raised = self.position + Vec3::new(0.0, self.step_height, 0.0);
                        let retry_dist = leftover.length();
                        let retry_dir = leftover.normalized();
                        let clear_above = world
                            .sweep_test(&self.capsule, raised, retry_dir, retry_dist + SKIN_WIDTH)
                            .map(|h| h.distance > retry_dist)
                            .unwrap_or(true);
                        let can_raise = world
                            .sweep_test(&self.capsule, self.position, Vec3::new(0.0, 1.0, 0.0), self.step_height)
                            .is_none();
                        if clear_above && can_raise {
                            self.position = raised + leftover;
                            remaining = Vec3::ZERO;
                            stepped_up = true;
                            break;
                        }
                    }

                    // slide:把剩余位移投影到命中面的切平面
                    let n = hit.normal;
                    let slide = leftover - n.scale(leftover.dot(n));
                    if slide.length() <= 1e-6 {
                        break;
                    }
                    remaining = slide;
                }
            }
        }

        // 2. 竖直方向:重力积分 + 向下 raycast 贴地
        self.vertical_velocity += self.gravity * dt;
        let dy = self.vertical_velocity * dt;
        let capsule_radius = match self.capsule {
            Shape::Capsule { radius, half_height } => (radius, half_height),
            _ => unreachable!("character controller only supports capsule"),
        };
        let half_total = capsule_radius.1 + capsule_radius.0;

        let mut ground_normal = Vec3::new(0.0, 1.0, 0.0);
        let mut grounded = false;
        if dy <= 0.0 {
            // 从胶囊中心向下发射线,命中距离 <= half_total + 皮肤厚度视为落地
            let ray_len = half_total + VERTICAL_RAYCAST_PADDING + (-dy);
            if let Some(hit) = world.raycast(
                self.position,
                Vec3::new(0.0, -1.0, 0.0),
                ray_len,
                None,
            ) {
                let floor_y = self.position.y - hit.distance + half_total;
                let would_penetrate = self.position.y + dy < floor_y + SKIN_WIDTH;
                let close_enough = hit.distance <= half_total + SKIN_WIDTH + (-dy);
                if would_penetrate || close_enough {
                    grounded = true;
                    ground_normal = hit.normal;
                    self.position.y = floor_y + SKIN_WIDTH;
                    self.vertical_velocity = 0.0;
                }
            }
        }
        if !grounded {
            self.position.y += dy;
        }

        // 3. 坡度处理:站在过陡的面上时,让角色沿斜面下滑
        if grounded {
            let cos_slope = ground_normal.dot(Vec3::new(0.0, 1.0, 0.0));
            let slope_angle = cos_slope.clamp(-1.0, 1.0).acos();
            if slope_angle > self.max_slope_angle {
                grounded = false;
                // 沿斜面下滑方向 = 重力在斜面上的投影
                let downhill = Vec3::new(0.0, -1.0, 0.0)
                    - ground_normal.scale(Vec3::new(0.0, -1.0, 0.0).dot(ground_normal));
                if downhill.length_sq() > 1e-8 {
                    let slide_speed = (-self.gravity) * dt * slope_angle.sin();
                    self.position = self.position + downhill.normalized().scale(slide_speed * dt);
                }
            }
        }

        self.on_ground = grounded;
        CharacterState {
            position: self.position,
            on_ground: grounded,
            ground_normal,
            stepped_up,
        }
    }
}

use crate::body::RigidBody;
use crate::collide::{broadphase_pairs, narrowphase};
use crate::math::Vec3;
use crate::solver::solve_contacts;

/// 物理世界。
/// v1 的固定步长离散仿真循环:
///   1. 施加重力,半隐式欧拉积分速度和位置(暂不做碰撞约束)
///   2. broadphase 粗筛可能碰撞的物体对
///   3. narrowphase 对每一对计算精确接触点
///   4. 序列脉冲求解器修正速度,避免穿透
///
/// 注意:v1 为了实现简单,把“先积分再解算”简化为一步(不像成熟引擎那样有
/// 单独的 predictive integration + 碰撞回退),在多数下落/堆叠场景下已经
/// 足够稳定,但高速物体可能穿透(需要 CCD,见 ROADMAP)。
pub struct World {
    pub bodies: Vec<RigidBody>,
    pub gravity: Vec3,
}

impl World {
    pub fn new() -> Self {
        World { bodies: Vec::new(), gravity: Vec3::new(0.0, -9.81, 0.0) }
    }

    pub fn add_body(&mut self, body: RigidBody) -> usize {
        self.bodies.push(body);
        self.bodies.len() - 1
    }

    pub fn step(&mut self, dt: f32) {
        for b in self.bodies.iter_mut() {
            b.integrate(self.gravity, dt);
        }

        let pairs = broadphase_pairs(&self.bodies);
        let mut contacts = Vec::with_capacity(pairs.len());
        for (i, j) in pairs {
            if let Some(c) = narrowphase(&self.bodies, i, j) {
                contacts.push(c);
            }
        }

        solve_contacts(&mut self.bodies, &contacts, dt);
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}

//! kirara-wasm: 把 kirara-core 包装成面向 JS 的 API。
//! 设计目标:让 three.js 侧保持轻量直接的调用方式,
//! 内部完全由 kirara-core 提供物理实现。

use wasm_bindgen::prelude::*;
use kirara_core::{World, RigidBody, Shape, Vec3};

#[wasm_bindgen]
pub struct KiraraWorld {
    world: World,
}

#[wasm_bindgen]
impl KiraraWorld {
    #[wasm_bindgen(constructor)]
    pub fn new() -> KiraraWorld {
        KiraraWorld { world: World::new() }
    }

    /// 添加一个静态地面平面,normal 需要归一化,offset 是原点到平面的有符号距离
    pub fn add_ground_plane(&mut self, nx: f32, ny: f32, nz: f32, offset: f32) -> usize {
        let body = RigidBody::new_static(
            Shape::Plane { normal: Vec3::new(nx, ny, nz), offset },
            Vec3::ZERO,
        );
        self.world.add_body(body)
    }

    pub fn add_sphere(&mut self, radius: f32, x: f32, y: f32, z: f32, mass: f32) -> usize {
        let body = RigidBody::new_dynamic(Shape::Sphere { radius }, Vec3::new(x, y, z), mass);
        self.world.add_body(body)
    }

    pub fn add_box(&mut self, hx: f32, hy: f32, hz: f32, x: f32, y: f32, z: f32, mass: f32) -> usize {
        let body = RigidBody::new_dynamic(
            Shape::Box { half_extents: Vec3::new(hx, hy, hz) },
            Vec3::new(x, y, z),
            mass,
        );
        self.world.add_body(body)
    }

    pub fn set_gravity(&mut self, x: f32, y: f32, z: f32) {
        self.world.gravity = Vec3::new(x, y, z);
    }

    pub fn step(&mut self, dt: f32) {
        self.world.step(dt);
    }

    pub fn body_count(&self) -> usize {
        self.world.bodies.len()
    }

    /// 批量导出所有刚体的 [x,y,z, qx,qy,qz,qw] * N,方便 three.js 侧一次性
    /// 用 Float32Array 读取,而不是逐个跨 wasm 边界调用(避免频繁调用开销)。
    pub fn get_transforms(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.world.bodies.len() * 7);
        for b in &self.world.bodies {
            let p = b.transform.position;
            let q = b.transform.rotation;
            out.push(p.x);
            out.push(p.y);
            out.push(p.z);
            out.push(q.x);
            out.push(q.y);
            out.push(q.z);
            out.push(q.w);
        }
        out
    }
}

impl Default for KiraraWorld {
    fn default() -> Self {
        Self::new()
    }
}

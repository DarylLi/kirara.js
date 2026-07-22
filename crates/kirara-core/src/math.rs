//! 最小可用的 3D 数学库:Vec3 / Quat / Mat3 / Transform
//! v1 目标:够用、正确、易读,不追求 SIMD 极限性能(留给 v2 用 glam/nalgebra 替换)。

use core::ops::{Add, Sub, Mul, Neg};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };

    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Vec3 { x, y, z }
    }

    pub fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn length_sq(self) -> f32 {
        self.dot(self)
    }

    pub fn length(self) -> f32 {
        self.length_sq().sqrt()
    }

    pub fn normalized(self) -> Vec3 {
        let l = self.length();
        if l > 1e-8 { self * (1.0 / l) } else { Vec3::ZERO }
    }

    pub fn scale(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 { Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z) }
}
impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 { Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z) }
}
impl Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f32) -> Vec3 { self.scale(s) }
}
impl Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 { Vec3::new(-self.x, -self.y, -self.z) }
}

/// 3x3 矩阵,行主序存储,用于惯性张量
#[derive(Clone, Copy, Debug)]
pub struct Mat3 {
    pub m: [[f32; 3]; 3],
}

impl Mat3 {
    pub const IDENTITY: Mat3 = Mat3 {
        m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    pub fn diagonal(x: f32, y: f32, z: f32) -> Mat3 {
        Mat3 { m: [[x, 0.0, 0.0], [0.0, y, 0.0], [0.0, 0.0, z]] }
    }

    pub fn mul_vec3(&self, v: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        )
    }

    pub fn mul_mat3(&self, o: &Mat3) -> Mat3 {
        let mut r = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0.0;
                for k in 0..3 {
                    s += self.m[i][k] * o.m[k][j];
                }
                r[i][j] = s;
            }
        }
        Mat3 { m: r }
    }

    pub fn transposed(&self) -> Mat3 {
        let m = self.m;
        Mat3 { m: [
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ]}
    }

    /// 逆矩阵(用于从局部惯性张量得到逆惯性张量),假设可逆
    pub fn inverse(&self) -> Mat3 {
        let m = &self.m;
        let a = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        let b = m[1][2] * m[2][0] - m[1][0] * m[2][2];
        let c = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        let det = m[0][0] * a + m[0][1] * b + m[0][2] * c;
        if det.abs() < 1e-12 {
            return Mat3::IDENTITY;
        }
        let inv_det = 1.0 / det;
        let d = m[0][2] * m[2][1] - m[0][1] * m[2][2];
        let e = m[0][0] * m[2][2] - m[0][2] * m[2][0];
        let f = m[0][1] * m[2][0] - m[0][0] * m[2][1];
        let g = m[0][1] * m[1][2] - m[0][2] * m[1][1];
        let h = m[0][2] * m[1][0] - m[0][0] * m[1][2];
        let i = m[0][0] * m[1][1] - m[0][1] * m[1][0];
        Mat3 { m: [
            [a * inv_det, d * inv_det, g * inv_det],
            [b * inv_det, e * inv_det, h * inv_det],
            [c * inv_det, f * inv_det, i * inv_det],
        ]}
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Quat {
    pub const IDENTITY: Quat = Quat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };

    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Quat {
        let a = axis.normalized();
        let half = angle * 0.5;
        let s = half.sin();
        Quat { x: a.x * s, y: a.y * s, z: a.z * s, w: half.cos() }
    }

    pub fn normalized(self) -> Quat {
        let l = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        if l > 1e-8 {
            Quat { x: self.x / l, y: self.y / l, z: self.z / l, w: self.w / l }
        } else {
            Quat::IDENTITY
        }
    }

    pub fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }

    /// 将四元数转换为旋转矩阵
    pub fn to_mat3(self) -> Mat3 {
        let Quat { x, y, z, w } = self;
        let (xx, yy, zz) = (x * x, y * y, z * z);
        let (xy, xz, yz) = (x * y, x * z, y * z);
        let (wx, wy, wz) = (w * x, w * y, w * z);
        Mat3 { m: [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]}
    }

    /// 按角速度 omega 对四元数做一个 dt 时间步的积分(一阶近似 + 重新归一化)
    pub fn integrate(self, omega: Vec3, dt: f32) -> Quat {
        let w = Quat { x: omega.x, y: omega.y, z: omega.z, w: 0.0 };
        let dq = w.mul(self);
        let q = Quat {
            x: self.x + dq.x * 0.5 * dt,
            y: self.y + dq.y * 0.5 * dt,
            z: self.z + dq.z * 0.5 * dt,
            w: self.w + dq.w * 0.5 * dt,
        };
        q.normalized()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Transform {
    pub position: Vec3,
    pub rotation: Quat,
}

impl Transform {
    pub fn identity() -> Self {
        Transform { position: Vec3::ZERO, rotation: Quat::IDENTITY }
    }

    /// 局部坐标 -> 世界坐标
    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        self.rotation.to_mat3().mul_vec3(p) + self.position
    }
}

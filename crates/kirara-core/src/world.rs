use crate::body::RigidBody;
use crate::collide::{broadphase_pairs, narrowphase};
use crate::constraint::{solve_constraints, Constraint};
use crate::math::{Transform, Vec3};
use crate::shape::{CompoundChild, MeshTriangle, Shape};
use crate::solver::solve_contacts;

const CCD_MOTION_THRESHOLD: f32 = 0.75;
const CCD_BACKOFF: f32 = 0.98;

/// `sweep_test` 的命中结果,语义与 `RaycastHit` 类似,但 `distance` 表示
/// 沿 `dir` 方向移动到达第一个接触所需的距离(不是 TOI 比例),`fraction`
/// 给出在 [0, 1] 区间内的归一化进度,便于上层用 `lerp` 直接定位。
#[derive(Clone, Copy, Debug)]
pub struct SweepHit {
    pub body: usize,
    pub point: Vec3,
    pub normal: Vec3,
    pub distance: f32,
    pub fraction: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct RaycastHit {
    pub body: usize,
    pub point: Vec3,
    pub normal: Vec3,
    pub distance: f32,
}

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
    pub constraints: Vec<Constraint>,
    pub gravity: Vec3,
}

impl World {
    pub fn new() -> Self {
        World {
            bodies: Vec::new(),
            constraints: Vec::new(),
            gravity: Vec3::new(0.0, -9.81, 0.0),
        }
    }

    pub fn add_body(&mut self, body: RigidBody) -> usize {
        self.bodies.push(body);
        self.bodies.len() - 1
    }

    pub fn add_constraint(&mut self, constraint: Constraint) -> usize {
        self.constraints.push(constraint);
        self.constraints.len() - 1
    }

    pub fn step(&mut self, dt: f32) {
        let previous_transforms: Vec<Transform> = self.bodies.iter().map(|b| b.transform).collect();
        for b in self.bodies.iter_mut() {
            b.begin_step();
            b.integrate(self.gravity, dt);
        }
        apply_ccd(&mut self.bodies, &previous_transforms);

        let pairs = broadphase_pairs(&self.bodies);
        let mut contacts = Vec::with_capacity(pairs.len());
        for (i, j) in pairs {
            if let Some(c) = narrowphase(&self.bodies, i, j) {
                contacts.push(c);
            }
        }

        solve_contacts(&mut self.bodies, &contacts, dt);
        solve_constraints(&mut self.bodies, &self.constraints, dt);

        for b in self.bodies.iter_mut() {
            b.update_sleep_state();
        }
    }

    pub fn raycast(&self, origin: Vec3, dir: Vec3, max_dist: f32, skip_body: Option<usize>) -> Option<RaycastHit> {
        if max_dist <= 0.0 || dir.length_sq() <= 1e-12 {
            return None;
        }

        let dir = dir.normalized();
        let mut best: Option<RaycastHit> = None;
        for (body, rigid_body) in self.bodies.iter().enumerate() {
            if Some(body) == skip_body {
                continue;
            }
            let hit = ray_shape(origin, dir, max_dist, body, &rigid_body.shape, rigid_body.transform);
            if let Some(hit) = hit {
                let replace = best.as_ref().map(|best_hit| hit.distance < best_hit.distance).unwrap_or(true);
                if replace {
                    best = Some(hit);
                }
            }
        }
        best
    }

    /// 形状扫掠测试:让 `shape` 从 `from` 出发沿 `dir` 平移 `max_dist`,
    /// 返回最早接触。Sphere 走精确解;其余形状用包围球近似(与 CCD 一致),
    /// 因此结果是保守的(可能略早报触,不会穿透目标)。
    pub fn sweep_test(&self, shape: &Shape, from: Vec3, dir: Vec3, max_dist: f32) -> Option<SweepHit> {
        if max_dist <= 0.0 || dir.length_sq() <= 1e-12 {
            return None;
        }
        let radius = sweep_shape_radius(shape)?;
        let dir = dir.normalized();

        let mut best: Option<SweepHit> = None;
        for (body, rigid_body) in self.bodies.iter().enumerate() {
            let hit = sweep_against_body(from, dir, max_dist, radius, body, &rigid_body.shape, rigid_body.transform);
            if let Some(hit) = hit {
                let replace = best.as_ref().map(|best_hit| hit.distance < best_hit.distance).unwrap_or(true);
                if replace {
                    best = Some(hit);
                }
            }
        }
        best
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}

fn apply_ccd(bodies: &mut [RigidBody], previous_transforms: &[Transform]) {
    for i in 0..bodies.len() {
        if bodies[i].is_static || bodies[i].is_sleeping {
            continue;
        }
        let radius = match shape_ccd_radius(&bodies[i].shape) {
            Some(radius) => radius,
            None => continue,
        };
        let start = previous_transforms[i].position;
        let end = bodies[i].transform.position;
        let delta = end - start;
        let travel = delta.length();
        if travel <= radius * CCD_MOTION_THRESHOLD {
            continue;
        }

        let dir = delta.scale(1.0 / travel);
        let mut best_toi = 1.0f32;
        let mut best_normal = None;
        for j in 0..bodies.len() {
            if i == j || !bodies[j].is_static {
                continue;
            }
            if let Some((toi, normal)) = sweep_shape_against_static(
                start,
                dir,
                travel,
                radius,
                &bodies[j].shape,
                bodies[j].transform,
            ) {
                if toi < best_toi {
                    best_toi = toi;
                    best_normal = Some(normal);
                }
            }
        }

        if let Some(normal) = best_normal {
            let clamped = (best_toi * CCD_BACKOFF).clamp(0.0, 1.0);
            bodies[i].transform.position = start + delta.scale(clamped);
            let vn = bodies[i].linear_velocity.dot(normal);
            if vn < 0.0 {
                bodies[i].linear_velocity = bodies[i].linear_velocity - normal.scale(vn);
            }
        }
    }
}

fn shape_ccd_radius(shape: &Shape) -> Option<f32> {
    match *shape {
        Shape::Sphere { radius } => Some(radius),
        Shape::Box { half_extents } => Some(half_extents.length()),
        _ => None,
    }
}

/// sweep_test 里扫掠形状取多大半径:Sphere 精确,Box/Capsule/凸包/Compound
/// 退化为包围球,Plane/TriangleMesh 不能作为扫掠形状。
fn sweep_shape_radius(shape: &Shape) -> Option<f32> {
    match *shape {
        Shape::Sphere { radius } => Some(radius),
        Shape::Box { half_extents } => Some(half_extents.length()),
        Shape::Capsule { half_height, radius } => Some(half_height + radius),
        Shape::ConvexHull { .. } | Shape::Compound { .. } => {
            let half = shape.local_aabb_half_extents();
            Some(half.length())
        }
        Shape::Plane { .. } | Shape::TriangleMesh { .. } => None,
    }
}

fn sweep_against_body(
    start: Vec3,
    dir: Vec3,
    max_dist: f32,
    radius: f32,
    body: usize,
    shape: &Shape,
    transform: Transform,
) -> Option<SweepHit> {
    match *shape {
        Shape::Sphere { radius: target_radius } => {
            sweep_sphere_sphere(start, dir, max_dist, radius, body, transform.position, target_radius)
        }
        Shape::Plane { normal, offset } => {
            let (toi, normal) = sweep_sphere_plane(start, dir, max_dist, radius, normal, offset)?;
            Some(make_sweep_hit(body, start, dir, max_dist, radius, toi, normal))
        }
        Shape::Box { half_extents } => {
            let (toi, normal) = sweep_sphere_box(start, dir, max_dist, radius, transform, half_extents)?;
            Some(make_sweep_hit(body, start, dir, max_dist, radius, toi, normal))
        }
        Shape::TriangleMesh { triangles } => {
            let (toi, normal) = sweep_sphere_triangle_mesh(start, dir, max_dist, radius, triangles, transform)?;
            Some(make_sweep_hit(body, start, dir, max_dist, radius, toi, normal))
        }
        Shape::Capsule { half_height, radius: capsule_radius } => {
            // 保守近似:用胶囊的包围球
            sweep_sphere_sphere(start, dir, max_dist, radius, body, transform.position, half_height + capsule_radius)
        }
        Shape::Compound { children } => {
            let mut best: Option<SweepHit> = None;
            for child in children {
                let child_transform = compose_transform(transform, child.transform);
                if let Some(hit) = sweep_against_body(start, dir, max_dist, radius, body, &child.shape, child_transform) {
                    let replace = best.as_ref().map(|best_hit| hit.distance < best_hit.distance).unwrap_or(true);
                    if replace {
                        best = Some(hit);
                    }
                }
            }
            best
        }
        Shape::ConvexHull { .. } => {
            // 保守近似:用凸包的包围球(以刚体质心为中心)
            sweep_sphere_sphere(start, dir, max_dist, radius, body, transform.position, shape.local_aabb_half_extents().length())
        }
    }
}

fn make_sweep_hit(body: usize, start: Vec3, dir: Vec3, max_dist: f32, radius: f32, toi: f32, normal: Vec3) -> SweepHit {
    let distance = toi * max_dist;
    let center = start + dir.scale(distance);
    SweepHit {
        body,
        point: center - normal.scale(radius),
        normal,
        distance,
        fraction: toi,
    }
}

/// 球-球扫掠:退化成"点射线 vs 半径相加的球"的解析解。
fn sweep_sphere_sphere(
    start: Vec3,
    dir: Vec3,
    max_dist: f32,
    radius: f32,
    body: usize,
    center: Vec3,
    target_radius: f32,
) -> Option<SweepHit> {
    let combined = radius + target_radius;
    let hit = ray_sphere(start, dir, max_dist, body, center, combined)?;
    let center_at_hit = start + dir.scale(hit.distance);
    Some(SweepHit {
        body,
        point: center_at_hit - hit.normal.scale(radius),
        normal: hit.normal,
        distance: hit.distance,
        fraction: hit.distance / max_dist.max(1e-6),
    })
}

fn sweep_shape_against_static(
    start: Vec3,
    dir: Vec3,
    max_dist: f32,
    radius: f32,
    shape: &Shape,
    transform: Transform,
) -> Option<(f32, Vec3)> {
    match *shape {
        Shape::Plane { normal, offset } => sweep_sphere_plane(start, dir, max_dist, radius, normal, offset),
        Shape::Box { half_extents } => sweep_sphere_box(start, dir, max_dist, radius, transform, half_extents),
        Shape::TriangleMesh { triangles } => sweep_sphere_triangle_mesh(start, dir, max_dist, radius, triangles, transform),
        _ => None,
    }
}

fn sweep_sphere_plane(
    start: Vec3,
    dir: Vec3,
    max_dist: f32,
    radius: f32,
    normal: Vec3,
    offset: f32,
) -> Option<(f32, Vec3)> {
    let denom = normal.dot(dir);
    if denom >= -1e-6 {
        return None;
    }
    let start_dist = normal.dot(start) - offset;
    let t = (radius - start_dist) / denom;
    if t < 0.0 || t > max_dist {
        return None;
    }
    Some((t / max_dist.max(1e-6), normal))
}

fn sweep_sphere_box(
    start: Vec3,
    dir: Vec3,
    max_dist: f32,
    radius: f32,
    transform: Transform,
    half_extents: Vec3,
) -> Option<(f32, Vec3)> {
    let expanded = half_extents + Vec3::new(radius, radius, radius);
    let hit = ray_box(start, dir, max_dist, 0, transform, expanded)?;
    Some((hit.distance / max_dist.max(1e-6), hit.normal))
}

fn sweep_sphere_triangle_mesh(
    start: Vec3,
    dir: Vec3,
    max_dist: f32,
    radius: f32,
    triangles: &'static [MeshTriangle],
    transform: Transform,
) -> Option<(f32, Vec3)> {
    let mut best: Option<(f32, Vec3)> = None;
    for tri in triangles {
        let a = transform.transform_point(tri.a);
        let b = transform.transform_point(tri.b);
        let c = transform.transform_point(tri.c);
        let normal = (b - a).cross(c - a).normalized();
        if normal.length_sq() <= 1e-12 {
            continue;
        }
        let offset_triangle = [
            a + normal.scale(radius),
            b + normal.scale(radius),
            c + normal.scale(radius),
        ];
        if let Some(hit) = ray_triangle(start, dir, max_dist, 0, offset_triangle[0], offset_triangle[1], offset_triangle[2]) {
            let candidate = (hit.distance / max_dist.max(1e-6), hit.normal);
            let replace = best.as_ref().map(|current| candidate.0 < current.0).unwrap_or(true);
            if replace {
                best = Some(candidate);
            }
        }
    }
    best
}

fn ray_sphere(origin: Vec3, dir: Vec3, max_dist: f32, body: usize, center: Vec3, radius: f32) -> Option<RaycastHit> {
    let oc = origin - center;
    let b = oc.dot(dir);
    let c = oc.dot(oc) - radius * radius;
    let disc = b * b - c;
    if disc < 0.0 {
        return None;
    }
    let sqrt_disc = disc.sqrt();
    let mut t = -b - sqrt_disc;
    if t < 0.0 {
        t = -b + sqrt_disc;
    }
    if t < 0.0 || t > max_dist {
        return None;
    }
    let point = origin + dir.scale(t);
    Some(RaycastHit {
        body,
        point,
        normal: (point - center).normalized(),
        distance: t,
    })
}

fn ray_plane(origin: Vec3, dir: Vec3, max_dist: f32, body: usize, normal: Vec3, offset: f32) -> Option<RaycastHit> {
    let denom = normal.dot(dir);
    if denom.abs() < 1e-6 {
        return None;
    }
    let t = (offset - normal.dot(origin)) / denom;
    if t < 0.0 || t > max_dist {
        return None;
    }
    Some(RaycastHit {
        body,
        point: origin + dir.scale(t),
        normal: if denom < 0.0 { normal } else { -normal },
        distance: t,
    })
}

fn ray_box(origin: Vec3, dir: Vec3, max_dist: f32, body: usize, transform: Transform, half_extents: Vec3) -> Option<RaycastHit> {
    let rot = transform.rotation.to_mat3();
    let inv_rot = rot.transposed();
    let local_origin = inv_rot.mul_vec3(origin - transform.position);
    let local_dir = inv_rot.mul_vec3(dir);

    let origin_arr = [local_origin.x, local_origin.y, local_origin.z];
    let dir_arr = [local_dir.x, local_dir.y, local_dir.z];
    let ext_arr = [half_extents.x, half_extents.y, half_extents.z];

    let mut t_min = 0.0f32;
    let mut t_max = max_dist;
    let mut enter_normal = Vec3::ZERO;
    let mut exit_normal = Vec3::ZERO;

    for axis in 0..3 {
        let o = origin_arr[axis];
        let d = dir_arr[axis];
        let e = ext_arr[axis];
        if d.abs() < 1e-6 {
            if o < -e || o > e {
                return None;
            }
            continue;
        }

        let inv_d = 1.0 / d;
        let mut t1 = (-e - o) * inv_d;
        let mut t2 = (e - o) * inv_d;
        let mut n1 = axis_normal(axis, -1.0);
        let mut n2 = axis_normal(axis, 1.0);
        if t1 > t2 {
            core::mem::swap(&mut t1, &mut t2);
            core::mem::swap(&mut n1, &mut n2);
        }
        if t1 > t_min {
            t_min = t1;
            enter_normal = n1;
        }
        if t2 < t_max {
            t_max = t2;
            exit_normal = n2;
        }
        if t_min > t_max {
            return None;
        }
    }

    let (distance, local_normal) = if t_min >= 0.0 {
        (t_min, enter_normal)
    } else if t_max >= 0.0 {
        (t_max, exit_normal)
    } else {
        return None;
    };

    if distance > max_dist {
        return None;
    }

    let point = origin + dir.scale(distance);
    Some(RaycastHit {
        body,
        point,
        normal: rot.mul_vec3(local_normal).normalized(),
        distance,
    })
}

fn axis_normal(axis: usize, sign: f32) -> Vec3 {
    match axis {
        0 => Vec3::new(sign, 0.0, 0.0),
        1 => Vec3::new(0.0, sign, 0.0),
        _ => Vec3::new(0.0, 0.0, sign),
    }
}

fn ray_shape(origin: Vec3, dir: Vec3, max_dist: f32, body: usize, shape: &Shape, transform: Transform) -> Option<RaycastHit> {
    match *shape {
        Shape::Sphere { radius } => ray_sphere(origin, dir, max_dist, body, transform.position, radius),
        Shape::Plane { normal, offset } => ray_plane(origin, dir, max_dist, body, normal, offset),
        Shape::Box { half_extents } => ray_box(origin, dir, max_dist, body, transform, half_extents),
        Shape::TriangleMesh { triangles } => ray_triangle_mesh(origin, dir, max_dist, body, triangles, transform),
        Shape::Compound { children } => ray_compound(origin, dir, max_dist, body, children, transform),
        Shape::Capsule { .. } => None,
        Shape::ConvexHull { .. } => None,
    }
}

fn ray_compound(
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
    body: usize,
    children: &'static [CompoundChild],
    parent: Transform,
) -> Option<RaycastHit> {
    let mut best: Option<RaycastHit> = None;
    for child in children {
        let child_transform = compose_transform(parent, child.transform);
        if let Some(hit) = ray_shape(origin, dir, max_dist, body, &child.shape, child_transform) {
            let replace = best.as_ref().map(|best_hit| hit.distance < best_hit.distance).unwrap_or(true);
            if replace {
                best = Some(hit);
            }
        }
    }
    best
}

fn compose_transform(parent: Transform, local: Transform) -> Transform {
    Transform {
        position: parent.transform_point(local.position),
        rotation: parent.rotation.mul(local.rotation).normalized(),
    }
}

fn ray_triangle_mesh(
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
    body: usize,
    triangles: &'static [MeshTriangle],
    transform: Transform,
) -> Option<RaycastHit> {
    let mut best: Option<RaycastHit> = None;
    for tri in triangles {
        let a = transform.transform_point(tri.a);
        let b = transform.transform_point(tri.b);
        let c = transform.transform_point(tri.c);
        if let Some(hit) = ray_triangle(origin, dir, max_dist, body, a, b, c) {
            let replace = best.as_ref().map(|best_hit| hit.distance < best_hit.distance).unwrap_or(true);
            if replace {
                best = Some(hit);
            }
        }
    }
    best
}

fn ray_triangle(origin: Vec3, dir: Vec3, max_dist: f32, body: usize, a: Vec3, b: Vec3, c: Vec3) -> Option<RaycastHit> {
    let ab = b - a;
    let ac = c - a;
    let p = dir.cross(ac);
    let det = ab.dot(p);
    if det.abs() < 1e-6 {
        return None;
    }
    let inv_det = 1.0 / det;
    let tvec = origin - a;
    let u = tvec.dot(p) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = tvec.cross(ab);
    let v = dir.dot(q) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let distance = ac.dot(q) * inv_det;
    if distance < 0.0 || distance > max_dist {
        return None;
    }

    let normal = ab.cross(ac).normalized();
    if normal.length_sq() <= 1e-12 {
        return None;
    }

    Some(RaycastHit {
        body,
        point: origin + dir.scale(distance),
        normal: if normal.dot(dir) < 0.0 { normal } else { -normal },
        distance,
    })
}

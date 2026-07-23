use crate::body::RigidBody;
use crate::math::Vec3;

const CONSTRAINT_ITERATIONS: usize = 12;
const CONSTRAINT_BAUMGARTE: f32 = 0.2;

#[derive(Clone, Copy, Debug)]
pub enum Constraint {
    Point2Point(Point2PointConstraint),
    Hinge(HingeConstraint),
    Generic6Dof(Generic6DofConstraint),
    Slider(SliderConstraint),
}

#[derive(Clone, Copy, Debug)]
pub struct Point2PointConstraint {
    pub a: usize,
    pub b: usize,
    pub pivot_a_local: Vec3,
    pub pivot_b_local: Vec3,
}

impl Point2PointConstraint {
    pub fn new(a: usize, b: usize, pivot_a_local: Vec3, pivot_b_local: Vec3) -> Self {
        Self { a, b, pivot_a_local, pivot_b_local }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct HingeConstraint {
    pub a: usize,
    pub b: usize,
    pub pivot_a_local: Vec3,
    pub pivot_b_local: Vec3,
    pub axis_a_local: Vec3,
    pub axis_b_local: Vec3,
}

impl HingeConstraint {
    pub fn new(
        a: usize,
        b: usize,
        pivot_a_local: Vec3,
        pivot_b_local: Vec3,
        axis_a_local: Vec3,
        axis_b_local: Vec3,
    ) -> Self {
        Self {
            a,
            b,
            pivot_a_local,
            pivot_b_local,
            axis_a_local: axis_a_local.normalized(),
            axis_b_local: axis_b_local.normalized(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AxisLock {
    pub x: bool,
    pub y: bool,
    pub z: bool,
}

impl AxisLock {
    pub fn all_locked() -> Self {
        Self { x: true, y: true, z: true }
    }

    pub fn from_bools(x: bool, y: bool, z: bool) -> Self {
        Self { x, y, z }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Generic6DofConstraint {
    pub a: usize,
    pub b: usize,
    pub pivot_a_local: Vec3,
    pub pivot_b_local: Vec3,
    pub linear_lock: AxisLock,
    pub angular_lock: AxisLock,
}

impl Generic6DofConstraint {
    pub fn new(
        a: usize,
        b: usize,
        pivot_a_local: Vec3,
        pivot_b_local: Vec3,
        linear_lock: AxisLock,
        angular_lock: AxisLock,
    ) -> Self {
        Self { a, b, pivot_a_local, pivot_b_local, linear_lock, angular_lock }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SliderConstraint {
    pub a: usize,
    pub b: usize,
    pub pivot_a_local: Vec3,
    pub pivot_b_local: Vec3,
    pub axis_a_local: Vec3,
    pub axis_b_local: Vec3,
}

impl SliderConstraint {
    pub fn new(
        a: usize,
        b: usize,
        pivot_a_local: Vec3,
        pivot_b_local: Vec3,
        axis_a_local: Vec3,
        axis_b_local: Vec3,
    ) -> Self {
        Self {
            a,
            b,
            pivot_a_local,
            pivot_b_local,
            axis_a_local: axis_a_local.normalized(),
            axis_b_local: axis_b_local.normalized(),
        }
    }
}

pub fn solve_constraints(bodies: &mut [RigidBody], constraints: &[Constraint], dt: f32) {
    for _ in 0..CONSTRAINT_ITERATIONS {
        for constraint in constraints {
            match *constraint {
                Constraint::Point2Point(p2p) => solve_point2point(bodies, p2p, dt),
                Constraint::Hinge(hinge) => solve_hinge(bodies, hinge, dt),
                Constraint::Generic6Dof(dof) => solve_generic_6dof(bodies, dof, dt),
                Constraint::Slider(slider) => solve_slider(bodies, slider, dt),
            }
        }
    }
}

fn solve_slider(bodies: &mut [RigidBody], slider: SliderConstraint, dt: f32) {
    solve_generic_6dof(
        bodies,
        Generic6DofConstraint::new(
            slider.a,
            slider.b,
            slider.pivot_a_local,
            slider.pivot_b_local,
            AxisLock::from_bools(false, true, true),
            AxisLock::all_locked(),
        ),
        dt,
    );

    if slider.a >= bodies.len() || slider.b >= bodies.len() || slider.a == slider.b {
        return;
    }

    let axis_a = bodies[slider.a]
        .transform
        .rotation
        .to_mat3()
        .mul_vec3(slider.axis_a_local)
        .normalized();
    let axis_b = bodies[slider.b]
        .transform
        .rotation
        .to_mat3()
        .mul_vec3(slider.axis_b_local)
        .normalized();
    if axis_a.length_sq() <= 1e-8 || axis_b.length_sq() <= 1e-8 {
        return;
    }

    solve_angular_alignment(bodies, slider.a, slider.b, axis_a, axis_b, dt);
}

fn solve_hinge(bodies: &mut [RigidBody], hinge: HingeConstraint, dt: f32) {
    solve_point2point(
        bodies,
        Point2PointConstraint::new(hinge.a, hinge.b, hinge.pivot_a_local, hinge.pivot_b_local),
        dt,
    );

    if hinge.a >= bodies.len() || hinge.b >= bodies.len() || hinge.a == hinge.b {
        return;
    }

    let axis_a = bodies[hinge.a]
        .transform
        .rotation
        .to_mat3()
        .mul_vec3(hinge.axis_a_local)
        .normalized();
    let axis_b = bodies[hinge.b]
        .transform
        .rotation
        .to_mat3()
        .mul_vec3(hinge.axis_b_local)
        .normalized();
    if axis_a.length_sq() <= 1e-8 || axis_b.length_sq() <= 1e-8 {
        return;
    }

    let basis_a = orthonormal_basis(axis_a);
    let basis_b = orthonormal_basis(axis_b);
    solve_angular_alignment(bodies, hinge.a, hinge.b, basis_a.0, basis_b.0, dt);
    solve_angular_alignment(bodies, hinge.a, hinge.b, basis_a.1, basis_b.1, dt);
}

fn solve_generic_6dof(bodies: &mut [RigidBody], joint: Generic6DofConstraint, dt: f32) {
    if joint.a >= bodies.len() || joint.b >= bodies.len() || joint.a == joint.b {
        return;
    }

    let rot_a = bodies[joint.a].transform.rotation.to_mat3();
    let rot_b = bodies[joint.b].transform.rotation.to_mat3();
    let ra = rot_a.mul_vec3(joint.pivot_a_local);
    let rb = rot_b.mul_vec3(joint.pivot_b_local);
    let world_a = bodies[joint.a].transform.position + ra;
    let world_b = bodies[joint.b].transform.position + rb;
    let error = world_b - world_a;
    let basis = [
        rot_a.mul_vec3(Vec3::new(1.0, 0.0, 0.0)).normalized(),
        rot_a.mul_vec3(Vec3::new(0.0, 1.0, 0.0)).normalized(),
        rot_a.mul_vec3(Vec3::new(0.0, 0.0, 1.0)).normalized(),
    ];

    for (locked, axis) in [
        (joint.linear_lock.x, basis[0]),
        (joint.linear_lock.y, basis[1]),
        (joint.linear_lock.z, basis[2]),
    ] {
        if locked {
            solve_linear_axis(bodies, joint.a, joint.b, ra, rb, axis, error, dt);
        }
    }

    let basis_a = [
        rot_a.mul_vec3(Vec3::new(1.0, 0.0, 0.0)).normalized(),
        rot_a.mul_vec3(Vec3::new(0.0, 1.0, 0.0)).normalized(),
        rot_a.mul_vec3(Vec3::new(0.0, 0.0, 1.0)).normalized(),
    ];
    let basis_b = [
        rot_b.mul_vec3(Vec3::new(1.0, 0.0, 0.0)).normalized(),
        rot_b.mul_vec3(Vec3::new(0.0, 1.0, 0.0)).normalized(),
        rot_b.mul_vec3(Vec3::new(0.0, 0.0, 1.0)).normalized(),
    ];
    for ((locked, ref_a), ref_b) in [
        ((joint.angular_lock.x, basis_a[0]), basis_b[0]),
        ((joint.angular_lock.y, basis_a[1]), basis_b[1]),
        ((joint.angular_lock.z, basis_a[2]), basis_b[2]),
    ] {
        if locked {
            solve_angular_alignment(bodies, joint.a, joint.b, ref_a, ref_b, dt);
        }
    }
}

fn solve_point2point(bodies: &mut [RigidBody], joint: Point2PointConstraint, dt: f32) {
    if joint.a >= bodies.len() || joint.b >= bodies.len() || joint.a == joint.b {
        return;
    }

    let rot_a = bodies[joint.a].transform.rotation.to_mat3();
    let rot_b = bodies[joint.b].transform.rotation.to_mat3();
    let ra = rot_a.mul_vec3(joint.pivot_a_local);
    let rb = rot_b.mul_vec3(joint.pivot_b_local);
    let world_a = bodies[joint.a].transform.position + ra;
    let world_b = bodies[joint.b].transform.position + rb;
    let error = world_b - world_a;
    let error_len = error.length();
    if error_len <= 1e-6 {
        return;
    }

    let dir = error.scale(1.0 / error_len);
    let vel_a = bodies[joint.a].linear_velocity + bodies[joint.a].angular_velocity.cross(ra);
    let vel_b = bodies[joint.b].linear_velocity + bodies[joint.b].angular_velocity.cross(rb);
    let rel_vel = vel_b - vel_a;
    let bias = (CONSTRAINT_BAUMGARTE / dt.max(1e-6)) * error_len;
    let eff_mass = effective_mass_along(bodies, joint.a, joint.b, ra, rb, dir).max(1e-6);
    let j = -(rel_vel.dot(dir) + bias) / eff_mass;
    let impulse = dir.scale(j);
    apply_impulse(bodies, joint.a, joint.b, ra, rb, impulse);
}

fn effective_mass_along(bodies: &[RigidBody], a: usize, b: usize, ra: Vec3, rb: Vec3, dir: Vec3) -> f32 {
    let mut k = bodies[a].inv_mass + bodies[b].inv_mass;
    if !bodies[a].is_static {
        let inv_inertia_a = bodies[a].inv_inertia_world();
        let ang_a = inv_inertia_a.mul_vec3(ra.cross(dir)).cross(ra);
        k += ang_a.dot(dir);
    }
    if !bodies[b].is_static {
        let inv_inertia_b = bodies[b].inv_inertia_world();
        let ang_b = inv_inertia_b.mul_vec3(rb.cross(dir)).cross(rb);
        k += ang_b.dot(dir);
    }
    k
}

fn angular_effective_mass(bodies: &[RigidBody], a: usize, b: usize, axis: Vec3) -> f32 {
    let mut k = 0.0;
    if !bodies[a].is_static {
        k += bodies[a].inv_inertia_world().mul_vec3(axis).dot(axis);
    }
    if !bodies[b].is_static {
        k += bodies[b].inv_inertia_world().mul_vec3(axis).dot(axis);
    }
    k
}

fn apply_impulse(bodies: &mut [RigidBody], a: usize, b: usize, ra: Vec3, rb: Vec3, impulse: Vec3) {
    if !bodies[a].is_static {
        if impulse.length_sq() > 0.0 {
            bodies[a].wake_up();
        }
        let inv_mass_a = bodies[a].inv_mass;
        let inv_inertia_a = bodies[a].inv_inertia_world();
        bodies[a].linear_velocity = bodies[a].linear_velocity - impulse.scale(inv_mass_a);
        bodies[a].angular_velocity = bodies[a].angular_velocity - inv_inertia_a.mul_vec3(ra.cross(impulse));
    }
    if !bodies[b].is_static {
        if impulse.length_sq() > 0.0 {
            bodies[b].wake_up();
        }
        let inv_mass_b = bodies[b].inv_mass;
        let inv_inertia_b = bodies[b].inv_inertia_world();
        bodies[b].linear_velocity = bodies[b].linear_velocity + impulse.scale(inv_mass_b);
        bodies[b].angular_velocity = bodies[b].angular_velocity + inv_inertia_b.mul_vec3(rb.cross(impulse));
    }
}

fn apply_angular_impulse(bodies: &mut [RigidBody], a: usize, b: usize, impulse: Vec3) {
    if !bodies[a].is_static {
        if impulse.length_sq() > 0.0 {
            bodies[a].wake_up();
        }
        let inv_inertia_a = bodies[a].inv_inertia_world();
        bodies[a].angular_velocity = bodies[a].angular_velocity - inv_inertia_a.mul_vec3(impulse);
    }
    if !bodies[b].is_static {
        if impulse.length_sq() > 0.0 {
            bodies[b].wake_up();
        }
        let inv_inertia_b = bodies[b].inv_inertia_world();
        bodies[b].angular_velocity = bodies[b].angular_velocity + inv_inertia_b.mul_vec3(impulse);
    }
}

fn solve_angular_alignment(bodies: &mut [RigidBody], a: usize, b: usize, ref_a: Vec3, ref_b: Vec3, dt: f32) {
    let error_axis = ref_a.cross(ref_b);
    let error = error_axis.length();
    if error <= 1e-6 {
        return;
    }

    let axis = error_axis.scale(1.0 / error);
    let rel_ang_vel = bodies[b].angular_velocity - bodies[a].angular_velocity;
    let bias = (CONSTRAINT_BAUMGARTE / dt.max(1e-6)) * error;
    let eff_mass = angular_effective_mass(bodies, a, b, axis).max(1e-6);
    let j = -(rel_ang_vel.dot(axis) + bias) / eff_mass;
    apply_angular_impulse(bodies, a, b, axis.scale(j));
}

fn solve_linear_axis(
    bodies: &mut [RigidBody],
    a: usize,
    b: usize,
    ra: Vec3,
    rb: Vec3,
    axis: Vec3,
    error: Vec3,
    dt: f32,
) {
    let axis = axis.normalized();
    if axis.length_sq() <= 1e-8 {
        return;
    }
    let error_scalar = error.dot(axis);
    if error_scalar.abs() <= 1e-6 {
        return;
    }

    let vel_a = bodies[a].linear_velocity + bodies[a].angular_velocity.cross(ra);
    let vel_b = bodies[b].linear_velocity + bodies[b].angular_velocity.cross(rb);
    let rel_vel = vel_b - vel_a;
    let bias = (CONSTRAINT_BAUMGARTE / dt.max(1e-6)) * error_scalar;
    let eff_mass = effective_mass_along(bodies, a, b, ra, rb, axis).max(1e-6);
    let j = -(rel_vel.dot(axis) + bias) / eff_mass;
    apply_impulse(bodies, a, b, ra, rb, axis.scale(j));
}

fn orthonormal_basis(axis: Vec3) -> (Vec3, Vec3) {
    let tangent = if axis.x.abs() < 0.8 {
        axis.cross(Vec3::new(1.0, 0.0, 0.0)).normalized()
    } else {
        axis.cross(Vec3::new(0.0, 1.0, 0.0)).normalized()
    };
    let bitangent = axis.cross(tangent).normalized();
    (tangent, bitangent)
}

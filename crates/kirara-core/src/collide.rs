//! 窄相碰撞检测(narrowphase)。
//! 当前实现的常见形状对:
//!   sphere-sphere / sphere-plane / box-plane / box-sphere / box-box
//!   capsule-sphere / capsule-plane / capsule-box / capsule-capsule
//!   sphere/capsule/box 对静态 triangle mesh

use crate::body::RigidBody;
use crate::math::{Transform, Vec3};
use crate::shape::{CompoundChild, MeshTriangle, Shape};

/// 一个接触点。法线方向定义为:从物体 A 指向物体 B。
#[derive(Clone, Copy, Debug)]
pub struct Contact {
    pub a: usize,
    pub b: usize,
    pub point: Vec3,
    pub normal: Vec3,
    pub penetration: f32,
}

#[derive(Clone, Copy)]
struct BroadphaseBody {
    index: usize,
    min: Vec3,
    max: Vec3,
    center: Vec3,
    is_static: bool,
}

#[derive(Clone, Debug)]
struct BvhNode {
    min: Vec3,
    max: Vec3,
    left: Option<usize>,
    right: Option<usize>,
    body: Option<usize>,
}

pub fn narrowphase(bodies: &[RigidBody], a: usize, b: usize) -> Option<Contact> {
    let (ra, rb) = (&bodies[a], &bodies[b]);
    shape_contact(a, &ra.shape, ra.transform, b, &rb.shape, rb.transform)
}

fn shape_contact(a: usize, shape_a: &Shape, ta: Transform, b: usize, shape_b: &Shape, tb: Transform) -> Option<Contact> {
    if let Shape::Compound { children } = shape_a {
        return best_child_contact(*children, ta, b, shape_b, tb, a, true);
    }
    if let Shape::Compound { children } = shape_b {
        return best_child_contact(*children, tb, a, shape_a, ta, b, false);
    }

    match (*shape_a, *shape_b) {
        (Shape::Sphere { radius: rra }, Shape::Sphere { radius: rrb }) => {
            sphere_sphere(a, b, ta.position, rra, tb.position, rrb)
        }
        (Shape::Sphere { radius }, Shape::Plane { normal, offset }) => {
            sphere_plane(a, b, ta.position, radius, normal, offset, false)
        }
        (Shape::Plane { normal, offset }, Shape::Sphere { radius }) => {
            sphere_plane(b, a, tb.position, radius, normal, offset, true)
        }
        (Shape::Box { half_extents }, Shape::Plane { normal, offset }) => {
            box_plane(a, b, ta, half_extents, normal, offset, false)
        }
        (Shape::Plane { normal, offset }, Shape::Box { half_extents }) => {
            box_plane(b, a, tb, half_extents, normal, offset, true)
        }
        (Shape::Sphere { radius }, Shape::Box { half_extents }) => {
            sphere_box(a, b, ta.position, radius, tb, half_extents, false)
        }
        (Shape::Box { half_extents }, Shape::Sphere { radius }) => {
            sphere_box(b, a, tb.position, radius, ta, half_extents, true)
        }
        (Shape::Box { half_extents: ha }, Shape::Box { half_extents: hb }) => {
            box_box(a, b, ta, ha, tb, hb)
        }
        (Shape::Sphere { radius }, Shape::TriangleMesh { triangles }) => {
            sphere_triangle_mesh(a, b, ta.position, radius, tb, triangles, false)
        }
        (Shape::TriangleMesh { triangles }, Shape::Sphere { radius }) => {
            sphere_triangle_mesh(b, a, tb.position, radius, ta, triangles, true)
        }
        (Shape::Box { half_extents }, Shape::TriangleMesh { triangles }) => {
            box_triangle_mesh(a, b, ta, half_extents, tb, triangles, false)
        }
        (Shape::TriangleMesh { triangles }, Shape::Box { half_extents }) => {
            box_triangle_mesh(b, a, tb, half_extents, ta, triangles, true)
        }
        (Shape::Capsule { half_height, radius }, Shape::Sphere { radius: sphere_radius }) => {
            capsule_sphere(a, b, ta, half_height, radius, tb.position, sphere_radius, false)
        }
        (Shape::Sphere { radius: sphere_radius }, Shape::Capsule { half_height, radius }) => {
            capsule_sphere(b, a, tb, half_height, radius, ta.position, sphere_radius, true)
        }
        (Shape::Capsule { half_height, radius }, Shape::Plane { normal, offset }) => {
            capsule_plane(a, b, ta, half_height, radius, normal, offset, false)
        }
        (Shape::Plane { normal, offset }, Shape::Capsule { half_height, radius }) => {
            capsule_plane(b, a, tb, half_height, radius, normal, offset, true)
        }
        (Shape::Capsule { half_height, radius }, Shape::Box { half_extents }) => {
            capsule_box(a, b, ta, half_height, radius, tb, half_extents, false)
        }
        (Shape::Box { half_extents }, Shape::Capsule { half_height, radius }) => {
            capsule_box(b, a, tb, half_height, radius, ta, half_extents, true)
        }
        (Shape::Capsule { half_height: ha, radius: ra_radius }, Shape::Capsule { half_height: hb, radius: rb_radius }) => {
            capsule_capsule(a, b, ta, ha, ra_radius, tb, hb, rb_radius)
        }
        (Shape::Capsule { half_height, radius }, Shape::TriangleMesh { triangles }) => {
            capsule_triangle_mesh(a, b, ta, half_height, radius, tb, triangles, false)
        }
        (Shape::TriangleMesh { triangles }, Shape::Capsule { half_height, radius }) => {
            capsule_triangle_mesh(b, a, tb, half_height, radius, ta, triangles, true)
        }
        _ => None,
    }
}

fn best_child_contact(
    children: &'static [CompoundChild],
    compound_transform: Transform,
    other_index: usize,
    other_shape: &Shape,
    other_transform: Transform,
    compound_index: usize,
    compound_is_a: bool,
) -> Option<Contact> {
    let mut best: Option<Contact> = None;
    for child in children {
        let child_transform = compose_transform(compound_transform, child.transform);
        let contact = if compound_is_a {
            shape_contact(compound_index, &child.shape, child_transform, other_index, other_shape, other_transform)
        } else {
            shape_contact(other_index, other_shape, other_transform, compound_index, &child.shape, child_transform)
        };
        if let Some(contact) = contact {
            let replace = best
                .as_ref()
                .map(|current| contact.penetration > current.penetration)
                .unwrap_or(true);
            if replace {
                best = Some(contact);
            }
        }
    }
    best
}

fn sphere_sphere(a: usize, b: usize, pa: Vec3, ra: f32, pb: Vec3, rb: f32) -> Option<Contact> {
    let delta = pb - pa;
    let dist_sq = delta.length_sq();
    let sum_r = ra + rb;
    if dist_sq >= sum_r * sum_r || dist_sq < 1e-12 {
        return None;
    }
    let dist = dist_sq.sqrt();
    let normal = delta.scale(1.0 / dist);
    let penetration = sum_r - dist;
    let point = pa + normal.scale(ra - penetration * 0.5);
    Some(Contact { a, b, point, normal, penetration })
}

fn sphere_plane(sphere_idx: usize, plane_idx: usize, center: Vec3, radius: f32, normal: Vec3, offset: f32, flip: bool) -> Option<Contact> {
    let dist = center.dot(normal) - offset;
    if dist >= radius {
        return None;
    }
    let penetration = radius - dist;
    let point = center - normal.scale(dist);
    if flip {
        // 此时 a = plane_idx, b = sphere_idx,法线需要指向 sphere 一侧保持“A->B”约定
        Some(Contact { a: plane_idx, b: sphere_idx, point, normal, penetration })
    } else {
        Some(Contact { a: sphere_idx, b: plane_idx, point, normal: -normal, penetration })
    }
}

fn box_plane(box_idx: usize, plane_idx: usize, transform: Transform, half_extents: Vec3, normal: Vec3, offset: f32, flip: bool) -> Option<Contact> {
    // 把穿透平面的顶点聚合成一个近似接触点。
    // 当一个面几乎完全贴地时,比“只取最深单点”更不容易制造持续抖动/扭矩。
    let corners = [
        Vec3::new(half_extents.x, half_extents.y, half_extents.z),
        Vec3::new(-half_extents.x, half_extents.y, half_extents.z),
        Vec3::new(half_extents.x, -half_extents.y, half_extents.z),
        Vec3::new(half_extents.x, half_extents.y, -half_extents.z),
        Vec3::new(-half_extents.x, -half_extents.y, half_extents.z),
        Vec3::new(-half_extents.x, half_extents.y, -half_extents.z),
        Vec3::new(half_extents.x, -half_extents.y, -half_extents.z),
        Vec3::new(-half_extents.x, -half_extents.y, -half_extents.z),
    ];
    let mut deepest_dist = f32::INFINITY;
    let mut point_sum = Vec3::ZERO;
    let mut count = 0usize;
    for c in corners {
        let world_p = transform.transform_point(c);
        let dist = world_p.dot(normal) - offset;
        if dist < 0.0 {
            let projected = world_p - normal.scale(dist);
            point_sum = point_sum + projected;
            count += 1;
            if dist < deepest_dist {
                deepest_dist = dist;
            }
        }
    }
    if count == 0 {
        return None;
    }
    let point = point_sum.scale(1.0 / count as f32);
    let penetration = -deepest_dist;
    if flip {
        Some(Contact { a: plane_idx, b: box_idx, point, normal, penetration })
    } else {
        Some(Contact { a: box_idx, b: plane_idx, point, normal: -normal, penetration })
    }
}

/// 用最近点近似 box-sphere:先把球心变换到 box 的局部坐标,clamp 后再转回世界坐标。
fn sphere_box(sphere_idx: usize, box_idx: usize, sphere_c: Vec3, radius: f32, box_transform: Transform, half_extents: Vec3, flip: bool) -> Option<Contact> {
    let rot = box_transform.rotation.to_mat3();
    let inv_rot = rot.transposed();
    let local = inv_rot.mul_vec3(sphere_c - box_transform.position);
    let clamped = Vec3::new(
        local.x.clamp(-half_extents.x, half_extents.x),
        local.y.clamp(-half_extents.y, half_extents.y),
        local.z.clamp(-half_extents.z, half_extents.z),
    );
    let closest = box_transform.position + rot.mul_vec3(clamped);
    let delta = sphere_c - closest;
    let dist_sq = delta.length_sq();
    if dist_sq >= radius * radius || dist_sq < 1e-12 {
        return None;
    }
    let dist = dist_sq.sqrt();
    let normal = delta.scale(1.0 / dist); // box -> sphere
    let penetration = radius - dist;
    if flip {
        Some(Contact { a: box_idx, b: sphere_idx, point: closest, normal, penetration })
    } else {
        Some(Contact { a: sphere_idx, b: box_idx, point: closest, normal: -normal, penetration })
    }
}

fn box_box(a: usize, b: usize, ta: Transform, ea: Vec3, tb: Transform, eb: Vec3) -> Option<Contact> {
    let axes_a = box_axes(ta);
    let axes_b = box_axes(tb);
    let center_delta = tb.position - ta.position;
    let t = [
        center_delta.dot(axes_a[0]),
        center_delta.dot(axes_a[1]),
        center_delta.dot(axes_a[2]),
    ];

    let mut r = [[0.0f32; 3]; 3];
    let mut abs_r = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = axes_a[i].dot(axes_b[j]);
            abs_r[i][j] = r[i][j].abs() + 1e-6;
        }
    }

    let ext_a = [ea.x, ea.y, ea.z];
    let ext_b = [eb.x, eb.y, eb.z];
    let mut min_penetration = f32::INFINITY;
    let mut best_axis = Vec3::ZERO;

    for i in 0..3 {
        let ra = ext_a[i];
        let rb = ext_b[0] * abs_r[i][0] + ext_b[1] * abs_r[i][1] + ext_b[2] * abs_r[i][2];
        let dist = t[i].abs();
        if dist > ra + rb {
            return None;
        }
        let penetration = ra + rb - dist;
        update_best_axis(
            &mut min_penetration,
            &mut best_axis,
            penetration,
            orient_axis(axes_a[i], center_delta),
        );
    }

    for j in 0..3 {
        let ra = ext_a[0] * abs_r[0][j] + ext_a[1] * abs_r[1][j] + ext_a[2] * abs_r[2][j];
        let rb = ext_b[j];
        let dist = (t[0] * r[0][j] + t[1] * r[1][j] + t[2] * r[2][j]).abs();
        if dist > ra + rb {
            return None;
        }
        let penetration = ra + rb - dist;
        update_best_axis(
            &mut min_penetration,
            &mut best_axis,
            penetration,
            orient_axis(axes_b[j], center_delta),
        );
    }

    for i in 0..3 {
        for j in 0..3 {
            let axis = axes_a[i].cross(axes_b[j]);
            let axis_len_sq = axis.length_sq();
            if axis_len_sq < 1e-8 {
                continue;
            }
            let ra = ext_a[(i + 1) % 3] * abs_r[(i + 2) % 3][j]
                + ext_a[(i + 2) % 3] * abs_r[(i + 1) % 3][j];
            let rb = ext_b[(j + 1) % 3] * abs_r[i][(j + 2) % 3]
                + ext_b[(j + 2) % 3] * abs_r[i][(j + 1) % 3];
            let dist = (t[(i + 2) % 3] * r[(i + 1) % 3][j]
                - t[(i + 1) % 3] * r[(i + 2) % 3][j])
                .abs();
            if dist > ra + rb {
                return None;
            }
            let penetration = ra + rb - dist;
            update_best_axis(
                &mut min_penetration,
                &mut best_axis,
                penetration,
                orient_axis(axis.scale(1.0 / axis_len_sq.sqrt()), center_delta),
            );
        }
    }

    let support_a = box_support(ta, ea, best_axis);
    let support_b = box_support(tb, eb, -best_axis);
    let point = (support_a + support_b).scale(0.5);
    Some(Contact { a, b, point, normal: best_axis, penetration: min_penetration })
}

fn sphere_triangle_mesh(
    sphere_idx: usize,
    mesh_idx: usize,
    center: Vec3,
    radius: f32,
    mesh_transform: Transform,
    triangles: &'static [MeshTriangle],
    flip: bool,
) -> Option<Contact> {
    let mut best: Option<Contact> = None;
    for &tri in triangles {
        let tri = transform_triangle(mesh_transform, tri);
        if let Some(contact) = sphere_triangle_contact(sphere_idx, mesh_idx, center, radius, tri, flip) {
            let replace = best
                .as_ref()
                .map(|current| contact.penetration > current.penetration)
                .unwrap_or(true);
            if replace {
                best = Some(contact);
            }
        }
    }
    best
}

fn sphere_triangle_contact(
    sphere_idx: usize,
    mesh_idx: usize,
    center: Vec3,
    radius: f32,
    tri: MeshTriangle,
    flip: bool,
) -> Option<Contact> {
    let closest = closest_point_on_triangle(center, tri.a, tri.b, tri.c);
    let delta = center - closest;
    let dist_sq = delta.length_sq();
    if dist_sq >= radius * radius {
        return None;
    }

    let base_normal = triangle_normal(tri)?;
    let tri_center = triangle_center(tri);
    let normal = if dist_sq > 1e-12 {
        delta.normalized()
    } else {
        orient_axis(base_normal, center - tri_center)
    };
    let penetration = radius - dist_sq.sqrt();
    if flip {
        Some(Contact { a: mesh_idx, b: sphere_idx, point: closest, normal, penetration })
    } else {
        Some(Contact { a: sphere_idx, b: mesh_idx, point: closest, normal: -normal, penetration })
    }
}

fn capsule_sphere(
    capsule_idx: usize,
    sphere_idx: usize,
    capsule_transform: Transform,
    half_height: f32,
    capsule_radius: f32,
    sphere_center: Vec3,
    sphere_radius: f32,
    flip: bool,
) -> Option<Contact> {
    let (seg_a, seg_b) = capsule_segment(capsule_transform, half_height);
    let closest = closest_point_on_segment(seg_a, seg_b, sphere_center);
    sphere_sphere(capsule_idx, sphere_idx, closest, capsule_radius, sphere_center, sphere_radius).map(|c| {
        if flip {
            Contact { a: sphere_idx, b: capsule_idx, point: c.point, normal: -c.normal, penetration: c.penetration }
        } else {
            c
        }
    })
}

fn capsule_plane(
    capsule_idx: usize,
    plane_idx: usize,
    capsule_transform: Transform,
    half_height: f32,
    radius: f32,
    normal: Vec3,
    offset: f32,
    flip: bool,
) -> Option<Contact> {
    let (seg_a, seg_b) = capsule_segment(capsule_transform, half_height);
    let dist_a = seg_a.dot(normal) - offset;
    let dist_b = seg_b.dot(normal) - offset;
    let (closest, dist) = if dist_a <= dist_b { (seg_a, dist_a) } else { (seg_b, dist_b) };
    if dist >= radius {
        return None;
    }
    let point = closest - normal.scale(dist);
    let penetration = radius - dist;
    if flip {
        Some(Contact { a: plane_idx, b: capsule_idx, point, normal, penetration })
    } else {
        Some(Contact { a: capsule_idx, b: plane_idx, point, normal: -normal, penetration })
    }
}

fn capsule_box(
    capsule_idx: usize,
    box_idx: usize,
    capsule_transform: Transform,
    half_height: f32,
    capsule_radius: f32,
    box_transform: Transform,
    half_extents: Vec3,
    flip: bool,
) -> Option<Contact> {
    let (seg_a, seg_b) = capsule_segment(capsule_transform, half_height);
    let rot = box_transform.rotation.to_mat3();
    let inv_rot = rot.transposed();
    let local_a = inv_rot.mul_vec3(seg_a - box_transform.position);
    let local_b = inv_rot.mul_vec3(seg_b - box_transform.position);

    let (t, local_seg_point) = closest_segment_point_to_aabb(local_a, local_b, half_extents);
    let local_box_point = clamp_vec3(local_seg_point, -half_extents, half_extents);
    let world_seg_point = seg_a + (seg_b - seg_a).scale(t);
    let world_box_point = box_transform.position + rot.mul_vec3(local_box_point);
    let delta = world_seg_point - world_box_point;
    let dist_sq = delta.length_sq();
    if dist_sq >= capsule_radius * capsule_radius {
        return None;
    }

    let normal = if dist_sq > 1e-12 {
        delta.normalized()
    } else {
        orient_axis(world_seg_point - box_transform.position, seg_a - box_transform.position)
    };
    let penetration = capsule_radius - dist_sq.sqrt();
    if flip {
        Some(Contact { a: box_idx, b: capsule_idx, point: world_box_point, normal, penetration })
    } else {
        Some(Contact { a: capsule_idx, b: box_idx, point: world_box_point, normal: -normal, penetration })
    }
}

fn capsule_capsule(
    a: usize,
    b: usize,
    ta: Transform,
    half_height_a: f32,
    radius_a: f32,
    tb: Transform,
    half_height_b: f32,
    radius_b: f32,
) -> Option<Contact> {
    let (a0, a1) = capsule_segment(ta, half_height_a);
    let (b0, b1) = capsule_segment(tb, half_height_b);
    let (ca, cb) = closest_points_between_segments(a0, a1, b0, b1);
    sphere_sphere(a, b, ca, radius_a, cb, radius_b)
}

fn capsule_triangle_mesh(
    capsule_idx: usize,
    mesh_idx: usize,
    capsule_transform: Transform,
    half_height: f32,
    radius: f32,
    mesh_transform: Transform,
    triangles: &'static [MeshTriangle],
    flip: bool,
) -> Option<Contact> {
    let (seg_a, seg_b) = capsule_segment(capsule_transform, half_height);
    let mut best: Option<Contact> = None;
    for &tri in triangles {
        let tri = transform_triangle(mesh_transform, tri);
        if let Some(contact) = capsule_triangle_contact(capsule_idx, mesh_idx, seg_a, seg_b, radius, tri, flip) {
            let replace = best
                .as_ref()
                .map(|current| contact.penetration > current.penetration)
                .unwrap_or(true);
            if replace {
                best = Some(contact);
            }
        }
    }
    best
}

fn capsule_triangle_contact(
    capsule_idx: usize,
    mesh_idx: usize,
    seg_a: Vec3,
    seg_b: Vec3,
    radius: f32,
    tri: MeshTriangle,
    flip: bool,
) -> Option<Contact> {
    let (capsule_point, tri_point) = closest_points_segment_triangle(seg_a, seg_b, tri);
    let delta = capsule_point - tri_point;
    let dist_sq = delta.length_sq();
    if dist_sq >= radius * radius {
        return None;
    }

    let base_normal = triangle_normal(tri)?;
    let tri_center = triangle_center(tri);
    let normal = if dist_sq > 1e-12 {
        delta.normalized()
    } else {
        orient_axis(base_normal, capsule_point - tri_center)
    };
    let penetration = radius - dist_sq.sqrt();
    if flip {
        Some(Contact { a: mesh_idx, b: capsule_idx, point: tri_point, normal, penetration })
    } else {
        Some(Contact { a: capsule_idx, b: mesh_idx, point: tri_point, normal: -normal, penetration })
    }
}

fn box_triangle_mesh(
    box_idx: usize,
    mesh_idx: usize,
    box_transform: Transform,
    half_extents: Vec3,
    mesh_transform: Transform,
    triangles: &'static [MeshTriangle],
    flip: bool,
) -> Option<Contact> {
    let mut best: Option<Contact> = None;
    for &tri in triangles {
        let tri = transform_triangle(mesh_transform, tri);
        if let Some(contact) = box_triangle_contact(box_idx, mesh_idx, box_transform, half_extents, tri, flip) {
            let replace = best
                .as_ref()
                .map(|current| contact.penetration > current.penetration)
                .unwrap_or(true);
            if replace {
                best = Some(contact);
            }
        }
    }
    best
}

fn box_triangle_contact(
    box_idx: usize,
    mesh_idx: usize,
    box_transform: Transform,
    half_extents: Vec3,
    tri: MeshTriangle,
    flip: bool,
) -> Option<Contact> {
    let base_normal = triangle_normal(tri)?;
    let tri_center = triangle_center(tri);
    let normal = orient_axis(base_normal, box_transform.position - tri_center);
    let corners = box_corners_world(box_transform, half_extents);

    let mut deepest_dist = f32::INFINITY;
    let mut point_sum = Vec3::ZERO;
    let mut count = 0usize;
    for corner in corners {
        let dist = (corner - tri.a).dot(normal);
        if dist < 0.0 {
            let projected = corner - normal.scale(dist);
            if point_in_triangle(projected, tri.a, tri.b, tri.c, normal) {
                point_sum = point_sum + projected;
                count += 1;
                if dist < deepest_dist {
                    deepest_dist = dist;
                }
            }
        }
    }

    if count == 0 {
        return None;
    }

    let point = point_sum.scale(1.0 / count as f32);
    let penetration = -deepest_dist;
    if flip {
        Some(Contact { a: mesh_idx, b: box_idx, point, normal, penetration })
    } else {
        Some(Contact { a: box_idx, b: mesh_idx, point, normal: -normal, penetration })
    }
}

fn box_axes(transform: Transform) -> [Vec3; 3] {
    let m = transform.rotation.to_mat3().m;
    [
        Vec3::new(m[0][0], m[1][0], m[2][0]).normalized(),
        Vec3::new(m[0][1], m[1][1], m[2][1]).normalized(),
        Vec3::new(m[0][2], m[1][2], m[2][2]).normalized(),
    ]
}

fn box_corners_world(transform: Transform, half_extents: Vec3) -> [Vec3; 8] {
    [
        transform.transform_point(Vec3::new(half_extents.x, half_extents.y, half_extents.z)),
        transform.transform_point(Vec3::new(-half_extents.x, half_extents.y, half_extents.z)),
        transform.transform_point(Vec3::new(half_extents.x, -half_extents.y, half_extents.z)),
        transform.transform_point(Vec3::new(half_extents.x, half_extents.y, -half_extents.z)),
        transform.transform_point(Vec3::new(-half_extents.x, -half_extents.y, half_extents.z)),
        transform.transform_point(Vec3::new(-half_extents.x, half_extents.y, -half_extents.z)),
        transform.transform_point(Vec3::new(half_extents.x, -half_extents.y, -half_extents.z)),
        transform.transform_point(Vec3::new(-half_extents.x, -half_extents.y, -half_extents.z)),
    ]
}

fn capsule_segment(transform: Transform, half_height: f32) -> (Vec3, Vec3) {
    let axis = transform.rotation.to_mat3().mul_vec3(Vec3::new(0.0, 1.0, 0.0)).normalized();
    let half = axis.scale(half_height);
    (transform.position - half, transform.position + half)
}

fn closest_point_on_segment(a: Vec3, b: Vec3, p: Vec3) -> Vec3 {
    let ab = b - a;
    let denom = ab.length_sq();
    if denom <= 1e-12 {
        return a;
    }
    let t = ((p - a).dot(ab) / denom).clamp(0.0, 1.0);
    a + ab.scale(t)
}

fn closest_points_between_segments(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> (Vec3, Vec3) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let (s, t) = if a <= 1e-12 && e <= 1e-12 {
        (0.0, 0.0)
    } else if a <= 1e-12 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 1e-12 {
            (((-c) / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s = if denom.abs() > 1e-12 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let t = (b * s + f) / e;
            if t < 0.0 {
                s = ((-c) / a).clamp(0.0, 1.0);
                (s, 0.0)
            } else if t > 1.0 {
                s = ((b - c) / a).clamp(0.0, 1.0);
                (s, 1.0)
            } else {
                (s, t)
            }
        }
    };

    (p1 + d1.scale(s), p2 + d2.scale(t))
}

fn closest_points_segment_triangle(seg_a: Vec3, seg_b: Vec3, tri: MeshTriangle) -> (Vec3, Vec3) {
    let mut best_seg = seg_a;
    let mut best_tri = closest_point_on_triangle(seg_a, tri.a, tri.b, tri.c);
    let mut best_dist_sq = (best_seg - best_tri).length_sq();

    let end_b = closest_point_on_triangle(seg_b, tri.a, tri.b, tri.c);
    let end_b_dist_sq = (seg_b - end_b).length_sq();
    if end_b_dist_sq < best_dist_sq {
        best_seg = seg_b;
        best_tri = end_b;
        best_dist_sq = end_b_dist_sq;
    }

    if let Some(normal) = triangle_normal(tri) {
        let seg_dir = seg_b - seg_a;
        let denom = normal.dot(seg_dir);
        if denom.abs() > 1e-12 {
            let t = normal.dot(tri.a - seg_a) / denom;
            if (0.0..=1.0).contains(&t) {
                let p = seg_a + seg_dir.scale(t);
                if point_in_triangle(p, tri.a, tri.b, tri.c, normal) {
                    return (p, p);
                }
            }
        }
    }

    for (ea, eb) in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
        let (seg_point, tri_point) = closest_points_between_segments(seg_a, seg_b, ea, eb);
        let dist_sq = (seg_point - tri_point).length_sq();
        if dist_sq < best_dist_sq {
            best_seg = seg_point;
            best_tri = tri_point;
            best_dist_sq = dist_sq;
        }
    }

    (best_seg, best_tri)
}

fn closest_point_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return a + ab.scale(v);
    }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return a + ac.scale(w);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let bc = c - b;
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + bc.scale(w);
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a + ab.scale(v) + ac.scale(w)
}

fn closest_segment_point_to_aabb(a: Vec3, b: Vec3, half_extents: Vec3) -> (f32, Vec3) {
    let mut left = 0.0f32;
    let mut right = 1.0f32;
    for _ in 0..24 {
        let m1 = left + (right - left) / 3.0;
        let m2 = right - (right - left) / 3.0;
        let p1 = a + (b - a).scale(m1);
        let p2 = a + (b - a).scale(m2);
        let d1 = distance_sq_point_aabb(p1, half_extents);
        let d2 = distance_sq_point_aabb(p2, half_extents);
        if d1 <= d2 {
            right = m2;
        } else {
            left = m1;
        }
    }
    let t = 0.5 * (left + right);
    (t, a + (b - a).scale(t))
}

fn distance_sq_point_aabb(p: Vec3, half_extents: Vec3) -> f32 {
    let c = clamp_vec3(p, -half_extents, half_extents);
    (p - c).length_sq()
}

fn clamp_vec3(v: Vec3, min: Vec3, max: Vec3) -> Vec3 {
    Vec3::new(
        v.x.clamp(min.x, max.x),
        v.y.clamp(min.y, max.y),
        v.z.clamp(min.z, max.z),
    )
}

fn box_support(transform: Transform, half_extents: Vec3, dir: Vec3) -> Vec3 {
    let axes = box_axes(transform);
    let extents = [half_extents.x, half_extents.y, half_extents.z];
    let mut p = transform.position;
    for i in 0..3 {
        let sign = if axes[i].dot(dir) >= 0.0 { 1.0 } else { -1.0 };
        p = p + axes[i].scale(extents[i] * sign);
    }
    p
}

fn transform_triangle(transform: Transform, tri: MeshTriangle) -> MeshTriangle {
    MeshTriangle {
        a: transform.transform_point(tri.a),
        b: transform.transform_point(tri.b),
        c: transform.transform_point(tri.c),
    }
}

fn triangle_normal(tri: MeshTriangle) -> Option<Vec3> {
    let n = (tri.b - tri.a).cross(tri.c - tri.a);
    if n.length_sq() <= 1e-12 {
        None
    } else {
        Some(n.normalized())
    }
}

fn triangle_center(tri: MeshTriangle) -> Vec3 {
    (tri.a + tri.b + tri.c).scale(1.0 / 3.0)
}

fn point_in_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3, normal: Vec3) -> bool {
    let ab = (b - a).cross(p - a).dot(normal);
    let bc = (c - b).cross(p - b).dot(normal);
    let ca = (a - c).cross(p - c).dot(normal);
    (ab >= -1e-5 && bc >= -1e-5 && ca >= -1e-5)
        || (ab <= 1e-5 && bc <= 1e-5 && ca <= 1e-5)
}

fn orient_axis(axis: Vec3, center_delta: Vec3) -> Vec3 {
    if axis.dot(center_delta) >= 0.0 { axis } else { -axis }
}

fn compose_transform(parent: Transform, local: Transform) -> Transform {
    Transform {
        position: parent.transform_point(local.position),
        rotation: parent.rotation.mul(local.rotation).normalized(),
    }
}

fn update_best_axis(min_penetration: &mut f32, best_axis: &mut Vec3, penetration: f32, axis: Vec3) {
    if penetration < *min_penetration {
        *min_penetration = penetration;
        *best_axis = axis;
    }
}

/// 粗筛基线版本:最朴素的 O(n^2) AABB 求交 + 静态平面特判。
#[cfg(test)]
fn broadphase_pairs_bruteforce(bodies: &[RigidBody]) -> Vec<(usize, usize)> {
    let n = bodies.len();
    let mut pairs = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            if bodies[i].is_static && bodies[j].is_static {
                continue;
            }
            if matches!(bodies[i].shape, Shape::Plane { .. }) || matches!(bodies[j].shape, Shape::Plane { .. }) {
                pairs.push((i, j));
                continue;
            }
            let ea = bodies[i].shape.local_aabb_half_extents();
            let eb = bodies[j].shape.local_aabb_half_extents();
            let delta = bodies[j].transform.position - bodies[i].transform.position;
            if delta.x.abs() <= ea.x + eb.x && delta.y.abs() <= ea.y + eb.y && delta.z.abs() <= ea.z + eb.z {
                pairs.push((i, j));
            }
        }
    }
    pairs
}

/// 粗筛(broadphase):每帧按当前 AABB 重建一棵 BVH,并保留与朴素版本一致的 pair 集合。
pub fn broadphase_pairs(bodies: &[RigidBody]) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    let mut broadphase_bodies = Vec::new();
    let mut plane_indices = Vec::new();

    for (index, body) in bodies.iter().enumerate() {
        if matches!(body.shape, Shape::Plane { .. }) {
            plane_indices.push(index);
            continue;
        }

        let half_extents = body.shape.local_aabb_half_extents();
        broadphase_bodies.push(BroadphaseBody {
            index,
            min: body.transform.position - half_extents,
            max: body.transform.position + half_extents,
            center: body.transform.position,
            is_static: body.is_static,
        });
    }

    if !broadphase_bodies.is_empty() {
        let mut body_indices: Vec<usize> = (0..broadphase_bodies.len()).collect();
        let mut nodes = Vec::with_capacity(broadphase_bodies.len() * 2);
        let root = build_bvh(&mut body_indices, &broadphase_bodies, &mut nodes);
        for query in 0..broadphase_bodies.len() {
            collect_bvh_pairs(query, root, &broadphase_bodies, &nodes, &mut pairs);
        }
    }

    for &plane_index in &plane_indices {
        for (index, body) in bodies.iter().enumerate() {
            if index == plane_index || (bodies[plane_index].is_static && body.is_static) {
                continue;
            }
            let (i, j) = if plane_index < index { (plane_index, index) } else { (index, plane_index) };
            pairs.push((i, j));
        }
    }

    pairs.sort_unstable();
    pairs
}

fn build_bvh(indices: &mut [usize], bodies: &[BroadphaseBody], nodes: &mut Vec<BvhNode>) -> usize {
    debug_assert!(!indices.is_empty());
    if indices.len() == 1 {
        let body = bodies[indices[0]];
        let node_index = nodes.len();
        nodes.push(BvhNode {
            min: body.min,
            max: body.max,
            left: None,
            right: None,
            body: Some(indices[0]),
        });
        return node_index;
    }

    let mut centroid_min = bodies[indices[0]].center;
    let mut centroid_max = bodies[indices[0]].center;
    for &idx in &indices[1..] {
        centroid_min.x = centroid_min.x.min(bodies[idx].center.x);
        centroid_min.y = centroid_min.y.min(bodies[idx].center.y);
        centroid_min.z = centroid_min.z.min(bodies[idx].center.z);
        centroid_max.x = centroid_max.x.max(bodies[idx].center.x);
        centroid_max.y = centroid_max.y.max(bodies[idx].center.y);
        centroid_max.z = centroid_max.z.max(bodies[idx].center.z);
    }

    let extent = centroid_max - centroid_min;
    let axis = if extent.y > extent.x && extent.y >= extent.z {
        1
    } else if extent.z > extent.x && extent.z >= extent.y {
        2
    } else {
        0
    };
    indices.sort_by(|&a, &b| component(bodies[a].center, axis).total_cmp(&component(bodies[b].center, axis)));

    let mid = indices.len() / 2;
    let (left_indices, right_indices) = indices.split_at_mut(mid);
    let left = build_bvh(left_indices, bodies, nodes);
    let right = build_bvh(right_indices, bodies, nodes);
    let node_index = nodes.len();
    nodes.push(BvhNode {
        min: min_vec3(nodes[left].min, nodes[right].min),
        max: max_vec3(nodes[left].max, nodes[right].max),
        left: Some(left),
        right: Some(right),
        body: None,
    });
    node_index
}

fn collect_bvh_pairs(
    query_idx: usize,
    node_idx: usize,
    bodies: &[BroadphaseBody],
    nodes: &[BvhNode],
    pairs: &mut Vec<(usize, usize)>,
) {
    let query = bodies[query_idx];
    let node = &nodes[node_idx];
    if !aabb_overlap(query.min, query.max, node.min, node.max) {
        return;
    }
    if let Some(body_idx) = node.body {
        if body_idx == query_idx {
            return;
        }
        let other = bodies[body_idx];
        if query.is_static && other.is_static {
            return;
        }
        let (i, j) = if query.index < other.index {
            (query.index, other.index)
        } else {
            (other.index, query.index)
        };
        if i != j && query.index < other.index {
            pairs.push((i, j));
        }
        return;
    }
    if let Some(left) = node.left {
        collect_bvh_pairs(query_idx, left, bodies, nodes, pairs);
    }
    if let Some(right) = node.right {
        collect_bvh_pairs(query_idx, right, bodies, nodes, pairs);
    }
}

fn aabb_overlap(a_min: Vec3, a_max: Vec3, b_min: Vec3, b_max: Vec3) -> bool {
    a_min.x <= b_max.x
        && a_max.x >= b_min.x
        && a_min.y <= b_max.y
        && a_max.y >= b_min.y
        && a_min.z <= b_max.z
        && a_max.z >= b_min.z
}

fn min_vec3(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z))
}

fn max_vec3(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z))
}

fn component(v: Vec3, axis: usize) -> f32 {
    match axis {
        1 => v.y,
        2 => v.z,
        _ => v.x,
    }
}

#[cfg(test)]
pub(crate) fn broadphase_pairs_ground_truth(bodies: &[RigidBody]) -> Vec<(usize, usize)> {
    let mut pairs = broadphase_pairs_bruteforce(bodies);
    pairs.sort_unstable();
    pairs
}

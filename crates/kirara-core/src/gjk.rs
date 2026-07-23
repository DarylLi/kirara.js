use crate::math::{Transform, Vec3};
use crate::shape::Shape;

const GJK_MAX_ITERS: usize = 24;
const GJK_EPS: f32 = 1e-6;

#[derive(Clone, Copy, Debug)]
pub struct GjkPoint {
    pub support_a: Vec3,
    pub support_b: Vec3,
    pub point: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub struct GjkResult {
    pub intersect: bool,
    pub distance: f32,
    pub closest_a: Vec3,
    pub closest_b: Vec3,
}

pub fn gjk_closest_points(
    shape_a: &Shape,
    transform_a: Transform,
    shape_b: &Shape,
    transform_b: Transform,
) -> Option<GjkResult> {
    shape_a.support_point_local(Vec3::new(1.0, 0.0, 0.0))?;
    shape_b.support_point_local(Vec3::new(1.0, 0.0, 0.0))?;

    let mut direction = transform_b.position - transform_a.position;
    if direction.length_sq() < GJK_EPS {
        direction = Vec3::new(1.0, 0.0, 0.0);
    }

    let mut simplex = Simplex::default();
    simplex.push(support(shape_a, transform_a, shape_b, transform_b, direction));
    direction = -simplex.points[0].point;

    for _ in 0..GJK_MAX_ITERS {
        if direction.length_sq() < GJK_EPS {
            let point = simplex.points[0];
            return Some(GjkResult {
                intersect: true,
                distance: 0.0,
                closest_a: point.support_a,
                closest_b: point.support_b,
            });
        }

        let new_point = support(shape_a, transform_a, shape_b, transform_b, direction);
        let progress = new_point.point.dot(direction);
        let previous = simplex.closest.point.dot(direction);
        if progress - previous < GJK_EPS {
            let c = simplex.closest;
            return Some(GjkResult {
                intersect: false,
                distance: c.point.length(),
                closest_a: c.support_a,
                closest_b: c.support_b,
            });
        }

        simplex.push(new_point);
        if simplex.reduce() {
            let c = simplex.closest;
            return Some(GjkResult {
                intersect: true,
                distance: 0.0,
                closest_a: c.support_a,
                closest_b: c.support_b,
            });
        }
        direction = -simplex.closest.point;
    }

    let c = simplex.closest;
    Some(GjkResult {
        intersect: c.point.length() < 1e-4,
        distance: c.point.length(),
        closest_a: c.support_a,
        closest_b: c.support_b,
    })
}

fn support(shape_a: &Shape, transform_a: Transform, shape_b: &Shape, transform_b: Transform, direction: Vec3) -> GjkPoint {
    let support_a = support_world(shape_a, transform_a, direction);
    let support_b = support_world(shape_b, transform_b, -direction);
    GjkPoint {
        support_a,
        support_b,
        point: support_a - support_b,
    }
}

fn support_world(shape: &Shape, transform: Transform, direction_world: Vec3) -> Vec3 {
    let rot = transform.rotation.to_mat3();
    let inv_rot = rot.transposed();
    let local_dir = inv_rot.mul_vec3(direction_world);
    let local = shape
        .support_point_local(local_dir)
        .unwrap_or(Vec3::ZERO);
    transform.position + rot.mul_vec3(local)
}

#[derive(Clone, Copy, Debug)]
struct ClosestPoint {
    point: Vec3,
    support_a: Vec3,
    support_b: Vec3,
}

#[derive(Clone, Copy, Debug)]
struct Simplex {
    points: [GjkPoint; 4],
    len: usize,
    closest: ClosestPoint,
}

impl Default for Simplex {
    fn default() -> Self {
        let zero = GjkPoint {
            support_a: Vec3::ZERO,
            support_b: Vec3::ZERO,
            point: Vec3::ZERO,
        };
        Simplex {
            points: [zero; 4],
            len: 0,
            closest: ClosestPoint {
                point: Vec3::ZERO,
                support_a: Vec3::ZERO,
                support_b: Vec3::ZERO,
            },
        }
    }
}

impl Simplex {
    fn push(&mut self, point: GjkPoint) {
        self.points[self.len.min(3)] = point;
        self.len = (self.len + 1).min(4);
        self.closest = ClosestPoint {
            point: point.point,
            support_a: point.support_a,
            support_b: point.support_b,
        };
    }

    fn reduce(&mut self) -> bool {
        match self.len {
            1 => {
                self.closest = ClosestPoint {
                    point: self.points[0].point,
                    support_a: self.points[0].support_a,
                    support_b: self.points[0].support_b,
                };
                self.points[0].point.length_sq() < GJK_EPS
            }
            2 => self.reduce_segment(),
            3 => self.reduce_triangle(),
            4 => self.reduce_tetrahedron(),
            _ => false,
        }
    }

    fn reduce_segment(&mut self) -> bool {
        let a = self.points[self.len - 1];
        let b = self.points[self.len - 2];
        let ab = b.point - a.point;
        let t = (-a.point).dot(ab) / ab.length_sq().max(GJK_EPS);
        if t <= 0.0 {
            self.points[0] = a;
            self.len = 1;
            self.closest = ClosestPoint {
                point: a.point,
                support_a: a.support_a,
                support_b: a.support_b,
            };
        } else if t >= 1.0 {
            self.points[0] = b;
            self.len = 1;
            self.closest = ClosestPoint {
                point: b.point,
                support_a: b.support_a,
                support_b: b.support_b,
            };
        } else {
            self.points[0] = a;
            self.points[1] = b;
            self.len = 2;
            self.closest = mix([a, b], [1.0 - t, t]);
        }
        self.closest.point.length_sq() < GJK_EPS
    }

    fn reduce_triangle(&mut self) -> bool {
        let a = self.points[self.len - 1];
        let b = self.points[self.len - 2];
        let c = self.points[self.len - 3];
        let res = closest_on_triangle_to_origin(a, b, c);
        self.len = res.count;
        for i in 0..res.count {
            self.points[i] = res.points[i];
        }
        self.closest = res.closest;
        self.closest.point.length_sq() < GJK_EPS
    }

    fn reduce_tetrahedron(&mut self) -> bool {
        let a = self.points[3];
        let b = self.points[2];
        let c = self.points[1];
        let d = self.points[0];

        if tetrahedron_contains_origin(a.point, b.point, c.point, d.point) {
            self.closest = ClosestPoint {
                point: Vec3::ZERO,
                support_a: (a.support_a + b.support_a + c.support_a + d.support_a).scale(0.25),
                support_b: (a.support_b + b.support_b + c.support_b + d.support_b).scale(0.25),
            };
            return true;
        }

        let mut best: Option<TriangleReduction> = None;
        for (fa, fb, fc, other) in [
            (a, b, c, d.point),
            (a, c, d, b.point),
            (a, d, b, c.point),
            (b, d, c, a.point),
        ] {
            if same_side_of_plane(fa.point, fb.point, fc.point, other, Vec3::ZERO) {
                let candidate = closest_on_triangle_to_origin(fa, fb, fc);
                let replace = best
                    .as_ref()
                    .map(|current| candidate.closest.point.length_sq() < current.closest.point.length_sq())
                    .unwrap_or(true);
                if replace {
                    best = Some(candidate);
                }
            }
        }

        if let Some(best) = best {
            self.len = best.count;
            for i in 0..best.count {
                self.points[i] = best.points[i];
            }
            self.closest = best.closest;
        }
        self.closest.point.length_sq() < GJK_EPS
    }
}

#[derive(Clone, Copy, Debug)]
struct TriangleReduction {
    points: [GjkPoint; 3],
    count: usize,
    closest: ClosestPoint,
}

fn closest_on_triangle_to_origin(a: GjkPoint, b: GjkPoint, c: GjkPoint) -> TriangleReduction {
    let ab = b.point - a.point;
    let ac = c.point - a.point;
    let ap = -a.point;

    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return TriangleReduction {
            points: [a, b, c],
            count: 1,
            closest: ClosestPoint {
                point: a.point,
                support_a: a.support_a,
                support_b: a.support_b,
            },
        };
    }

    let bp = -b.point;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return TriangleReduction {
            points: [b, a, c],
            count: 1,
            closest: ClosestPoint {
                point: b.point,
                support_a: b.support_a,
                support_b: b.support_b,
            },
        };
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return TriangleReduction {
            points: [a, b, c],
            count: 2,
            closest: mix([a, b], [1.0 - v, v]),
        };
    }

    let cp = -c.point;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return TriangleReduction {
            points: [c, a, b],
            count: 1,
            closest: ClosestPoint {
                point: c.point,
                support_a: c.support_a,
                support_b: c.support_b,
            },
        };
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return TriangleReduction {
            points: [a, c, b],
            count: 2,
            closest: mix([a, c], [1.0 - w, w]),
        };
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return TriangleReduction {
            points: [b, c, a],
            count: 2,
            closest: mix([b, c], [1.0 - w, w]),
        };
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    TriangleReduction {
        points: [a, b, c],
        count: 3,
        closest: mix([a, b, c], [1.0 - v - w, v, w]),
    }
}

fn mix<const N: usize>(points: [GjkPoint; N], weights: [f32; N]) -> ClosestPoint {
    let mut point = Vec3::ZERO;
    let mut support_a = Vec3::ZERO;
    let mut support_b = Vec3::ZERO;
    for i in 0..N {
        point = point + points[i].point.scale(weights[i]);
        support_a = support_a + points[i].support_a.scale(weights[i]);
        support_b = support_b + points[i].support_b.scale(weights[i]);
    }
    ClosestPoint { point, support_a, support_b }
}

fn same_side_of_plane(a: Vec3, b: Vec3, c: Vec3, other: Vec3, p: Vec3) -> bool {
    let normal = (b - a).cross(c - a);
    let sign_other = normal.dot(other - a);
    let sign_p = normal.dot(p - a);
    sign_other * sign_p <= 0.0
}

fn tetrahedron_contains_origin(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> bool {
    let abc = same_side_of_plane(a, b, c, d, Vec3::ZERO);
    let acd = same_side_of_plane(a, c, d, b, Vec3::ZERO);
    let adb = same_side_of_plane(a, d, b, c, Vec3::ZERO);
    let bdc = same_side_of_plane(b, d, c, a, Vec3::ZERO);
    abc && acd && adb && bdc
}

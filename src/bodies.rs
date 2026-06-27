
use std::sync::Arc;
use std::sync::Mutex;
use crate::point::*;
use crate::renderer::{Triangle, sphere, cube};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Shape {
    Sphere,
    Cube,
}

pub struct SolverInput<'a> {
    pub size: &'a [f64],
    pub shape: &'a [Shape],
    pub inv_mass: &'a [f64],
    pub inv_inertia: &'a [Point],

    pub position: &'a mut [Point],
    pub prev_position: &'a [Point],

    pub rotation: &'a mut [Quaternion],
    pub prev_rotation: &'a [Quaternion],
}

impl SolverInput<'_> {
    pub fn inv_mass(&self, body: usize, normal: Point, point: Point) -> f64 {
        if self.inv_mass[body] == 0.0 { return 0.0; }

        let w = self.inv_mass[body];
        let x =
            self.rotation[body].apply_inverse((point - self.position[body]).cross(normal));

        return w + (x*x).dot(self.inv_inertia[body]);
    }

    pub fn apply_correction(&mut self, body: usize, normal: Point, lambda: f64, point: Point) {
        let delta = lambda * normal;

        self.position[body] += self.inv_mass[body] * delta;

        let mut domega =
            self.rotation[body].apply_inverse((point - self.position[body]).cross(delta));

        domega =
            0.5 * self.rotation[body].apply( domega * self.inv_inertia[body] );

        let drot =
            Quaternion::new(0.0, domega.x, domega.y, domega.z) * self.rotation[body];

        self.rotation[body].w += 0.5 * drot.w;
        self.rotation[body].x += 0.5 * drot.x;
        self.rotation[body].y += 0.5 * drot.y;
        self.rotation[body].z += 0.5 * drot.z;
        self.rotation[body] = self.rotation[body].normalize();
    }

    pub fn sphere_intersect(&self, id0: usize, id1: usize) -> Option<(Point, Point)> {
        let square_dist: f64 = (self.position[id0] - self.position[id1]).norm_square();

        if square_dist < (self.size[id0] + self.size[id1]).powi(2) {
            let normal: Point =
                (self.position[id0] - self.position[id1]) *
                (1.0 / f64::sqrt(square_dist));

            let p1: Point = self.position[id0] - normal * self.size[id0];
            let p2: Point = self.position[id1] + normal * self.size[id1];
            return Some((p1, p2))
        }

        None
    }

    // A point at a distance of at least radius is sure of not intersecting the body
    pub fn radius(&self, id: usize) -> f64 {
        match self.shape[id] {
            Shape::Cube => self.size[id] * f64::sqrt(3.0),
            Shape::Sphere => self.size[id],
        }
    }

    pub fn cube_points(&self, id: usize) -> [Point; 8] {
        let mut pts: [Point; 8] = [Point::zero(); 8];

        for i in 0..8 {
            let x0: f64 = if i & 1 != 0 { self.size[id] } else { -self.size[id] };
            let y0: f64 = if i & 2 != 0 { self.size[id] } else { -self.size[id] };
            let z0: f64 = if i & 4 != 0 { self.size[id] } else { -self.size[id] };
            pts[i] = self.rotation[id] * Point::new(x0, y0, z0) + self.position[id];
        }

        pts
    }

    pub fn cube_intersect_segment(&self, id: usize, origin: Point, inv_direction: Point) -> f64 {
        let t1: Point = (-self.size[id] - origin) * inv_direction;
        let t2: Point = (self.size[id] - origin) * inv_direction;

        let vmin: Point = t1.min(t2);
        let vmax: Point = t1.max(t2);
        let tmin: f64 = f64::max(vmin.x, f64::max(vmin.y, vmin.z));
        let tmax: f64 = f64::min(vmax.x, f64::min(vmax.y, vmax.z));

        if tmin > tmax || tmax < 0.0-1e-4 || tmin > 1.0+1e-4 { return f64::MAX; }

        if tmin <= 0.0 { return 0.0; }
        if tmax >= 1.0 { return 1.0; }
        return (tmin + tmax) * 0.5;
    }

    pub fn half_cube_intersect(&self, id0: usize, id1: usize) -> Option<(Point, Point)> {
        let pts0: [Point; 8] = self.cube_points(id0);
        let mut lpts0: [Point; 8] = pts0;

        for i in 0..8 {
            lpts0[i] = self.rotation[id1].inverse() * (pts0[i] - self.position[id1]);
        }

        let mut result: Option<(Point, Point)> = None;
        for i in 0..8 {
            for j in i+1..8 {
                if (j-1-i) & (j-i) != 0 { continue; }
                let origin: Point = lpts0[i];
                let direction: Point = lpts0[j] - origin;
                let inv_direction: Point = Point::splat(1.0) / direction;
                let t: f64 = self.cube_intersect_segment(id1, origin, inv_direction);

                if t >= 0.0 && t <= 1.0 {
                    let p0: Point = pts0[i] + (pts0[j] - pts0[i]) * t;
                    let lp0: Point = lpts0[i] + (lpts0[j] - lpts0[i]) * t;
                    let lp1: Point;

                    //assert!(lp0.x <= self.size[id1]+1e-3);
                    //assert!(lp0.y <= self.size[id1]+1e-3);
                    //assert!(lp0.z <= self.size[id1]+1e-3);
                    //assert!(lp0.x >= -self.size[id1]-1e-3);
                    //assert!(lp0.y >= -self.size[id1]-1e-3);
                    //assert!(lp0.z >= -self.size[id1]-1e-3);

                    if lp0.x.abs() > lp0.y.abs() && lp0.x.abs() > lp0.z.abs() {
                        let x1: f64 = if lp0.x >= 0.0 { self.size[id1] } else { -self.size[id1] };
                        lp1 = lp0.set_axis(0, x1);
                    } else if lp0.y.abs() > lp0.z.abs() {
                        let y1: f64 = if lp0.y >= 0.0 { self.size[id1] } else { -self.size[id1] };
                        lp1 = lp0.set_axis(1, y1);
                    } else {
                        let z1: f64 = if lp0.z >= 0.0 { self.size[id1] } else { -self.size[id1] };
                        lp1 = lp0.set_axis(2, z1);
                    }

                    let p1: Point = self.rotation[id1] * lp1 + self.position[id1];

                    if let Some((p0_, p1_)) = result {
                        if (p0 - p1).norm_square() > (p0_ - p1_).norm_square() {
                            result = Some((p0, p1));
                        }
                    } else { result = Some((p0, p1)); }
                }
            }
        }

        return result;
    }

    pub fn half_cube_intersect2(&self, id0: usize, id1: usize) -> Option<(Point, Point)> {
        let pts0 = self.cube_points(id0);
        for i in 0..8 {
            let p0 = pts0[i];
            let lp0 = self.rotation[id1].inverse() * (p0 - self.position[id1]);

            if lp0.x < -self.size[id1] || self.size[id1] < lp0.x { continue; }
            if lp0.y < -self.size[id1] || self.size[id1] < lp0.y { continue; }
            if lp0.z < -self.size[id1] || self.size[id1] < lp0.z { continue; }

            let lp1: Point;
            if lp0.x.abs() > lp0.y.abs() && lp0.x.abs() > lp0.z.abs() {
                let x1: f64 = if lp0.x >= 0.0 { self.size[id1] } else { -self.size[id1] };
                lp1 = lp0.set_axis(0, x1);
            } else if lp0.y.abs() > lp0.z.abs() {
                let y1: f64 = if lp0.y >= 0.0 { self.size[id1] } else { -self.size[id1] };
                lp1 = lp0.set_axis(1, y1);
            } else {
                let z1: f64 = if lp0.z >= 0.0 { self.size[id1] } else { -self.size[id1] };
                lp1 = lp0.set_axis(2, z1);
            }

            let p1: Point =
                self.rotation[id1] * lp1 + self.position[id1];

            return Some((p0, p1));
        }

        None
    }

    pub fn intersect(&self, id0: usize, id1: usize) -> Option<(Point, Point)> {
        let center_dist = (self.position[id0] - self.position[id1]).norm_square();
        let radius = self.radius(id0) + self.radius(id1);
        if center_dist > radius.powi(2) { return None; }

        if self.shape[id0] == Shape::Sphere && self.shape[id1] == Shape::Sphere {
            return self.sphere_intersect(id0, id1);
        }

        if self.shape[id0] == Shape::Cube && self.shape[id1] == Shape::Cube {
            if let Some((p1, p0)) = self.half_cube_intersect(id1, id0) {
                return Some((p0, p1));
            }
            return self.half_cube_intersect(id0, id1);
        }

        None
    }
}

pub trait LocalSolver {
    fn solve_local(&mut self, input: SolverInput<'_>, dt: f64);
}

pub struct Bodies {
    pub prev_position: Vec<Point>,
    pub prev_rotation: Vec<Quaternion>,
    pub rotation: Vec<Quaternion>,
    pub position: Vec<Point>,
    pub speed: Vec<Point>,
    pub omega: Vec<Point>,
    pub shape: Vec<Shape>,
    pub size: Vec<f64>,

    pub inv_mass: Vec<f64>,
    pub inv_inertia: Vec<Point>,

    pub constraints: Vec<Arc<Mutex<dyn LocalSolver>>>,
}

impl Bodies {
    pub fn new() -> Bodies {
        Self {
            size: vec![],
            shape: vec![],
            omega: vec![],
            rotation: vec![],
            position: vec![],
            constraints: vec![],
            prev_rotation: vec![],
            prev_position: vec![],
            inv_inertia: vec![],
            inv_mass: vec![],
            speed: vec![],
        }
    }

    pub fn add_point(&mut self, x0: Point, v0: Point, inv_mass: f64, inv_inertia: Point) -> usize {
        self.prev_rotation.push(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        self.rotation.push(Quaternion::new(1.0, 0.0, 0.0, 0.0));
        let len: usize = self.position.len();
        self.inv_inertia.push(inv_inertia);
        self.omega.push(Point::zero());
        self.shape.push(Shape::Cube);
        self.inv_mass.push(inv_mass);
        self.prev_position.push(x0);
        self.position.push(x0);
        self.size.push(0.25);
        self.speed.push(v0);
        len
    }

    pub fn add_constraint(&mut self, constraint: Arc<Mutex<dyn LocalSolver>>) {
        self.constraints.push(constraint);
    }

    pub fn render(&self, out: &mut Vec<Triangle>) {
        use sdl2::pixels::Color;
        for i in 0..self.position.len() {
            if self.shape[i] == Shape::Sphere {
                let radius = self.size[i];
                let center = self.position[i];
                let color = Color::RGB(255, (i%255) as u8, 0);
                out.extend(sphere(center, radius, color, 20));
            }

            if self.shape[i] == Shape::Cube {
                let size = self.size[i];
                let center = self.position[i];
                let color = Color::RGB(255, (i%255) as u8, 0);
                let mut cube = cube(center, size, color);

                for tri in cube.iter_mut() {
                    for v in tri.vertex.iter_mut() {
                        *v = self.rotation[i] * (*v - self.position[i]) + self.position[i];
                    }
                }

                out.extend(cube);
            }
        }
    }

    #[inline(never)]
    fn update_positions(&mut self, dt: f64) {
        for i in 0..self.position.len() {
            self.prev_position[i] = self.position[i];
            self.prev_rotation[i] = self.rotation[i];
        }

        for i in 0..self.position.len() {
            self.speed[i] += Point::new(0.0, 0.0, -dt * 9.81);

            self.position[i] =
                self.prev_position[i] + self.speed[i] * dt;

            let omega = self.omega[i];
            let drot =
                Quaternion::new(0.0, omega.x, omega.y, omega.z) * self.rotation[i];

            self.rotation[i].w += 0.5 * dt * drot.w;
            self.rotation[i].x += 0.5 * dt * drot.x;
            self.rotation[i].y += 0.5 * dt * drot.y;
            self.rotation[i].z += 0.5 * dt * drot.z;
            self.rotation[i] = self.rotation[i].normalize();
        }
    }

    #[inline(never)]
    fn update_velocities(&mut self, dt: f64) {
        for i in 0..self.position.len() {
            self.speed[i] = (self.position[i] - self.prev_position[i]) * (1.0 / dt);

            let drot: Quaternion = self.rotation[i] * self.prev_rotation[i].inverse();
            self.omega[i] = Point::new(drot.x, drot.y, drot.z) * (2.0 / dt);
            if drot.w < 0.0 { self.omega[i] *= -1.0; }

            let speed_norm = self.speed[i].norm();
            if speed_norm >= 10.0 { self.speed[i] *= 10.0 / speed_norm; }

            let omega_norm = self.omega[i].norm();
            if omega_norm >= 10.0 { self.omega[i] *= 10.0 / omega_norm; }
        }
    }

    #[inline(never)]
    pub fn step(&mut self, dt: f64) {
        self.update_positions(dt);

        for cnstr in self.constraints.iter_mut() {
            let inputs = SolverInput {
                prev_position: &self.prev_position,
                prev_rotation: &self.prev_rotation,
                inv_inertia: &self.inv_inertia,
                position: &mut self.position,
                rotation: &mut self.rotation,
                inv_mass: &self.inv_mass,
                shape: &self.shape,
                size: &self.size,
            };
            cnstr.lock().unwrap().solve_local(inputs, dt);
        }

        self.update_velocities(dt);
    }
}

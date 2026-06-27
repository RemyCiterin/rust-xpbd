pub mod point;
pub mod renderer;
pub mod bodies;

use std::sync::Arc;
use std::sync::Mutex;
use bodies::*;
use point::*;

pub const GROUND_Z: f64 = -4.0;


pub struct MouseConstraint {
    pub index: usize,
    pub origin: Point,
    pub direction: Point,
    pub t: f64,
}

impl Default for MouseConstraint {
    fn default() -> Self {
        Self {
            index: usize::MAX,
            origin: Point::zero(),
            direction: Point::zero(),
            t: 0.0
        }
    }
}

impl LocalSolver for MouseConstraint {
    fn solve_local(&mut self, input: SolverInput<'_>, _dt: f64) {
        if self.index < input.position.len() {
            input.position[self.index] =
                0.5 * (self.origin + self.t * self.direction) +
                0.5 * input.position[self.index];
        }
    }
}

pub struct Collider {
    aa: Vec<Point>,
    bb: Vec<Point>,
    first: Vec<usize>,
    length: Vec<usize>,
    childs: Vec<usize>,
    ids: Vec<usize>,
    radius: f64,
}

impl Collider {
    pub fn new(radius: f64) -> Self {
        Self {
            radius,
            aa: vec![],
            bb: vec![],
            ids: vec![],
            first: vec![],
            childs: vec![],
            length: vec![],
        }
    }

    pub fn add(&mut self, point: usize) {
        self.ids.push(point);
    }

    fn push(&mut self) -> usize {
        self.aa.push(Point::zero());
        self.bb.push(Point::zero());
        self.length.push(0);
        self.childs.push(0);
        self.first.push(0);
        self.aa.len() - 1
    }

    pub fn intersect_ray(&self, index: usize, points: &[Point], r: &mut Ray) -> Option<usize> {
        let d0 = r.intersect_aabb(self.aa[index], self.bb[index]);
        if d0 == f64::MAX { return None; }

        if self.childs[index] == 0 {
            let first = self.first[index];
            let length = self.length[index];

            let mut best_id: Option<usize> = None;
            for &id in self.ids[first..first+length].iter() {
                let dist =
                    r.intersect_aabb(
                        points[id]-self.radius,
                        points[id]+self.radius);

                if dist != f64::MAX {
                    best_id = Some(id);
                    r.best_t = dist;
                }
            }

            return best_id;
        }

        let x1 = self.intersect_ray(self.childs[index], points, r);
        let x2 = self.intersect_ray(self.childs[index]+1, points, r);
        x2.or(x1)
    }

    pub fn collide_pairs(&self, i0: usize, i1: usize, out: &mut Vec<(usize, usize)>) {
        if self.bb[i0].x + 1.0 < self.aa[i1].x { return; }
        if self.bb[i0].y + 1.0 < self.aa[i1].y { return; }
        if self.bb[i0].z + 1.0 < self.aa[i1].z { return; }
        if self.bb[i1].x + 1.0 < self.aa[i0].x { return; }
        if self.bb[i1].y + 1.0 < self.aa[i0].y { return; }
        if self.bb[i1].z + 1.0 < self.aa[i0].z { return; }

        if self.childs[i0] == 0 && self.childs[i1] == 0 {
            let f0: usize = self.first[i0];
            let f1: usize = self.first[i1];
            let l0: usize = self.length[i0];
            let l1: usize = self.length[i1];

            for &i in &self.ids[f0..f0+l0] {
                for &j in &self.ids[f1..f1+l1] {
                    if i != j { out.push((i, j)); }
                }
            }

            return;
        }

        if self.childs[i0] == 0 {
            self.collide_pairs(i0, self.childs[i1], out);
            self.collide_pairs(i0, self.childs[i1]+1, out);
            return;
        }

        if self.childs[i1] == 0 {
            self.collide_pairs(self.childs[i0], i1, out);
            self.collide_pairs(self.childs[i0]+1, i1, out);
            return;
        }

        self.collide_pairs(self.childs[i0], self.childs[i1], out);
        self.collide_pairs(self.childs[i0]+1, self.childs[i1], out);
        self.collide_pairs(self.childs[i0], self.childs[i1]+1, out);
        self.collide_pairs(self.childs[i0]+1, self.childs[i1]+1, out);
    }

    pub fn clear(&mut self) {
        self.length.clear();
        self.childs.clear();
        self.first.clear();
        self.aa.clear();
        self.bb.clear();
    }

    pub fn build(&mut self, input: &SolverInput<'_>, index: usize, first: usize, length: usize) {
        self.length[index] = length;
        self.first[index] = first;

        let mut aa: Point = Point::new(f64::MAX, f64::MAX, f64::MAX);
        let mut bb: Point = Point::new(f64::MIN, f64::MIN, f64::MIN);
        for &i in self.ids[first..first+length].iter() {
            aa = (input.position[i] - input.radius(i)).min(aa);
            bb = (input.position[i] + input.radius(i)).max(bb);
        }

        let range: Point = bb - aa;
        self.aa[index] = aa;
        self.bb[index] = bb;

        if length < 2 {
            self.childs[index] = 0;
            return;
        }

        self.childs[index] = self.push();
        self.push();

        let axis: usize = if range.x > range.y {
            if range.x > range.z { 0 } else { 2 }
        } else {
            if range.y > range.z { 1 } else { 2 }
        };

        let mut l0: usize = 0;
        let mut l1: usize = length;
        let threshold: f64 = (aa+bb).get_axis(axis) / 2.0;

        while l0 < l1 {
            if input.position[self.ids[first+l0]].get_axis(axis) < threshold {
                l0 += 1;
            } else {
                self.ids.swap(first+l0, first+l1-1);
                l1 -= 1;
            }
        }

        let size: usize = l0;

        if size == 0 || size == length {
            self.childs[index] = 0;
            return;
        }

        self.build(&input, self.childs[index], first, size);
        self.build(&input, self.childs[index]+1, first+size, length-size);
    }
}

impl LocalSolver for Collider {
    fn solve_local(&mut self, mut input: SolverInput<'_>, _dt: f64) {
        self.clear();
        self.push();
        self.build(&input, 0, 0, self.ids.len());

        let mut collisions: Vec<(usize, usize)> = vec![];
        self.collide_pairs(0, 0, &mut collisions);

        for (id0, id1) in collisions {
                if let Some((x0, x1)) = input.intersect(id0, id1) {
                    let c: f64 = (x0 - x1).norm();
                    if c < 1e-8 { continue; }

                    let normal: Point = (x0 - x1) * (1.0 / c);
                    let w0: f64 = input.inv_mass(id0, normal, x0);
                    let w1: f64 = input.inv_mass(id1, normal, x1);

                    let normal_lambda: f64 = -c / (w0 + w1);
                    input.apply_correction(id0, normal, normal_lambda, x0);
                    input.apply_correction(id1, normal, -normal_lambda, x1);

                    let r0: Point = input.rotation[id0].inverse() * (x0 - input.position[id0]);
                    let r1: Point = input.rotation[id1].inverse() * (x1 - input.position[id1]);
                    let prev_x0: Point = input.prev_rotation[id0] * r0 + input.prev_position[id0];
                    let prev_x1: Point = input.prev_rotation[id1] * r1 + input.prev_position[id1];

                    let mut delta_x: Point = (x0 - prev_x0) - (x1 - prev_x1);
                    delta_x -= normal * delta_x.dot(normal);

                    let c = delta_x.norm();
                    if c < 1e-8 { continue; }

                    let normal = delta_x * (1.0 / c);
                    let w0: f64 = input.inv_mass(id0, normal, x0);
                    let w1: f64 = input.inv_mass(id1, normal, x1);

                    let tangent_lambda: f64 = -c / (w0 + w1);
                    if tangent_lambda.abs() > normal_lambda.abs() * 0.5 { continue; }
                    //println!("{} {}", normal_lambda / _dt.powi(2), tangent_lambda / _dt.powi(2));
                    input.apply_correction(id0, normal, tangent_lambda, x0);
                    input.apply_correction(id1, normal, -tangent_lambda, x1);
                }
        }
    }
}

pub struct GroundSolver {
    radius: f64
}

impl LocalSolver for GroundSolver {
    fn solve_local(&mut self, mut input: SolverInput<'_>, _dt: f64) {
        for i in 0..input.position.len() {
            let mut correction: Point = Point::zero();
            let mut p: Point = Point::zero();
            let mut collision: bool = false;


            if input.shape[i] == Shape::Sphere && input.position[i].z < GROUND_Z - self.radius {
                let target = input.position[i].set_axis(2, GROUND_Z - self.radius);
                p = input.position[i] - Point::new(0.0, 0.0, self.radius);
                correction = target - input.position[i];
                collision = true;
            }

            if input.shape[i] == Shape::Cube {
                let mut best_collision: f64 = GROUND_Z;
                for j in 0..8 {
                    let x = if j & 1 == 0 { self.radius } else { -self.radius };
                    let y = if j & 2 == 0 { self.radius } else { -self.radius };
                    let z = if j & 4 == 0 { self.radius } else { -self.radius };

                    let pt =
                        input.rotation[i] * Point::new(x, y, z) +
                        input.position[i];

                    if pt.z < best_collision {
                        correction = Point::new(0.0, 0.0, GROUND_Z - pt.z);
                        best_collision = pt.z;
                        collision = true;
                        p = pt;
                    }
                }
            }

            if collision {
                let r: Point = input.rotation[i].inverse() * (p - input.position[i]);
                let prev_p: Point = input.prev_rotation[i] * r + input.prev_position[i];

                let mut delta_p: Point = p - prev_p;
                delta_p.z = 0.0;

                correction -= delta_p;
                let normal = correction.normalize();
                let lambda = correction.norm() / input.inv_mass(i, normal, p);
                input.apply_correction(i, normal, lambda, p);
            }
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum LineKind {
    Exact,
    Min,
    Max,
}

#[derive(Clone)]
pub struct LineSolver {
    kinds: Vec<LineKind>,
    lambdas: Vec<f64>,
    lengths: Vec<f64>,
    idx0: Vec<usize>,
    idx1: Vec<usize>,
    w0: Vec<f64>,
    w1: Vec<f64>,
    alpha: f64,
}

impl LineSolver {
    pub fn new(alpha: f64) -> Self {
        Self {
            lambdas: vec![],
            lengths: vec![],
            kinds: vec![],
            alpha: alpha,
            idx0: vec![],
            idx1: vec![],
            w0: vec![],
            w1: vec![],
        }
    }

    pub fn add(&mut self, id0: usize, id1: usize, w0: f64, w1: f64, length: f64, kind: LineKind) {
        self.lengths.push(length);
        self.lambdas.push(0.0);
        self.kinds.push(kind);
        self.idx0.push(id0);
        self.idx1.push(id1);
        self.w0.push(w0);
        self.w1.push(w1);
    }
}

impl LocalSolver for LineSolver {
    fn solve_local(&mut self, input: SolverInput<'_>, dt: f64) {
        for i in 0..self.lengths.len() {
            let x0: Point = input.position[self.idx0[i]];
            let x1: Point = input.position[self.idx1[i]];
            let lambda = self.lambdas[i];
            let l0: f64 = self.lengths[i];
            let l: f64 = (x0 - x1).norm();
            let w0: f64 = self.w0[i];
            let w1: f64 = self.w1[i];
            let c: f64 = l - l0;

            if self.kinds[i] == LineKind::Min && l >= l0 { continue; }
            if self.kinds[i] == LineKind::Max && l <= l0 { continue; }
            if c.abs() < 1e-4 { continue; }

            let alpha = self.alpha / (dt*dt);
            let d_lambda = (-c - lambda * alpha) / (w0 + w1 + alpha);
            self.lambdas[i] += d_lambda;

            let c0: Point = (x0 - x1) * (1.0 / l);
            let c1: Point = (x1 - x0) * (1.0 / l);
            let d_lambda =
                - c / (alpha + w0 * c0.norm_square() + w1 * c1.norm_square());

            input.position[self.idx0[i]] += w0 * d_lambda * c0;
            input.position[self.idx1[i]] += w1 * d_lambda * c1;
        }
    }
}


use renderer::*;

pub fn gen_free_fall(solver: &mut Bodies) {
    for stage in 3..128 {
        let offset: f64 = if stage % 2 == 0 { 0.5 } else { -0.5 };
        solver.add_point(
            Point::new(5.0, 0.0, 2.0 * stage as f64 + offset),
            Point::zero(),
            1.0,
            Point::splat(1.0)
        );

        solver.add_point(
            Point::new(2.0, 0.0, 2.0 * stage as f64 - offset),
            Point::zero(),
            1.0,
            Point::splat(1.0)
        );
    }
}

pub fn gen_ladder(lines: &mut LineSolver, solver: &mut Bodies) {
    let mut prev_lhs: Option<usize> = None;
    let mut prev_rhs: Option<usize> = None;
    for stage in 3..128 {
        let offset: f64 = if stage % 2 == 0 { 0.5 } else { -0.5 };
        let lhs =
            solver.add_point(
                Point::new(-5.0, 0.0, 2.0 * stage as f64 + offset),
                Point::zero(),
                1.0,
                Point::splat(1.0)
            );

        let rhs =
            solver.add_point(
                Point::new(-2.0, 0.0, 2.0 * stage as f64 - offset),
                Point::zero(),
                1.0,
                Point::splat(1.0)
            );

        let length = (solver.position[rhs] - solver.position[lhs]).norm();
        lines.add(lhs, rhs, 1.0, 1.0, length, LineKind::Exact);

        if let Some(p) = prev_lhs {
            let length = (solver.position[p] - solver.position[lhs]).norm();
            lines.add(lhs, p, 1.0, 1.0, length, LineKind::Exact);
        }

        if let Some(p) = prev_rhs {
            let length = (solver.position[p] - solver.position[rhs]).norm();
            lines.add(rhs, p, 1.0, 1.0, length, LineKind::Exact);
        }

        prev_lhs = Some(lhs);
        prev_rhs = Some(rhs);
    }
}

pub fn gen_cube(
    center: Point,
    radius: f64,
    solver: &mut Bodies,
    lines: &mut LineSolver) {

    let mut points: Vec<usize> = vec![];

    for i in -1..=1 {
        for j in -1..=1 {
            for k in -1..=1 {
                if i == 0 || j == 0 || k == 0 { continue; }
                let p: Point = center +
                    Point::new(i as f64, j as f64, k as f64) * radius;
                let idx =
                    solver.add_point(
                        p, Point::zero(),
                        1.0, Point::splat(1.0));
                points.push(idx);
            }
        }
    }

    for i in 0..8 {
        for j in i+1..8 {
            let p0: usize = points[i];
            let p1: usize = points[j];
            let dist: f64 = (solver.position[p0] - solver.position[p1]).norm();
            lines.add(p0, p1,
                1.0, 1.0, dist, LineKind::Exact
            );
        }
    }
}

fn event_loop() {
    let mut solver = Bodies::new();

    let lines = LineSolver::new(0.0000001);
    let ground =
        Arc::new(Mutex::new(GroundSolver{radius: 0.5}));
    let collisions =
        Arc::new(Mutex::new(Collider::new(0.5)));
    let mouse_constraint =
        Arc::new(Mutex::new(MouseConstraint::default()));

    //gen_ladder(&mut lines, &mut solver);
    //gen_free_fall(&mut solver);
    //gen_cube(Point::new(0.0, 0.0, 15.0), 1.0, &mut solver, &mut lines);

    //for i in 0..10 {
    //    for j in 0..10 {
    //        let idx0 =
    //            solver.add_point(
    //                Point::new(i as f64 - 4.0, j as f64, 6.0),
    //                Point::new(0.01, 0.0, 0.0),
    //                1.0, Point::splat(1.0));
    //        let idx1 =
    //            solver.add_point(
    //                Point::new(i as f64 - 4.0, j as f64, 8.0),
    //                Point::new(0.0, 0.0, 0.0),
    //                1.0, Point::splat(1.0));
    //        collisions.lock().unwrap().add(idx0);
    //        collisions.lock().unwrap().add(idx1);
    //
    //        //lines.add(idx0, idx1, 1.0, 1.0, 2.0, LineKind::Min);
    //        //lines.add(idx0, idx1, 1.0, 1.0, 2.4, LineKind::Max);
    //    }
    //}

    solver.add_constraint(ground);
    let lines_clone = lines.clone();
    solver.add_constraint(Arc::new(Mutex::new(lines)));
    solver.add_constraint(mouse_constraint.clone());
    solver.add_constraint(collisions.clone());

    let mut renderer =
        Renderer::new(3.141/4.0, 320.0/240.0, 2.0, 1.0);

    let transform = |p: Point|
        Point::new(-p.x/10.0, p.z/10.0, p.y/10.0 + 1.5);

    let transform_inverse_o = |p: Point|
        Point::new(-p.x*10.0, (p.z-1.5)*10.0, p.y*10.0);

    let transform_inverse_d = |p: Point|
        Point::new(-p.x*10.0, p.z*10.0, p.y*10.0);

    let mut triangles: Vec<Triangle> = vec![];
    let instant = std::time::Instant::now();

    let mut mouse: MouseState = MouseState { down: false, x: 0.0, y: 0.0 };

    for frames in 0.. {
        for _ in 0..10 {
            solver.step(1.0 / (60.0 * 10.0));
        }

        if frames % 100 == 0 {
            let id = solver.add_point(
                Point::new(0.0, 0.0, 2.0),
                Point::zero(), 1.0, Point::splat(1.0));
            solver.omega[id] = Point::new(0.0, 0.01, 0.0);
            //solver.size[id] = 1.0;
            collisions.lock().unwrap().add(id);
        }

        let fps: f64 = 1000.0 * frames as f64 / instant.elapsed().as_millis() as f64;
        print!("\rstep: {} tri count: {} fps: {:.4}    ", frames, triangles.len(), fps);

        let prev_down: bool = mouse.down;
        mouse = renderer.events();

        let mut ray = renderer.to_ray(mouse.x, mouse.y);
        ray = Ray::new(
            transform_inverse_o(ray.origin),
            transform_inverse_d(ray.direction));

        mouse_constraint.lock().unwrap().origin = ray.origin;
        mouse_constraint.lock().unwrap().direction = ray.direction;

        if !mouse.down {
            mouse_constraint.lock().unwrap().index = usize::MAX;
        }

        if !prev_down && mouse.down {
            for (i, &p) in solver.position.iter().enumerate() {
                let t =
                    ray.intersect_aabb(p - 0.5, p + 0.5);

                if t < f64::MAX {
                    mouse_constraint.lock().unwrap().t = t;
                    mouse_constraint.lock().unwrap().index = i;
                    ray.best_t = t;
                }
            }
        }

        renderer.clear();
        for (i, j) in lines_clone.idx0.iter().zip(lines_clone.idx1.iter()) {
            let p0 = solver.position[*i];
            let p1 = solver.position[*j];

            renderer.line(
                transform(p0),
                transform(p1),
                sdl2::pixels::Color::RGB(0, 255, 0));
        }

        //std::thread::sleep(std::time::Duration::from_millis(1000/120));
        triangles.clear();


        solver.render(&mut triangles);
        for tri in triangles.iter_mut() {
            *tri = Triangle::new(
                transform(tri.vertex[0]),
                transform(tri.vertex[1]),
                transform(tri.vertex[2]),
                tri.color);
        }

        let mut ray = renderer.to_ray(mouse.x, mouse.y);
        ray = Ray::new(
            transform_inverse_o(ray.origin),
            transform_inverse_d(ray.direction));
        let pointed: Point = ray.origin + ray.direction;
        let color = sdl2::pixels::Color::RGB(5, 255, 5);
        triangles.extend(
            sphere(transform(pointed), 0.05, color, 40));

        if frames % 1 == 0 { renderer.draw(&triangles); }
        use std::io::*;
        std::io::stdout().flush().unwrap();
    }
}

fn main() {
    let guard =
        pprof::ProfilerGuardBuilder::default().frequency(1000)
        .blocklist(&["libc", "libgcc", "pthread", "vdso"])
        .build().unwrap();

    event_loop();

    if let Ok(report) = guard.report().build() {
        let file = std::fs::File::create("flamegraph.svg").unwrap();
        report.flamegraph(file).unwrap();
    };
}

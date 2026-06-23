
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Point {pub x: f64, pub y: f64, pub z: f64}

impl Point {
    pub fn new(x: f64, y: f64, z: f64) -> Point {
        Point{x, y, z}
    }

    pub fn zero() -> Point {
        Point::new(0.0, 0.0, 0.0)
    }

    pub fn splat(x: f64) -> Point {
        Point::new(x, x, x)
    }

    pub fn dot(self, other: Point) -> f64 {
        let tmp: Point = self * other;
        tmp.x + tmp.y + tmp.z
    }

    pub fn norm(self) -> f64 {
        f64::sqrt(self.dot(self))
    }

    pub fn get_axis(self, x: usize) -> f64 {
        match x {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => panic!(),
        }
    }

    pub fn set_axis(self, x: usize, v: f64) -> Point {
        match x {
            0 => Point::new(v, self.y, self.z),
            1 => Point::new(self.x, v, self.z),
            2 => Point::new(self.x, self.y, v),
            _ => panic!(),
        }
    }

    pub fn norm_square(self) -> f64 {
        self.dot(self)
    }

    pub fn min(self, other: Point) -> Point {
        Point{
            x: f64::min(self.x, other.x),
            y: f64::min(self.y, other.y),
            z: f64::min(self.z, other.z),
        }
    }

    pub fn max(self, other: Point) -> Point {
        Point{
            x: f64::max(self.x, other.x),
            y: f64::max(self.y, other.y),
            z: f64::max(self.z, other.z),
        }
    }

    pub fn cross(&self, other: Point) -> Point {
        Point::new(
            self.y * other.z - other.y * self.z,
            self.z * other.x - other.z * self.x,
            self.x * other.y - other.x * self.y
        )
    }

    pub fn det(&self, b: Point, c: Point) -> f64 {
        self.cross(b).dot(c)
    }

    pub fn comatrix(mat: [Point; 3]) -> [Point; 3] {
        [mat[1].cross(mat[2]), mat[2].cross(mat[0]), mat[0].cross(mat[1])]
    }

    pub fn transpose(mat: [Point; 3]) -> [Point; 3] {
        return [
            Point::new(mat[0].x, mat[1].x, mat[2].x),
            Point::new(mat[0].y, mat[1].y, mat[2].y),
            Point::new(mat[0].z, mat[1].z, mat[2].z),
        ];
    }

    pub fn inverse(mat: [Point; 3]) -> Option<[Point; 3]> {
        let det = mat[0].det(mat[1], mat[2]);
        if det.abs() < 1e-5 { return None; }

        let inv_det = Point::splat(1.0 / det);
        let com = Self::transpose(Self::comatrix(mat));
        return Some([ com[0] * inv_det, com[1] * inv_det, com[2] * inv_det ]);

    }

    pub fn normalize(self) -> Self {
        self / Self::splat(self.norm())
    }
}

impl std::ops::Neg for Point {
    type Output = Self;
    fn neg(self) -> Point {
        Point{x: -self.x, y: -self.y, z: -self.z}
    }
}

impl std::ops::Add for Point {
    type Output = Point;
    fn add(self, other: Point) -> Point {
        Point{x: self.x + other.x, y: self.y + other.y, z: self.z + other.z}
    }
}

impl std::ops::Add<f64> for Point {
    type Output = Point;
    fn add(self, other: f64) -> Point {
        Point{x: self.x + other, y: self.y + other, z: self.z + other}
    }
}

impl std::ops::Add<Point> for f64 {
    type Output = Point;
    fn add(self, other: Point) -> Point {
        Point{x: self + other.x, y: self + other.y, z: self + other.z}
    }
}

impl std::ops::AddAssign<f64> for Point {
    fn add_assign(&mut self, other: f64) {
        *self = *self + other;
    }
}

impl std::ops::AddAssign for Point {
    fn add_assign(&mut self, other: Point) {
        *self = *self + other;
    }
}

impl std::ops::Sub for Point {
    type Output = Point;
    fn sub(self, other: Point) -> Point {
        Point{x: self.x - other.x, y: self.y - other.y, z: self.z - other.z}
    }
}

impl std::ops::Sub<f64> for Point {
    type Output = Point;
    fn sub(self, other: f64) -> Point {
        Point{x: self.x - other, y: self.y - other, z: self.z - other}
    }
}

impl std::ops::Sub<Point> for f64 {
    type Output = Point;
    fn sub(self, other: Point) -> Point {
        Point{x: self - other.x, y: self - other.y, z: self - other.z}
    }
}

impl std::ops::SubAssign for Point {
    fn sub_assign(&mut self, other: Point) {
        *self = *self - other;
    }
}

impl std::ops::SubAssign<f64> for Point {
    fn sub_assign(&mut self, other: f64) {
        *self = *self - other;
    }
}

impl std::ops::Mul for Point {
    type Output = Point;
    fn mul(self, other: Point) -> Point {
        Point{x: self.x * other.x, y: self.y * other.y, z: self.z * other.z}
    }
}

impl std::ops::Mul<f64> for Point {
    type Output = Point;
    fn mul(self, other: f64) -> Point {
        Point{x: self.x * other, y: self.y * other, z: self.z * other}
    }
}

impl std::ops::Mul<Point> for f64 {
    type Output = Point;
    fn mul(self, other: Point) -> Point {
        Point{x: self * other.x, y: self * other.y, z: self * other.z}
    }
}

impl std::ops::MulAssign for Point {
    fn mul_assign(&mut self, other: Point) {
        *self = *self * other;
    }
}

impl std::ops::MulAssign<f64> for Point {
    fn mul_assign(&mut self, other: f64) {
        *self = *self * other;
    }
}

impl std::ops::Div for Point {
    type Output = Point;
    fn div(self, other: Point) -> Point {
        Point{x: self.x / other.x, y: self.y / other.y, z: self.z / other.z}
    }
}

impl std::ops::DivAssign for Point {
    fn div_assign(&mut self, other: Point) {
        *self = *self / other;
    }
}


pub struct Ray {
    pub origin: Point,
    pub direction: Point,
    pub inv_direction: Point,
    pub best_t: f64,
}

impl Ray {
    pub fn new(origin: Point, direction: Point) -> Ray {
        Self {
            origin,
            direction,
            inv_direction: Point::splat(1.0) / direction,
            best_t: f64::MAX
        }
    }

    pub fn intersect_aabb(&self, aa: Point, bb: Point) -> f64 {
        let t1 = (aa - self.origin) * self.inv_direction;
        let t2 = (bb - self.origin) * self.inv_direction;

        let vmin = t1.min(t2);
        let vmax = t1.max(t2);
        let tmin = f64::max(vmin.x, f64::max(vmin.y, vmin.z));
        let tmax = f64::min(vmax.x, f64::min(vmax.y, vmax.z));

        if tmax >= tmin && tmax >= 0.0 {
            if tmin >= self.best_t { return f64::MAX; }
            return tmin.max(0.0);
        }

        f64::MAX
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn zero() -> Quaternion {
        Quaternion::new(0.0, 0.0, 0.0, 0.0)
    }

    pub fn splat(x: f64) -> Quaternion {
        Quaternion::new(x, x, x, x)
    }

    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Quaternion {
        Self {x, y, z, w}
    }

    pub fn new_axis(mut axis: Point, angle: f64) -> Quaternion {
        if axis.norm() > 1e-6 { axis = axis.normalize(); }

        let sin_angle: f64 = f64::sin(angle / 2.0);
        let cos_angle: f64 = f64::cos(angle / 2.0);

        Quaternion::new(
            cos_angle,
            axis.x * sin_angle,
            axis.y * sin_angle,
            axis.z * sin_angle,
        )
    }

    pub fn inverse(&self) -> Quaternion {
        Quaternion::new(self.w, -self.x, -self.y, -self.z)
    }

    pub fn matrix3(&self) -> [[f64; 3]; 3] {
        let mut m: [[f64; 3]; 3] = [[0f64; 3]; 3];

        m[0][0] = 1.0 - 2.0 * self.y.powi(2) - 2.0 * self.z.powi(2);
        m[0][1] = 2.0 * self.x * self.y - 2.0 * self.w * self.z;
        m[0][2] = 2.0 * self.x * self.z + 2.0 * self.w * self.y;

        m[1][0] = 2.0 * self.x * self.y + 2.0 * self.w * self.z;
        m[1][1] = 1.0 - (2.0 * self.x.powi(2)) - (2.0 * self.z.powi(2));
        m[1][2] = 2.0 * self.y * self.z - 2.0 * self.w * self.x;

        m[2][0] = 2.0 * self.x * self.z - 2.0 * self.w * self.y;
        m[2][1] = 2.0 * self.y * self.z + 2.0 * self.w * self.x;
        m[2][2] = 1.0 - (2.0 * self.x.powi(2)) - (2.0 * self.y.powi(2));

        m
    }

    pub fn matrix4(&self) -> [[f64; 4]; 4] {
        let mut m: [[f64; 4]; 4] = [[0f64; 4]; 4];

        m[0][0] = 1.0 - 2.0 * self.y.powi(2) - 2.0 * self.z.powi(2);
        m[0][1] = 2.0 * self.x * self.y - 2.0 * self.w * self.z;
        m[0][2] = 2.0 * self.x * self.z + 2.0 * self.w * self.y;

        m[1][0] = 2.0 * self.x * self.y + 2.0 * self.w * self.z;
        m[1][1] = 1.0 - (2.0 * self.x.powi(2)) - (2.0 * self.z.powi(2));
        m[1][2] = 2.0 * self.y * self.z - 2.0 * self.w * self.x;

        m[2][0] = 2.0 * self.x * self.z - 2.0 * self.w * self.y;
        m[2][1] = 2.0 * self.y * self.z + 2.0 * self.w * self.x;
        m[2][2] = 1.0 - (2.0 * self.x.powi(2)) - (2.0 * self.y.powi(2));
        m[3][3] = 1.0;

        m
    }

    pub fn square_norm(&self) -> f64 {
        self.w.powi(2) + self.x.powi(2) + self.y.powi(2) + self.z.powi(2)
    }

    pub fn norm(&self) -> f64 {
        self.square_norm().sqrt()
    }

    pub fn normalize(&self) -> Quaternion {
        let len = self.norm();
        Quaternion {
            w: self.w / len,
            x: self.x / len,
            y: self.y / len,
            z: self.z / len,
        }
    }

    pub fn apply(&self, p: Point) -> Point {
        let m: [[f64; 3]; 3] = self.matrix3();
        Point::new(
            m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z,
            m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z,
            m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z,
        )
    }

    pub fn apply_inverse(&self, p: Point) -> Point {
        self.inverse().apply(p)
    }
}

impl std::ops::Mul for Quaternion {
    type Output = Self;

    fn mul(self, other: Quaternion) -> Quaternion {
        Quaternion::new(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y + self.y * other.w + self.z * other.x - self.x * other.z,
            self.w * other.z + self.z * other.w + self.x * other.y - self.y * other.x
        )
    }
}

impl std::ops::Mul<Point> for Quaternion {
    type Output = Point;

    fn mul(self, other: Point) -> Point {
        self.apply(other)
    }
}

use sdl2::video::Window;
use sdl2::render::Canvas;
use sdl2::pixels::Color;
use sdl2::rect;

use crate::point::*;

//use rayon::prelude::*;

pub const WIDTH: u32 = 640*2;
pub const HEIGHT: u32 = 480*2;

pub struct Triangle {
    /// Set of vertices used to represent the triangle
    pub vertex: [Point; 3],

    /// Uniform color of the triangle
    pub color: Color,

    /// Only meaningfull for triangles after projection
    pub inv_det: f64,
}

impl Triangle {
    pub fn new(p1: Point, p2: Point, p3: Point, color: Color) -> Triangle {
        Self { vertex: [p1, p2, p3], color, inv_det: 0.0 }
    }
}

pub fn square(a: Point, b: Point, c: Point, d: Point, color: Color) -> [Triangle; 2] {
    return [
        Triangle::new(a, b, c, color),
        Triangle::new(b, c, d, color),
    ];
}

pub fn sphere(center: Point, radius: f64, color: Color, points: usize) -> Vec<Triangle> {
    let mut ret = vec![];

    let point = |i: usize, j: usize| {
        let theta = 2.0 * std::f64::consts::PI * (i as f64) / points as f64;
        let alpha = 2.0 * std::f64::consts::PI * (j as f64) / points as f64;
        Point::new(
            f64::sin(theta),
            f64::cos(theta) * f64::sin(alpha),
            f64::cos(theta) * f64::cos(alpha),
        )
    };

    for i in 0..points {
        for j in 0..points {
            let p1 = center + Point::splat(radius) * point(i, j);
            let p2 = center + Point::splat(radius) * point(i+1, j);
            let p3 = center + Point::splat(radius) * point(i, j+1);
            let p4 = center + Point::splat(radius) * point(i+1, j+1);
            ret.push(Triangle::new(p1, p2, p3, color));
            ret.push(Triangle::new(p2, p4, p3, color));
        }
    }

    return ret;
}

pub fn cube(center: Point, size: f64, color: Color) -> Vec<Triangle> {
    let mut out = Vec::with_capacity(12);

    for axis in 0..3 {
        for dir in 0..2 {
            let plane: f64 = if dir == 0 { size } else { -size };
            let a: Point = center + match axis {
                0 => Point::new(plane, size, size),
                1 => Point::new(size, plane, size),
                _ => Point::new(size, size, plane)};

            let b: Point = center + match axis {
                0 => Point::new(plane, -size, size),
                1 => Point::new(-size, plane, size),
                _ => Point::new(-size, size, plane)};

            let c: Point = center + match axis {
                0 => Point::new(plane, -size, -size),
                1 => Point::new(-size, plane, -size),
                _ => Point::new(-size, -size, plane)};

            let d: Point = center + match axis {
                0 => Point::new(plane, size, -size),
                1 => Point::new(size, plane, -size),
                _ => Point::new(size, -size, plane)};

            out.push(Triangle::new(a, b, c, color));
            out.push(Triangle::new(a, d, c, color));
        }
    }

    out
}

#[derive(Copy, Clone, PartialEq)]
pub struct MouseState {
    pub down: bool,
    pub x: f64,
    pub y: f64,
}

pub struct Renderer {
    image: Vec<u8>,
    z_buffer: Vec<f64>,
    canvas: Canvas<Window>,
    ctx: sdl2::Sdl,

    mouse: MouseState,

    angle: f64,
    ratio: f64,
    far: f64,
    near: f64,
}

pub fn into_x_coord(x: f64) -> i32 {
    (((x+1.0) * WIDTH as f64) / 2.0) as i32
}

pub fn from_x_coord(i: i32) -> f64 {
    2.0 * i as f64 / WIDTH as f64 - 1.0
}

pub fn into_y_coord(y: f64) -> i32 {
    (((y+1.0) * HEIGHT as f64) / 2.0) as i32
}

pub fn from_y_coord(j: i32) -> f64 {
    2.0 * j as f64 / HEIGHT as f64 - 1.0
}

impl Renderer {
    pub fn new(angle: f64, ratio: f64, far: f64, near: f64) -> Self {
        sdl2::hint::set("SDL_NO_SIGNAL_HANDLERS", "1");
        let ctx = sdl2::init().unwrap();
        let video = ctx.video().unwrap();
        let window = video.window("graph", WIDTH, HEIGHT)
            .build().unwrap();
        let canvas = window.into_canvas().build().unwrap();

        let z_buffer =
            (0..WIDTH*HEIGHT)
            .map(|_| f64::MAX)
            .collect();

        let image =
            (0..4*WIDTH*HEIGHT)
            .map(|_| 0)
            .collect();

        Self {
            mouse: MouseState{down: false, x: 0.0, y: 0.0},
            ctx,
            angle,
            ratio,
            far,
            near,
            canvas,
            z_buffer,
            image,
        }
    }

    pub fn events(&mut self) -> MouseState {
        let mut event_pump = self.ctx.event_pump().unwrap();

        use sdl2::event::*;
        use sdl2::mouse::MouseButton;
        for event in event_pump.poll_iter() {
            match event {
                Event::MouseMotion { mousestate, x, y , ..} => {
                    self.mouse.down = mousestate.is_mouse_button_pressed(MouseButton::Left);
                    self.mouse.x = from_x_coord(x);
                    self.mouse.y = from_y_coord(y);
                }
                Event::MouseButtonDown { mouse_btn,  x, y, .. } => {
                    if mouse_btn == MouseButton::Left { self.mouse.down = true; }
                    self.mouse.x = from_x_coord(x);
                    self.mouse.y = from_y_coord(y);
                }
                Event::MouseButtonUp { mouse_btn,  x, y, .. } => {
                    if mouse_btn == MouseButton::Left { self.mouse.down = true; }
                    self.mouse.x = from_x_coord(x);
                    self.mouse.y = from_y_coord(y);
                }
                _ => {}
            }
        }

        self.mouse
    }

    pub fn to_ray(&self, x: f64, y: f64) -> Ray {
        let origin =
            Point::new(0.0, 0.0, 0.0);
        let direction = Point::new(
            2.0 * -x * f64::tan(self.angle / 2.0) * self.ratio,
            2.0 * -y * f64::tan(self.angle / 2.0),
            2.0 * self.near);

        Ray::new(origin, direction)
    }

    pub fn projection_matrix(&self) -> [[f64; 4]; 4] {
        let h: f64 = 1.0 / f64::tan(self.angle / 2.0);
        let w: f64 = h / self.ratio;

        let mut mat: [[f64; 4]; 4] = [[0.0; 4]; 4];

        mat[0][0] = w;
        mat[1][1] = h;
        mat[2][2] = -self.far / (self.far - self.near);
        mat[2][3] = self.far * self.near / (self.far - self.near);
        mat[3][2] = -1.0;

        mat
    }

    pub fn clear(&mut self) {
        self.canvas.set_draw_color(Color::RGB(0, 0, 0));
        self.canvas.clear();
        for i in 0..WIDTH*HEIGHT {
            self.z_buffer[i as usize] = f64::MAX;
            self.image[3 * i as usize + 0] = 0;
            self.image[3 * i as usize + 1] = 0;
            self.image[3 * i as usize + 2] = 0;
        }
    }

    pub fn line(&mut self, mut start: Point, mut end: Point, color: Color) {
        let m = self.projection_matrix();

        let prj_point = |p: Point| {
            let mut r = Point::new(
                p.x * m[0][0] + p.y * m[0][1] + p.z * m[0][2] + m[0][3],
                p.x * m[1][0] + p.y * m[1][1] + p.z * m[1][2] + m[1][3],
                p.x * m[2][0] + p.y * m[2][1] + p.z * m[2][2] + m[2][3],
            );
            let w = p.x * m[3][0] + p.y * m[3][1] + p.z * m[3][2] + m[3][3];

            if w != 1.0 { r /= Point::splat(w); }
            return r;
        };

        start = prj_point(start);
        end = prj_point(end);

        self.canvas.set_draw_color(color);
        let src_x = into_x_coord(start.x);
        let src_y = into_y_coord(start.y);
        let dst_x = into_x_coord(end.x);
        let dst_y = into_y_coord(end.y);

        self.canvas
            .draw_line(
                rect::Point::new(src_x, src_y),
                rect::Point::new(dst_x, dst_y))
            .unwrap();
    }

    pub fn draw(&mut self, triangles: &[Triangle]) {
        let m = self.projection_matrix();

        let prj_point = |p: Point| {
            let mut r = Point::new(
                p.x * m[0][0] + p.y * m[0][1] + p.z * m[0][2] + m[0][3],
                p.x * m[1][0] + p.y * m[1][1] + p.z * m[1][2] + m[1][3],
                p.x * m[2][0] + p.y * m[2][1] + p.z * m[2][2] + m[2][3],
            );
            let w = p.x * m[3][0] + p.y * m[3][1] + p.z * m[3][2] + m[3][3];

            if w != 1.0 { r /= Point::splat(w); }
            if w > 0.0 { r *= -1.0; }
            return r;
        };

        let mut projected: Vec<Triangle> = vec![];

        for tri in triangles.iter() {
            let v0:Point = prj_point(tri.vertex[0]);
            let v1:Point = prj_point(tri.vertex[1]);
            let v2:Point = prj_point(tri.vertex[2]);

            let a: f64 = v1.x - v0.x;
            let b: f64 = v2.x - v0.x;
            let c: f64 = v1.y - v0.y;
            let d: f64 = v2.y - v0.y;
            let det: f64 = a*d - b*c;

            let prj = Triangle{
                vertex: [v0, v1, v2],
                inv_det: 1.0 / det,
                color: tri.color,
            };

            projected.push(prj);
        }

        for tri in projected {
            let aa: Point = tri.vertex[0].min(tri.vertex[1]).min(tri.vertex[2]);
            let bb: Point = tri.vertex[0].max(tri.vertex[1]).max(tri.vertex[2]);
            let a: f64 = tri.vertex[1].x - tri.vertex[0].x;
            let b: f64 = tri.vertex[2].x - tri.vertex[0].x;
            let c: f64 = tri.vertex[1].y - tri.vertex[0].y;
            let d: f64 = tri.vertex[2].y - tri.vertex[0].y;
            let mut color: Color = tri.color;

            if bb.x < -1.0 { continue; }
            if bb.y < -1.0 { continue; }
            if aa.x > 1.0 { continue; }
            if aa.y > 1.0 { continue; }

            let mut normal =
                (tri.vertex[1] - tri.vertex[0])
                .cross(tri.vertex[2] - tri.vertex[0])
                .normalize();

            let center: Point =
                (tri.vertex[0] + tri.vertex[1] + tri.vertex[2]) / Point::splat(3.0);
            if (center + Point::new(0.0, 0.0, self.far)).dot(normal) < -1e-5 {
                normal *= Point::splat(-1.0); }

            let light: f64 =
                0.5 + 0.5 * Point::new(0.0, 1.0, 0.0).normalize().dot(normal);
            color.r = (color.r as f64 * light) as u8;
            color.g = (color.g as f64 * light) as u8;
            color.b = (color.b as f64 * light) as u8;

            for i in into_x_coord(aa.x)..into_x_coord(bb.x)+1 {
                for j in into_y_coord(aa.y)..into_y_coord(bb.y)+1 {
                    if i < 0 || i >= WIDTH as i32 { continue; }
                    if j < 0 || j >= HEIGHT as i32 { continue; }

                    let x: f64 = from_x_coord(i) - tri.vertex[0].x;
                    let y: f64 = from_y_coord(j) - tri.vertex[0].y;

                    let v: f64 = tri.inv_det * (d * x - b * y);
                    let w: f64 = tri.inv_det * (y * a - c * x);
                    let u: f64 = 1.0 - v - w;

                    if u < 0.0 || u > 1.0 { continue; }
                    if v < 0.0 || v > 1.0 { continue; }
                    if w < 0.0 || w > 1.0 { continue; }

                    let z: f64 =
                        u * tri.vertex[0].z +
                        v * tri.vertex[1].z +
                        w * tri.vertex[2].z;

                    if z < 0.0 { continue; }
                    if z >= self.z_buffer[(j*WIDTH as i32+i) as usize] { continue; }
                    self.z_buffer[(j*WIDTH as i32+i) as usize] = z;

                    self.image[3 * (j*WIDTH as i32+i) as usize + 0] = color.r;
                    self.image[3 * (j*WIDTH as i32+i) as usize + 1] = color.g;
                    self.image[3 * (j*WIDTH as i32+i) as usize + 2] = color.b;
                }
            }
        }

        let texture_creator = self.canvas.texture_creator();

        use sdl2::surface::Surface;
        use sdl2::render::Texture;
        use sdl2::pixels::PixelFormatEnum::RGB24;

        let surface =
            Surface::from_data(
                &mut self.image,
                WIDTH,
                HEIGHT,
                WIDTH*3,
                RGB24)
            .unwrap();

        let texture =
            Texture::from_surface(&surface, &texture_creator).unwrap();

        self.canvas.copy(&texture, None, None).unwrap();
        self.canvas.present();

    }
}

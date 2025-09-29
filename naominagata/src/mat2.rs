use oort_api::prelude::*;

/// A 2x2 matrix of the form:
/// ```
/// [a b]
/// [c d]
/// ```
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Mat2 {
    pub xx: f64,
    pub xy: f64,
    pub yx: f64,
    pub yy: f64,
}

impl Mat2 {
    pub fn new(a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { xx: a, xy: b, yx: c, yy: d }
    }

    pub fn identity() -> Mat2 {
        Mat2 {
            xx: 1.0,
            xy: 0.0,
            yx: 0.0,
            yy: 1.0,
        }
    }

    pub fn zero() -> Mat2 {
        Mat2 {
            xx: 0.0,
            xy: 0.0,
            yx: 0.0,
            yy: 0.0,
        }
    }

    pub fn add(&self, other: &Mat2) -> Mat2 {
        Mat2 {
            xx: self.xx + other.xx,
            xy: self.xy + other.xy,
            yx: self.yx + other.yx,
            yy: self.yy + other.yy,
        }
    }

    pub fn mul(&self, other: &Mat2) -> Mat2 {
        Mat2 {
            xx: self.xx * other.xx + self.xy * other.yx,
            xy: self.xx * other.xy + self.xy * other.yy,
            yx: self.yx * other.xx + self.yy * other.yx,
            yy: self.yx * other.xy + self.yy * other.yy,
        }
    }

    /// Scalar multiplication (immutable method).
    pub fn scale(&self, k: f64) -> Mat2 {
        Mat2 {
            xx: self.xx * k,
            xy: self.xy * k,
            yx: self.yx * k,
            yy: self.yy * k,
        }
    }

    pub fn det(&self) -> f64 {
        self.xx * self.yy - self.xy * self.yx
    }

    pub fn inv(&self) -> Mat2 {
        let det = self.det();
        Mat2 {
            xx: self.yy / det,
            xy: -self.xy / det,
            yx: -self.yx / det,
            yy: self.xx / det,
        }
    }

    pub fn mul_vec(&self, vec: &Vec2) -> Vec2 {
        Vec2 {
            x: self.xx * vec.x + self.xy * vec.y,
            y: self.yx * vec.x + self.yy * vec.y,
        }
    }

    pub fn solve(&self, vec: &Vec2) -> Vec2 {
        let inv = self.inv();
        inv.mul_vec(vec)
    }

    pub fn transpose(&self) -> Mat2 {
        Mat2 {
            xx: self.xx,
            xy: self.yx,
            yx: self.xy,
            yy: self.yy,
        }
    }

    pub fn trace(&self) -> f64 {
        self.xx + self.yy
    }

    pub fn frobenius_norm(&self) -> f64 {
        (self.xx * self.xx + self.xy * self.xy + self.yx * self.yx + self.yy * self.yy).sqrt()
    }

    pub fn is_symmetric(&self) -> bool {
        self.xy == self.yx
    }
}

// Operator overloading -------------------------------------------------------
use core::ops::{Add, AddAssign, Mul};

impl Add for Mat2 {
    type Output = Mat2;

    fn add(self, rhs: Mat2) -> Mat2 {
        Mat2 {
            xx: self.xx + rhs.xx,
            xy: self.xy + rhs.xy,
            yx: self.yx + rhs.yx,
            yy: self.yy + rhs.yy,
        }
    }
}

impl AddAssign for Mat2 {
    fn add_assign(&mut self, rhs: Mat2) {
        *self = *self + rhs;
    }
}

// Matrix * Matrix → Matrix
impl Mul<Mat2> for Mat2 {
    type Output = Mat2;

    fn mul(self, rhs: Mat2) -> Mat2 {
        Mat2 {
            xx: self.xx * rhs.xx + self.xy * rhs.yx,
            xy: self.xx * rhs.xy + self.xy * rhs.yy,
            yx: self.yx * rhs.xx + self.yy * rhs.yx,
            yy: self.yx * rhs.xy + self.yy * rhs.yy,
        }
    }
}

// Matrix * scalar → Matrix
impl Mul<f64> for Mat2 {
    type Output = Mat2;

    fn mul(self, k: f64) -> Mat2 {
        Mat2 {
            xx: self.xx * k,
            xy: self.xy * k,
            yx: self.yx * k,
            yy: self.yy * k,
        }
    }
}

// scalar * Matrix → Matrix
impl Mul<Mat2> for f64 {
    type Output = Mat2;

    fn mul(self, m: Mat2) -> Mat2 {
        m * self
    }
}

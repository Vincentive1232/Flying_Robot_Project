#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]

use core::ops::{Add, Sub, Mul, Neg};
use libm::atan2f;

// use std::{result, task::ready};
use crate::estimator_bindings::{self, quaternion_s, vec3_s};
extern crate libm;


/*****************************************************************
 * Quaternion structure definition
 ****************************************************************/
 #[derive(Clone, Copy, Debug)]
 pub struct Quaternion {
    pub qw: f32,
    pub qx: f32,
    pub qy: f32,
    pub qz: f32,
}

impl Quaternion {
    // construction function
    pub fn new(qw: f32, qx: f32, qy: f32, qz: f32) -> Self {
        Quaternion{qw, qx, qy, qz}
    }

    // conjugate function
    pub fn conjugate(&self) -> Self {
        Quaternion {
            qw: self.qw,
            qx: -self.qx,
            qy: -self.qy,
            qz: -self.qz,
        }
    }

    pub fn norm(&self) -> f32 {
        libm::sqrtf(self.qw * self.qw + self.qx * self.qx + self.qy * self.qy + self.qz * self.qz)
    }
}

// define the conversion from estimator_bindings to my_math types and vice versa
impl From<quaternion_s> for Quaternion {
    fn from(c_quaternion: quaternion_s) -> Self {
        unsafe {
            Quaternion::new(
                c_quaternion.__bindgen_anon_1.__bindgen_anon_2.w, 
                c_quaternion.__bindgen_anon_1.__bindgen_anon_2.x, 
                c_quaternion.__bindgen_anon_1.__bindgen_anon_2.y, 
                c_quaternion.__bindgen_anon_1.__bindgen_anon_2.z
            )
        }
    }
}

impl From<Quaternion> for quaternion_s {
    fn from(r_quaternion: Quaternion) -> Self {
        // let q1 = stabilizer_types::quaternion_s__bindgen_ty_1__bindgen_ty_1 { q0: r_quaternion.qw, q1: r_quaternion.qx, q2: r_quaternion.qy, q3: r_quaternion.qz };
        let q2 = estimator_bindings::quaternion_s__bindgen_ty_1__bindgen_ty_2 { w: r_quaternion.qw, x: r_quaternion.qx, y: r_quaternion.qy, z: r_quaternion.qz };

        let __bindgen_anon_1_data = estimator_bindings::quaternion_s__bindgen_ty_1 { __bindgen_anon_2: q2 };

        quaternion_s { 
            __bindgen_anon_1: __bindgen_anon_1_data
        }
    }
}

// Reload Add
impl Add for Quaternion {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Quaternion {
            qw: self.qw + other.qw,
            qx: self.qx + other.qx,
            qy: self.qy + other.qy,
            qz: self.qz + other.qz,
        }
    }
}

// Reload Multiply
impl Mul for Quaternion {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        Self {
            qw: self.qw*other.qw - self.qx*other.qx - self.qy*other.qy - self.qz*other.qz,
            qx: self.qx*other.qw + self.qw*other.qx - self.qz*other.qy + self.qy*other.qz,
            qy: self.qy*other.qw + self.qz*other.qx + self.qw*other.qy - self.qx*other.qz,
            qz: self.qz*other.qw - self.qy*other.qx + self.qx*other.qy + self.qw*other.qz,
        }
    }
}

//Reload Neg
impl Neg for Quaternion {
    type Output = Self;

    fn neg(self) -> Self {
        Quaternion {
            qw: -self.qw,
            qx: -self.qx,
            qy: -self.qy,
            qz: -self.qz,
        }
    }
}

//Reload *： f32*Vector3
impl Mul<Quaternion> for f32 {
    type Output = Quaternion;

    fn mul(self, q: Quaternion) -> Quaternion {
        Quaternion {
            qw: self * q.qw,
            qx: self * q.qx,
            qy: self * q.qy,
            qz: self * q.qz,
        }
    }
}



/*****************************************************************
 * Vector3 structure definition
 ****************************************************************/
 #[derive(Clone, Copy, Debug)]
 pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Vector3{x, y, z}
    }

    // calculate cross product
    pub fn cross(self, other: Vector3) -> Vector3 {
        Vector3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn dot_product(self, vec: Vector3) -> f32 {
        self.x*vec.x + self.y*vec.y + self.z*vec.z
    }

    pub fn norm(self) -> f32 {
        libm::sqrtf(self.x*self.x + self.y*self.y + self.z*self.z)
    }
}

// define the conversion from stabilizer_types to my_math types and vice versa
impl From<vec3_s> for Vector3 {
    fn from(c_vec3: vec3_s) -> Self {
        Vector3::new(c_vec3.x, c_vec3.y, c_vec3.z)
    }
}

impl From<Vector3> for vec3_s {
    fn from(r_vec3: Vector3) -> Self {
        vec3_s { timestamp: 0, x: r_vec3.x, y: r_vec3.y, z: r_vec3.z }
    }
}


//Reload Add
impl Add for Vector3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Vector3 {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

//Reload Sub
impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Vector3 {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

//Reload Neg
impl Neg for Vector3 {
    type Output = Self;

    fn neg(self) -> Self {
        Vector3 {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

//Reload *: Vector3*f32
impl Mul<f32> for Vector3 {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Vector3 {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

//Reload *： f32*Vector3
impl Mul<Vector3> for f32 {
    type Output = Vector3;

    fn mul(self, vec: Vector3) -> Vector3 {
        Vector3 {
            x: self * vec.x,
            y: self * vec.y,
            z: self * vec.z,
        }
    }
}


/*****************************************************************
 * Vector4 structure definition
 ****************************************************************/
 #[derive(Clone, Copy, Debug)]
 pub struct Vector4 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub k: f32,
}

impl Vector4 {
    pub fn new(x: f32, y: f32, z: f32, k: f32) -> Self {
        Vector4{x, y, z, k}
    }
}

//Reload Add
impl Add for Vector4 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Vector4 {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
            k: self.k + other.k,
        }
    }
}

//Reload Sub
impl Sub for Vector4 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Vector4 {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
            k: self.k - other.k,
        }
    }
}

//Reload Neg
impl Neg for Vector4 {
    type Output = Self;

    fn neg(self) -> Self {
        Vector4 {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            k: -self.k,
        }
    }
}

//Reload Mul with a f32
impl Mul<f32> for Vector4 {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Vector4 {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
            k: self.k * scalar,
        }
    }
}

//Reload *： f32*Vector3
impl Mul<Vector4> for f32 {
    type Output = Vector4;

    fn mul(self, vec: Vector4) -> Vector4 {
        Vector4 {
            x: self * vec.x,
            y: self * vec.y,
            z: self * vec.z,
            k: self * vec.k,
        }
    }
}



/*****************************************************************
 * Matrix(3x3) structure definition
 ****************************************************************/
 #[derive(Clone, Copy, Debug)]
 pub struct Matrix3 {
    data: [[f32; 3]; 3],
}

impl Matrix3 {
    pub fn new(data: [[f32; 3]; 3]) -> Self {
        Matrix3 { data }
    }

    pub fn transpose(self) -> Self {
        Matrix3::new([
            [self.data[0][0], self.data[1][0], self.data[2][0]],
            [self.data[0][1], self.data[1][1], self.data[2][1]],
            [self.data[0][2], self.data[1][2], self.data[2][2]],
        ])
    }
}

// Reload Multiplication with a 3-dim vector
impl Mul<Vector3> for Matrix3 {
    type Output = Vector3;

    fn mul(self, vec: Vector3) -> Vector3 {
        Vector3 {
            x: self.data[0][0] * vec.x + self.data[0][1] * vec.y + self.data[0][2] * vec.z,
            y: self.data[1][0] * vec.x + self.data[1][1] * vec.y + self.data[1][2] * vec.z,
            z: self.data[2][0] * vec.x + self.data[2][1] * vec.y + self.data[2][2] * vec.z,
        }
    }
}

// Reload Multiplication with a 3-dim vector
impl Sub<Matrix3> for Matrix3 {
    type Output = Matrix3;

    fn sub(self, mat: Matrix3) -> Matrix3 {
        Matrix3::new([
            [self.data[0][0]-mat.data[0][0], self.data[0][1]-mat.data[0][1], self.data[0][2]-mat.data[0][2]],
            [self.data[1][0]-mat.data[1][0], self.data[1][1]-mat.data[1][1], self.data[1][2]-mat.data[1][2]],
            [self.data[2][0]-mat.data[2][0], self.data[2][1]-mat.data[2][1], self.data[2][2]-mat.data[2][2]],
        ])
    }
}

// Reload Multiplication with a matrix
impl Mul<Matrix3> for Matrix3 {
    type Output = Matrix3;

    fn mul(self, rhs: Matrix3) -> Matrix3 {
        let mut result = [[0.0; 3]; 3];

        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    result[i][j] += self.data[i][k] * rhs.data[k][j];
                }
            }
        }

        Matrix3::new(result)
    }
}



/*****************************************************************
 * Matrix(4x4) structure definition
 ****************************************************************/
 #[derive(Clone, Copy, Debug)]
 pub struct Matrix4 {
    data: [[f32; 4]; 4],
}

impl Matrix4 {
    pub fn new(data: [[f32; 4]; 4]) -> Self {
        Matrix4 { data }
    }
}

// Reload Multiplication with a 4-dim vector
impl Mul<Vector4> for Matrix4 {
    type Output = Vector4;

    fn mul(self, vec: Vector4) -> Vector4 {
        Vector4 {
            x: self.data[0][0] * vec.x + self.data[0][1] * vec.y + self.data[0][2] * vec.z + self.data[0][3] * vec.k,
            y: self.data[1][0] * vec.x + self.data[1][1] * vec.y + self.data[1][2] * vec.z + self.data[1][3] * vec.k,
            z: self.data[2][0] * vec.x + self.data[2][1] * vec.y + self.data[2][2] * vec.z + self.data[2][3] * vec.k,
            k: self.data[3][0] * vec.x + self.data[3][1] * vec.y + self.data[3][2] * vec.z + self.data[3][3] * vec.k,
        }
    }
}




/*****************************************************************
 * Matrix structure definition using generics
 * NEED TO GUARANTEE THAT THE DIMENSIONS MATCH!!!!!!
 ****************************************************************/
 #[derive(Clone, Copy, Debug)]
pub struct Mat<const Rows: usize, const Columns: usize> {
    pub m: [[f32; Columns]; Rows],
}

/// Innitialize as an all zero matrix
impl<const Rows: usize, const Columns: usize> Mat<Rows, Columns> {
    pub fn new() -> Self {
        Mat {
            m: [[0.0; Columns]; Rows],
        }
    }
}

// Reload Add
impl<const Rows: usize, const Columns: usize>
    Add for Mat<Rows, Columns>
{
    type Output = Self;

    fn add(self, other:Self) -> Self {
        let mut result = [[0.0; Columns]; Rows];
        for i in 0..Rows {
            for j in 0..Columns {
                result[i][j] = self.m[i][j] + other.m[i][j];
            }
        }

        Mat { m: result }
    }
}

// Reload Sub
impl<const Rows: usize, const Columns: usize>
Sub for Mat<Rows, Columns>
{
    type Output = Self;

    fn sub(self, other:Self) -> Self {
        let mut result = [[0.0; Columns]; Rows];
        for i in 0..Rows {
            for j in 0..Columns {
                result[i][j] = self.m[i][j] - other.m[i][j];
            }
        }

        Mat { m: result }
    }
}

// Reload Mul (Matrix * Matrix)
impl<const R1: usize, const C1: usize, const C2: usize>
    Mul<Mat<C1, C2>> for Mat<R1, C1>
{
    type Output = Mat<R1, C2>;

    fn mul(self, other: Mat<C1, C2>) -> Self::Output {
        let mut result = [[0.0; C2]; R1];

        for i in 0..R1 {
            for j in 0..C2 {
                for k in 0..C1 {
                    result[i][j] += self.m[i][k] * other.m[k][j];
                }
            }
        }

        Mat { m: result }
    }
}

// Reload Mul (Matrix * Matrix)
impl<const Rows: usize, const Columns: usize> Mul<f32> for Mat<Rows, Columns> {
    type Output = Mat<Rows, Columns>;
    fn mul(self, scalar: f32) -> Self::Output {
        let mut result = [[0.0; Columns]; Rows];
        for i in 0..Rows {
            for j in 0..Columns {
                result[i][j] = self.m[i][j] * scalar;
            }
        }
        Mat { m: result }
    }
}

// Transpose
impl<const Rows: usize, const Columns: usize> Mat<Rows, Columns> {
    pub fn transpose(&self) -> Mat<Columns, Rows> {
        let mut result = [[0.0; Rows]; Columns];
        for i in 0..Rows {
            for j in 0..Columns {
                result[j][i] = self.m[i][j];
            }
        }
        Mat { m: result }
    }
}

// Subtracts identity matrix from this matrix (only valid for square matrices).
impl<const Rows: usize, const Columns: usize> Mat<Rows, Columns> {
    pub fn sub_identity(&self) -> Self {
        let mut result = self.clone();
        for i in 0..Rows.min(Columns) {
            result.m[i][i] -= 1.0;
        }
        result
    }
}





/*****************************************************************
 * Other utils
 ****************************************************************/
pub fn sin_approx(x: f32) -> f32 {
    let mut term = x;
    let mut sum = x;
    let x_squared = x * x;
    let mut sign = -1.0;

    for n  in (3..20).step_by(2) {
        term *= x_squared / ((n - 1) as f32 * n as f32);
        sum += sign * term;
        sign *= -1.0;
    }
    
    sum
}

pub fn cos_approx(x: f32) -> f32 {
    let mut term = 1.0;
    let mut sum = 1.0;
    let x_squared = x * x;
    let mut sign = -1.0;

    for n  in (2..20).step_by(2) {
        term *= x_squared / ((n - 1) as f32 * n as f32);
        sum += sign * term;
        sign *= -1.0;
    }
    
    sum
}

pub fn my_pow(x: f32) -> f32 {
    x * x
}

pub fn vector_augument(vec: Vector3) -> Quaternion {
    Quaternion::new(0.0, vec.x, vec.y, vec.z)
}


pub fn circle_dot(q: Quaternion, vec: Vector3) -> Vector3 {
    let q_temp = Quaternion::new(0.0, vec.x, vec.y, vec.z);
    let q_conj = q.conjugate();
    let q_result = q*q_temp*q_conj;

    Vector3::new(q_result.qx, q_result.qy, q_result.qz)
}


pub fn skew(omega: Vector3) -> Matrix3 {
    let rot_matrix = Matrix3::new([
        [0.0, -omega.z, omega.y],
        [omega.z, 0.0, -omega.x],
        [-omega.y, omega.x, 0.0],
    ]);

    rot_matrix
}


pub fn vee(rot: Matrix3) -> Vector3 {
    Vector3::new(-rot.data[1][2],rot.data[0][2],-rot.data[0][1])
}


pub fn clamp(val: f32, min: f32, max: f32) -> f32 {
    let mut val_r = val;
    if val > max {
        val_r = max;
    }
    else if val < min {
        val_r = min;
    }
    val_r
}


pub fn v3_clamp(vec3: Vector3, min: f32, max: f32) -> Vector3 {
    let clamp_x = clamp(vec3.x, min, max);
    let clamp_y = clamp(vec3.y, min, max);
    let clamp_z = clamp(vec3.z, min, max);

    Vector3::new(clamp_x, clamp_y, clamp_z)
}


pub fn quaternion_to_matrix(q: Quaternion) -> Matrix3 {
    let mut rot_matrix: Matrix3 = Matrix3::new([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]);

    rot_matrix.data[0][0] = q.qw*q.qw + q.qx*q.qx - q.qy*q.qy - q.qz*q.qz;
    rot_matrix.data[0][1] = 2.0 * (q.qx*q.qy - q.qw*q.qz);
    rot_matrix.data[0][2] = 2.0 * (q.qw*q.qy + q.qx*q.qz);

    rot_matrix.data[1][0] = 2.0 * (q.qx*q.qy + q.qw*q.qz);
    rot_matrix.data[1][1] = q.qw*q.qw - q.qx*q.qx + q.qy*q.qy - q.qz*q.qz;
    rot_matrix.data[1][2] = 2.0 * (q.qy*q.qz - q.qw*q.qx);

    rot_matrix.data[2][0] = 2.0 * (q.qx*q.qz - q.qw*q.qy);
    rot_matrix.data[2][1] = 2.0 * (q.qw*q.qx + q.qy*q.qz);
    rot_matrix.data[2][2] = q.qw*q.qw - q.qx*q.qx - q.qy*q.qy + q.qz*q.qz;

    rot_matrix
}


pub fn quaternion_to_matrix_mat(q: Quaternion) -> Mat<3, 3> {

    let mut rot_matrix: Mat<3, 3_> = Mat::new();

    rot_matrix.m[0][0] = q.qw*q.qw + q.qx*q.qx - q.qy*q.qy - q.qz*q.qz;
    rot_matrix.m[0][1] = 2.0 * (q.qx*q.qy - q.qw*q.qz);
    rot_matrix.m[0][2] = 2.0 * (q.qw*q.qy + q.qx*q.qz);

    rot_matrix.m[1][0] = 2.0 * (q.qx*q.qy + q.qw*q.qz);
    rot_matrix.m[1][1] = q.qw*q.qw - q.qx*q.qx + q.qy*q.qy - q.qz*q.qz;
    rot_matrix.m[1][2] = 2.0 * (q.qy*q.qz - q.qw*q.qx);

    rot_matrix.m[2][0] = 2.0 * (q.qx*q.qz - q.qw*q.qy);
    rot_matrix.m[2][1] = 2.0 * (q.qw*q.qx + q.qy*q.qz);
    rot_matrix.m[2][2] = q.qw*q.qw - q.qx*q.qx - q.qy*q.qy + q.qz*q.qz;

    rot_matrix
}


/// Turn Quaternion into Roll, Pitch, Yaw (units: rad)
pub fn quaternion_to_rpy(q: Quaternion) -> Vector3 {
    let qw = q.qw;
    let qx = q.qx;
    let qy = q.qy;
    let qz = q.qz;

    // 计算 Roll (X 轴旋转)
    let sinr_cosp = 2.0 * (qw * qx + qy * qz);
    let cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    let roll = atan2f(sinr_cosp, cosr_cosp);

    // 计算 Pitch (Y 轴旋转)
    let sinp = 2.0 * (qw * qy - qz * qx);
    let pitch = if sinp >= 1.0 {
        3.141592653589793 / 2.0 // π/2
    } else if sinp <= -1.0 {
        -3.141592653589793 / 2.0 // -π/2
    } else {
        libm::asinf(sinp)
    };

    // 计算 Yaw (Z 轴旋转)
    let siny_cosp = 2.0 * (qw * qz + qx * qy);
    let cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    let yaw = atan2f(siny_cosp, cosy_cosp);

    Vector3::new(roll, pitch, yaw)
}



pub fn rotation_update(q: Quaternion, vec: Vector3, dt: f32) -> Quaternion {
    let omega_global = circle_dot(q, vec);
    // let omega_global = vec;

    let half_dt_omega = Quaternion::new(0.0, dt * omega_global.x, dt * omega_global.y, dt * omega_global.z);

    let mut q_next = q * half_dt_omega;

    q_next = (1.0/q_next.norm()) * q_next;

    q_next
}



/*****************************************************************
 * Utils about thrust/force/rpm transformation
 ****************************************************************/
pub fn cmd_2_thrust(cmd: f32) -> f32 {
    let param_1:f32 = 0.109e-6;
    let param_2:f32 = -2.106e-4;
    let param_3:f32 = 0.154;

    return (param_1*(cmd*cmd) + param_2*cmd + param_3)/1000.0;
}


pub fn thrust_2_torque(f_i: f32) -> f32 {
    let param_1:f32 = 0.005964552;
    let param_2:f32 = 1.563383e-5;

    return param_1*f_i + param_2;
}


pub fn combine_thrust_torque(cmd_1: f32, cmd_2: f32, cmd_3: f32, cmd_4: f32) -> Vector4 {

    let a_2: f32 = 0.04/1.414;

    let thrust_1 = cmd_2_thrust(cmd_1);
    let thrust_2 = cmd_2_thrust(cmd_2);
    let thrust_3 = cmd_2_thrust(cmd_3);
    let thrust_4 = cmd_2_thrust(cmd_4);

    let torque_1 = thrust_2_torque(thrust_1);
    let torque_2 = thrust_2_torque(thrust_2);
    let torque_3 = thrust_2_torque(thrust_3);
    let torque_4 = thrust_2_torque(thrust_4);

    
    let force_f = thrust_1 + thrust_2 + thrust_3 + thrust_4;
    let tau_x = (-torque_1 - torque_2 + torque_3 + torque_4) * a_2;
    let tau_y = (-torque_1 + torque_2 + torque_3 - torque_4) * a_2;
    let tau_z = (-torque_1 + torque_2 - torque_3 + torque_4) * a_2;
    
    /*
    let force_f = thrust_1 + thrust_2 + thrust_3 + thrust_4;
    let tau_x = (-thrust_1 - thrust_2 + thrust_3 + thrust_4) * a_2;
    let tau_y = (-thrust_1 + thrust_2 + thrust_3 - thrust_4) * a_2;
    let tau_z = (-thrust_1 + thrust_2 - thrust_3 + thrust_4) * a_2;
    */

    Vector4::new(force_f, tau_x, tau_y, tau_z)
}


pub fn rpm_to_force(rpm: f32) -> f32 {
    let p = [2.50077341e-08, -5.40422570e-05, -1.51910248e-01];

    let force_in_grams = p[0] * rpm*rpm + p[1] * rpm + p[2];

    let mut force_in_newton = force_in_grams * 9.81 / 1000.0;

    force_in_newton = force_in_newton.max(0.0);
    force_in_newton.min(0.012*9.81)
    // force_in_newton
}

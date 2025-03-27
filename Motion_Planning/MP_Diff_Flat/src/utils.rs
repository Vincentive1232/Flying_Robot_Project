use std::ops::{Add, Sub, Mul, Neg};

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
        (self.qw * self.qw + self.qx * self.qx + self.qy * self.qy + self.qz * self.qz).sqrt()
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
        (self.x*self.x + self.y*self.y + self.z*self.z).sqrt()
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
 * Other utils
 ****************************************************************/
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


pub fn quaternion_to_matrix2(q: Quaternion) -> Matrix3 {
    let q_norm = (1.0/q.norm()) * q;

    let mut rot_matrix: Matrix3 = Matrix3::new([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ]);

    rot_matrix.data[0][0] = 1.0 - 2.0*(q_norm.qy*q_norm.qy + q_norm.qz*q_norm.qz);
    rot_matrix.data[0][1] = 2.0 * (q_norm.qx*q_norm.qy - q_norm.qw*q_norm.qz);
    rot_matrix.data[0][2] = 2.0 * (q_norm.qw*q_norm.qy + q_norm.qx*q_norm.qz);

    rot_matrix.data[1][0] = 2.0 * (q_norm.qx*q_norm.qy + q_norm.qw*q_norm.qz);
    rot_matrix.data[1][1] = 1.0 - 2.0*(q_norm.qx*q_norm.qx + q_norm.qz*q_norm.qz);
    rot_matrix.data[1][2] = 2.0 * (q_norm.qy*q_norm.qz - q_norm.qw*q_norm.qx);

    rot_matrix.data[2][0] = 2.0 * (q_norm.qx*q_norm.qz - q_norm.qw*q_norm.qy);
    rot_matrix.data[2][1] = 2.0 * (q_norm.qw*q_norm.qx + q_norm.qy*q_norm.qz);
    rot_matrix.data[2][2] = 1.0 - 2.0*(q_norm.qx*q_norm.qx + q_norm.qy*q_norm.qy);

    rot_matrix
}


pub fn rotation_matrix_to_rpy(rot: Matrix3) -> Vector3 {
    let roll = rot.data[2][1].atan2(rot.data[2][2]); // atan2(R32, R33)
    let pitch = (-rot.data[2][0]).asin(); // asin(-R31)
    let yaw = rot.data[1][0].atan2(rot.data[0][0]); // atan2(R21, R11)

    Vector3 {x: roll, y: pitch, z: yaw}
}


pub fn rotation_matrix_to_quaternion(rot: Matrix3) -> Quaternion {
    let trace = rot.data[0][0] + rot.data[1][1] + rot.data[2][2]; // R11 + R22 + R33
    let mut quat = Quaternion::new(1.0, 0.0, 0.0, 0.0);

    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        quat.qw = 0.25 * s;
        quat.qx = (rot.data[2][1] - rot.data[1][2]) / s;
        quat.qy = (rot.data[0][2] - rot.data[2][0]) / s;
        quat.qz = (rot.data[1][0] - rot.data[0][1]) / s;
    } 
    else if (rot.data[0][0] > rot.data[1][1]) && (rot.data[0][0] > rot.data[2][2]) {
        let s = (1.0 + rot.data[0][0] - rot.data[1][1] - rot.data[2][2]).sqrt() * 2.0;
        quat.qw = (rot.data[2][1] - rot.data[1][2]) / s;
        quat.qx = 0.25 * s;
        quat.qy = (rot.data[0][1] + rot.data[1][0]) / s;
        quat.qz = (rot.data[0][2] + rot.data[2][0]) / s;
    } 
    else if rot.data[1][1] > rot.data[2][2] {
        let s = (1.0 + rot.data[1][1] - rot.data[0][0] - rot.data[2][2]).sqrt() * 2.0;
        quat.qw = (rot.data[0][2] - rot.data[2][0]) / s;
        quat.qx = (rot.data[0][1] + rot.data[1][0]) / s;
        quat.qy = 0.25 * s;
        quat.qz = (rot.data[1][2] + rot.data[2][1]) / s;
    } 
    else {
        let s = (1.0 + rot.data[2][2] - rot.data[0][0] - rot.data[1][1]).sqrt() * 2.0;
        quat.qw = (rot.data[1][0] - rot.data[0][1]) / s;
        quat.qx = (rot.data[0][2] + rot.data[2][0]) / s;
        quat.qy = (rot.data[1][2] + rot.data[2][1]) / s;
        quat.qz = 0.25 * s;
    }

    quat
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

    let force_in_grams = p[0] * rpm.powi(2) + p[1] * rpm + p[2];

    let mut force_in_newton = force_in_grams * 9.81 / 1000.0;

    force_in_newton = force_in_newton.max(0.0);
    force_in_newton.min(0.012*9.81)
    // force_in_newton
}
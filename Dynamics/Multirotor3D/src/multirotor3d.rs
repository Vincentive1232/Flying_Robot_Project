use nalgebra::{Quaternion, Matrix4, Matrix3, Vector3, Vector4};
use super::utils;


/************************************************
* Configuration Space
* Format:
* p(position): Vector3 <x, y, z>
* v(velocity): Vector3 <vx, vy, vz>
* q(angle): UnitQuaternion <qw, qx, qy, qz>
* q_dot(angular velocity): Vector3 <wx, wy, wz>
* NOTE: q_dot should be Vector4, but for the convenience here we 
*       define it as Quaternion.
************************************************/
#[derive(Debug)]
pub struct Multirotor3DState {
    pub p: Vector3<f32>,
    pub v: Vector3<f32>,
    pub q: Quaternion<f32>,
    pub q_dot: Vector3<f32>,
}

/************************************************
* Action Space
* Format: Vector4 <rpm1, rpm2, rpm3, rpm4>.
************************************************/
#[derive(Debug)]
pub struct Multirotor3DAction {
    pub a: Vector4<f32>,
}

#[derive(Debug)]
pub struct Multirotor3D {
    pub mass: f32,
    pub g: f32,
    pub j: Matrix3<f32>,
    pub j_inv: Matrix3<f32>,
    pub l: f32,
    pub dt: f32,
    pub state: Multirotor3DState,
    pub b0: Matrix4<f32>,
}

impl Multirotor3D {
    pub fn step(&mut self, action: Multirotor3DAction) {
        // let pi_f32 = std::f32::consts::PI;
        let f_vec = Vector3::new(0.0, 0.0, action.a[0]);
        let tau_vec = Vector3::new(action.a[1], action.a[2], action.a[3]);
        let g_vec = Vector3::new(0.0, 0.0, self.g);


        self.state.p = self.state.p + self.state.v * self.dt;
        self.state.v = self.state.v + (g_vec + (1.0/self.mass) * utils::circle_dot(self.state.q, f_vec)) * self.dt;
        self.state.q = self.state.q + (0.5*self.state.q*utils::vector_augument(self.state.q_dot)) * self.dt;
        self.state.q_dot = self.state.q_dot + (self.j_inv*((self.j*self.state.q_dot).cross(&self.state.q_dot) + tau_vec)) * self.dt;
    }
}
use crate::utils::{Vector3, Vector4};

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
    pub p: utils::Vector3,
    pub v: utils::Vector3,
    pub q: utils::Quaternion,
    pub q_dot: utils::Vector3,
}

/************************************************
* Action Space
* Format: Vector4 <rpm1, rpm2, rpm3, rpm4>.
************************************************/
#[derive(Debug)]
pub struct Multirotor3DAction {
    pub a: utils::Vector4,
}

#[derive(Debug)]
pub struct Multirotor3D {
    pub mass: f32,
    pub g: f32,
    pub j: utils::Matrix3,
    pub j_inv: utils::Matrix3,
    pub b0: utils::Matrix4,
    pub l: f32,
    pub dt: f32,
    pub state: Multirotor3DState,
    pub kp: utils::Matrix3,
    pub kv: utils::Matrix3,
    pub kr: utils::Matrix3,
    pub k_omega: utils::Matrix3,
    pub ki_p: utils::Matrix3,
    pub ki: utils::Matrix3,
    pub i_error_att: utils::Vector3,    // angular integration error, to avoid calculating w and w_dot
    pub i_error_pos: utils::Vector3,
    pub ref_quat: utils::Quaternion,
    pub ref_ang_vel: utils::Vector3,
    pub ref_ang_acc: utils::Vector3,
    pub ang_acc: utils::Vector3,
}

impl Multirotor3D {
    // Pure Euler integration using quaternion
    pub fn step_euler(&mut self, action: Multirotor3DAction) {
        // let pi_f32 = std::f32::consts::PI;
        let f_vec = utils::Vector3::new(0.0, 0.0, action.a.x);
        let tau_vec = utils::Vector3::new(action.a.y, action.a.z, action.a.k);
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);


        self.state.p = self.state.p + self.dt * self.state.v;
        self.state.v = self.state.v + self.dt * (g_vec + (1.0/self.mass) * utils::circle_dot(self.state.q, f_vec));
        self.state.q = self.state.q + self.dt * (0.5 * (self.state.q)*utils::vector_augument(self.state.q_dot)); //换了相乘顺序，但不知道对不对
        self.state.q_dot = self.state.q_dot + self.dt * (self.j_inv*((self.j*self.state.q_dot).cross(self.state.q_dot) + tau_vec));

        self.state.q = (1.0/self.state.q.norm()) * self.state.q;
        // println!("{:?}", self.state.q.norm());
    }


    // Pure RK4 integration using quaternion
    pub fn step_rk4(&mut self, action: Multirotor3DAction) {
        let f_vec = utils::Vector3::new(0.0, 0.0, action.a.x);
        let tau_vec = utils::Vector3::new(action.a.y, action.a.z, action.a.k);
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);

        let k1 = Multirotor3DState{
            p:      self.state.v,
            v:      (g_vec + (1.0/self.mass) * utils::circle_dot(self.state.q, f_vec)),
            q:      (0.5 * (self.state.q*utils::vector_augument(self.state.q_dot))),
            q_dot:  (self.j_inv*((self.j*self.state.q_dot).cross(self.state.q_dot) + tau_vec)),
        };

        // Here might be a bit confusing since I just used the same struccture
        // to represent the differential equations' results
        let mut state_temp = Multirotor3DState {
            p:      self.state.p + 0.5 * self.dt * k1.p,
            v:      self.state.v + 0.5 * self.dt * k1.v,
            q:      self.state.q + 0.5 * self.dt * k1.q,
            q_dot:  self.state.q_dot + 0.5 * self.dt * k1.q_dot,
        };
        state_temp.q = (1.0/state_temp.q.norm()) * state_temp.q;


        let k2 = Multirotor3DState{
            p:      state_temp.v,
            v:      (g_vec + (1.0/self.mass) * utils::circle_dot(state_temp.q, f_vec)),
            q:      (0.5 * (state_temp.q*utils::vector_augument(state_temp.q_dot))),
            q_dot:  (self.j_inv*((self.j*state_temp.q_dot).cross(state_temp.q_dot) + tau_vec)),
        };

        state_temp = Multirotor3DState {
            p:      self.state.p + 0.5 * self.dt * k2.p,
            v:      self.state.v + 0.5 * self.dt * k2.v,
            q:      self.state.q + 0.5 * self.dt * k2.q,
            q_dot:  self.state.q_dot + 0.5 * self.dt * k2.q_dot,
        };
        state_temp.q = (1.0/state_temp.q.norm()) * state_temp.q;


        let k3 = Multirotor3DState{
            p:      state_temp.v,
            v:      (g_vec + (1.0/self.mass) * utils::circle_dot(state_temp.q, f_vec)),
            q:      (0.5 * (state_temp.q*utils::vector_augument(state_temp.q_dot))),
            q_dot:  (self.j_inv*((self.j*state_temp.q_dot).cross(state_temp.q_dot) + tau_vec)),
        };


        state_temp = Multirotor3DState {
            p:      self.state.p + self.dt * k3.p,
            v:      self.state.v + self.dt * k3.v,
            q:      self.state.q + self.dt * k3.q,
            q_dot:  self.state.q_dot + self.dt * k3.q_dot,
        };
        state_temp.q = (1.0/state_temp.q.norm()) * state_temp.q;

        
        let k4 = Multirotor3DState{
            p:      state_temp.v,
            v:      self.dt * (g_vec + (1.0/self.mass) * utils::circle_dot(state_temp.q, f_vec)),
            q:      self.dt * (0.5 * (state_temp.q*utils::vector_augument(state_temp.q_dot))),
            q_dot:  self.dt * (self.j_inv*((self.j*state_temp.q_dot).cross(state_temp.q_dot) + tau_vec)),
        };

        self.state.p = self.state.p + (1.0/6.0)*self.dt*(k1.p + 2.0*k2.p + 2.0*k3.p + k4.p);
        self.state.v = self.state.v + (1.0/6.0)*self.dt*(k1.v + 2.0*k2.v + 2.0*k3.v + k4.v);
        self.state.q = self.state.q + (1.0/6.0)*self.dt*(k1.q + 2.0*k2.q + 2.0*k3.q + k4.q);
        self.state.q_dot = self.state.q_dot + (1.0/6.0)*self.dt*(k1.q_dot + 2.0*k2.q_dot + 2.0*k3.q_dot + k4.q_dot);

        self.state.q = (1.0/self.state.q.norm()) * self.state.q;

    }


    pub fn step_euler_fake_double(&mut self, action: Multirotor3DAction, q: utils::Quaternion, q_dot: utils::Vector3, v: utils::Vector3, dt: f32) {
        // let pi_f32 = std::f32::consts::PI;
        self.dt = dt;
        let f_vec = utils::Vector3::new(0.0, 0.0, action.a.x);
        let tau_vec = utils::Vector3::new(action.a.y, action.a.z, action.a.k);
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);
        let rot_matrix = utils::quaternion_to_matrix(self.state.q);
        // let rot_matrix = utils::quaternion_to_matrix(utils::Quaternion{qw: 0.0, qx: 0.0, qy: 0.0, qz: 1.0});
        // println!("matrix: {:?}", rot_matrix);

        let a = g_vec + (1.0/self.mass) * (rot_matrix * f_vec);

        self.state.p = self.state.p + self.dt * self.state.v;
        // self.state.v = v;
        self.state.v = self.state.v + self.dt * a;
        // self.state.q = q;
        self.state.q = self.state.q + self.dt * (0.5 * (self.state.q)*utils::vector_augument(self.state.q_dot));
        // self.state.q = utils::Quaternion::new(0.0, 0.0, 0.0, 1.0);
        // self.state.q_dot = utils::Vector3::new(0.0, 0.0, 0.0);
        self.state.q_dot = q_dot;
        // self.state.q_dot = self.state.q_dot + self.dt * (self.j_inv*((self.j*self.state.q_dot).cross(self.state.q_dot) + tau_vec));

        // self.state.q_dot = self.state.q_dot + self.dt * (self.j_inv*(tau_vec - (utils::skew(self.state.q_dot)*self.j*self.state.q_dot)));
        println!("q_dot_dot: {:?}", (self.j_inv*(tau_vec - (utils::skew(self.state.q_dot)*self.j*self.state.q_dot))));
        // println!("q_dot_dot_cross: {:?}", self.dt * (self.j_inv*((self.j*self.state.q_dot).cross(self.state.q_dot) + tau_vec)));
        // println!("q_dot: {:?}", self.state.q_dot);
        self.state.q = (1.0/self.state.q.norm()) * self.state.q;
    }


    pub fn step_rk4_fake_double(&mut self, action: Multirotor3DAction, q: utils::Quaternion, v: utils::Vector3, q_dot:utils::Vector3, q_dot_next: utils::Vector3) {
        let f_vec = utils::Vector3::new(0.0, 0.0, action.a.x);
        let tau_vec = utils::Vector3::new(action.a.y, action.a.z, action.a.k);
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);

        let k1 = Multirotor3DState{
            p:      self.state.v,
            v:      (g_vec + (1.0/self.mass) * utils::circle_dot(self.state.q, f_vec)),
            q:      (0.5 * (self.state.q*utils::vector_augument(self.state.q_dot))),
            q_dot:  q_dot,
        };

        // Here might be a bit confusing since I just used the same struccture
        // to represent the differential equations' results
        let mut state_temp = Multirotor3DState {
            p:      self.state.p + 0.5 * self.dt * k1.p,
            v:      self.state.v + 0.5 * self.dt * k1.v,
            q:      self.state.q + 0.5 * self.dt * k1.q,
            q_dot:  q_dot + 0.5 * (q_dot_next - q_dot),
        };
        state_temp.q = (1.0/state_temp.q.norm()) * state_temp.q;


        let k2: Multirotor3DState = Multirotor3DState{
            p:      state_temp.v,
            v:      (g_vec + (1.0/self.mass) * utils::circle_dot(state_temp.q, f_vec)),
            q:      (0.5 * (state_temp.q*utils::vector_augument(state_temp.q_dot))),
            q_dot:  q_dot,
        };

        state_temp = Multirotor3DState {
            p:      self.state.p + 0.5 * self.dt * k2.p,
            v:      self.state.v + 0.5 * self.dt * k2.v,
            q:      self.state.q + 0.5 * self.dt * k2.q,
            q_dot:  q_dot + 0.5 * (q_dot_next - q_dot),
        };
        state_temp.q = (1.0/state_temp.q.norm()) * state_temp.q;


        let k3 = Multirotor3DState{
            p:      state_temp.v,
            v:      (g_vec + (1.0/self.mass) * utils::circle_dot(state_temp.q, f_vec)),
            q:      (0.5 * (state_temp.q*utils::vector_augument(state_temp.q_dot))),
            q_dot:  q_dot,
        };


        state_temp = Multirotor3DState {
            p:      self.state.p + self.dt * k3.p,
            v:      self.state.v + self.dt * k3.v,
            q:      self.state.q + self.dt * k3.q,
            q_dot:  q_dot_next,
        };
        state_temp.q = (1.0/state_temp.q.norm()) * state_temp.q;


        let k4 = Multirotor3DState{
            p:      state_temp.v,
            v:      self.dt * (g_vec + (1.0/self.mass) * utils::circle_dot(state_temp.q, f_vec)),
            q:      self.dt * (0.5 * (state_temp.q*utils::vector_augument(state_temp.q_dot))),
            q_dot:  q_dot,
        };

        self.state.p = self.state.p + (1.0/6.0)*self.dt*(k1.p + 2.0*k2.p + 2.0*k3.p + k4.p);
        self.state.v = self.state.v + (1.0/6.0)*self.dt*(k1.v + 2.0*k2.v + 2.0*k3.v + k4.v);
        // self.state.q = self.state.q + (1.0/6.0)*self.dt*(k1.q + 2.0*k2.q + 2.0*k3.q + k4.q);
        self.state.q = q;
        self.state.q_dot = q_dot;

        self.state.q = (1.0/self.state.q.norm()) * self.state.q;

    }


    pub fn step_euler_crazyflies2(&mut self, action: Multirotor3DAction) {
        let force_1 = utils::rpm_to_force(action.a.x);
        let force_2 = utils::rpm_to_force(action.a.y);
        let force_3 = utils::rpm_to_force(action.a.z);
        let force_4 = utils::rpm_to_force(action.a.k);

        let force = Vector4::new(force_1, force_2, force_3, force_4);
        
        let eta = self.b0 * force;
        let fu = Vector3::new(0.0, 0.0, eta.x);
        let tau_u = Vector3::new(eta.y, eta.z, eta.k);
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);

        let pos_next = self.state.p + self.dt*self.state.v;
        let vel_next = self.state.v + self.dt*(g_vec + (1.0/self.mass) * utils::circle_dot(self.state.q, fu));

        let q_next = utils::rotation_update(self.state.q, self.state.q_dot, self.dt);
        let q_dot_next = self.state.q_dot + self.dt * (self.j_inv*((self.j*self.state.q_dot).cross(self.state.q_dot) + tau_u));
        println!("q_dot_dot: {:?}", self.j_inv*((self.j*self.state.q_dot).cross(self.state.q_dot) + tau_u));
        println!("tau_u: {:?}", tau_u);
        println!("eta:{:?}", eta);
        
        self.state.p = pos_next;
        self.state.v = vel_next;
        self.state.q = q_next;
        self.state.q_dot = q_dot_next;

        
        if self.state.p.z < 0.0 {
            self.state.p.z = 0.0;
            self.state.v.x = 0.0;
            self.state.v.y = 0.0;
            self.state.v.z = 0.0;
            self.state.q_dot.x = 0.0;
            self.state.q_dot.y = 0.0;
            self.state.q_dot.z = 0.0;
        }
        
    }


    // Lee Controller example
    pub fn lee_controller_example(&mut self, pos_d: utils::Vector3, vel_d: utils::Vector3, acc_d: utils::Vector3, jerk_d: utils::Vector3, snap_d: utils::Vector3) -> Multirotor3DAction {
        // Linear Errors
        let ep = utils::v3_clamp(pos_d - self.state.p, -100.0, 100.0);
        let ev = utils::v3_clamp(vel_d - self.state.v, -100.0, 100.0);
        // let ep = pos_d - self.state.p;
        // let ev = vel_d - self.state.v;
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);
        let e3 = utils::Vector3::new(0.0, 0.0, 1.0);
        let rot_matrix = utils::quaternion_to_matrix(self.state.q);
        

        // Thrust Calculation
        let fd = self.mass * (acc_d + self.kp*ep + self.kv*ev - g_vec);
        let f = fd.dot_product(rot_matrix*e3);



        // Rotation Errors (take phi=0 to avoid using sin and cos)
        let phi_dot: f32 = 0.0;
        let xbd = utils::Vector3::new(1.0, 0.0, 0.0);
        let zbd = (1.0/fd.norm()) * fd;
        let z_x_cross = zbd.cross(xbd);
        let ybd = (1.0/z_x_cross.norm()) * z_x_cross;
        let y_z_cross = ybd.cross(zbd);
        let rot_matrix_d = utils::Matrix3::new([
            [y_z_cross.x, ybd.x, zbd.x],
            [y_z_cross.y, ybd.y, zbd.y],
            [y_z_cross.z, ybd.z, zbd.z],
        ]);

        let hw = (self.mass/f) * (jerk_d - zbd.dot_product(jerk_d)*zbd);
        let w_xd = -hw.dot_product(ybd);
        let w_yd = hw.dot_product(xbd);
        let w_zd = phi_dot*e3.dot_product(zbd);
        let w_d = utils::Vector3::new(w_xd, w_yd, w_zd);
        println!("w_d: {:?}", w_d);

        let er = 0.5*utils::vee(rot_matrix_d.transpose()*rot_matrix - rot_matrix.transpose()*rot_matrix_d);
        let ew: Vector3 = self.state.q_dot - rot_matrix.transpose()*rot_matrix_d*w_d;

        self.i_error_att = self.i_error_att + self.dt*er;

        let tau_u = -(self.kr*er) - (self.k_omega*ew) - self.ki*self.i_error_att + self.state.q_dot.cross(self.j*self.state.q_dot);

        let action = utils::Vector4::new(f, tau_u.x, tau_u.y, tau_u.z);
        // println!("w_d: {:?}", action);

        Multirotor3DAction{
            a: action
        }

    }


    pub fn lee_controller(&mut self, pos_d: utils::Vector3, vel_d: utils::Vector3, acc_d: utils::Vector3, jerk_d: utils::Vector3, snap_d: utils::Vector3) -> Multirotor3DAction {
        // Linear Errors
        let ep = utils::v3_clamp(pos_d - self.state.p, -100.0, 100.0);
        let ev = utils::v3_clamp(vel_d - self.state.v, -100.0, 100.0);
        // let ep = pos_d - self.state.p;
        // let ev = vel_d - self.state.v;
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);
        let e3 = utils::Vector3::new(0.0, 0.0, 1.0);
        let rot_matrix = utils::quaternion_to_matrix(self.state.q);

        // Thrust Calculation
        let fd = self.mass * (acc_d + self.kp*ep + self.kv*ev - g_vec);
        let f = fd.dot_product(rot_matrix*e3);



        // Rotation Errors (take phi=0 to avoid using sin and cos)
        let phi_dot: f32 = 0.0;
        let xbd = utils::Vector3::new(1.0, 0.0, 0.0);
        let zbd = (1.0/fd.norm()) * fd;
        let z_x_cross = zbd.cross(xbd);
        let ybd = (1.0/z_x_cross.norm()) * z_x_cross;
        let y_z_cross = ybd.cross(zbd);
        let rot_matrix_d = utils::Matrix3::new([
            [y_z_cross.x, ybd.x, zbd.x],
            [y_z_cross.y, ybd.y, zbd.y],
            [y_z_cross.z, ybd.z, zbd.z],
        ]);

        let hw = (self.mass/f) * (jerk_d - zbd.dot_product(jerk_d)*zbd);
        let w_xd = -hw.dot_product(ybd);
        let w_yd = hw.dot_product(xbd);
        let w_zd = phi_dot*e3.dot_product(zbd);
        let w_d = utils::Vector3::new(w_xd, w_yd, w_zd);

        let hw_dot = (self.mass/f) * (snap_d - zbd.dot_product(snap_d)*zbd);
        let w_xd_dot = -hw_dot.dot_product(ybd);
        let w_yd_dot = hw_dot.dot_product(xbd);
        let w_zd_dot = phi_dot*e3.dot_product(zbd);
        let w_d_dot = utils::Vector3::new(w_xd_dot, w_yd_dot, w_zd_dot);
        self.ang_acc = w_d_dot;

        let er = 0.5*utils::vee(rot_matrix_d.transpose()*rot_matrix - rot_matrix.transpose()*rot_matrix_d);
        let ew: Vector3 = self.state.q_dot - rot_matrix.transpose()*rot_matrix_d*w_d;

        self.i_error_att = self.i_error_att + self.dt*er;
        let temp = utils::skew(self.state.q_dot)*rot_matrix.transpose()*rot_matrix_d*w_d - rot_matrix.transpose()*rot_matrix_d*w_d_dot;

        let tau_u = -(self.kr*er) - (self.k_omega*ew) + self.state.q_dot.cross(self.j*self.state.q_dot) - self.j*temp;

        let action = utils::Vector4::new(f, tau_u.x, tau_u.y, tau_u.z);

        Multirotor3DAction{
            a: action
        }

    }


    pub fn differential_flatness(&mut self, pos_d: utils::Vector3, vel_d: utils::Vector3, acc_d: utils::Vector3, jerk_d: utils::Vector3, snap_d: utils::Vector3) -> Multirotor3DAction {
        
        // ===== Compute thrust f =====
        let phi: f32 = 0.0;
        let phi_dot: f32 = 0.0;
        let phi_dot_dot: f32 = 0.0;
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);
        let xc = utils::Vector3::new(libm::cosf(phi), libm::sinf(phi), 0.0);
        let yc = utils::Vector3::new(-libm::sinf(phi), libm::cosf(phi), 0.0);
        let xb = (1.0/(yc.cross(acc_d - g_vec)).norm()) * ((yc.cross(acc_d - g_vec)));
        let yb = (1.0/(acc_d - g_vec).cross(xb).norm()) * ((acc_d - g_vec).cross(xb));
        let zb = xb.cross(yb);
        let rot_matrix = utils::Matrix3::new([
            [xb.x, yb.x, zb.x],
            [xb.y, yb.y, zb.y],
            [xb.z, yb.z, zb.z],
        ]);
        self.ref_quat = utils::rotation_matrix_to_quaternion(rot_matrix);
        

        let c = zb.dot_product(acc_d - g_vec);
        let f = self.mass * c;


        // ===== Compute torque tau =====
        // ===== 1. Compute omega w =====
        let d1 = xb.dot_product(jerk_d);
        let d2 = (-yb).dot_product(jerk_d);

        let b3 = (-yc).dot_product(zb);
        let c3 = (yc.cross(zb)).norm();
        let d3 = phi_dot * xc.dot_product(xb);

        println!("c: {:?}", c);
        let wx = d2 / c;
        let wy = d1 / c;
        let wz = (c*d3 - b3*d1) / (c*c3);

        let w = utils::Vector3::new(wx, wy, wz);
        self.ref_ang_vel = w;
        
        // ===== 2. Compute omega dot w_dot =====
        let c_dot = zb.dot_product(jerk_d);
        let e1 = xb.dot_product(snap_d) - 2.0*c_dot*wy - c*wx*wz;
        let e2 = -yb.dot_product(snap_d) - 2.0*c_dot*wx + c*wy*wz;
        let e3 = phi_dot_dot*xc.dot_product(xb) + 2.0*phi_dot*wz*xc.dot_product(yb) - 2.0*phi_dot*wy*xc.dot_product(zb) - wx*wy*yc.dot_product(yb) - wx*wz*yc.dot_product(zb);

        let wx_dot = e2 / c;
        let wy_dot = e1 / c;
        let wz_dot = (c*e3 - b3*e1) / (c*c3);

        let w_dot = utils::Vector3::new(wx_dot, wy_dot, wz_dot);
        self.ref_ang_acc = w_dot;
        self.ang_acc = w_dot;

        // ===== 3. Compute tau =====
        let tau_u = self.j*w_dot - (self.j*w).cross(w);


        // ===== Finalize the action =====
        let action = utils::Vector4::new(f, tau_u.x, tau_u.y, tau_u.z);

        Multirotor3DAction{
            a: action
        }
    }

}
use crate::utils::{circle_dot, Vector3, Vector4};

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
}

impl Multirotor3D {
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
    }



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




    pub fn step_euler_fake(&mut self, action: Multirotor3DAction, q: utils::Quaternion, q_dot: utils::Vector3, v: utils::Vector3) {
        // let pi_f32 = std::f32::consts::PI;
        let f_vec = utils::Vector3::new(0.0, 0.0, action.a.x);
        let tau_vec = utils::Vector3::new(action.a.y, action.a.z, action.a.k);
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);


        self.state.p = self.state.p + self.dt * self.state.v;
        self.state.v = self.state.v + self.dt * (g_vec + (1.0/self.mass) * utils::circle_dot(self.state.q, f_vec));
        self.state.q = self.state.q + self.dt * (0.5 * (self.state.q)*utils::vector_augument(self.state.q_dot));
        self.state.q_dot = q_dot;

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


    pub fn step_euler_correction(&mut self, action: Multirotor3DAction, p: utils::Vector3, v: utils::Vector3, q: utils::Quaternion, q_dot: utils::Vector3, dt: f32, i: usize) {
        // let pi_f32 = std::f32::consts::PI;
        if i%100 == 0
        {
            self.state.p = p;
            self.state.v = v;
            self.state.q = q;
            self.state.q_dot = q_dot;
        }
        else{
            self.dt = dt;
            let f_vec = utils::Vector3::new(0.0, 0.0, action.a.x);
            let tau_vec = utils::Vector3::new(action.a.y, action.a.z, action.a.k);
            let g_vec = utils::Vector3::new(0.0, 0.0, self.g);
            let rot_matrix = utils::quaternion_to_matrix(self.state.q);

            let a = g_vec + (1.0/self.mass) * (rot_matrix * f_vec);

            self.state.p = self.state.p + self.dt * self.state.v;
            
            self.state.v = self.state.v + self.dt * (g_vec + (1.0/self.mass) * utils::circle_dot(self.state.q, f_vec));
            // self.state.v = self.state.v + self.dt * a;
            // self.state.v = v;
            
            // let temp_q_dot = circle_dot(self.state.q, self.state.q_dot);
            // self.state.q = self.state.q + self.dt * (0.5 * utils::vector_augument(temp_q_dot) * self.state.q);
            self.state.q = self.state.q + self.dt * (0.5 * (self.state.q) * utils::vector_augument(self.state.q_dot));
            // self.state.q = utils::Quaternion::new(0.0, 0.0, 0.0, 1.0);
            // self.state.q = q;
            self.state.q = (1.0/self.state.q.norm()) * self.state.q;
            
            self.state.q_dot = self.state.q_dot + self.dt * (self.j_inv*((self.j*self.state.q_dot).cross(self.state.q_dot) + tau_vec));
            //self.state.q_dot = self.state.q_dot + self.dt * (self.j_inv*(tau_vec - (utils::skew(self.state.q_dot)*self.j*self.state.q_dot)));
            // self.state.q_dot = utils::Vector3::new(0.0, 0.0, 0.0);
            // self.state.q_dot = q_dot;
            // println!("q_dot_dot: {:?}", (self.j_inv*(tau_vec - (utils::skew(self.state.q_dot)*self.j*self.state.q_dot))));
            // println!("q_dot_dot_cross: {:?}", self.dt * (self.j_inv*((self.j*self.state.q_dot).cross(self.state.q_dot) + tau_vec)));
            // println!("q_dot: {:?}", self.state.q_dot);
        }
    }


    pub fn step_rk4_fake(&mut self, action: Multirotor3DAction, q_dot:utils::Vector3, q_dot_next: utils::Vector3) {
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
        self.state.q = self.state.q + (1.0/6.0)*self.dt*(k1.q + 2.0*k2.q + 2.0*k3.q + k4.q);
        self.state.q_dot = q_dot;

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


    pub fn step_rk4_correction(&mut self, action: Multirotor3DAction, p: utils::Vector3, v: utils::Vector3, q: utils::Quaternion, q_dot: utils::Vector3, dt: f32, i: usize) {
        self.dt = dt;
        let f_vec = utils::Vector3::new(0.0, 0.0, action.a.x);
        let tau_vec = utils::Vector3::new(action.a.y, action.a.z, action.a.k);
        let g_vec = utils::Vector3::new(0.0, 0.0, self.g);
        
        if i%10 == 0
        {
            self.state.p = p;
            self.state.v = v;
            self.state.q = q;
            self.state.q_dot = q_dot;
        }
        else
        {
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
}
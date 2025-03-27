use crate::stabilizer_types::{self, control_t, sensorData_t, setpoint_t, state_t};
use crate::math as my_math;


/************************************************
* Configuration Space
* Format:
* p(position): Vector3 <x, y, z>
* v(velocity): Vector3 <vx, vy, vz>
* q(angle): Quaternion <qw, qx, qy, qz>
* q_dot(angular velocity): Vector3 <wx, wy, wz>
* NOTE: q_dot should be Vector4, but for the convenience here we 
*       define it as Quaternion.
************************************************/
#[derive(Debug, Copy, Clone)]
pub struct Multirotor3DState {
    pub p: my_math::Vector3,
    pub v: my_math::Vector3,
    pub q: my_math::Quaternion,
    pub w: my_math::Vector3,
}



#[derive(Debug, Copy, Clone)]
pub struct controller_Lee {
    pub mass: f32,
    pub g: f32,
    pub pi: f32,
    pub J: my_math::Matrix3,
    pub l: f32,
    pub dt: f32,
    pub state: Multirotor3DState,
    pub kp: my_math::Matrix3,
    pub kp_limit: f32,
    pub kv: my_math::Matrix3,
    pub kv_limit: f32,
    pub kr: my_math::Matrix3,
    pub k_omega: my_math::Matrix3,
    pub ki: my_math::Matrix3,
    pub i_error_att: my_math::Vector3
}

impl controller_Lee {
    pub fn controller_Lee_Init() -> Self {
        controller_Lee {
            mass: 0.034,
            g: -9.81,
            pi: 3.1415926,
            J: my_math::Matrix3::new([
                    [16.571710e-6, 0.0, 0.0],
                    [0.0, 16.655602e-6, 0.0],
                    [0.0, 0.0, 29.261652e-6],
                ]),
            l: 0.08,
            dt: 1.0/1000.0,
            state: Multirotor3DState {
                p: my_math::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
                v: my_math::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
                q: my_math::Quaternion {qw: 0.0, qx: 0.0, qy: 0.0, qz: 1.0},  
                w: my_math::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
            },

            // Initialize the gains
            kp: my_math::Matrix3::new([
                [7.0, 0.0, 0.0],
                [0.0, 7.0, 0.0],
                [0.0, 0.0, 7.0],
            ]),
            kp_limit: 100.0,
            kv: my_math::Matrix3::new([
                [4.0, 0.0, 0.0],
                [0.0, 4.0, 0.0],
                [0.0, 0.0, 4.0],
            ]),
            kv_limit: 100.0,
            kr: my_math::Matrix3::new([
                [0.007, 0.0, 0.0],
                [0.0, 0.007, 0.0],
                [0.0, 0.0, 0.008],
            ]),
            k_omega: my_math::Matrix3::new([
                [0.00115, 0.0, 0.0],
                [0.0, 0.00115, 0.0],
                [0.0, 0.0, 0.002],
            ]),
            ki: my_math::Matrix3::new([
                [0.03, 0.0, 0.0],
                [0.0, 0.03, 0.0],
                [0.0, 0.0, 0.03],
            ]),
            i_error_att: my_math::Vector3 {x: 0.0, y: 0.0, z: 0.0},
        }
    }


    pub fn controller_Lee(&mut self, control: *mut control_t, setpoint: *const setpoint_t, sensors: *const sensorData_t, state: *const state_t, tick: u32) {
        unsafe {
            if !sensors.is_null() {
                let setpoint_data: &setpoint_t = &*setpoint;
                let state_data: &state_t = &*state;
                let sensors_data: &sensorData_t = &*sensors;

                // define the reference trajectory
                let pos_d: my_math::Vector3 = setpoint_data.position.into();
                let vel_d: my_math::Vector3 = setpoint_data.velocity.into();
                let acc_d: my_math::Vector3 = setpoint_data.acceleration.into();
                let jerk_d: my_math::Vector3 = setpoint_data.jerk.into();

                // define the current states
                let pos_c: my_math::Vector3 = state_data.position.into();
                let vel_c: my_math::Vector3 = state_data.velocity.into();
                let q_c: my_math::Quaternion = state_data.attitudeQuaternion.into();
                let w_c_deg: my_math::Vector3 = my_math::Vector3::new(
                    sensors_data.gyro.__bindgen_anon_1.x, 
                    sensors_data.gyro.__bindgen_anon_1.y, 
                    sensors_data.gyro.__bindgen_anon_1.z
                );

                // Convert and Normalize rotation variables (sensor data in deg/s -> rad/s)
                let w_c_rad = (self.pi / 180.0) * w_c_deg;
                let rot_matrix = my_math::quaternion_to_matrix(q_c);

                // define g_vector and e3 vector
                let g_vec = my_math::Vector3::new(0.0, 0.0, self.g);
                let e3 = my_math::Vector3::new(0.0, 0.0, 1.0);

                ///////////////////////////////////////////////////////////////////////////////////////
                // Linear Errors
                let ep = my_math::v3_clamp(pos_d - pos_c, -self.kp_limit, self.kp_limit);
                let ev = my_math::v3_clamp(vel_d - vel_c, -self.kv_limit, self.kv_limit);

                // Thrust
                let fd = self.mass * (acc_d + self.kp*ep + self.kv*ev - g_vec);
                let f = fd.dot_product(rot_matrix*e3);
                ///////////////////////////////////////////////////////////////////////////////////////
                 
                
                // Rotation Errors (take phi=0 to avoid using sin and cos)
                let phi_dot: f32 = 0.0;
                let xbd = my_math::Vector3::new(1.0, 0.0, 0.0);
                let zbd = (1.0/fd.norm()) * fd;
                let z_x_cross = zbd.cross(xbd);
                let ybd = (1.0/z_x_cross.norm()) * z_x_cross;
                let y_z_cross = ybd.cross(zbd);
                let rot_matrix_d = my_math::Matrix3::new([
                    [y_z_cross.x, ybd.x, zbd.x],
                    [y_z_cross.y, ybd.y, zbd.y],
                    [y_z_cross.z, ybd.z, zbd.z],
                ]);

                let hw = (self.mass/f) * (jerk_d - zbd.dot_product(jerk_d)*zbd);
                let w_xd = -hw.dot_product(ybd);
                let w_yd = hw.dot_product(xbd);
                let w_zd = phi_dot*e3.dot_product(zbd);
                let w_d = my_math::Vector3::new(w_xd, w_yd, w_zd);
                
                /*
                let hw_dot = (self.mass/f) * (snap_d - zbd.dot_product(snap_d)*zbd);
                let w_xd_dot = -hw_dot.dot_product(ybd);
                let w_yd_dot = hw_dot.dot_product(xbd);
                let w_zd_dot = phi_dot*e3.dot_product(zbd);
                let w_d_dot = my_math::Vector3::new(w_xd_dot, w_yd_dot, w_zd_dot);
                */

                let er: my_math::Vector3 = 0.5*my_math::vee(rot_matrix_d.transpose()*rot_matrix - rot_matrix.transpose()*rot_matrix_d);
                let ew: my_math::Vector3 = w_c_rad - rot_matrix.transpose()*rot_matrix_d*w_d;

                self.i_error_att = self.i_error_att + self.dt*er;

                let tau_u = -(self.kr*er) - (self.k_omega*ew) - self.ki*self.i_error_att + w_c_rad.cross(self.J*w_c_rad);

                let control_ref = &mut *control;
                control_ref.controlMode = stabilizer_types::control_mode_e_controlModeForceTorque;
                control_ref.__bindgen_anon_1.__bindgen_anon_2 = stabilizer_types::control_s__bindgen_ty_1__bindgen_ty_2 {
                    thrustSi: f,
                    __bindgen_anon_1: stabilizer_types::control_s__bindgen_ty_1__bindgen_ty_2__bindgen_ty_1 {
                        torque: [tau_u.x, tau_u.y, tau_u.z],
                    }
                };
            }
        }
    }
}

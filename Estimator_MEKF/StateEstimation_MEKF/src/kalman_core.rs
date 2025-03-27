#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]

extern crate libm;

use crate::math::{self as my_math};

/*****************************************************************
 * constant indexes 
 ****************************************************************/
const KC_State_X: usize = 0;
const KC_State_Y: usize = 1;
const KC_State_Z: usize = 2;

const KC_State_Px: usize = 3;
const KC_State_Py: usize = 4;
const KC_State_Pz: usize = 5;

const KC_State_D0: usize = 6;
const KC_State_D1: usize = 7;
const KC_State_D2: usize = 8;

const KC_State_Dim: usize = 9;

pub const GRAVITY_MAGNITUDE: f32 = 9.81;
pub const DEG_TO_GRAD: f32 = 0.01745;
pub const FLOW_RESOLUTION: f32 = 0.1;

const MAX_COVARIANCE: f32 = 100.0;
const MIN_COVARIANCE: f32 = 1e-6;



// Measurement noise model
const expPointA: f32 = 2.5;
const expStdA: f32 = 0.0025; // STD at elevation expPointA [m]
const expPointB: f32 = 4.0;
const expStdB:f32 = 0.2;    // STD at elevation expPointB [m]



/*****************************************************************
 * Matrix structure definition using generics
 * The estimated states are listed as follows:
 *   - X, Y, Z: the position in the global frame
 *   - Vx, Vy, Vz: the velocity in the body frame
 *   - D0, D1, D2: attitude error
 ****************************************************************/
 #[derive(Debug)]
pub struct kalmanCoreData {
    // 3 to-be-estimated state vector
    pub state: my_math::Mat<KC_State_Dim, 1>,
    /* 
    pub position_global: my_math::Mat<3, 1>,
    pub velocity_body: my_math::Mat<3, 1>,
    pub attitude_error: my_math::Mat<3, 1>,
    */

    // rotation in quaternion to allow easier normalization
    pub q: my_math::Quaternion,

    // rotation in rotation matrix, set in stateFinalization and used in prediction step
    pub R: my_math::Mat<3, 3>,

    // Covariance Matrix
    pub Pm: my_math::Mat<KC_State_Dim, KC_State_Dim>,

    // Process Noise Covariance Matrix
    pub Rm: my_math::Mat<KC_State_Dim, KC_State_Dim>,

    // Measurement Noise Covariance Matrix
    pub Qm: my_math::Mat<KC_State_Dim, KC_State_Dim>,

    pub baroReferenceHeight: f32,

    pub initialQuaternion: my_math::Quaternion,

    // Tracks whether an update to the state has been made, and the state therefore requires finalization
    pub isUpdated: bool,
    pub lastPredictionMs: f32,
    pub lastUpdateMs: f32,
    pub lastFlowUpdate: f32,
    pub lastProcessNoiseUpdateMs: f32,
}

impl kalmanCoreData {
    pub fn new() -> Self {
        kalmanCoreData {
            state: my_math::Mat::new(),
            q: my_math::Quaternion::new(1.0, 0.0, 0.0, 0.0),
            R: my_math::Mat::new(),                    
            Pm: my_math::Mat::new(),                   
            Rm: my_math::Mat::new(),                   
            Qm: my_math::Mat::new(),                   
            baroReferenceHeight: 0.0,
            initialQuaternion: my_math::Quaternion::new(1.0, 0.0, 0.0, 0.0),
            isUpdated: false,                 
            lastPredictionMs: 0.0,              
            lastUpdateMs: 0.0,
            lastFlowUpdate: 0.0,                  
            lastProcessNoiseUpdateMs: 0.0,      
        }
    }
}


#[derive(Debug)]
pub struct kalmanCoreParams{
    // standard deviation initialization
    pub stdDevInitialPosition_xy: f32,
    pub stdDevInitialPosition_z: f32,
    pub stdDevInitialVelocity: f32,
    pub stdDevInitialAttitude_rollpitch: f32,
    pub stdDevInitialAttitude_yaw: f32,

    // process noise initialization
    pub procNoiseAcc_xy: f32,
    pub procNoiseAcc_z: f32,
    pub procNoiseVel: f32,
    pub procNoisePos: f32,
    pub procNoiseAtt: f32,

    // measurement noise initialization
    pub measNoiseBaro: f32,
    pub measNoiseGyro_rollpitch: f32,
    pub measNoiseGyro_yaw: f32,

    pub initialX: f32,
    pub initialY: f32,
    pub initialZ: f32,
 
    pub initialYaw: f32,
}

impl kalmanCoreParams {
    pub fn new() -> Self {
        kalmanCoreParams {
            stdDevInitialPosition_xy: 0.1,
            stdDevInitialPosition_z: 0.1,
            stdDevInitialVelocity: 0.01,
            stdDevInitialAttitude_rollpitch: 0.01,
            stdDevInitialAttitude_yaw: 0.01,
            procNoiseAcc_xy: 0.5,
            procNoiseAcc_z: 1.0,
            procNoiseVel: 0.0,
            procNoisePos: 0.0,
            procNoiseAtt: 0.0,
            measNoiseBaro: 2.0,
            measNoiseGyro_rollpitch: 0.1,
            measNoiseGyro_yaw: 0.1,
            initialX: 0.0,
            initialY: 0.0,
            initialZ: 0.0,
            initialYaw: 0.0,
        }
    }
}



pub struct kalmanCoreEstimator{
    pub kalmanCoreData_est: kalmanCoreData,
    pub kalmanCoreParams_est: kalmanCoreParams
}


impl kalmanCoreEstimator{
    pub fn new() -> Self {
        kalmanCoreEstimator {
            kalmanCoreData_est: kalmanCoreData::new(),
            kalmanCoreParams_est: kalmanCoreParams::new(),
        }
    }

    // Kalman Filter Parameters Initialization
    pub fn kalmanCoreInit(&mut self, nowMs: f32, Initial_State: my_math::Mat<KC_State_Dim, 1>){

        // ===== Initialize the Params of the Kalman Filter =====

        // ===== Initialize the Data/States of the Kalman Filter(Initialize to Groundtruth) =====
        self.kalmanCoreData_est.state = Initial_State;
        // self.kalmanCoreData_est.state = my_math::Mat{m: [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]};

        // ===== reset the attitude quaternion =====
        self.kalmanCoreData_est.initialQuaternion = my_math::Quaternion{
            qw: libm::cosf(self.kalmanCoreParams_est.initialYaw / 2.0),
            qx: 0.0,
            qy: 0.0,
            qz: libm::cosf(self.kalmanCoreParams_est.initialYaw / 2.0)
        };

        // ===== Set the initial rotation matrix as an identity matrix =====
        self.kalmanCoreData_est.R = my_math::quaternion_to_matrix_mat(self.kalmanCoreData_est.initialQuaternion);
        
        
        // ===== Initialize the diagonal value of the state covariance matrix =====
        self.kalmanCoreData_est.Pm = my_math::Mat::new();

        // ===== Position Initialization =====
        self.kalmanCoreData_est.Pm.m[KC_State_X][KC_State_X] = libm::powf(self.kalmanCoreParams_est.stdDevInitialPosition_xy, 2.0);
        self.kalmanCoreData_est.Pm.m[KC_State_Y][KC_State_Y] = libm::powf(self.kalmanCoreParams_est.stdDevInitialPosition_xy, 2.0);
        self.kalmanCoreData_est.Pm.m[KC_State_Z][KC_State_Z] = libm::powf(self.kalmanCoreParams_est.stdDevInitialPosition_z, 2.0);  
        
        // ===== Velocity Initialization =====
        self.kalmanCoreData_est.Pm.m[KC_State_Px][KC_State_Px] = libm::powf(self.kalmanCoreParams_est.stdDevInitialVelocity, 2.0);
        self.kalmanCoreData_est.Pm.m[KC_State_Py][KC_State_Py] = libm::powf(self.kalmanCoreParams_est.stdDevInitialVelocity, 2.0);
        self.kalmanCoreData_est.Pm.m[KC_State_Pz][KC_State_Pz] = libm::powf(self.kalmanCoreParams_est.stdDevInitialVelocity, 2.0);

        // ===== Error Delta Initialization =====
        self.kalmanCoreData_est.Pm.m[KC_State_D0][KC_State_D0] = libm::powf(self.kalmanCoreParams_est.stdDevInitialAttitude_rollpitch, 2.0);
        self.kalmanCoreData_est.Pm.m[KC_State_D1][KC_State_D1] = libm::powf(self.kalmanCoreParams_est.stdDevInitialAttitude_rollpitch, 2.0);
        self.kalmanCoreData_est.Pm.m[KC_State_D2][KC_State_D2] = libm::powf(self.kalmanCoreParams_est.stdDevInitialAttitude_yaw, 2.0);
        
        // Initialize the Process Noise Covariance as an all zero matrix
        self.kalmanCoreData_est.Rm = my_math::Mat::new();
        
        self.kalmanCoreData_est.baroReferenceHeight = 0.0;

        self.kalmanCoreData_est.isUpdated = false;
        self.kalmanCoreData_est.lastPredictionMs = nowMs;
        self.kalmanCoreData_est.lastProcessNoiseUpdateMs = nowMs;
    }


    pub fn set_Quat_and_R(&mut self, q: my_math::Quaternion) {
        self.kalmanCoreData_est.q = q;
        self.kalmanCoreData_est.R = my_math::quaternion_to_matrix_mat(q);
    }


    // Kalman Filter Prediction Step
    // Design predict the next state and push forward the covariance
    pub fn predictDt(&mut self, dt: f32, acc: my_math::Vector3, gyro: my_math::Vector3) {

        // ===== Define the linearized dynamics matrix as an all zero matrix =====
        unsafe{
            let mut Am: my_math::Mat<9, 9> = my_math::Mat::new();

            // ===== Here we define the matrix according to the Jacobian provided by sympy =====
            // ===== Initialize the diagonal as 1.0 =====
            Am.m[KC_State_X][KC_State_X] = 1.0;
            Am.m[KC_State_Y][KC_State_Y] = 1.0;
            Am.m[KC_State_Z][KC_State_Z] = 1.0;

            Am.m[KC_State_Px][KC_State_Px] = 1.0;
            Am.m[KC_State_Py][KC_State_Py] = 1.0;
            Am.m[KC_State_Pz][KC_State_Pz] = 1.0;

            Am.m[KC_State_D0][KC_State_D0] = 1.0;
            Am.m[KC_State_D1][KC_State_D1] = 1.0;
            Am.m[KC_State_D2][KC_State_D2] = 1.0;

            // ===== Here we don't use the combination of the elements of quaternion but the element in the Rotation Matrix =====
            // ===== The Rotation Matrix R should be finalized after each update step =====
            Am.m[KC_State_X][KC_State_Px] = self.kalmanCoreData_est.R.m[0][0]*dt;
            Am.m[KC_State_Y][KC_State_Px] = self.kalmanCoreData_est.R.m[1][0]*dt;
            Am.m[KC_State_Z][KC_State_Px] = self.kalmanCoreData_est.R.m[2][0]*dt;

            Am.m[KC_State_X][KC_State_Py] = self.kalmanCoreData_est.R.m[0][1]*dt;
            Am.m[KC_State_Y][KC_State_Py] = self.kalmanCoreData_est.R.m[1][1]*dt;
            Am.m[KC_State_Z][KC_State_Py] = self.kalmanCoreData_est.R.m[2][1]*dt;
            
            Am.m[KC_State_X][KC_State_Pz] = self.kalmanCoreData_est.R.m[0][2]*dt;
            Am.m[KC_State_Y][KC_State_Pz] = self.kalmanCoreData_est.R.m[1][2]*dt;
            Am.m[KC_State_Z][KC_State_Pz] = self.kalmanCoreData_est.R.m[2][2]*dt;


            /*
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // ?????
            // according to Sympy, all other elements should be 0, but in the kalman_core.c there are still some other parts
            // ===== position from attitude error =====
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[0][2] - 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[0][1]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[1][2] - 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[1][1]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[2][2] - 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[2][1]) * dt;

            Am.m[KC_State_Z][KC_State_D0] = (
                                -self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[0][2] + 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[0][0]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                -self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[1][2] + 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[1][0]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                -self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[2][2] + 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[2][0]) * dt;  

            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[0][1] - 
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[0][0]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[1][1] -
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[1][0]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[2][1] - 
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[2][0]) * dt;

            
            // ===== body-frame velocity from body-frame velocity =====
            Am.m[KC_State_Px][KC_State_Px] =  1.0;
            Am.m[KC_State_Py][KC_State_Px] = -gyro.__bindgen_anon_1.z*dt;
            Am.m[KC_State_Pz][KC_State_Px] =  gyro.__bindgen_anon_1.y*dt;

            Am.m[KC_State_Px][KC_State_Py] =  gyro.__bindgen_anon_1.z*dt;
            Am.m[KC_State_Py][KC_State_Py] =  1.0;
            Am.m[KC_State_Pz][KC_State_Py] = -gyro.__bindgen_anon_1.x*dt;

            Am.m[KC_State_Px][KC_State_Pz] = -gyro.__bindgen_anon_1.y*dt;
            Am.m[KC_State_Py][KC_State_Pz] =  gyro.__bindgen_anon_1.x*dt;
            Am.m[KC_State_Pz][KC_State_Pz] =  1.0;


            // ===== body-frame velocity from attitude error =====
            Am.m[KC_State_Px][KC_State_D0] =  0.0;
            Am.m[KC_State_Py][KC_State_D0] = -GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][2]*dt;
            Am.m[KC_State_Pz][KC_State_D0] =  GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][1]*dt;

            Am.m[KC_State_Px][KC_State_D1] =  GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][2]*dt;
            Am.m[KC_State_Py][KC_State_D1] =  0.0;
            Am.m[KC_State_Pz][KC_State_D1] = -GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][0]*dt;

            Am.m[KC_State_Px][KC_State_D2] = -GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][1]*dt;
            Am.m[KC_State_Py][KC_State_D2] =  GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][0]*dt;
            Am.m[KC_State_Pz][KC_State_D2] =  0.0;


            // ===== attitude error from attitude error =====
            let d0: f32 = gyro.__bindgen_anon_1.x * (dt/2.0);
            let d1: f32 = gyro.__bindgen_anon_1.y * (dt/2.0);
            let d2: f32 = gyro.__bindgen_anon_1.z * (dt/2.0);

            Am.m[KC_State_D0][KC_State_D0] =  1.0 - d1*d1/2.0 - d2*d2/2.0;
            Am.m[KC_State_D0][KC_State_D1] =  d2 + d0*d1/2.0;
            Am.m[KC_State_D0][KC_State_D2] = -d1 + d0*d2/2.0;

            Am.m[KC_State_D1][KC_State_D0] = -d2 + d0*d1/2.0;
            Am.m[KC_State_D1][KC_State_D1] =  1.0 - d0*d0/2.0 - d2*d2/2.0;
            Am.m[KC_State_D1][KC_State_D2] =  d0 + d1*d2/2.0;

            Am.m[KC_State_D2][KC_State_D0] =  d1 + d0*d2/2.0;
            Am.m[KC_State_D2][KC_State_D1] = -d0 + d1*d2/2.0;
            Am.m[KC_State_D2][KC_State_D2] =  1.0 - d0*d0/2.0 - d1*d1/2.0;
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            */

            /*
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // ?????
            // according to Sympy, all other elements should be 0, but in the kalman_core.c there are still some other parts
            // ===== position from attitude error =====
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[0][2] - 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[0][1]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[1][2] - 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[1][1]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[2][2] - 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[2][1]) * dt;

            Am.m[KC_State_Z][KC_State_D0] = (
                                -self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[0][2] + 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[0][0]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                -self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[1][2] + 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[1][0]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                -self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[2][2] + 
                                self.kalmanCoreData_est.state.m[KC_State_Pz][0] * self.kalmanCoreData_est.R.m[2][0]) * dt;  

            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[0][1] - 
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[0][0]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[1][1] -
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[1][0]) * dt;
            Am.m[KC_State_Z][KC_State_D0] = (
                                self.kalmanCoreData_est.state.m[KC_State_Px][0] * self.kalmanCoreData_est.R.m[2][1] - 
                                self.kalmanCoreData_est.state.m[KC_State_Py][0] * self.kalmanCoreData_est.R.m[2][0]) * dt;

            
            // ===== body-frame velocity from body-frame velocity =====
            Am.m[KC_State_Px][KC_State_Px] =  1.0;
            Am.m[KC_State_Py][KC_State_Px] = -gyro.z*dt;
            Am.m[KC_State_Pz][KC_State_Px] =  gyro.y*dt;

            Am.m[KC_State_Px][KC_State_Py] =  gyro.z*dt;
            Am.m[KC_State_Py][KC_State_Py] =  1.0;
            Am.m[KC_State_Pz][KC_State_Py] = -gyro.x*dt;

            Am.m[KC_State_Px][KC_State_Pz] = -gyro.y*dt;
            Am.m[KC_State_Py][KC_State_Pz] =  gyro.x*dt;
            Am.m[KC_State_Pz][KC_State_Pz] =  1.0;


            // ===== body-frame velocity from attitude error =====
            Am.m[KC_State_Px][KC_State_D0] =  0.0;
            Am.m[KC_State_Py][KC_State_D0] = -GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][2]*dt;
            Am.m[KC_State_Pz][KC_State_D0] =  GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][1]*dt;

            Am.m[KC_State_Px][KC_State_D1] =  GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][2]*dt;
            Am.m[KC_State_Py][KC_State_D1] =  0.0;
            Am.m[KC_State_Pz][KC_State_D1] = -GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][0]*dt;

            Am.m[KC_State_Px][KC_State_D2] = -GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][1]*dt;
            Am.m[KC_State_Py][KC_State_D2] =  GRAVITY_MAGNITUDE*self.kalmanCoreData_est.R.m[2][0]*dt;
            Am.m[KC_State_Pz][KC_State_D2] =  0.0;


            // ===== attitude error from attitude error =====
            let d0: f32 = gyro.x * (dt/2.0);
            let d1: f32 = gyro.y * (dt/2.0);
            let d2: f32 = gyro.z * (dt/2.0);

            Am.m[KC_State_D0][KC_State_D0] =  1.0 - d1*d1/2.0 - d2*d2/2.0;
            Am.m[KC_State_D0][KC_State_D1] =  d2 + d0*d1/2.0;
            Am.m[KC_State_D0][KC_State_D2] = -d1 + d0*d2/2.0;

            Am.m[KC_State_D1][KC_State_D0] = -d2 + d0*d1/2.0;
            Am.m[KC_State_D1][KC_State_D1] =  1.0 - d0*d0/2.0 - d2*d2/2.0;
            Am.m[KC_State_D1][KC_State_D2] =  d0 + d1*d2/2.0;

            Am.m[KC_State_D2][KC_State_D0] =  d1 + d0*d2/2.0;
            Am.m[KC_State_D2][KC_State_D1] = -d0 + d1*d2/2.0;
            Am.m[KC_State_D2][KC_State_D2] =  1.0 - d0*d0/2.0 - d1*d1/2.0;
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            */

            // ===== Reset Step =====
            self.kalmanCoreData_est.state.m[KC_State_D0][0] = 0.0;
            self.kalmanCoreData_est.state.m[KC_State_D1][0] = 0.0;
            self.kalmanCoreData_est.state.m[KC_State_D2][0] = 0.0;
            

            // ===== Covariance Update =====
            self.kalmanCoreData_est.Pm = Am * self.kalmanCoreData_est.Pm * Am.transpose();   // GPG'
            self.addProcessNoiseDt(dt); // + R

            
            // ===== Prediction Step =====
            // ===== Using the equations in the slides =====
            let p_ref: my_math::Vector3 = my_math::Vector3 { x: self.kalmanCoreData_est.state.m[KC_State_X][0], 
                                                             y: self.kalmanCoreData_est.state.m[KC_State_Y][0], 
                                                             z: self.kalmanCoreData_est.state.m[KC_State_Z][0] 
                                                            };
            let b_ref: my_math::Vector3 = my_math::Vector3 { x: self.kalmanCoreData_est.state.m[KC_State_Px][0], 
                                                             y: self.kalmanCoreData_est.state.m[KC_State_Py][0], 
                                                             z: self.kalmanCoreData_est.state.m[KC_State_Pz][0] 
                                                            };
            let delta_vec_ref: my_math::Vector3 = my_math::Vector3 { x: self.kalmanCoreData_est.state.m[KC_State_D0][0], 
                                                                     y: self.kalmanCoreData_est.state.m[KC_State_D1][0], 
                                                                     z: self.kalmanCoreData_est.state.m[KC_State_D2][0] 
                                                                    };                                              
            let g: my_math::Vector3 = my_math::Vector3 { x: 0.0, y: 0.0, z: -GRAVITY_MAGNITUDE };
            let og: my_math::Vector3 = my_math::Vector3 { x: gyro.x, y: gyro.y, z: gyro.z};
            let oa: my_math::Vector3 = my_math::Vector3 { x: acc.x, y: acc.y, z: acc.z };
            /*
            println!("oa: {:?}", my_math::circle_dot((self.kalmanCoreData_est.q).conjugate(), g) + oa);
            println!("og: {:?}", og);
            println!("dt: {:?}", dt);
            println!("p_ref: {:?}", p_ref);
            println!("b_ref: {:?}", b_ref);
            */
             

            let p_next: my_math::Vector3 = p_ref + my_math::circle_dot(self.kalmanCoreData_est.q, b_ref) * dt;
            let b_next: my_math::Vector3 = b_ref + (my_math::circle_dot((self.kalmanCoreData_est.q).conjugate(), g) + oa) * dt;
            let delta_next: my_math::Vector3 = delta_vec_ref + og * dt;


            self.kalmanCoreData_est.state.m[KC_State_X][0] = p_next.x;
            self.kalmanCoreData_est.state.m[KC_State_Y][0] = p_next.y;
            self.kalmanCoreData_est.state.m[KC_State_Z][0] = p_next.z;

            self.kalmanCoreData_est.state.m[KC_State_Px][0] = b_next.x;
            self.kalmanCoreData_est.state.m[KC_State_Py][0] = b_next.y;
            self.kalmanCoreData_est.state.m[KC_State_Pz][0] = b_next.z;

            self.kalmanCoreData_est.state.m[KC_State_D0][0] = delta_next.x;
            self.kalmanCoreData_est.state.m[KC_State_D1][0] = delta_next.y;
            self.kalmanCoreData_est.state.m[KC_State_D2][0] = delta_next.z;

            // ===== update the quaternion using the prediction value =====
            // let angle = (og*dt).norm() + 1e-6;
            // let ca: f32 = libm::cosf(angle/2.0);
            // let sa = libm::sinf(angle/2.0);
            // let dq: my_math::Quaternion = my_math::Quaternion { qw: ca, qx: sa*(og*dt).x/angle, qy: sa*(og*dt).y/angle, qz: sa*(og*dt).z/angle };

            // self.kalmanCoreData_est.q = (1.0/(self.kalmanCoreData_est.q * dq).norm()) * (self.kalmanCoreData_est.q * dq);
            // self.kalmanCoreData_est.R = my_math::quaternion_to_matrix_mat(self.kalmanCoreData_est.q);

            self.kalmanCoreData_est.isUpdated = true;
            
        }
    }


    
    // This function is doing "+ R" part of "P_next = GPG' + R" 
    pub fn addProcessNoiseDt(&mut self, t: f32){
        // let dt = t - self.kalmanCoreData_est.lastProcessNoiseUpdateMs;
        let dt = 0.0005;
        println!("dt_cov: {:?}", dt);
        self.kalmanCoreData_est.Pm.m[KC_State_X][KC_State_X] += libm::powf(self.kalmanCoreParams_est.procNoiseAcc_xy*dt*dt + self.kalmanCoreParams_est.procNoiseVel*dt + self.kalmanCoreParams_est.procNoisePos, 2.0);  // add process noise on position
        self.kalmanCoreData_est.Pm.m[KC_State_Y][KC_State_Y] += libm::powf(self.kalmanCoreParams_est.procNoiseAcc_xy*dt*dt + self.kalmanCoreParams_est.procNoiseVel*dt + self.kalmanCoreParams_est.procNoisePos, 2.0);  // add process noise on position
        self.kalmanCoreData_est.Pm.m[KC_State_Z][KC_State_Z] += libm::powf(self.kalmanCoreParams_est.procNoiseAcc_z*dt*dt + self.kalmanCoreParams_est.procNoiseVel*dt + self.kalmanCoreParams_est.procNoisePos, 2.0);   // add process noise on position

        self.kalmanCoreData_est.Pm.m[KC_State_Px][KC_State_Px] += libm::powf(self.kalmanCoreParams_est.procNoiseAcc_xy*dt + self.kalmanCoreParams_est.procNoiseVel, 2.0); // add process noise on velocity
        self.kalmanCoreData_est.Pm.m[KC_State_Py][KC_State_Py] += libm::powf(self.kalmanCoreParams_est.procNoiseAcc_xy*dt + self.kalmanCoreParams_est.procNoiseVel, 2.0); // add process noise on velocity
        self.kalmanCoreData_est.Pm.m[KC_State_Pz][KC_State_Pz] += libm::powf(self.kalmanCoreParams_est.procNoiseAcc_z*dt + self.kalmanCoreParams_est.procNoiseVel, 2.0);  // add process noise on velocity

        self.kalmanCoreData_est.Pm.m[KC_State_D0][KC_State_D0] += libm::powf(self.kalmanCoreParams_est.measNoiseGyro_rollpitch * dt + self.kalmanCoreParams_est.procNoiseAtt, 2.0);
        self.kalmanCoreData_est.Pm.m[KC_State_D1][KC_State_D1] += libm::powf(self.kalmanCoreParams_est.measNoiseGyro_rollpitch * dt + self.kalmanCoreParams_est.procNoiseAtt, 2.0);
        self.kalmanCoreData_est.Pm.m[KC_State_D2][KC_State_D2] += libm::powf(self.kalmanCoreParams_est.measNoiseGyro_yaw * dt + self.kalmanCoreParams_est.procNoiseAtt, 2.0);


        // ===== Guarantee the Symmetry for the Predicted Covariance Matrix
        
        for i in 0..KC_State_Dim {
            for j in i..KC_State_Dim {
                let mut p = 0.5 * self.kalmanCoreData_est.Pm.m[i][j] + 0.5 * self.kalmanCoreData_est.Pm.m[j][i];
                
                if p.is_nan() || p > MAX_COVARIANCE {
                    p = MAX_COVARIANCE;
                } else if i == j && p < MIN_COVARIANCE {
                    p = MIN_COVARIANCE;
                }

                self.kalmanCoreData_est.Pm.m[i][j] = p;
                self.kalmanCoreData_est.Pm.m[j][i] = p;
            }
        }
        self.kalmanCoreData_est.lastProcessNoiseUpdateMs = t;
        
    }



    pub fn kalmanCorePredict(&mut self, acc: my_math::Vector3, gyro: my_math::Vector3, nowMs: f32){
        // let dt = ((nowMs - self.kalmanCoreData_est.lastPredictionMs) as f32) / 1000.0;
        let dt = nowMs - self.kalmanCoreData_est.lastPredictionMs;
        self.predictDt(dt, acc, gyro);
        self.kalmanCoreData_est.lastPredictionMs = nowMs;
    }


    // Kalman Filter Scalar Update
    // Design to get rid of the matrix inverse computation. We use row by row update instead.
    pub fn kalmanCoreScalarUpdate(&mut self, Hm: my_math::Mat<1, 9>, error: f32, std_meas_noise: f32){

        // ===== PH' Calculation =====
        let HmT = Hm.transpose();
        let PHmT = self.kalmanCoreData_est.Pm * HmT;
        
        // ===== HPH' + R Calculation =====
        let Q: f32 = std_meas_noise * std_meas_noise;
        let mut HPH_Q = Q;
        for i in 0..KC_State_Dim {
            HPH_Q += Hm.m[0][i] * PHmT.m[i][0]; // This is only possible when we do scalar update
        }
        assert!(!HPH_Q.is_nan(), "HPH_Q is NaN!");

        // ===== State Update =====
        let mut K = my_math::Mat{m: [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]};
        for i in 0..KC_State_Dim {
            K.m[i][0] = PHmT.m[i][0] / HPH_Q;
            self.kalmanCoreData_est.state.m[i][0] += K.m[i][0] * error;  // The error here is (real measurement - predicted measurement) 
        }

        // ===== Covariance Update =====
        // ===== Here we use Joseph-Form Update to guarantee the symmetricity of the covariance =====
        // ===== The update function is written as below =====
        // ===== P_k = (I - KH)*P_(k-1)*(I - KH)' + K*Q*K' =====
        
        let KH = K * (Hm);
        
        let KH_minus_I = KH.sub_identity();
        let tmpNN1 = KH_minus_I * self.kalmanCoreData_est.Pm;
        let tmpNN2 = tmpNN1 * KH_minus_I.transpose();
        self.kalmanCoreData_est.Pm = tmpNN2;
        
        
        /*
        let mut Identity: my_math::Mat<9, 9> = my_math::Mat::new();
        for i in 0..KC_State_Dim {
            Identity.m[i][i] = 1.0;
        }
        let I_minus_KH = Identity - KH;
        self.kalmanCoreData_est.Pm = I_minus_KH * self.kalmanCoreData_est.Pm;
        */

        // ===== Update covariance and ensure boundedness =====
        
        for i in 0..KC_State_Dim {
            for j in i..KC_State_Dim {
                let v = K.m[i][0] * Q * K.m[j][0];
                let mut p = 0.5 * self.kalmanCoreData_est.Pm.m[i][j] + 0.5 * self.kalmanCoreData_est.Pm.m[j][i] + v;
                
                if p.is_nan() || p > MAX_COVARIANCE {
                    p = MAX_COVARIANCE;
                } else if i == j && p < MIN_COVARIANCE {
                    p = MIN_COVARIANCE;
                }

                self.kalmanCoreData_est.Pm.m[i][j] = p;
                self.kalmanCoreData_est.Pm.m[j][i] = p;
            }
        }
        

        self.kalmanCoreData_est.isUpdated = true;

    }


    // Finalize all value after one prediction and correction step
    pub fn kalmanCoreFinalize(&mut self){
        // ===== Update Quaternion =====
        let delta_vec_ref: my_math::Vector3 = my_math::Vector3 { x: self.kalmanCoreData_est.state.m[KC_State_D0][0], 
                                                                 y: self.kalmanCoreData_est.state.m[KC_State_D1][0], 
                                                                 z: self.kalmanCoreData_est.state.m[KC_State_D2][0] 
                                                                };                                                   
        let q_delta_ref: my_math::Quaternion = my_math::Quaternion { qw: libm::cosf(delta_vec_ref.norm()/2.0),
                                                                     qx: delta_vec_ref.x * (libm::sinf(delta_vec_ref.norm()/2.0)/(delta_vec_ref.norm()+1e-6)),
                                                                     qy: delta_vec_ref.y * (libm::sinf(delta_vec_ref.norm()/2.0)/(delta_vec_ref.norm()+1e-6)),
                                                                     qz: delta_vec_ref.z * (libm::sinf(delta_vec_ref.norm()/2.0)/(delta_vec_ref.norm()+1e-6)) 
                                                                    };                                                            
        self.kalmanCoreData_est.q = (1.0/(self.kalmanCoreData_est.q*q_delta_ref).norm()) * (self.kalmanCoreData_est.q * q_delta_ref);
        
        // ===== Update Rotation Matrix =====
        self.kalmanCoreData_est.R = my_math::quaternion_to_matrix_mat(self.kalmanCoreData_est.q);
    }


    // Measurement model where the measurement is the absolute height
    pub fn kalmanCoreUpdateWithAbsoluteHeight(&mut self, height: f32){
        let mut H: my_math::Mat<1, KC_State_Dim> = my_math::Mat{m: [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]};
        H.m[0][KC_State_Z] = 1.0;

        let error_height = height - self.kalmanCoreData_est.state.m[KC_State_Z][0];

        // ===== Compute the varying standard deviation =====
        let expCoeff: f32 = libm::logf(expStdB / expStdA) / (expPointB - expPointA);
        let distance: f32 = height;
        let stdDev_height: f32 = expStdA * (1.0  + libm::expf( expCoeff * (distance - expPointA)));

        self.kalmanCoreScalarUpdate(H, error_height, stdDev_height);
    }


    // Measurement model where the measurement is the absolute height
    pub fn kalmanCoreUpdateWithToF(&mut self, height: f32){
        let mut H: my_math::Mat<1, KC_State_Dim> = my_math::Mat{m: [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]};

        if (libm::fabsf(self.kalmanCoreData_est.R.m[2][2])) > 0.1 && (self.kalmanCoreData_est.R.m[2][2] > 0.0) {
            let mut angle: f32 = libm::fabsf(libm::acosf(self.kalmanCoreData_est.R.m[2][2])) - DEG_TO_GRAD * (15.0 / 2.0);
            if angle < 0.0 {
                angle = 0.0;
            }
        
            H.m[0][KC_State_Z] = 1.0 / libm::cosf(angle);

            let error_height = height - self.kalmanCoreData_est.state.m[KC_State_Z][0]/libm::cosf(angle);

            // ===== Compute the varying standard deviation =====
            let expCoeff: f32 = libm::logf(expStdB / expStdA) / (expPointB - expPointA);
            let distance: f32 = height;
            let stdDev_height: f32 = expStdA * (1.0  + libm::expf( expCoeff * (distance - expPointA)));

            self.kalmanCoreScalarUpdate(H, error_height, stdDev_height);
        }
    }


    // Measurement model where the measurement is the Optical Flow
    pub fn kalmanCoreUpdateWithFlow(&mut self, nowMs: f32, flow: my_math::Vector3){
        // ===== define some constant parameters for the velocity estimation =====
        // let dt: f32 = ((nowMs - self.kalmanCoreData_est.lastFlowUpdate) as f32) / 1000.0;
        let dt: f32 = nowMs - self.kalmanCoreData_est.lastFlowUpdate;
        println!("dt: {:?}", dt);

        let Npix: f32 = 350.0;
        let thetapix: f32 = 0.71674;

        
        // ===== Saturate elevation in prediction and correction to avoid singularities =====
        let z_g: f32;
        if self.kalmanCoreData_est.state.m[KC_State_Z][0] < 0.1 
        {
            z_g = 0.1;
        }
        else
        {
            z_g = self.kalmanCoreData_est.state.m[KC_State_Z][0];
        }

        // ===== X velocity prediction and update =====
        // ===== 1. Predict the number if accumulated pixels in the X-direction =====
        let mut Hx = my_math::Mat{m: [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]};
        let predictedNX: f32 = ((dt * Npix) / (thetapix*z_g)) * self.kalmanCoreData_est.state.m[KC_State_Px][0];
        let measuredNX: f32 = flow.x;

        // ===== 2. Derive measurement equation w.r.t dx =====
        Hx.m[0][KC_State_Z] = -(Npix * dt * self.kalmanCoreData_est.state.m[KC_State_Px][0]) / (thetapix*(z_g * z_g));
        Hx.m[0][KC_State_Px] = (Npix * dt) / (thetapix * z_g);

        // ===== 3. Compute the error =====
        let error_Px: f32 = measuredNX - predictedNX;
        println!("error x: {:?}", error_Px);

        // ===== 4. Update =====
        self.kalmanCoreScalarUpdate(Hx, error_Px, flow.z*FLOW_RESOLUTION);
        // self.kalmanCoreData_est.state.m[3][0] = ((thetapix*z_g) / (dt * Npix)) * measuredNX;


        // ===== Y velocity prediction and update =====
        // ===== 1. Predict the number if accumulated pixels in the X-direction =====
        let mut Hy = my_math::Mat{m: [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]};
        let predictedNY: f32 = ((dt * Npix) / (thetapix*z_g)) * self.kalmanCoreData_est.state.m[KC_State_Py][0];
        let measuredNY: f32 = flow.y;

        // ===== 2. Derive measurement equation w.r.t dy =====
        Hy.m[0][KC_State_Z] = -(Npix * dt * self.kalmanCoreData_est.state.m[KC_State_Py][0]) / (thetapix*(z_g * z_g));
        Hy.m[0][KC_State_Py] = (Npix * dt) / (thetapix * z_g);

        // ===== 3. Compute the error =====
        let error_Py: f32 = measuredNY - predictedNY;
        println!("error y: {:?}", error_Py);

        // ===== 4. Update =====
        self.kalmanCoreScalarUpdate(Hy, error_Py, flow.z*FLOW_RESOLUTION);
        // self.kalmanCoreData_est.state.m[4][0] = ((thetapix*z_g) / (dt * Npix)) * flow.y;

        self.kalmanCoreData_est.lastFlowUpdate = nowMs;

        
    }

}
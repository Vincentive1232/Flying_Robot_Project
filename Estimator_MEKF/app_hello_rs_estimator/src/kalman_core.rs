#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]

extern crate libm;

use crate::math::{self as my_math};
use crate::estimator_bindings;

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
pub const GRAD_TO_DEG: f32 = 52.295;
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



// ===== Kalman Filter Estimator Params, including noise level. =====
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


#[derive(Debug)]
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


    // Kalman Filter Parameters Initialization (Zero Initial State)
    pub fn kalmanCoreInit_Onboard() -> Self {
        let mut MEKF_Estimator = kalmanCoreEstimator::new();

        // ===== Initialize the Data/States of the Kalman Filter(Initialize to Groundtruth) =====
        // MEKF_Estimator.kalmanCoreData_est.state = Initial_State;
        MEKF_Estimator.kalmanCoreData_est.state = my_math::Mat{m: [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]};

        // ===== reset the attitude quaternion =====
        MEKF_Estimator.kalmanCoreData_est.initialQuaternion = my_math::Quaternion{
            qw: libm::cosf(MEKF_Estimator.kalmanCoreParams_est.initialYaw / 2.0),
            qx: 0.0,
            qy: 0.0,
            qz: libm::cosf(MEKF_Estimator.kalmanCoreParams_est.initialYaw / 2.0)
        };

        // ===== Set the initial rotation matrix as an identity matrix =====
        MEKF_Estimator.kalmanCoreData_est.R = my_math::Mat{m:[[1.0, 0.0, 0.0], 
                                                              [0.0, 1.0, 0.0], 
                                                              [0.0, 0.0, 1.0]]};
        
        
        // ===== Initialize the diagonal value of the state covariance matrix =====
        MEKF_Estimator.kalmanCoreData_est.Pm = my_math::Mat::new();

        // ===== Position Initialization =====
        MEKF_Estimator.kalmanCoreData_est.Pm.m[KC_State_X][KC_State_X] = libm::powf(MEKF_Estimator.kalmanCoreParams_est.stdDevInitialPosition_xy, 2.0);
        MEKF_Estimator.kalmanCoreData_est.Pm.m[KC_State_Y][KC_State_Y] = libm::powf(MEKF_Estimator.kalmanCoreParams_est.stdDevInitialPosition_xy, 2.0);
        MEKF_Estimator.kalmanCoreData_est.Pm.m[KC_State_Z][KC_State_Z] = libm::powf(MEKF_Estimator.kalmanCoreParams_est.stdDevInitialPosition_z, 2.0);  
        
        // ===== Velocity Initialization =====
        MEKF_Estimator.kalmanCoreData_est.Pm.m[KC_State_Px][KC_State_Px] = libm::powf(MEKF_Estimator.kalmanCoreParams_est.stdDevInitialVelocity, 2.0);
        MEKF_Estimator.kalmanCoreData_est.Pm.m[KC_State_Py][KC_State_Py] = libm::powf(MEKF_Estimator.kalmanCoreParams_est.stdDevInitialVelocity, 2.0);
        MEKF_Estimator.kalmanCoreData_est.Pm.m[KC_State_Pz][KC_State_Pz] = libm::powf(MEKF_Estimator.kalmanCoreParams_est.stdDevInitialVelocity, 2.0);

        // ===== Error Delta Initialization =====
        MEKF_Estimator.kalmanCoreData_est.Pm.m[KC_State_D0][KC_State_D0] = libm::powf(MEKF_Estimator.kalmanCoreParams_est.stdDevInitialAttitude_rollpitch, 2.0);
        MEKF_Estimator.kalmanCoreData_est.Pm.m[KC_State_D1][KC_State_D1] = libm::powf(MEKF_Estimator.kalmanCoreParams_est.stdDevInitialAttitude_rollpitch, 2.0);
        MEKF_Estimator.kalmanCoreData_est.Pm.m[KC_State_D2][KC_State_D2] = libm::powf(MEKF_Estimator.kalmanCoreParams_est.stdDevInitialAttitude_yaw, 2.0);
        
        // Initialize the Process Noise Covariance as an all zero matrix
        MEKF_Estimator.kalmanCoreData_est.Rm = my_math::Mat::new();
        
        MEKF_Estimator.kalmanCoreData_est.baroReferenceHeight = 0.0;

        MEKF_Estimator.kalmanCoreData_est.isUpdated = false;
        MEKF_Estimator.kalmanCoreData_est.lastPredictionMs = 0.0;
        MEKF_Estimator.kalmanCoreData_est.lastProcessNoiseUpdateMs = 0.0;

        MEKF_Estimator
    }


    pub fn kalmanCoreInit_Onboard_new() -> Self {
        kalmanCoreEstimator{
            kalmanCoreData_est: kalmanCoreData{
                state: my_math::Mat::new(),
                q: my_math::Quaternion::new(1.0, 0.0, 0.0, 0.0),
                R: my_math::Mat { m: ([[1.0, 0.0, 0.0],
                                       [0.0, 1.0, 0.0],
                                       [0.0, 0.0, 1.0]]) },                    
                Pm: my_math::Mat { m: ([[libm::powf(0.1, 2.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, libm::powf(0.1, 2.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, libm::powf(0.1, 2.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, libm::powf(0.01, 2.0), 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, libm::powf(0.01, 2.0), 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, libm::powf(0.01, 2.0), 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, libm::powf(0.01, 2.0), 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, libm::powf(0.01, 2.0), 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, libm::powf(0.01, 2.0)]])
                                        },                   
                Rm: my_math::Mat::new(),                   
                Qm: my_math::Mat::new(),                   
                baroReferenceHeight: 0.0,
                initialQuaternion: my_math::Quaternion::new(1.0, 0.0, 0.0, 0.0),
                isUpdated: false,                 
                lastPredictionMs: 0.0,              
                lastUpdateMs: 0.0,
                lastFlowUpdate: 0.0,                  
                lastProcessNoiseUpdateMs: 0.0,   
            },
            kalmanCoreParams_est: kalmanCoreParams{
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


    pub fn set_Quat_and_R(&mut self, q: my_math::Quaternion) {
        self.kalmanCoreData_est.q = q;
        self.kalmanCoreData_est.R = my_math::quaternion_to_matrix_mat(q);
    }


    // Kalman Filter Prediction Step
    // Design predict the next state and push forward the covariance
    pub fn predictDt(&mut self, dt: f32, acc: my_math::Vector3, gyro: my_math::Vector3) {
        /* Here we discretize (euler forward) and linearise the quadrocopter dynamics in order
         * to push the covariance forward.
         *
         * QUADROCOPTER DYNAMICS (see paper):
         *
         * \dot{x} = R(I + [[d]])p
         * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 //drag negligible
         * \dot{d} = \omega
         *
         * where [[.]] is the cross-product matrix of .
         *       \omega are the gyro measurements
         *       e3 is the column vector [0 0 1]'
         *       I is the identity
         *       R is the current attitude as a rotation matrix
         *       f/m is the mass-normalized motor force (acceleration in the body's z direction)
         *       g is gravity
         *       x, p, d are the quad's states
         * note that d (attitude error) is zero at the beginning of each iteration,
         * since error information is incorporated into R after each Kalman update.
         */

        // ===== Define the linearized dynamics matrix as an all zero matrix =====
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


    
    // This function is doing "+ R" part of "P_next = GPG' + R" 
    pub fn addProcessNoiseDt(&mut self, dt: f32){
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
    }



    pub fn kalmanCorePredict(&mut self, acc: my_math::Vector3, gyro: my_math::Vector3, dt: f32){
        // let dt = ((nowMs - self.kalmanCoreData_est.lastPredictionMs) as f32) / 1000.0;
        // let dt = nowMs - self.kalmanCoreData_est.lastPredictionMs;
        self.predictDt(dt, acc, gyro);
        // self.kalmanCoreData_est.lastPredictionMs = nowMs;
    }
    
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
    pub fn kalmanCoreUpdateWithAbsoluteHeight(&mut self, height: estimator_bindings::tofMeasurement_s){
        let mut H: my_math::Mat<1, KC_State_Dim> = my_math::Mat{m: [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]};
        H.m[0][KC_State_Z] = 1.0;

        let height_in_m = height.distance;
        let error_height = height_in_m - self.kalmanCoreData_est.state.m[KC_State_Z][0];

        // ===== Compute the varying standard deviation =====
        let expCoeff: f32 = libm::logf(expStdB / expStdA) / (expPointB - expPointA);
        let stdDev_height: f32 = expStdA * (1.0  + libm::expf( expCoeff * (height_in_m - expPointA)));

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
    pub fn kalmanCoreUpdateWithFlow(&mut self, flow: estimator_bindings::flowMeasurement_s){
        unsafe {
            // ===== define some constant parameters for the velocity estimation =====
            // let dt: f32 = ((nowMs - self.kalmanCoreData_est.lastFlowUpdate) as f32) / 1000.0;
            let dt: f32 = 0.01;
            // let dt: f32 = flow.dt;

            let Npix: f32 = 350.0;
            let thetapix: f32 = 0.71674;

            // ===== change the units of the IMU measurements =====
            // let omegaX_b: f32 = gyro.x * DEG_TO_GRAD;
            // let omegaY_b: f32 = gyro.y * DEG_TO_GRAD;


            // let dx_g: f32 = self.kalmanCoreData_est.state.m[KC_State_Px][0];
            // let dy_g: f32 = self.kalmanCoreData_est.state.m[KC_State_Py][0];
            let z_g: f32;
            // ===== Saturate elevation in prediction and correction to avoid singularities =====
            if self.kalmanCoreData_est.state.m[KC_State_Z][0] < 0.1 
            {
                z_g = 0.1;
            }
            else{
                z_g = self.kalmanCoreData_est.state.m[KC_State_Z][0];
            }

            // ===== X velocity prediction and update =====
            // ===== 1. Predict the number if accumulated pixels in the X-direction =====
            let mut Hx = my_math::Mat{m: [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]};
            let predictedNX: f32 = ((dt * Npix) / (thetapix*z_g)) * self.kalmanCoreData_est.state.m[KC_State_Px][0];
            let mut measuredNX: f32 = flow.__bindgen_anon_1.__bindgen_anon_1.dpixelx.into();
            measuredNX = measuredNX;

            // ===== 2. Derive measurement equation w.r.t dx =====
            Hx.m[0][KC_State_Z] = -(Npix * dt * self.kalmanCoreData_est.state.m[KC_State_Px][0]) / (thetapix*(z_g * z_g));
            Hx.m[0][KC_State_Px] = (Npix * dt) / (thetapix * z_g);

            // ===== 3. Compute the error =====
            let error_Px: f32 = measuredNX - predictedNX;

            // ===== 4. Update =====
            let stdDevX: f32 = flow.stdDevX.into();
            self.kalmanCoreScalarUpdate(Hx, error_Px, 0.2);


            // ===== Y velocity prediction and update =====
            // ===== 1. Predict the number if accumulated pixels in the X-direction =====
            let mut Hy = my_math::Mat{m: [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]};
            let predictedNY: f32 = ((dt * Npix) / (thetapix*z_g)) * self.kalmanCoreData_est.state.m[KC_State_Py][0];
            // let predictedNY: f32 = (dt * Npix / thetapix) * ((dy_g * self.kalmanCoreData_est.R.m[2][2] / z_g) - omegaX_b);
            let mut measuredNY: f32 = flow.__bindgen_anon_1.__bindgen_anon_1.dpixely.into();
            measuredNY = measuredNY;

            // ===== 2. Derive measurement equation w.r.t dy =====
            // Hy.m[KC_State_Z][0] = (Npix * dt / thetapix) * ((self.kalmanCoreData_est.R.m[2][2] * dy_g) / (-z_g * z_g));
            // Hy.m[KC_State_Py][0] = (Npix * dt / thetapix) * (self.kalmanCoreData_est.R.m[2][2] / z_g);
            Hy.m[0][KC_State_Z] = -(Npix * dt * self.kalmanCoreData_est.state.m[KC_State_Py][0]) / (thetapix*(z_g * z_g));
            Hy.m[0][KC_State_Py] = (Npix * dt) / (thetapix * z_g);

            // ===== 3. Compute the error =====
            let error_Py: f32 = measuredNY - predictedNY;

            // ===== 4. Update =====
            let stdDevY: f32 = flow.stdDevY.into();
            self.kalmanCoreScalarUpdate(Hy, error_Py, 0.2);

            // self.kalmanCoreData_est.lastFlowUpdate = nowMs;
        }
    }


    pub fn set_estimate_state_val(&mut self, state: *mut estimator_bindings::state_t, acc_meas: my_math::Vector3) {
        unsafe {
            let state_estimate = &mut *state;
            state_estimate.position = estimator_bindings::vec3_s {
                timestamp: 0,
                x: self.kalmanCoreData_est.state.m[0][0],
                y: self.kalmanCoreData_est.state.m[1][0],
                z: self.kalmanCoreData_est.state.m[2][0]
            };
            let mut world_vel: my_math::Vector3 = my_math::Vector3::new(self.kalmanCoreData_est.state.m[3][0], self.kalmanCoreData_est.state.m[4][0], self.kalmanCoreData_est.state.m[5][0]);
            world_vel = my_math::circle_dot(self.kalmanCoreData_est.q, world_vel);
            state_estimate.velocity = estimator_bindings::vec3_s {
                timestamp: 0,
                x: world_vel.x,
                y: world_vel.y,
                z: world_vel.z
            };
            state_estimate.attitudeQuaternion = estimator_bindings::quaternion_t {
                __bindgen_anon_1: estimator_bindings::quaternion_s__bindgen_ty_1 {
                    __bindgen_anon_2: estimator_bindings::quaternion_s__bindgen_ty_1__bindgen_ty_2 {
                        x: self.kalmanCoreData_est.q.qx,
                        y: self.kalmanCoreData_est.q.qy,
                        z: self.kalmanCoreData_est.q.qz,
                        w: self.kalmanCoreData_est.q.qw,
                    }
                }
            };
            let mut rpy_estimate = my_math::quaternion_to_rpy(self.kalmanCoreData_est.q);
            rpy_estimate = rpy_estimate * GRAD_TO_DEG;
            state_estimate.attitude = estimator_bindings::attitude_s {
                timestamp: 0,
                roll: rpy_estimate.x,
                pitch: -rpy_estimate.y,
                yaw: rpy_estimate.z
            };
            state_estimate.acc = estimator_bindings::vec3_s {
                timestamp: 0,
                x: acc_meas.x,
                y: acc_meas.y,
                z: acc_meas.z
            };
        }
    }

}
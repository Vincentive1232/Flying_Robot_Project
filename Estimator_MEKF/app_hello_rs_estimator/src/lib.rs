#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use panic_halt as _;
use core::cell::UnsafeCell;

pub mod math;
pub mod stabilizer_types;
pub mod estimator_bindings;
pub mod myController_lee;
pub mod kalman_core;

use math::{self as my_math};
use estimator_bindings::{control_t, sensorData_t, setpoint_t, stabilizerStep_t, state_t};
use myController_lee::controller_Lee;
use kalman_core::kalmanCoreEstimator;


// Define a global controller variable
static mut Controller: UnsafeCell<Option<controller_Lee>> = UnsafeCell::new(None);

// Define a global estimator variable
static mut Estimator: UnsafeCell<Option<kalmanCoreEstimator>>= UnsafeCell::new(None);


extern "C" {
    pub fn vTaskDelay(ticks: u32);
    pub fn consolePutchar(ch: i32) -> i32;
}

fn console_print(msg: &str) {
    for c in msg.as_bytes() {
        unsafe{ consolePutchar(*c as i32); }
    }
}


#[no_mangle]
pub extern "C" fn appMain() -> i32 {
    console_print("Hello from CF2 Estimator!\n");

    loop {
        unsafe { vTaskDelay(1000); }
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTreeInit() {
    unsafe {
        let controller_ptr = Controller.get();
        *controller_ptr = Some(controller_Lee::controller_Lee_Init());
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTreeTest() -> bool {
    return true
}


#[no_mangle]
pub unsafe extern "C" fn controllerOutOfTree (
    control: *mut control_t,
    setpoint: *const setpoint_t,
    sensors: *const sensorData_t,
    state: *const state_t,
    tick: u32
) {
    unsafe {
        if let Some(ref mut controller) = *Controller.get() {
            controller.controller_Lee(control, setpoint, sensors, state, tick);
        }
        
    }
}


#[no_mangle]
pub extern "C" fn estimatorOutOfTreeInit() {
    unsafe {
        let estimator_ptr = Estimator.get();
        *estimator_ptr = Some(kalmanCoreEstimator::kalmanCoreInit_Onboard_new());
    }
}

#[no_mangle]
pub extern "C" fn estimatorOutOfTreeTest() -> bool {
    return true
}


#[no_mangle]
pub unsafe extern "C" fn estimatorOutOfTree (
    state: *mut state_t, 
    stabilizerStep: stabilizerStep_t 
) {
    unsafe {

        let mut acc_valid_flag: bool = false;
        let mut gyro_valid_flag: bool = false;
        let mut acc_meas: my_math::Vector3 = my_math::Vector3::new(0.0, 0.0, 0.0);
        let mut gyro_meas: my_math::Vector3 = my_math::Vector3::new(0.0, 0.0, 0.0);

        let mut measurement = estimator_bindings::measurement_t {
            type_: 255,
            data: estimator_bindings::measurement_t__bindgen_ty_1 {
                gyroscope: estimator_bindings::gyroscopeMeasurement_t {
                    gyro: estimator_bindings::Axis3f {
                            axis: [0.0, 0.0, 0.0]
                    }
                },
            }
        };

        if let Some(ref mut estimator) = *Estimator.get() 
        {   
            let state_estimate = &mut *state;
            /* 
            estimator.kalmanCoreData_est.state.m[0][0] = state_estimate.position.x;
            estimator.kalmanCoreData_est.state.m[1][0] = state_estimate.position.y;
            estimator.kalmanCoreData_est.state.m[2][0] = state_estimate.position.z;

            estimator.kalmanCoreData_est.q.qx = state_estimate.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_2.x;
            estimator.kalmanCoreData_est.q.qy = state_estimate.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_2.y;
            estimator.kalmanCoreData_est.q.qz = state_estimate.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_2.z;
            estimator.kalmanCoreData_est.q.qw = state_estimate.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_2.w;

            let mut body_vel: my_math::Vector3 = my_math::Vector3::new(state_estimate.velocity.x, state_estimate.velocity.y, state_estimate.velocity.z);
            body_vel = my_math::circle_dot(estimator.kalmanCoreData_est.q.conjugate(), body_vel);
            estimator.kalmanCoreData_est.state.m[3][0] = body_vel.x;
            estimator.kalmanCoreData_est.state.m[4][0] = body_vel.y;
            estimator.kalmanCoreData_est.state.m[5][0] = body_vel.z;

            estimator.set_Quat_and_R(estimator.kalmanCoreData_est.q);
            */

            loop {
                let ok = estimator_bindings::estimatorDequeue(&mut measurement);

                if !ok { break;}

                let t = measurement.type_ & 0xFF;

                if t == estimator_bindings::MeasurementType_MeasurementTypeTDOA {
                    continue;
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypePosition {
                    continue;
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypePose {
                    continue;
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypeDistance {
                    continue;
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypeTOF {
                    estimator.kalmanCoreUpdateWithAbsoluteHeight(measurement.data.tof);
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypeAbsoluteHeight {
                    continue;
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypeFlow {
                    estimator.kalmanCoreUpdateWithFlow(measurement.data.flow);
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypeYawError {
                    continue;
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypeSweepAngle {
                    continue;
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypeGyroscope {
                    gyro_valid_flag = true;
                    gyro_meas = my_math::Vector3::new(measurement.data.gyroscope.gyro.__bindgen_anon_1.x.into(), 
                                                      measurement.data.gyroscope.gyro.__bindgen_anon_1.y.into(), 
                                                      measurement.data.gyroscope.gyro.__bindgen_anon_1.z.into()
                                                    );
                    gyro_meas = gyro_meas * kalman_core::DEG_TO_GRAD;
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypeAcceleration {
                    acc_valid_flag = true;
                    acc_meas = my_math::Vector3::new(measurement.data.acceleration.acc.__bindgen_anon_1.x.into(), 
                                                     measurement.data.acceleration.acc.__bindgen_anon_1.y.into(), 
                                                     measurement.data.acceleration.acc.__bindgen_anon_1.z.into()
                                                    );
                    acc_meas = acc_meas * kalman_core::GRAVITY_MAGNITUDE;
                }
                else if t == estimator_bindings::MeasurementType_MeasurementTypeBarometer {
                    continue;                 
                }
                else {
                    continue;
                }

                if acc_valid_flag && gyro_valid_flag {
                    estimator.kalmanCoreFinalize();
                    estimator.set_estimate_state_val(state, acc_meas);
                    estimator.kalmanCorePredict(acc_meas, gyro_meas, 0.001);
                    acc_valid_flag = false;
                    gyro_valid_flag = false;
                }
                
                // estimator.addProcessNoiseDt(0.001); // Here is the biggest change
            }
        }
    }
}

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]

use std::error::Error;
use std::fs::File;
// use csv::Writer;
use kalman_core::kalmanCoreEstimator;
use std::io::Write;
use csv::ReaderBuilder;

pub mod math;
pub mod stabilizer_types;
pub mod kalman_core;

use crate::math as my_math;

fn main() -> Result<(), Box<dyn Error>> {

    let path = "data/output.txt";
    let mut file_out = File::create(path).ok().unwrap();

    let path_gt = "data/groundtruth.txt";
    let mut file_out_gt = File::create(path_gt).ok().unwrap();

    /*
    let path_vis = "data/output.csv";
    let file_vis = File::create(path_vis)?;
    let mut wtr_vis = Writer::from_writer(file_vis);
    */

    // open csv flight data file
    let file = File::open("data/figure8.csv")?;

    // create csv file reader
    let mut rdr = ReaderBuilder::new()
        .delimiter(b',')
        .from_reader(file);

    // let mut timestamps: Vec<f32> = Vec::new();
    let mut ts: Vec<f32> = Vec::new();
    let mut accels: Vec<my_math::Vector3> = Vec::new();
    let mut gyros: Vec<my_math::Vector3> = Vec::new();
    let mut motions: Vec<my_math::Vector3> = Vec::new();
    let mut ranges: Vec<f32> = Vec::new();
    let mut estimate_quaternions: Vec<my_math::Quaternion> = Vec::new();
    let mut estimate_vels: Vec<my_math::Vector3> = Vec::new();
    let mut estimate_positions: Vec<my_math::Vector3> = Vec::new();
    let _pi: f32 = 3.1415926;
    let mut count: usize = 0;

    let mut gyro_is_valid: bool = false;
    let mut accel_is_valid: bool = false;
    let mut height_is_valid: bool = false;
    let mut flow_is_valid: bool = false;

    // kalmanCoreEstimator Initialization
    // get the initial time and the groundtruth start state
    let mut t_init: f32 = 0.0;
    let mut estimate_quaternion_init: my_math::Quaternion = my_math::Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let mut estimate_vel_init: my_math::Vector3 = my_math::Vector3::new(0.0, 0.0, 0.0);
    let mut estimate_position_init: my_math::Vector3 = my_math::Vector3::new(0.0, 0.0, 0.0);
    for result in rdr.records() {
        let data = result?;
        
        t_init = data[0].parse()?;

        estimate_quaternion_init = my_math::Quaternion {
            qw: data[11].parse()?,
            qx: data[12].parse()?,
            qy: data[13].parse()?,
            qz: data[14].parse()?,
        };

        estimate_vel_init = my_math::Vector3 {
            x: data[15].parse()?,
            y: data[16].parse()?,
            z: data[17].parse()?,
        };

        estimate_position_init = my_math::Vector3 {
            x: data[18].parse()?,
            y: data[19].parse()?,
            z: data[20].parse()?,
        };

        if !t_init.is_nan() && !estimate_position_init.x.is_nan() && !estimate_vel_init.x.is_nan() && !estimate_quaternion_init.qw.is_nan() {
            break;
        }
    }
    

    let mut kalmanCoreEstimator_struct = kalmanCoreEstimator::new();
    estimate_vel_init = my_math::circle_dot(estimate_quaternion_init.conjugate(), estimate_vel_init);
    let Initial_State: math::Mat<9, 1> = my_math::Mat{m: [[estimate_position_init.x], [estimate_position_init.y], [estimate_position_init.z],
                                                                        [estimate_vel_init.x], [estimate_vel_init.y], [estimate_vel_init.z],
                                                                        [0.0], [0.0], [0.0]]};
    kalmanCoreEstimator_struct.kalmanCoreInit(t_init, Initial_State);
    kalmanCoreEstimator_struct.set_Quat_and_R(estimate_quaternion_init);
    

    /*
    let quaternion_zero_init = my_math::Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let Initial_State: math::Mat<9, 1> = my_math::Mat{m: [[0.0], [0.0], [0.0],
                                                                        [0.0], [0.0], [0.0],
                                                                        [0.0], [0.0], [0.0]]};
    kalmanCoreEstimator_struct.kalmanCoreInit(t_init, Initial_State);
    kalmanCoreEstimator_struct.set_Quat_and_R(quaternion_zero_init);
    */



    // ===== write the initial estimation in to a txt file =====
    _ = writeln!(file_out, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
    kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[0][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[1][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[2][0],
    kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[3][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[4][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[5][0],
    kalmanCoreEstimator_struct.kalmanCoreData_est.q.qw, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qx, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qy, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qz      
    );
    // ===== write the initial estimation in to a txt file =====

    // ===== write the groundtruth in to a txt file =====
    _ = writeln!(file_out_gt, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
    estimate_position_init.x, estimate_position_init.y, estimate_position_init.z,
    estimate_vel_init.x, estimate_vel_init.y, estimate_vel_init.z,
    estimate_quaternion_init.qw, estimate_quaternion_init.qx, estimate_quaternion_init.qy, estimate_quaternion_init.qz      
    );
    // ===== write the groundtruth in to a txt file =====



    let mut t: f32;
    let mut accel = my_math::Vector3::new(0.0, 0.0, 0.0);
    let mut gyro = my_math::Vector3::new(0.0, 0.0, 0.0);
    let mut motion = my_math::Vector3::new(0.0, 0.0, 0.0);
    let mut height: f32 = 0.0;

    let mut estimate_quaternion = my_math::Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let mut estimate_vel = my_math::Vector3::new(0.0, 0.0,0.0);
    let mut estimate_position = my_math::Vector3::new(0.0, 0.0, 0.0);
    

    for result in rdr.records() {
        let data = result?;
        count += 1;

        // ===== Make Sure the Innitialization is correct =====
        /*
        if count <= 18 {
            continue;
        }
        */

        t = data[0].parse()?;
        ts.push(t);


        let accel_val_check: f32 = data[1].parse()?;
        if !accel_val_check.is_nan() {
            accel = my_math::Vector3 {
                x: data[1].parse()?,
                y: data[2].parse()?,
                z: data[3].parse()?,
            };
            accels.push(accel);
            accel_is_valid = true;
        }
        

        let gyro_val_check: f32 = data[4].parse()?;
        if !gyro_val_check.is_nan() {
            gyro = my_math::Vector3 {
                x: data[4].parse()?,
                y: data[5].parse()?,
                z: data[6].parse()?,
            };
            gyros.push(gyro);
            gyro_is_valid = true;
        }


        let motion_val_check: f32 = data[7].parse()?;
        if !motion_val_check.is_nan() {
            motion = my_math::Vector3 {
                x: data[7].parse()?,
                y: data[8].parse()?,
                z: data[9].parse()?,
            };
            motions.push(motion);
            flow_is_valid = true;
        }


        let height_check: f32 = data[10].parse()?;
        if !height_check.is_nan() {
            height = data[10].parse()?;
            ranges.push(height);
            height_is_valid = true;
        }
        

        let estimate_quaternion_check: f32 = data[11].parse()?;
        if !estimate_quaternion_check.is_nan() {
            estimate_quaternion = my_math::Quaternion {
                qw: data[11].parse()?,
                qx: data[12].parse()?,
                qy: data[13].parse()?,
                qz: data[14].parse()?,
            };
            estimate_quaternions.push(estimate_quaternion);
        }


        let estimate_vel_check: f32 = data[15].parse()?;
        if !estimate_vel_check.is_nan() {
            estimate_vel = my_math::Vector3 {
                x: data[15].parse()?,
                y: data[16].parse()?,
                z: data[17].parse()?,
            };
            estimate_vels.push(estimate_vel);
        }


        let estimate_position_check: f32 = data[18].parse()?;
        if !estimate_position_check.is_nan() {
            estimate_position = my_math::Vector3 {
                x: data[18].parse()?,
                y: data[19].parse()?,
                z: data[20].parse()?,
            };
            estimate_positions.push(estimate_position);
            // ===== write the groundtruth in to a txt file =====
            _ = writeln!(file_out_gt, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
                        estimate_position.x, estimate_position.y, estimate_position.z,
                        estimate_vel.x, estimate_vel.y, estimate_vel.z,
                        estimate_quaternion.qw, estimate_quaternion.qx, estimate_quaternion.qy, estimate_quaternion.qz      
                        );
            // ===== write the groundtruth in to a txt file =====
            
            
            // ===== write the estimation in to a txt file =====
            let body_vel: my_math::Vector3 = my_math::Vector3::new(kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[3][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[4][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[5][0]);
            let world_vel: my_math::Vector3 = my_math::circle_dot(kalmanCoreEstimator_struct.kalmanCoreData_est.q, body_vel);
            _ = writeln!(file_out, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
                        kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[0][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[1][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[2][0],
                        world_vel.x, world_vel.y, world_vel.z,
                        kalmanCoreEstimator_struct.kalmanCoreData_est.q.qw, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qx, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qy, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qz      
                        );
            // ===== write the estimation in to a txt file =====
            
        }


        // ===== MEKF Estimator ===== *kalman_core::GRAVITY_MAGNITUDE
        // ===== Prediction Step =====
        if gyro_is_valid && accel_is_valid {
            // println!("first: {:?}", t);
            // break;
            kalmanCoreEstimator_struct.kalmanCoreFinalize();
            println!("before prediction: {:?}, {:?}, {:?}", kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[0][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[1][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[2][0]);
            kalmanCoreEstimator_struct.kalmanCorePredict(accel*kalman_core::GRAVITY_MAGNITUDE, gyro*kalman_core::DEG_TO_GRAD, t);
            // kalmanCoreEstimator_struct.kalmanCoreFinalize((t*1000.0) as i32);
            gyro_is_valid = false;
            accel_is_valid = false;
            println!("after prediction: {:?}, {:?}, {:?}", kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[0][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[1][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[2][0]);
            // print!("\n");

            /* 
            let body_vel: my_math::Vector3 = my_math::Vector3::new(kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[3][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[4][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[5][0]);
            let world_vel: my_math::Vector3 = my_math::circle_dot(kalmanCoreEstimator_struct.kalmanCoreData_est.q.conjugate(), body_vel);
            _ = writeln!(file_out, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
                        kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[0][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[1][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[2][0],
                        world_vel.x, world_vel.y, world_vel.z,
                        kalmanCoreEstimator_struct.kalmanCoreData_est.q.qw, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qx, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qy, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qz      
                        );
            // ===== write the estimation in to a txt file =====
            */
        }

        // ===== Height Correction Step =====
        if height_is_valid {
            kalmanCoreEstimator_struct.kalmanCoreUpdateWithAbsoluteHeight(height/1000.0);
            // kalmanCoreEstimator_struct.kalmanCoreUpdateWithToF(height/1000.0);
            height_is_valid = false;
            print!("Height Update! \n");

            /* 
            // ===== write the estimation in to a txt file =====
            let body_vel: my_math::Vector3 = my_math::Vector3::new(kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[3][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[4][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[5][0]);
            let world_vel: my_math::Vector3 = my_math::circle_dot(kalmanCoreEstimator_struct.kalmanCoreData_est.q, body_vel);
            _ = writeln!(file_out, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
                        kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[0][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[1][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[2][0],
                        world_vel.x, world_vel.y, world_vel.z,
                        kalmanCoreEstimator_struct.kalmanCoreData_est.q.qw, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qx, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qy, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qz      
                        );
            // ===== write the estimation in to a txt file =====
            */
        }

        // ===== Flow Correction Step =====
        if flow_is_valid {
            kalmanCoreEstimator_struct.kalmanCoreUpdateWithFlow(t, motion);
            // kalmanCoreEstimator_struct.kalmanCoreFinalize((t*1000.0) as i32);
            flow_is_valid = false;
            print!("Flow Update! \n");
            /*
            // ===== write the estimation in to a txt file =====
            let body_vel: my_math::Vector3 = my_math::Vector3::new(kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[3][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[4][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[5][0]);
            let world_vel: my_math::Vector3 = my_math::circle_dot(kalmanCoreEstimator_struct.kalmanCoreData_est.q, body_vel);
            _ = writeln!(file_out, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
                        kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[0][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[1][0], kalmanCoreEstimator_struct.kalmanCoreData_est.state.m[2][0],
                        world_vel.x, world_vel.y, world_vel.z,
                        kalmanCoreEstimator_struct.kalmanCoreData_est.q.qw, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qx, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qy, kalmanCoreEstimator_struct.kalmanCoreData_est.q.qz      
                        );
            // ===== write the estimation in to a txt file =====
            */
        }
        // kalmanCoreEstimator_struct.addProcessNoiseDt(t);

    }   
    Ok(())
}
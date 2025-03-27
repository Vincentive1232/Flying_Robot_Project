use std::error::Error;
use std::fs::File;
use csv::Writer;
use std::io::{self, Write};
use csv::ReaderBuilder;
// use serde::Deserialize;

pub mod multirotor3d;
pub mod utils;
pub mod vis;

use multirotor3d::Multirotor3D;
use multirotor3d::Multirotor3DState;
use multirotor3d::Multirotor3DAction;

fn main() -> Result<(), Box<dyn Error>> {
    let mode_propagate: i8 = 0;
    let mode_action: i8 = 1;

    let path: &str;
    if mode_action == 0 {
        path = "output_control.txt";
    }
    else {
        path = "output_diff_flat.txt";
    }
    let mut file_out = File::create(path).ok().unwrap();

    let path_vis = "data/output.csv";
    let file_vis = File::create(path_vis)?;
    let mut wtr_vis = Writer::from_writer(file_vis);

    // open csv flight data file
    let file = File::open("data/aggressive_traj_0001.csv")?;

    // create csv file reader
    let mut rdr = ReaderBuilder::new()
        .delimiter(b',')
        .from_reader(file);

    // let mut timestamps: Vec<f32> = Vec::new();
    let mut ts: Vec<f32> = Vec::new();
    let mut positions: Vec<utils::Vector3> = Vec::new();
    let mut velocities: Vec<utils::Vector3> = Vec::new();
    let mut accelerations: Vec<utils::Vector3> = Vec::new();
    let mut jerks: Vec<utils::Vector3> = Vec::new();
    let mut snaps: Vec<utils::Vector3> = Vec::new();
    let _pi: f32 = 3.1415926;
    let mut count: usize = 0;

    for result in rdr.records() {
        let data = result?;
        count += 1;

        let t: f32 = data[0].parse()?;
        ts.push(t);

        let position = utils::Vector3 {
            x: data[1].parse()?,
            y: data[2].parse()?,
            z: data[3].parse()?,
        };
        positions.push(position);

        let velocity = utils::Vector3 {
            x: data[4].parse()?,
            y: data[5].parse()?,
            z: data[6].parse()?,
        };
        velocities.push(velocity);

        let acceleration = utils::Vector3 {
            x: data[7].parse()?,
            y: data[8].parse()?,
            z: data[9].parse()?,
        };
        accelerations.push(acceleration);

        let jerk = utils::Vector3 {
            x: data[10].parse()?,
            y: data[11].parse()?,
            z: data[12].parse()?,
        };
        jerks.push(jerk);

        let snap = utils::Vector3 {
            x: data[13].parse()?,
            y: data[14].parse()?,
            z: data[15].parse()?,
        };
        snaps.push(snap);

    }
    // println!("positions: {:?}", positions);
    // println!("velocities: {:?}", velocities);


    // Define the simulation start point and simulation end point
    // for comparison with the groundtruth
    let sim_start: usize = 0;
    let sim_end: usize = count;


    // Parameters define 1
    let kappa_f: f32 = 1.5590e-8;  //1.6175
    let kappa_tau: f32 = 1.858e-8;
    let a: f32 = 0.046/1.414; 
    // Define robot_with_acuation_matrix(kappas)
    let mut robot_with_kappas: Multirotor3D = Multirotor3D { 
        mass: 0.034, 
        g: -9.81, 
        /*
        j: utils::Matrix3::new([
            [16.57171e-6, 0.830806e-6,  0.718277e-6],
            [0.830806e-6, 16.655602e-6, 1.800197e-6],
            [0.718277e-6, 1.800197e-6,  29.261652e-6],
        ]), 
        j_inv: utils::Matrix3::new([
            [60544.85769, -2878.57629, -1309.08447],
            [-2878.57629, 60587.61977, -3656.17889],
            [-1309.08447, -3656.17889, 34431.48485],
        ]), 
        */
        j: utils::Matrix3::new([
            [16.571710e-6, 0.0, 0.0],
            [0.0, 16.655602e-6, 0.0],
            [0.0, 0.0, 29.261652e-6],
        ]),
        j_inv: utils::Matrix3::new([
            [60343.803, 0.0, 0.0],
            [0.0, 60039.859, 0.0],
            [0.0, 0.0, 34174.421],
        ]),
        b0: utils::Matrix4::new([
            [kappa_f, kappa_f, kappa_f, kappa_f],
            [-kappa_f*a, -kappa_f*a, kappa_f*a, kappa_f*a],
            [-kappa_f*a, kappa_f*a, kappa_f*a, -kappa_f*a],
            [-kappa_tau, kappa_tau, -kappa_tau, kappa_tau],
        ]),
        l: 0.08, 
        dt: 0.002, 
        state: Multirotor3DState{ 
            p: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
            v: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
            q: utils::Quaternion {qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0},  
            q_dot: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
        },
        kp: utils::Matrix3::new([
            [7.0, 0.0, 0.0],
            [0.0, 7.0, 0.0],
            [0.0, 0.0, 7.0],
        ]),
        kv: utils::Matrix3::new([
            [4.0, 0.0, 0.0],
            [0.0, 4.0, 0.0],
            [0.0, 0.0, 4.0],
        ]),
        ki_p: utils::Matrix3::new([
            [5.0, 0.0, 0.0],
            [0.0, 8.0, 0.0],
            [0.0, 0.0, 4.0],
        ]),
        kr: utils::Matrix3::new([
            [0.007, 0.0, 0.0],
            [0.0, 0.007, 0.0],
            [0.0, 0.0, 0.008],
        ]),
        ki: utils::Matrix3::new([
            [0.00115, 0.0, 0.0],
            [0.0, 0.00115, 0.0],
            [0.0, 0.0, 0.002],
        ]),
        k_omega: utils::Matrix3::new([
            [0.03, 0.0, 0.0],
            [0.0, 0.03, 0.0],
            [0.0, 0.0, 0.03],
        ]),
        i_error_att: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0},
        i_error_pos: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0},
        ref_quat: utils::Quaternion {qw: 0.0, qx: 0.0, qy: 0.0, qz: 0.0},
        ref_ang_vel: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0},
        ref_ang_acc: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0},
        ang_acc: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}
    };


    // Parameters define 2
    let arm_length: f32 = 0.046;
    let arm: f32 = 0.707106781 * arm_length;
    let t2t: f32 = 0.006;
    // Define robot
    let mut robot: Multirotor3D = Multirotor3D {
        mass: 0.0347,//0.0351 
        g: -9.81, 
        j: utils::Matrix3::new([
            [16.571710e-6, 0.0, 0.0],
            [0.0, 16.655602e-6, 0.0],
            [0.0, 0.0, 29.261652e-6],
        ]),
        j_inv: utils::Matrix3::new([
            [60343.803, 0.0, 0.0],
            [0.0, 60039.859, 0.0],
            [0.0, 0.0, 34174.421],
        ]),
        b0: utils::Matrix4::new([
            [1.0, 1.0, 1.0, 1.0],
            [-arm, -arm, arm, arm],
            [-arm, arm, arm, -arm],
            [-t2t, t2t, -t2t, t2t],
        ]),
        l: 0.08, 
        dt: 0.0001, 
        state: Multirotor3DState{ 
            p: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
            v: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
            q: utils::Quaternion {qw: 1.0, qx: 0.0, qy: 0.0, qz: 0.0},  
            q_dot: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
        },
        kp: utils::Matrix3::new([
            [7.0, 0.0, 0.0],
            [0.0, 7.0, 0.0],
            [0.0, 0.0, 7.0],
        ]),
        kv: utils::Matrix3::new([
            [4.0, 0.0, 0.0],
            [0.0, 4.0, 0.0],
            [0.0, 0.0, 4.0],
        ]),
        ki_p: utils::Matrix3::new([
            [5.0, 0.0, 0.0],
            [0.0, 8.0, 0.0],
            [0.0, 0.0, 4.0],
        ]),
        kr: utils::Matrix3::new([
            [0.007, 0.0, 0.0],
            [0.0, 0.007, 0.0],
            [0.0, 0.0, 0.008],
        ]),
        k_omega: utils::Matrix3::new([
            [0.00115, 0.0, 0.0],
            [0.0, 0.00115, 0.0],
            [0.0, 0.0, 0.002],
        ]),
        ki: utils::Matrix3::new([
            [0.03, 0.0, 0.0],
            [0.0, 0.03, 0.0],
            [0.0, 0.0, 0.03],
        ]),
        i_error_att: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0},
        i_error_pos: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0},
        ref_quat: utils::Quaternion {qw: 0.0, qx: 0.0, qy: 0.0, qz: 0.0},
        ref_ang_vel: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0},
        ref_ang_acc: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0},
        ang_acc: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}
    };
    // println!("{:?}", robot);

    
    let mut action_force =  Multirotor3DAction {
        a: utils::Vector4::new(0.0, 0.0, 0.0, 0.0),
    };

    for i in sim_start..sim_end {
        // println!("States: {:?}", robot.state);
        // let tmp_force = utils::Vector4::new(utils::rpm_to_force(actions[i].a.x), utils::rpm_to_force(actions[i].a.y), utils::rpm_to_force(actions[i].a.z), utils::rpm_to_force(actions[i].a.k));
        /*
        let tmp_force = utils::Vector4::new(utils::rpm_to_force(20000.0), utils::rpm_to_force(20100.0), utils::rpm_to_force(20000.0), utils::rpm_to_force(20000.0));
        println!("Actions{:?}: {:?}", i, robot.b0 * tmp_force);
        
        let action_force = Multirotor3DAction {
            a: robot.b0 * tmp_force,
        };
        */
        if mode_action == 0 {
            action_force = robot.lee_controller(
                positions[i], 
                velocities[i], 
                accelerations[i], 
                jerks[i], 
                snaps[i]
            );
        }
        else {
            action_force = robot.differential_flatness(
                positions[i], 
                velocities[i], 
                accelerations[i], 
                jerks[i], 
                snaps[i]
            );

        }
        
        if mode_propagate == 0 {
            robot.step_euler(action_force);
            // robot_2.step_euler_fake(action_force, states[i+1].q, states[i+1].q_dot, states[i+1].v);
            // robot_2.step_euler_fake_double(action_force, states[i+1].q, states[i+1].q_dot, states[i+1].v, ts[i+1]-ts[i]);
            // robot_2.step_euler_correction(action_force, states[i].p, states[i].v, states[i].q, states[i].q_dot, ts[i+1]-ts[i], i+1);
            // robot_2.step_euler_crazyflies2(action_force);
        }
        else if mode_propagate == 1 {
            robot.step_rk4(action_force);
            // robot_2.step_rk4_fake(action_force, states[i].q_dot, states[i+1].q_dot);
            // robot_2.step_rk4_fake_double(action_force, states[i].q, states[i].v, states[i].q_dot, states[i+1].q_dot);
            // robot_2.step_rk4_correction(action_force, states[i].p, states[i].v, states[i].q, states[i].q_dot, ts[i+1]-ts[i], i+1);
        }
        else {
            println!("No specific integrating method!!!!!")
        }


        _ = writeln!(file_out, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
            robot.state.p.x, robot.state.p.y, robot.state.p.z,
            positions[i].x, positions[i].y, positions[i].z,
            robot.state.v.x, robot.state.v.y, robot.state.v.z,
            velocities[i].x, velocities[i].y, velocities[i].z,
            robot.state.q.qw, robot.state.q.qx, robot.state.q.qy, robot.state.q.qz,
            robot.state.q_dot.x, robot.state.q_dot.y, robot.state.q_dot.z,
            robot.ref_quat.qw, robot.ref_quat.qx, robot.ref_quat.qy, robot.ref_quat.qz,
            robot.ref_ang_vel.x, robot.ref_ang_vel.y, robot.ref_ang_vel.z, 
            robot.ref_ang_acc.x, robot.ref_ang_acc.y, robot.ref_ang_acc.z,
            robot.ang_acc.x, robot.ang_acc.y, robot.ang_acc.z       
        );

        wtr_vis.write_record(&[
            ts[i].to_string(),
            robot.state.p.x.to_string(), robot.state.p.y.to_string(), robot.state.p.z.to_string(),
            robot.state.v.x.to_string(), robot.state.v.y.to_string(), robot.state.v.z.to_string(),
            robot.state.q.qw.to_string(), robot.state.q.qx.to_string(), robot.state.q.qy.to_string(), robot.state.q.qz.to_string(),
            robot.state.q_dot.x.to_string(), robot.state.q_dot.y.to_string(), robot.state.q_dot.z.to_string()
        ])?;

        wtr_vis.flush()?;
    }

    // _ = vis::visualization_meshcat();
    _ = vis::visualization_meshcat_obj(sim_start, sim_end-1);

    Ok(())
}
    
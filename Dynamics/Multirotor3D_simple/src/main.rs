use std::error::Error;
use std::fs::File;
use csv::Writer;
use std::io::Write;
use csv::ReaderBuilder;
// use serde::Deserialize;

pub mod multirotor3d;
pub mod utils;
pub mod vis;

use multirotor3d::Multirotor3D;
use multirotor3d::Multirotor3DState;
use multirotor3d::Multirotor3DAction;

fn main() -> Result<(), Box<dyn Error>> {

    let path = "output.txt";
    let mut file_out = File::create(path).ok().unwrap();

    let path_vis = "data/output.csv";
    let file_vis = File::create(path_vis)?;
    let mut wtr_vis = Writer::from_writer(file_vis);

    // open csv flight data file
    let file = File::open("data/real_flight_data.csv")?;

    // create csv file reader
    let mut rdr = ReaderBuilder::new()
        .delimiter(b',')
        .from_reader(file);

    // let mut timestamps: Vec<f32> = Vec::new();
    let mut states: Vec<Multirotor3DState> = Vec::new();
    let mut actions: Vec<Multirotor3DAction> = Vec::new();
    let mut ts: Vec<f32> = Vec::new();
    // let pi: f32 = 3.1415926;

    for result in rdr.records() {
        let data = result?;

        let t: f32 = data[0].parse()?;
        ts.push(t);

        let state = Multirotor3DState {
            p: utils::Vector3::new(data[1].parse()?, data[2].parse()?, data[3].parse()?),
            v: utils::Vector3::new(data[4].parse()?, data[5].parse()?, data[6].parse()?),
            q: utils::Quaternion::new(data[7].parse()?, data[8].parse()?, data[9].parse()?, data[10].parse()?),
            q_dot: utils::Vector3::new(data[11].parse()?, data[12].parse()?, data[13].parse()?),
        };
        states.push(state);

        let mut action = Multirotor3DAction {
            a: utils::Vector4::new(data[14].parse()?, data[15].parse()?, data[16].parse()?, data[17].parse()?),
        };
        
        // converted from rpm to rad/s
        /*
        action.a.x = (action.a.x*((2.0*pi)/60.0))*(action.a.x*((2.0*pi)/60.0));
        action.a.y = (action.a.y*((2.0*pi)/60.0))*(action.a.y*((2.0*pi)/60.0));
        action.a.z = (action.a.z*((2.0*pi)/60.0))*(action.a.z*((2.0*pi)/60.0));
        action.a.k = (action.a.k*((2.0*pi)/60.0))*(action.a.k*((2.0*pi)/60.0));
        */
        
        action.a.x = action.a.x;
        action.a.y = action.a.y;
        action.a.z = action.a.z;
        action.a.k = action.a.k;
        

        actions.push(action);

    }
    // println!("States: {:?}", states);
    // println!("Actions: {:?}", actions);


    // Define the simulation start point and simulation end point
    // for comparison with the groundtruth
    let sim_start: usize = 0;
    let sim_end: usize = 4000;

    // Parameters define 1
    let kappa_f: f32 = 1.5590e-8;  //1.6175
    let kappa_tau: f32 = 1.858e-8;
    let a: f32 = 0.046/1.414; 
    // Define robot_1
    let mut _robot: Multirotor3D = Multirotor3D { 
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
            p: states[sim_start].p, 
            v: states[sim_start].v,
            q: states[sim_start].q, 
            q_dot: states[sim_start].q_dot,
        },
    };

    // Parameters define 2
    let arm_length: f32 = 0.046;
    let arm: f32 = 0.707106781 * arm_length;
    let t2t: f32 = 0.006;
    // Define robot_2
    let mut robot_2: Multirotor3D = Multirotor3D {
        mass: 0.034,//0.0351 
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
        dt: 0.002, 
        
        state: Multirotor3DState{ 
            p: states[sim_start].p, 
            v: states[sim_start].v,
            q: states[sim_start].q, 
            q_dot: states[sim_start].q_dot,
        },
        /*
        state: Multirotor3DState{ 
            p: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
            v: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
            q: utils::Quaternion {qw: 0.0, qx: 0.0, qy: 0.0, qz: 1.0},  
            q_dot: utils::Vector3 {x: 0.0, y: 0.0, z: 0.0}, 
        },
        */
    };
    println!("{:?}", robot_2);

    let mode: i8 = 0;

    /*
    for i in sim_start..sim_end {
        // println!("States: {:?}", robot.state);
        
        println!("Actions{:?}: {:?}", i, robot.b0 * actions[i].a);
        let action_force = Multirotor3DAction {
            a: robot.b0 * actions[i].a,
        };
        
        /*
        println!("Actions: {:?}", combine_thrust_torque(actions[i].a.x, actions[i].a.y, actions[i].a.z, actions[i].a.k));
        let action_force = Multirotor3DAction {
            a: combine_thrust_torque(actions[i].a.x, actions[i].a.y, actions[i].a.z, actions[i].a.k),
        };
        */
        if mode == 0 {
            // robot.step_euler(action_force);
            robot.step_euler_fake(action_force,  states[i].q_dot, );
        }
        else if mode == 1 {
            // robot.step_rk4(action_force);
            robot.step_rk4_fake(action_force, states[i].q_dot, states[i+1].q_dot);
        }
        else {
            println!("No specific integrating method!!!!!")
        }
        writeln!(file_out, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
            robot.state.p.x, robot.state.p.y, robot.state.p.z,
            states[i].p.x, states[i].p.y, states[i].p.z,
            robot.state.v.x, robot.state.v.y, robot.state.v.z,
            states[i].v.x, states[i].v.y, states[i].v.z,
            robot.state.q.qw, robot.state.q.qx, robot.state.q.qy, robot.state.q.qz,
            states[i].q.qw, states[i].q.qx, states[i].q.qy, states[i].q.qz,
            robot.state.q_dot.x, robot.state.q_dot.y, robot.state.q_dot.z,
            states[i].q_dot.x, states[i].q_dot.y, states[i].q_dot.z,
        );
    }
    */

    for i in sim_start..sim_end {
        // println!("States: {:?}", robot.state);
        let tmp_force = utils::Vector4::new(utils::rpm_to_force(actions[i].a.x), utils::rpm_to_force(actions[i].a.y), utils::rpm_to_force(actions[i].a.z), utils::rpm_to_force(actions[i].a.k));
        // let tmp_force = utils::Vector4::new(utils::rpm_to_force(20000.0), utils::rpm_to_force(20001.0), utils::rpm_to_force(20000.0), utils::rpm_to_force(20000.0));
        println!("Actions{:?}: {:?}", i, robot_2.b0 * tmp_force);
        
        let action_force = Multirotor3DAction {
            a: robot_2.b0 * tmp_force,
        };
        
        
        if mode == 0 {
            // robot_2.step_euler(action_force);
            // robot_2.step_euler_fake(action_force, states[i+1].q, states[i+1].q_dot, states[i+1].v);
            // robot_2.step_euler_fake_double(action_force, states[i+1].q, states[i+1].q_dot, states[i+1].v, ts[i+1]-ts[i]);
            robot_2.step_euler_correction(action_force, states[i].p, states[i].v, states[i].q, states[i].q_dot, ts[i+1]-ts[i], i+1);
            // robot_2.step_euler_crazyflies2(action_force);
        }
        else if mode == 1 {
            // robot_2.step_rk4(action_force);
            // robot_2.step_rk4_fake(action_force, states[i].q_dot, states[i+1].q_dot);
            // robot_2.step_rk4_fake_double(action_force, states[i].q, states[i].v, states[i].q_dot, states[i+1].q_dot);
            robot_2.step_rk4_correction(action_force, states[i].p, states[i].v, states[i].q, states[i].q_dot, ts[i+1]-ts[i], i+1);
        }
        else {
            println!("No specific integrating method!!!!!")
        }


        _ = writeln!(file_out, "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}",
            robot_2.state.p.x, robot_2.state.p.y, robot_2.state.p.z,
            states[i].p.x, states[i].p.y, states[i].p.z,
            robot_2.state.v.x, robot_2.state.v.y, robot_2.state.v.z,
            states[i].v.x, states[i].v.y, states[i].v.z,
            robot_2.state.q.qw, robot_2.state.q.qx, robot_2.state.q.qy, robot_2.state.q.qz,
            states[i].q.qw, states[i].q.qx, states[i].q.qy, states[i].q.qz,
            robot_2.state.q_dot.x, robot_2.state.q_dot.y, robot_2.state.q_dot.z,
            states[i].q_dot.x, states[i].q_dot.y, states[i].q_dot.z,
        );

        wtr_vis.write_record(&[
            ts[i].to_string(),
            robot_2.state.p.x.to_string(), robot_2.state.p.y.to_string(), robot_2.state.p.z.to_string(),
            robot_2.state.v.x.to_string(), robot_2.state.v.y.to_string(), robot_2.state.v.z.to_string(),
            robot_2.state.q.qw.to_string(), robot_2.state.q.qx.to_string(), robot_2.state.q.qy.to_string(), robot_2.state.q.qz.to_string(),
            robot_2.state.q_dot.x.to_string(), robot_2.state.q_dot.y.to_string(), robot_2.state.q_dot.z.to_string()
        ])?;

        wtr_vis.flush()?;
    }

    // _ = vis::visualization_meshcat();
    _ = vis::visualization_meshcat_obj();

    Ok(())
}
    
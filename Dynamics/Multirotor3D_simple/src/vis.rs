use std::error::Error;
use std::fs::File;
use csv::ReaderBuilder;
use std::time::Duration;

use meshcat::types::*;
use meshcat::utils as mesh_utils;
use nalgebra::Isometry3;

use super::utils;
use super::multirotor3d;

use multirotor3d::Multirotor3DState;

pub fn visualization_meshcat() -> Result<(), Box<dyn Error>> {

    // open csv flight data file
    // let file = File::open("data/real_flight_data.csv")?;
    let file = File::open("data/output.csv")?;

    // create csv file reader
    let mut rdr = ReaderBuilder::new()
        .delimiter(b',')
        .from_reader(file);

    // let mut timestamps: Vec<f32> = Vec::new();
    let mut states: Vec<Multirotor3DState> = Vec::new();
    let mut ts: Vec<f32> = Vec::new();

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
    }


    let meshcat: Meshcat = Meshcat::new("tcp://127.0.0.1:6000");

    /*************************************** Body Definition ***********************************************/
    meshcat.set_object(
        "/robot1",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Box { 
            width: 0.092, 
            height: 0.008, 
            depth: 0.005,
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0, 0.0, 0.0),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.785),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x00ffff).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot2",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Box { 
            width: 0.092, 
            height: 0.008, 
            depth: 0.005,
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0, 0.0, 0.0),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, -0.785),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x00ffff).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot3",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Box { 
            width: 0.025, 
            height: 0.025, 
            depth: 0.005,
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0, 0.0, 0.0),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x00ffff).build())
        .build(),
    )?;
    /*************************************** Body Definition ***********************************************/



    /*************************************** Motors Definition ***********************************************/
    // propeller parameters
    let motor_radius_top = 0.005;
    let motor_radius_bottom = 0.003;
    let motor_height = 0.01;

    meshcat.set_object(
        "/robot_left",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: motor_radius_top, 
            radius_bottom: motor_radius_bottom, 
            height: motor_height, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.0325, 0.0325, 0.005),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0xff0000).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_left_back",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: motor_radius_top, 
            radius_bottom: motor_radius_bottom, 
            height: motor_height, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.0325, -0.0325, 0.005),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0xff0000).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_right",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: motor_radius_top, 
            radius_bottom: motor_radius_bottom, 
            height: motor_height, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0325, 0.0325, 0.005),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0xff0000).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_right_back",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: motor_radius_top, 
            radius_bottom: motor_radius_bottom, 
            height: motor_height, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0325, -0.0325, 0.005),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0xff0000).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_left_top",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: 0.001, 
            radius_bottom: 0.002, 
            height: 0.005, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.0325, 0.0325, 0.012),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x00ff00).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_left_back_top",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: 0.001, 
            radius_bottom: 0.002, 
            height: 0.005, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.0325, -0.0325, 0.012),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x00ff00).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_right_top",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: 0.001, 
            radius_bottom: 0.002, 
            height: 0.005, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0325, 0.0325, 0.012),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x00ff00).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_right_back_top",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: 0.001, 
            radius_bottom: 0.002, 
            height: 0.005, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0325, -0.0325, 0.012),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x00ff00).build())
        .build(),
    )?;
    /*************************************** Motors Definition ***********************************************/


    let propellers_radius = 0.025;
    let propellers_height = 0.001;
    /*************************************** Propellers Definition ***********************************************/
    meshcat.set_object(
        "/robot_propeller_left",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: propellers_radius, 
            radius_bottom: propellers_radius, 
            height: propellers_height, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.0325, 0.0325, 0.01),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x002f00).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_propeller_left_back",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: propellers_radius, 
            radius_bottom: propellers_radius, 
            height: propellers_height, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.0325, -0.0325, 0.01),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x002f00).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_propeller_right",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: propellers_radius, 
            radius_bottom: propellers_radius, 
            height: propellers_height, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0325, 0.0325, 0.01),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x002f00).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_propeller_right_back",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Cylinder { 
            radius_top: propellers_radius, 
            radius_bottom: propellers_radius, 
            height: propellers_height, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0325, -0.0325, 0.01),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x002f00).opacity(1.0).build())
        .build(),
    )?;
    /*************************************** Propellers Definition ***********************************************/


    let sim_start: usize = 0;
    let sim_end: usize = 4600;

    for i in sim_start..sim_end {
        meshcat.set_transform(
            "/robot1",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot2",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot3",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot_left",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot_left_back",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot_right",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot_right_back",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot_left_top",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot_left_back_top",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot_right_top",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;

        meshcat.set_transform(
            "/robot_right_back_top",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;


        
        meshcat.set_transform(
            "/robot_propeller_left",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;
        meshcat.set_transform(
            "/robot_propeller_left_back",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;
        meshcat.set_transform(
            "/robot_propeller_right",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;
        meshcat.set_transform(
            "/robot_propeller_right_back",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;


        std::thread::sleep(Duration::from_millis(1));
    }

    Ok(())
}




pub fn visualization_meshcat_obj() -> Result<(), Box<dyn Error>> {

    // open csv flight data file
    // let file = File::open("data/real_flight_data.csv")?;
    let file = File::open("data/output.csv")?;

    // create csv file reader
    let mut rdr = ReaderBuilder::new()
        .delimiter(b',')
        .from_reader(file);

    // let mut timestamps: Vec<f32> = Vec::new();
    let mut states: Vec<Multirotor3DState> = Vec::new();
    let mut ts: Vec<f32> = Vec::new();

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
    }


    let meshcat: Meshcat = Meshcat::new("tcp://127.0.0.1:6000");

    meshcat.set_object(
        "/drone",
        LumpedObject::builder()
            .geometries(vec![Geometry::new(mesh_utils::load_mesh(
                "obj/cf2_assembly.obj",
            )?)])
            .object(Object::new(
                Isometry3::from_parts(
                    nalgebra::Translation3::new(0.0, 0.0, 0.0),
                    nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
                ),
                ObjectType::Mesh,
            ))
            .material(Material::builder().color(0xffa500).build())
            .build(),
    )?;


    let sim_start: usize = 0;
    let sim_end: usize = 4600;

    for i in sim_start..sim_end {
        meshcat.set_transform(
            "/drone",
            Isometry3::from_parts(
                nalgebra::Translation3::new(states[i].p.x.into(), states[i].p.y.into(), states[i].p.z.into()),
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(states[i].q.qw.into(), states[i].q.qx.into(), states[i].q.qy.into(), states[i].q.qz.into())),
            ),
        )?;


        std::thread::sleep(Duration::from_millis(5));
    }

    Ok(())
}

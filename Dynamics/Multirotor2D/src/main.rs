use std::error::Error;
use std::time::Duration;

use meshcat::types::*;
use nalgebra::Isometry3;

pub mod multirotor2d;

use multirotor2d::Multirotor2D;
use multirotor2d::Multirotor2DState;
use multirotor2d::Multirotor2DAction;

fn main() -> Result<(), Box<dyn Error>> {
    let meshcat: Meshcat = Meshcat::new("tcp://127.0.0.1:6000");
    meshcat.set_object(
        "/robot1",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Box { 
            width: 0.7071, 
            height: 0.08, 
            depth: 0.05,
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
            width: 0.7071, 
            height: 0.08, 
            depth: 0.05,
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
            width: 0.25, 
            height: 0.25, 
            depth: 0.05,
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


/*************************************** Motors Definition ***********************************************/
    // propeller parameters
    let motor_radius_top = 0.05;
    let motor_radius_bottom = 0.03;
    let motor_height = 0.1;

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
                nalgebra::Translation3::new(-0.2, 0.2, 0.05),
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
                nalgebra::Translation3::new(-0.2, -0.2, 0.05),
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
                nalgebra::Translation3::new(0.2, 0.2, 0.05),
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
                nalgebra::Translation3::new(0.2, -0.2, 0.05),
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
            radius_top: 0.01, 
            radius_bottom: 0.02, 
            height: 0.05, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.2, 0.2, 0.12),
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
            radius_top: 0.01, 
            radius_bottom: 0.02, 
            height: 0.05, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.2, -0.2, 0.12),
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
            radius_top: 0.01, 
            radius_bottom: 0.02, 
            height: 0.05, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.2, 0.2, 0.12),
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
            radius_top: 0.01, 
            radius_bottom: 0.02, 
            height: 0.05, 
            radial_segments: 32, 
            height_segments: 1, 
            theta_start: 0.0, 
            theta_length: 2.0 * std::f64::consts::PI
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.2, -0.2, 0.12),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x00ff00).build())
        .build(),
    )?;
/*************************************** Motors Definition ***********************************************/



/*************************************** Propellers Definition ***********************************************/
    meshcat.set_object(
        "/robot_propeller_left",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Box { 
            width: 0.35, 
            height: 0.01, 
            depth: 0.01,
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.2, 0.2, 0.1),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x002fff).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_propeller_left_back",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Box { 
            width: 0.35, 
            height: 0.01, 
            depth: 0.01,
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(-0.2, -0.2, 0.1),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x002fff).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_propeller_right",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Box { 
            width: 0.35, 
            height: 0.01, 
            depth: 0.01,
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.2, 0.2, 0.1),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x002fff).build())
        .build(),
    )?;

    meshcat.set_object(
        "/robot_propeller_right_back",
        LumpedObject::builder()
        .geometries(vec![Geometry::new(GeometryType::Box { 
            width: 0.35, 
            height: 0.01, 
            depth: 0.01,
        })])
        .object(Object::new(
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.2, -0.2, 0.1),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
            ObjectType::Mesh,
        ))
        .material(Material::builder().color(0x002fff).build())
        .build(),
    )?;
/*************************************** Propellers Definition ***********************************************/


    let mut robot: Multirotor2D = Multirotor2D { mass: 0.1, g: 9.81, jyy: 0.000333, l: 0.1, dt: 0.1, state: Multirotor2DState{ x: 0.0, x_dot: 0.0, z: 0.0, z_dot: 0.0, theta: 0.0, theta_dot:0.0} };
    println!("{:?}", robot);

    // let mut yaw_tmp = 0.50;

    for _ in 1..60 {
        robot.step(Multirotor2DAction { f1: 0.49999, f2: 0.5 });
        let theta_tmp = -robot.state.theta;

        /*
        yaw_tmp += 0.50;
        yaw_tmp = yaw_tmp % (2.0*std::f64::consts::PI);

        // 初始的旋转（保持物体在世界坐标系中的旋转）
        let world_rotation = UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0);

        // 绕物体自身 Y 轴的旋转
        let local_rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw_tmp);

        // 合并旋转：物体在世界坐标系中的旋转 * 绕自身 Y 轴的旋转
        let combined_rotation = world_rotation * local_rotation;
        */
        
        meshcat.set_transform(
            "/robot1",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot2",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot3",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot_left",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot_left_back",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot_right",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot_right_back",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot_left_top",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot_left_back_top",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot_right_top",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;

        meshcat.set_transform(
            "/robot_right_back_top",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;


        
        meshcat.set_transform(
            "/robot_propeller_left",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;
        meshcat.set_transform(
            "/robot_propeller_left_back",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;
        meshcat.set_transform(
            "/robot_propeller_right",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;
        meshcat.set_transform(
            "/robot_propeller_right_back",
            Isometry3::from_parts(
                nalgebra::Translation3::new(robot.state.x.into(), 0.0, robot.state.z.into()),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, theta_tmp.into(), 0.0),
            ),
        )?;
        std::thread::sleep(Duration::from_millis(100));
        println!("{:?}", robot.state);
    }

    Ok(())
}
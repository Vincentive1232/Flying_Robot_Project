use std::error::Error;
use std::fs::File;
use csv::ReaderBuilder;
use serde::Deserialize;

use nalgebra::{Quaternion, Matrix4, Matrix3, Vector3, Vector4};

pub mod multirotor3d;
pub mod utils;

use multirotor3d::Multirotor3D;
use multirotor3d::Multirotor3DState;
use multirotor3d::Multirotor3DAction;

fn main() -> Result<(), Box<dyn Error>> {
    // open csv flight data file
    let file = File::open("data/real_flight_data.csv")?;

    // create csv file reader
    let mut rdr = ReaderBuilder::new()
        .delimiter(b',')
        .from_reader(file);

    // let mut timestamps: Vec<f32> = Vec::new();
    let mut states: Vec<Multirotor3DState> = Vec::new();
    let mut actions: Vec<Multirotor3DAction> = Vec::new();

    for result in rdr.records() {
        let data = result?;

        // let t = data[0].parse()?;

        let state = Multirotor3DState {
            p: Vector3::new(data[1].parse()?, data[2].parse()?, data[3].parse()?),
            v: Vector3::new(data[4].parse()?, data[5].parse()?, data[6].parse()?),
            q: Quaternion::new(data[7].parse()?, data[8].parse()?, data[9].parse()?, data[10].parse()?),
            q_dot: Vector3::new(data[11].parse()?, data[12].parse()?, data[13].parse()?),
        };
        states.push(state);

        let action = Multirotor3DAction {
            a: Vector4::new(data[14].parse()?, data[15].parse()?, data[16].parse()?, data[17].parse()?),
        };
        actions.push(action);

    }
    println!("States: {:?}", states);
    println!("Actions: {:?}", actions);

    Ok(())
}

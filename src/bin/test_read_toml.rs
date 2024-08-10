use rust_robotics::models::{base::System, bicycle_kinematic};

fn main() {
    let mut vehicle: bicycle_kinematic::Model = bicycle_kinematic::Model::new(0.0, 0.0);
    println!(
        "Before reading: lf = {}, lr = {}",
        vehicle.length_front, vehicle.length_rear
    );

    // TODO: have to be in top level directory. Have a feature to give a relative path or
    // absolute path and still be able to read the file
    vehicle.read("configs/bike_kinem_example.toml");
    println!(
        "Before reading: lf = {}, lr = {}",
        vehicle.length_front, vehicle.length_rear
    );
}

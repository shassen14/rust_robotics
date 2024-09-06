# rust_robotics
This project was inspired by [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics), the desire to learn Rust, and previous work I've done. I am still learning, so please feel free to contribute any way you see fit. Thanks!

# Table of Contents 
* [Current Project State](#current-project-state)
* [Requirements/Quick Install](#requirementsquick-install)


# Current Project State
Architecturally, the project is a WIP. The vision is to work on algorithms in the following manner (i.e. system modeling -> controls -> path/motion planning -> inertial sensor imitation -> localization -> basic visual sensor imitation -> mapping).

![Basic Robotics Pipeline](https://github.com/shassen14/rust_robotics_media/blob/main/general/basic_robotics_pipeline.drawio.png)

**NOTE: Since this is the beginning, the program is very volatile and could easily have breaking changes**

## Robotic Algorithms Implemented
1. There is a simple ODE (Ordinary Differential Equation) solver implementation being from the Runge-Kutta family (Euler all the way up to RK4).
    * This is helpful to model systems (kinematics, dynamics, empirical) and find their solution/states. 
    * Code [here](/src/num_methods/runge_kutta.rs)
2. A base interface (rust trait) to model systems. Code [here](/src/models/base.rs)
    * Notable models halfway implemented are the following:
        * [Kinematic Bicycle](/src/models/ground_vehicles/bicycle_kinematic.rs)
        * [N Joint 2D Robotic Arm](/src/models/humanoid/n_joint_arm2.rs)

# Requirements/Quick Install
1. [Rust Installation](https://www.rust-lang.org/tools/install) for your specified OS
2. Clone this repo and change your directory into this repo using the command line interface
3. `cargo build --release`
    * Release build optimizes for storage size and cpu usage

## Run Example Binaries
**Assumption is you are in the repo directory in the command line interface**

**Once this repo is more flushed out, these files will be moved to an examples directory**

* [N Joint 2D Robotic Arm Example](/src/bin/test_n_joint_arm2.rs)
    * `cargo run --release --bin test_n_joint_arm2 configs/examples/animation_n_joint_arm2.toml`
![N Joint Robotic Arm Simulation](https://github.com/shassen14/rust_robotics_media/blob/main/models/n_joint_robotic_arm_2d.gif)

* Kinematic Bicycle Model Examples
    * [Simple Drive Around with Keyboard](/src/bin/test_bicycle_kinematic.rs) 
        * `cargo run --release --bin test_bicycle_kinematic configs/examples/animation_bicycle.toml configs/examples/bicycle_kinematic.toml`
    * [Drive Around with Keyboard while Recording Path to CSV File](/src/bin/write_bike_path.rs)
        * `cargo run --release --bin write_back_path configs/examples/animation_bicycle.toml configs/examples/bicycle_kinematic.toml`
        * [Hard coded CSV File](/logs/examples/example_path.csv)
    * [Read Path from CSV and Path Track with Pure Pursuit](/src/bin/read_bike_path.rs) 
        * `cargo run --release --bin read_bike_path configs/examples/animation_bicycle.toml configs/examples/bicycle_kinematic.toml`
![Pure Pursuit on a Predetermined Path using Kinematic Bicycle Model](https://github.com/shassen14/rust_robotics_media/blob/main/models/bicycle_kinematic_pure_pursuit.gif)
    






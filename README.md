# rust_robotics
This project was inspired by [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics), the desire to learn Rust, and previous work I've done. I am still learning, so please feel free to contribute any way you see fit. Thanks!

# Table of Contents 
* [Current Project State](#current-project-state)
* [Requirements/Quick Install](#requirementsquick-install)

# Current Project State
Architecturally, the project is a WIP. The vision is to work on algorithms in the following manner (i.e. system modeling -> controls -> path/motion planning -> inertial sensor imitation -> localization -> basic visual sensor imitation -> mapping).

## Basic Robotics Pipeline
This is generally how data flows to get robots to achieve goals autonomously. We will work on algorithms unintuitively from the bottom to upstream.

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

### Path Planning
* [Dijkstras](examples/path_planning/dijkstras.rs)
    * `cargo run --release --example dijkstras -- --dijkstras-config configs/path_planning/dijkstras_params.toml`
        * The toml file can be edited in such a way to change how many obstacles and their sizes as well as how big the map is

### Controls
* [N Joint 2D Robotic Arm Example](/src/bin/test_n_joint_arm2.rs)
    * `cargo run --release --example n_joint_arm_2d configs/controls/n_joint_arm2_params.toml`
        * The toml file can be edited in such a way to change the controls, and the number of linkages for the robotic arm

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


# To-Do's

## Robotics Related
1. Base interface for controls
    * Base interface for pathtracking as an extension?
1. Double check jacobians for all models
1. Feedforward control input for all models


## Non-Robotics Related
1. Function returns Result<> instead of the expected values for error handling
1. Function arguments have std::option for optional arguments. This will help with default API values.
1. Ability to move axis range while plotting for animations. 
    * Like a panning camera to center around an object
1. Multiplot function wrappers to graph multiple plots at the same time in animations
    * Need to have mouse point position converted to graph location
1. 3D plotting for animations
1. Combination of 3D plots and 2D plots
1. Utilities for command line argument for binaries
1. CSV to SQLite
    * [external packagege](https://github.com/rusqlite/rusqlite)
1. Move binaries to examples
    * Examples need a generic program flow
1. Github workflow
1. Multiple Robotic arms simulation
    * Assumption of starting point to be 0 needs to be removed
1. Path Planning (Dijkstras)
    * What if start or goal position are in obstacles
    * what if obstacles are intersecting
    * Be okay with a goal position not being obtainable in a convex obstacle shape like a donut






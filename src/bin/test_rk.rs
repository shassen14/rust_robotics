use nalgebra as na;
use rust_robotics::models::base::System;
use rust_robotics::models::ca_1dof::Ca1dof;
use rust_robotics::num_methods::runge_kutta;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let x0: f64 = 0.;
    let vel0: f64 = 1.;
    let accel0: f64 = 1.;
    let state0: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(x0, vel0, accel0);
    let start: f64 = 0.;
    let end: f64 = 10.;
    let step: f64 = 0.01;
    // let total_time = end - start;

    let mut t0 = start;
    let mut tf = t0 + step;
    let mut result = state0;

    // let expected_result = na::SVector::<f64, 3>::new(
    //     x0 + vel0 * total_time + 0.5 * accel0 * total_time * total_time,
    //     vel0 + accel0 * total_time,
    //     accel0,
    // );

    let veh: Ca1dof = Ca1dof {};

    while tf <= end {
        result = Ca1dof::propagate(
            &veh,
            &result,
            &na::SVector::<f64, 0>::zeros(),
            t0,
            step,
            runge_kutta::rk4,
        );
        t0 = tf;
        tf += step;
        // println!("t0: {}, tf: {}, result: {}", t0, tf, result);
    }

    Ok(())
}

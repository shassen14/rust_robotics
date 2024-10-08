extern crate nalgebra as na;

use crate::models::base;

pub struct Model;

impl Model {
    fn calculate_f() -> na::SMatrix<f64, 3, 3> {
        // input is column based instead of row based
        // pos_dot = vel
        // vel_dot = acc
        // acc_dot = 0.0
        na::SMatrix::<f64, 3, 3>::from_array_storage(na::ArrayStorage([
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
        ]))
    }
}

/// Constant Acceleration 1 DOF
///
/// System x_dot: [position_dot, velocity_dot, acceleration_dot]
/// System x: [position, velocity, acceleration]
/// System u: none
///
impl base::System<f64, 3, 0> for Model {
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, 3>,
        _u: &nalgebra::SVector<f64, 0>,
        _t: f64,
    ) -> na::SVector<f64, 3> {
        na::SVector::<f64, 3>::new(x[1], x[2], 0.)
    }

    fn calculate_jacobian(
        &self,
        _x: &nalgebra::SVector<f64, 3>,
        _u: &nalgebra::SVector<f64, 0>,
        _t: f64,
    ) -> (na::SMatrix<f64, 3, 3>, na::SMatrix<f64, 3, 0>) {
        (Model::calculate_f(), na::SMatrix::<f64, 3, 0>::zeros())
    }

    fn calculate_input(
        &self,
        _x: &nalgebra::SVector<f64, 3>,
        _x_dot: &nalgebra::SVector<f64, 3>,
        _t: f64,
    ) -> nalgebra::SVector<f64, 0> {
        na::SVector::<f64, 0>::zeros()
    }

    /// No parameters to read for this model
    fn read(&mut self, _: &str) -> () {}
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*;
    use crate::num_methods::runge_kutta;

    #[test]
    fn ca1dof_prop() {
        let x0: f64 = 0.;
        let vel0: f64 = 1.;
        let accel0: f64 = 1.;
        let state0: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(x0, vel0, accel0);
        let start: f64 = 0.;
        let end: f64 = 10.;
        let step: f64 = 0.01;
        let total_time = end - start;

        let mut t0 = start;
        let mut tf = t0 + step;
        let mut result = state0;

        let expected_result = na::SVector::<f64, 3>::new(
            x0 + vel0 * total_time + 0.5 * accel0 * total_time * total_time,
            vel0 + accel0 * total_time,
            accel0,
        );

        let veh: Model = Model {};

        while tf <= end {
            result = base::System::propagate(
                &veh,
                &result,
                &na::SVector::<f64, 0>::zeros(),
                t0,
                step,
                runge_kutta::rk4,
            );
            t0 = tf;
            tf += step;
        }

        let max_error: f64 = 1e-11;
        let total_error = expected_result - result;

        println!("Result: {},  Expected Result: {}", result, expected_result);
        for i in 0..3 {
            println!(
                "i: {},  total error: {},  max error: {}\n",
                i,
                total_error[i].abs(),
                max_error.abs()
            );
            assert!(total_error[i].abs() < max_error.abs())
        }
    }

    #[test]
    fn ca1dof_f_matrix() {
        // TODO: test random time propagates and F should be the same
        let answer = na::SMatrix::<f64, 3, 3>::from_array_storage(na::ArrayStorage([
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
        ]));

        println!("{}", Model::calculate_f());
        assert_eq!(Model::calculate_f(), answer)
    }
}

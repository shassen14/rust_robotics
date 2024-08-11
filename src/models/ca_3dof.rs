extern crate nalgebra as na;

use crate::models::base;

// TODO: can I make this trait SystemF and implement it for any system?
// Seems to be difficult since trait will only allow consts and there will be some
// variables in dynamic systems with mass, inertia, length, etc.

pub struct Model;

impl Model {
    fn calculate_f() -> na::SMatrix<f64, 9, 9> {
        na::SMatrix::<f64, 9, 9>::from_array_storage(na::ArrayStorage([
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [1., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 1., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 1., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 1., 0.],
        ]))
    }
}

impl base::System<f64, 9, 0> for Model {
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, 9>,
        _u: &nalgebra::SVector<f64, 0>,
        _t: f64,
    ) -> na::SVector<f64, 9> {
        na::SVector::<f64, 9>::from_vec(vec![x[1], x[2], 0., x[4], x[5], 0., x[7], x[8], 0.])
    }

    fn get_jacobian(
        &self,
        _x: &nalgebra::SVector<f64, 9>,
        _u: &nalgebra::SVector<f64, 0>,
        _t: f64,
    ) -> (na::SMatrix<f64, 9, 9>, na::SMatrix<f64, 9, 0>) {
        (Model::calculate_f(), na::SMatrix::<f64, 9, 0>::zeros())
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
    fn ca3dof_prop() {
        let x0: f64 = 1.;
        let vel_x0: f64 = 2.;
        let accel_x0: f64 = 1.5;
        let y0: f64 = 0.;
        let vel_y0: f64 = 1.;
        let accel_y0: f64 = 0.5;
        let z0: f64 = 0.;
        let vel_z0: f64 = 0.;
        let accel_z0: f64 = 3.;
        let state0: na::SVector<f64, 9> =
            na::SVector::<f64, 9>::from_array_storage(na::ArrayStorage([[
                x0, vel_x0, accel_x0, y0, vel_y0, accel_y0, z0, vel_z0, accel_z0,
            ]]));
        let start: f64 = 0.;
        let end: f64 = 10.;
        let step: f64 = 0.01;
        let total_time = end - start;

        let mut t0 = start;
        let mut tf = t0 + step;
        let mut result = state0;

        let expected_result = na::SVector::<f64, 9>::from_array_storage(na::ArrayStorage([[
            x0 + vel_x0 * total_time + 0.5 * accel_x0 * total_time * total_time,
            vel_x0 + accel_x0 * total_time,
            accel_x0,
            y0 + vel_y0 * total_time + 0.5 * accel_y0 * total_time * total_time,
            vel_y0 + accel_y0 * total_time,
            accel_y0,
            z0 + vel_z0 * total_time + 0.5 * accel_z0 * total_time * total_time,
            vel_z0 + accel_z0 * total_time,
            accel_z0,
        ]]));

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
            println!("t0: {}, tf: {}, result: {}", t0, tf, result);
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
    fn ca3dof_f_matrix() {
        // TODO: test random time propagates and F should be the same
        let answer = na::SMatrix::<f64, 9, 9>::from_array_storage(na::ArrayStorage([
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [1., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 1., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 1., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 1., 0.],
        ]));

        println!("{}", Model::calculate_f());
        assert_eq!(Model::calculate_f(), answer)
    }
}

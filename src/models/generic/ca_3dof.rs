extern crate nalgebra as na;

use crate::models::base;

// TODO: can I make this trait SystemF and implement it for any system?
// Seems to be difficult since trait will only allow consts and there will be some
// variables in dynamic systems with mass, inertia, length, etc.

pub struct Model;

impl Model {
    /// Returns the derivative of the state given the state and the time.
    ///
    /// The state is defined as:
    /// * `x[0]`: position x
    /// * `x[1]`: position y
    /// * `x[2]`: position z
    /// * `x[3]`: velocity x
    /// * `x[4]`: velocity y
    /// * `x[5]`: velocity z
    /// * `x[6]`: acceleration x
    /// * `x[7]`: acceleration y
    /// * `x[8]`: acceleration z
    ///
    /// The returned matrix is column-major, meaning that the first column is
    /// the derivative of the first state, the second column is the derivative
    /// of the second state, etc.
    const fn calculate_f() -> na::SMatrix<f64, 9, 9> {
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

/// Constant Acceleration 1 DOF
///
/// System x_dot: [position_x_dot, velocity_x_dot, acceleration_x_dot, ..
///             .. position_y_dot, velocity_y_dot, acceleration_y_dot, ..
///             .. position_z_dot, velocity_z_dot, acceleration_z_dot]
/// System x: [position_x, velocity_x, acceleration_x, ..
///         .. position_y, velocity_y, acceleration_y, ..
///         .. position_z, velocity_z, acceleration_z]
/// System u: none
///
impl base::System<f64, 9, 0> for Model {
    /// Returns the derivative of the state given the state and the time.
    ///
    /// The state is defined as:
    /// * `x[0]`: position x
    /// * `x[1]`: velocity x
    /// * `x[2]`: acceleration x
    /// * `x[3]`: position y
    /// * `x[4]`: velocity y
    /// * `x[5]`: acceleration y
    /// * `x[6]`: position z
    /// * `x[7]`: velocity z
    /// * `x[8]`: acceleration z
    ///
    /// The returned vector is:
    /// * `x_dot[0]`: velocity x
    /// * `x_dot[1]`: acceleration x
    /// * `x_dot[2]`: 0.0 (constant acceleration)
    /// * `x_dot[3]`: velocity y
    /// * `x_dot[4]`: acceleration y
    /// * `x_dot[5]`: 0.0 (constant acceleration)
    /// * `x_dot[6]`: velocity z
    /// * `x_dot[7]`: acceleration z
    /// * `x_dot[8]`: 0.0 (constant acceleration)
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, 9>,
        _u: &nalgebra::SVector<f64, 0>,
        _t: f64,
    ) -> na::SVector<f64, 9> {
        // [0, 1, 0, 0, 0, 0, 0, 0, 0], pos_x_dot = vel_x_dot
        // [0, 0, 1, 0, 0, 0, 0, 0, 0], vel_x_dot = acc_x_dot
        // [0, 0, 0, 0, 0, 0, 0, 0, 0], acc_x_dot = 0 (constant)
        // [0, 0, 0, 0, 1, 0, 0, 0, 0], pos_y_dot = vel_y_dot
        // [0, 0, 0, 0, 0, 1, 0, 0, 0], vel_y_dot = acc_y_dot
        // [0, 0, 0, 0, 0, 0, 0, 0, 0], acc_y_dot = 0 (constant)
        // [0, 0, 0, 0, 0, 0, 0, 1, 0], pos_z_dot = vel_z_dot
        // [0, 0, 0, 0, 0, 0, 0, 0, 1], vel_z_dot = acc_z_dot
        // [0, 0, 0, 0, 0, 0, 0, 0, 0], acc_z_dot = 0 (constant)
        na::SVector::<f64, 9>::from_vec(vec![x[1], x[2], 0., x[4], x[5], 0., x[7], x[8], 0.])
    }

    /// Returns the jacobian of the system given the state and the time.
    ///
    /// The first returned matrix is the jacobian of the state with respect to
    /// the state.
    ///
    /// The second returned matrix is the jacobian of the state with respect to
    /// the input.
    fn calculate_jacobian(
        &self,
        _x: &nalgebra::SVector<f64, 9>,
        _u: &nalgebra::SVector<f64, 0>,
        _t: f64,
    ) -> (na::SMatrix<f64, 9, 9>, na::SMatrix<f64, 9, 0>) {
        (Model::calculate_f(), na::SMatrix::<f64, 9, 0>::zeros())
    }

    /// This function is a no-op and is only defined because it is required by
    /// the trait `base::System`. No parameters to read for this model
    fn calculate_input(
        &self,
        _x: &nalgebra::SVector<f64, 9>,
        _x_dot: &nalgebra::SVector<f64, 9>,
        _t: f64,
    ) -> nalgebra::SVector<f64, 0> {
        na::SVector::<f64, 0>::zeros()
    }

    /// This function is a no-op and is only defined because it is required by
    /// the trait `base::System`. No parameters to read for this model
    fn read(&mut self, _: &str) -> () {}
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{models::base::System, num_methods::runge_kutta};
    use approx::assert_relative_eq;

    ////////////////////////////////////////////////////////////////////////////////
    // test get_derivatives
    ////////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_get_derivatives_zero_state() {
        let model = Model;
        let x = na::SVector::<f64, 9>::zeros();
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let expected = na::SVector::<f64, 9>::zeros();
        let result = model.get_derivatives(&x, &u, t);
        for i in 0..9 {
            assert_eq!(result[i], expected[i]);
        }
    }

    #[test]
    fn test_get_derivatives_non_zero_state() {
        let model = Model;
        let x = na::SVector::<f64, 9>::from_vec(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let expected =
            na::SVector::<f64, 9>::from_vec(vec![2.0, 3.0, 0.0, 5.0, 6.0, 0.0, 8.0, 9.0, 0.0]);
        let result = model.get_derivatives(&x, &u, t);
        for i in 0..9 {
            assert_eq!(result[i], expected[i]);
        }
    }

    #[test]
    fn test_get_derivatives_constant_acceleration() {
        let model = Model;
        let x = na::SVector::<f64, 9>::from_vec(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let expected =
            na::SVector::<f64, 9>::from_vec(vec![2.0, 3.0, 0.0, 5.0, 6.0, 0.0, 8.0, 9.0, 0.0]);
        let result = model.get_derivatives(&x, &u, t);
        for i in 0..9 {
            assert_eq!(result[i], expected[i]);
        }
    }

    #[test]
    #[should_panic]
    fn test_get_derivatives_non_constant_acceleration() {
        let model = Model;
        let x = na::SVector::<f64, 9>::from_vec(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let expected =
            na::SVector::<f64, 9>::from_vec(vec![2.0, 3.0, 1.0, 5.0, 6.0, 1.0, 8.0, 9.0, 1.0]);
        let result = model.get_derivatives(&x, &u, t);
        for i in 0..9 {
            assert_eq!(result[i], expected[i]);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // test calculate_jacobian
    ////////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_calculate_jacobian_state_jacobian() {
        let model = Model;
        let x = na::SVector::<f64, 9>::zeros();
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let (state_jacobian, _) = model.calculate_jacobian(&x, &u, t);
        let expected_state_jacobian = Model::calculate_f();
        for i in 0..9 {
            for j in 0..9 {
                assert_relative_eq!(
                    state_jacobian[(i, j)],
                    expected_state_jacobian[(i, j)],
                    epsilon = 1e-6
                );
            }
        }
    }
    #[test]
    fn test_calculate_jacobian_input_jacobian() {
        let model = Model;
        let x = na::SVector::<f64, 9>::zeros();
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let (_, input_jacobian) = model.calculate_jacobian(&x, &u, t);
        let expected_input_jacobian = na::SMatrix::<f64, 9, 0>::zeros();
        for i in 0..9 {
            for j in 0..0 {
                assert_relative_eq!(
                    input_jacobian[(i, j)],
                    expected_input_jacobian[(i, j)],
                    epsilon = 1e-6
                );
            }
        }
    }
    #[test]
    fn test_calculate_jacobian_shapes() {
        let model = Model;
        let x = na::SVector::<f64, 9>::zeros();
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let (state_jacobian, input_jacobian) = model.calculate_jacobian(&x, &u, t);
        assert_eq!(state_jacobian.shape(), (9, 9));
        assert_eq!(input_jacobian.shape(), (9, 0));
    }

    ////////////////////////////////////////////////////////////////////////////////
    // test calculate_input
    ///////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_calculate_input_zero_vector() {
        let model = Model;
        let x = na::SVector::<f64, 9>::zeros();
        let x_dot = na::SVector::<f64, 9>::zeros();
        let t = 0.0;
        let result = model.calculate_input(&x, &x_dot, t);
        assert_eq!(result.len(), 0);
    }
    #[test]
    fn test_calculate_input_no_panic() {
        let model = Model;
        let x = na::SVector::<f64, 9>::from_vec(vec![1.0; 9]);
        let x_dot = na::SVector::<f64, 9>::from_vec(vec![2.0; 9]);
        let t = 1.0;
        let result = model.calculate_input(&x, &x_dot, t);
        assert_eq!(result.len(), 0);
    }
    #[test]
    fn test_calculate_input_same_result() {
        let model = Model;
        let x1 = na::SVector::<f64, 9>::zeros();
        let x_dot1 = na::SVector::<f64, 9>::zeros();
        let t1 = 0.0;
        let result1 = model.calculate_input(&x1, &x_dot1, t1);
        let x2 = na::SVector::<f64, 9>::from_vec(vec![1.0; 9]);
        let x_dot2 = na::SVector::<f64, 9>::from_vec(vec![2.0; 9]);
        let t2 = 1.0;
        let result2 = model.calculate_input(&x2, &x_dot2, t2);
        assert_eq!(result1.len(), result2.len());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // test prop
    ////////////////////////////////////////////////////////////////////////////////

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
                &runge_kutta::rk4,
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

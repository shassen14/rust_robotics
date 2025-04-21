extern crate nalgebra as na;

use crate::models::base;

pub struct Model;

impl Model {
    /// Returns the derivative of the state given the state and the time.
    ///
    /// The state is defined as:
    /// * `x[0]`: position
    /// * `x[1]`: velocity
    /// * `x[2]`: acceleration
    ///
    /// The returned matrix is column-major, meaning that the first column is
    /// the derivative of the first state, the second column is the derivative
    /// of the second state, etc.
    fn calculate_f() -> na::SMatrix<f64, 3, 3> {
        // input is column based instead of row based
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
impl base::System<f64, 3, 0> for Model {
    /// Returns the derivative of the state given the state and the time.
    ///
    /// The returned vector is:
    /// * `x_dot[0]`: velocity
    /// * `x_dot[1]`: acceleration
    /// * `x_dot[2]`: 0.0 (constant acceleration)
    fn get_derivatives(
        &self,
        x: &nalgebra::SVector<f64, 3>,
        _u: &nalgebra::SVector<f64, 0>,
        _t: f64,
    ) -> na::SVector<f64, 3> {
        na::SVector::<f64, 3>::new(x[1], x[2], 0.)
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

    /// This function is a no-op and is only defined because it is required by
    /// the trait `base::System`. No parameters to read for this model
    ///
    /// # Parameters
    ///
    /// * `self` - The model object
    /// * `_` - The filename to read from, which is ignored
    ///
    /// # Returns
    ///
    /// * `()` - An empty tuple
    fn read(&mut self, _: &str) -> () {}
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::base::System;
    use crate::num_methods::runge_kutta;
    use approx::assert_relative_eq;
    use nalgebra as na;

    /////////////////////////////////////////////////////////////////////////////////
    // test get_derivatives
    /////////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_get_derivatives_zero_state() {
        let model = Model;
        let x = na::SVector::<f64, 3>::zeros();
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let expected = na::SVector::<f64, 3>::zeros();
        let result = model.get_derivatives(&x, &u, t);
        for i in 0..3 {
            assert_relative_eq!(result[i], expected[i]);
        }
    }
    #[test]
    fn test_get_derivatives_non_zero_state() {
        let model = Model;
        let x = na::SVector::<f64, 3>::new(1.0, 2.0, 3.0);
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let expected = na::SVector::<f64, 3>::new(2.0, 3.0, 0.0);
        let result = model.get_derivatives(&x, &u, t);
        for i in 0..3 {
            assert_relative_eq!(result[i], expected[i]);
        }
    }
    #[test]
    fn test_get_derivatives_large_state_values() {
        let model = Model;
        let x = na::SVector::<f64, 3>::new(1000.0, 2000.0, 3000.0);
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let expected = na::SVector::<f64, 3>::new(2000.0, 3000.0, 0.0);
        let result = model.get_derivatives(&x, &u, t);
        for i in 0..3 {
            assert_relative_eq!(result[i], expected[i]);
        }
    }
    #[test]
    fn test_get_derivatives_negative_state_values() {
        let model = Model;
        let x = na::SVector::<f64, 3>::new(-1.0, -2.0, -3.0);
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let expected = na::SVector::<f64, 3>::new(-2.0, -3.0, 0.0);
        let result = model.get_derivatives(&x, &u, t);
        for i in 0..3 {
            assert_relative_eq!(result[i], expected[i]);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////
    // test calculate_input
    /////////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_calculate_jacobian_state_jacobian() {
        let model = Model;
        let x = na::SVector::<f64, 3>::new(1.0, 2.0, 3.0);
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let (state_jacobian, _) = model.calculate_jacobian(&x, &u, t);
        let expected_state_jacobian = Model::calculate_f();
        for i in 0..3 {
            for j in 0..3 {
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
        let x = na::SVector::<f64, 3>::new(1.0, 2.0, 3.0);
        let u = na::SVector::<f64, 0>::zeros();
        let t = 0.0;
        let (_, input_jacobian) = model.calculate_jacobian(&x, &u, t);
        let expected_input_jacobian = na::SMatrix::<f64, 3, 0>::zeros();
        for i in 0..3 {
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
    fn test_calculate_jacobian_different_inputs() {
        let model = Model;
        let x_values = vec![
            na::SVector::<f64, 3>::new(1.0, 2.0, 3.0),
            na::SVector::<f64, 3>::new(-1.0, -2.0, -3.0),
            na::SVector::<f64, 3>::new(0.0, 0.0, 0.0),
        ];
        let u = na::SVector::<f64, 0>::zeros();
        let t_values = vec![0.0, 1.0, -1.0];
        for x in x_values {
            for t in t_values.clone() {
                let (state_jacobian, input_jacobian) = model.calculate_jacobian(&x, &u, t);
                let expected_state_jacobian = Model::calculate_f();
                let expected_input_jacobian = na::SMatrix::<f64, 3, 0>::zeros();
                for i in 0..3 {
                    for j in 0..3 {
                        assert_relative_eq!(
                            state_jacobian[(i, j)],
                            expected_state_jacobian[(i, j)],
                            epsilon = 1e-6
                        );
                    }
                }
                for i in 0..3 {
                    for j in 0..0 {
                        assert_relative_eq!(
                            input_jacobian[(i, j)],
                            expected_input_jacobian[(i, j)],
                            epsilon = 1e-6
                        );
                    }
                }
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////
    // test calculate_input
    /////////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_calculate_input_valid_inputs() {
        let model = Model;
        let x = na::SVector::<f64, 3>::new(1.0, 2.0, 3.0);
        let x_dot = na::SVector::<f64, 3>::new(4.0, 5.0, 6.0);
        let t = 1.0;
        let result = model.calculate_input(&x, &x_dot, t);
        assert_eq!(result, na::SVector::<f64, 0>::zeros());
    }
    #[test]
    fn test_calculate_input_zero_vectors() {
        let model = Model;
        let x = na::SVector::<f64, 3>::zeros();
        let x_dot = na::SVector::<f64, 3>::zeros();
        let t = 0.0;
        let result = model.calculate_input(&x, &x_dot, t);
        assert_eq!(result, na::SVector::<f64, 0>::zeros());
    }
    #[test]
    fn test_calculate_input_large_values() {
        let model = Model;
        let x = na::SVector::<f64, 3>::new(1e10, 2e10, 3e10);
        let x_dot = na::SVector::<f64, 3>::new(4e10, 5e10, 6e10);
        let t = 1e10;
        let result = model.calculate_input(&x, &x_dot, t);
        assert_eq!(result, na::SVector::<f64, 0>::zeros());
    }

    /////////////////////////////////////////////////////////////////////////////////
    // test propagate
    /////////////////////////////////////////////////////////////////////////////////

    #[test]
    fn test_propagate() {
        let model = Model;
        let x = na::SVector::<f64, 3>::new(1.0, 2.0, 3.0);
        let u = na::SVector::<f64, 0>::zeros();
        let start = 0.0;
        let end = 1.0;
        let step = 0.1;
        let mut t = start;
        let mut result = x;
        let mut expected_t = 0.0;

        while t <= end {
            result = base::System::propagate(&model, &result, &u, t, step, &runge_kutta::rk4);
            t += step;
            expected_t += step;
            let expected = na::SVector::<f64, 3>::new(
                1.0 + 2.0 * expected_t + 0.5 * 3.0 * expected_t * expected_t,
                2.0 + 3.0 * expected_t,
                3.0,
            );
            for i in 0..3 {
                assert_relative_eq!(result[i], expected[i]);
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////
    // test calculate_f
    /////////////////////////////////////////////////////////////////////////////////

    #[test]
    fn ca1dof_f_matrix_matches_expected() {
        let expected = na::SMatrix::<f64, 3, 3>::from_array_storage(na::ArrayStorage([
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
        ]));
        let result = Model::calculate_f();
        for i in 0..3 {
            for j in 0..3 {
                assert_relative_eq!(result[(i, j)], expected[(i, j)], epsilon = 1e-6);
            }
        }
    }
}

use crate::num_methods::defs;
use nalgebra as na;
use num_traits::Zero;

// TODO: Add a `write`` function to write to a toml for the system parameters
// This would be helpful with system identification
// TODO: Add `estimate model` function to estimate the parameters given the
// current state and state_dot

/// System is a public interface that all models should implement to
/// learn or to adjust details about the system where
/// T Type, N number of states, M number of inputs
///
pub trait SystemD<T>
where
    T: Zero + std::cmp::PartialOrd,
{
    /// Propagates the system state forward by one time step using a provided integration function.
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state [Nx1]
    /// * `u` - System's current control input [Mx1]
    /// * `t` - Current timestamp
    /// * `dt` - Timestep for integration
    /// * `integrator` - Integration function used to perform the state update over the time step
    ///
    /// # Returns
    ///
    /// * `na::DVector<T>` - The state of the system at time `t + dt`
    ///
    /// # Panics
    ///
    /// * If `dt` is negative
    fn propagate(
        &self,
        x: &na::DVector<T>,
        u: &na::DVector<T>,
        t: T,
        dt: T,
        integrator: &dyn defs::IntegrationFnD<T>,
    ) -> na::DVector<T>
    where
        T: std::ops::Add<Output = T> + Copy,
    {
        // Check that dt is not negative
        assert!(dt >= T::zero(), "dt cannot be negative");

        // Define a closure that computes the derivatives of the system state
        let func = |func_x: &na::DVector<T>, func_t: T| -> na::DVector<T> {
            self.get_derivatives(func_x, u, func_t)
        };

        // Use the provided integrator to compute the state at the next time step
        integrator(&func, x, t, t + dt)
    }

    /// Derivative function x_dot = f(x, u, t) which is a function of
    /// the system's current state, control input, and time
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state [Nx1]
    /// * `u` - System's current control input [Mx1]
    /// * `t` - Current timestamp
    /// * Returns rate of change of the system's states
    fn get_derivatives(&self, x: &na::DVector<T>, u: &na::DVector<T>, t: T) -> na::DVector<T>;

    /// Calculate system's jacobian with respect to state and input partial derivatives
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state [Nx1]
    /// * `u` - System's current control input [Mx1]
    /// * `t` - Current timestamp
    /// * Returns a pair of matrices where the first is the partial derivative of f(x, u, t)
    /// with respect to each state (x), and the second is the partial derivative of f(x, u, t)
    /// with respect to each control input (u). [(NxN), (NxM)] since there are n states and
    /// M inputs. These are the A and B matrices for linear systems
    fn calculate_jacobian(
        &self,
        x: &na::DVector<T>,
        u: &na::DVector<T>,
        t: T,
    ) -> (na::DMatrix<T>, na::DMatrix<T>);

    /// Calculate system's control inputs given the system's current states
    /// to produce a desired rate of change.
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state
    /// * `x_dot_desired` - System's desired rate of change
    /// * `t` - Current timestamp
    /// * Returns control inputs required to obtain the desired rate of change
    fn calculate_input(
        &self,
        x: &na::DVector<T>,
        x_dot_desired: &na::DVector<T>,
        t: T,
    ) -> na::DVector<T>;

    /// Read a toml file to change the model's parameters if any
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, and values that could be changed from the toml file
    /// * `filename` - File name to read values
    fn read(&mut self, filename: &str) -> ();
}

/// SystemH is a public interface that humanoid models should implement where
/// T Type, N number of states, M number of inputs, D dimensions
///
/// This trait implementation should not be implemented alone.
/// It should be paired with System if implemented.
pub trait SystemHD<T, const D: u8> {
    /// Obtains the ratio of number of states to number of dimensions
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * Returns the number of states to number of dimensions ratio
    fn num_states_to_num_dim_ratio(&self) -> u8;

    /// Calculates a feasible initial state where the positions for each link end
    /// are feasible given their length and desired angles
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `angles_desired` - Ordered desired angles from joint 0 -> N relative to the joint prior
    /// * Returns the feasible joint position states where the velocites and accelerations are
    /// zero. The positions/angles are considered to be global.
    fn calculate_feasible_state_initial(&self, angles_desired: &[T]) -> na::DVector<T>;

    /// Inverse Kinematics calculates the angle differences from the current state
    /// to the desired position
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `position_desired` - Desired position for the end effector in order (x,y) or (x,y,z)
    /// * `x` - Current state which should have the positions of each joint
    /// * Returns the angle error for each joint
    fn inverse_kinematics(&self, position_desired: Vec<T>, x: &na::DVector<T>) -> Vec<T>;
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use crate::num_methods::runge_kutta;
//     use approx::assert_relative_eq;
//     use nalgebra as na;

//     // Define a simple SystemD implementation for testing
//     struct SystemDImpl;

//     impl SystemD<f64> for SystemDImpl {
//         fn get_derivatives(
//             &self,
//             x: &na::DVector<f64>,
//             u: &na::DVector<f64>,
//             t: f64,
//         ) -> na::DVector<f64> {
//             // Simple derivative function for testing
//             x.component_mul(&x) + u.component_mul(&u)
//         }
//         fn calculate_input(
//             &self,
//             x: &nalgebra::DVector<f64>,
//             x_dot_desired: &nalgebra::DVector<f64>,
//             t: f64,
//         ) -> nalgebra::DVector<f64> {
//             todo!()
//         }

//         fn calculate_jacobian(
//             &self,
//             x: &nalgebra::DVector<f64>,
//             u: &nalgebra::DVector<f64>,
//             t: f64,
//         ) -> (nalgebra::DMatrix<f64>, nalgebra::DMatrix<f64>) {
//             todo!()
//         }

//         fn read(&mut self, filename: &str) -> () {
//             todo!()
//         }
//     }

//     #[test]
//     fn test_propagate_rk4() {
//         let system = SystemDImpl;
//         let x = na::DVector::from_element(1, 1.0);
//         let u = na::DVector::from_element(1, 0.0);
//         let t = 0.0;
//         let dt = 1.0;
//         let result = system.propagate(&x, &u, t, dt, &runge_kutta::rk4d);
//         assert_relative_eq!(
//             result[0],
//             na::DVector::from_element(1, 2.0)[0],
//             epsilon = 1e-6
//         );
//     }

//     #[test]
//     fn test_propagate_zero_dt() {
//         let system = SystemDImpl;
//         let x = na::DVector::from_element(1, 1.0);
//         let u = na::DVector::from_element(1, 0.0);
//         let t = 0.0;
//         let dt = 0.0;
//         let result = system.propagate(&x, &u, t, dt, &runge_kutta::rk4d);
//         assert_eq!(result, x);
//     }

//     #[test]
//     #[should_panic]
//     fn test_propagate_negative_dt() {
//         let system = SystemDImpl;
//         let x = na::DVector::from_element(1, 1.0);
//         let u = na::DVector::from_element(1, 0.0);
//         let t = 0.0;
//         let dt = -1.0;
//         system.propagate(&x, &u, t, dt, &runge_kutta::rk4d);
//     }
// }

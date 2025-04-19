use crate::num_methods::defs;
use nalgebra as na;

// TODO: Add a `write`` function to write to a toml for the system parameters
// This would be helpful with system identification
// TODO: Add `estimate model` function to east imate the parameters given the
// current state and state_dot

/// System is a public interface that all models should implement to
/// learn or to adjust details about the system where
/// T Type, N number of states, M number of inputs
///
pub trait System<T, const N: usize, const M: usize> {
    /// Propagates one time step given a function x_dot = f(x, u, t)
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state
    /// * `u` - System's current control input
    /// * `t` - Current timestamp
    /// * `dt` - Timestep
    /// * `integrator` - Integration function to integrate over one timestep
    /// * Returns Final State x(t+ dt)
    fn propagate(
        &self,
        x: &na::SVector<T, N>,
        u: &na::SVector<T, M>,
        t: T,
        dt: T,
        integrator: impl defs::IntegrationFn<T, N>,
    ) -> na::SVector<T, N>
    where
        T: std::ops::Add<Output = T> + Copy,
    {
        let func = |func_x: &na::SVector<T, N>, func_t: T| -> na::SVector<T, N> {
            self.get_derivatives(func_x, u, func_t)
        };
        integrator(&func, x, t, t + dt)
    }

    // TODO: is this needed if propagate exists? May want to delete
    /// Propagates one time step given the state space model x_dot = Ax + Bu
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state
    /// * `u` - System's current control input
    /// * `t` - Current timestamp
    /// * `dt` - Timestep
    /// * `integrator` - Integration function to integrate over one timestep
    /// * Returns Final State x(t+ dt)
    // fn propagate_linear(
    //     &self,
    //     x: &na::SVector<T, N>,
    //     u: &na::SVector<T, M>,
    //     t: T,
    //     dt: T,
    //     integrator: impl defs::IntegrationFn<T, N>,
    // ) -> na::SVector<T, N>
    // where
    //     T: std::ops::Add<Output = T> + Copy,
    //     na::SMatrix<T, N, N>: std::ops::Mul<na::SVector<T, N>, Output = na::SVector<T, N>>,
    //     na::SMatrix<T, N, M>: std::ops::Mul<na::SVector<T, M>, Output = na::SVector<T, N>>,
    //     na::SVector<T, N>: std::ops::Add<Output = na::SVector<T, N>>,
    // {
    //     let func = |func_x: &na::SVector<T, N>, func_t: T| -> na::SVector<T, N> {
    //         let linear_model = self.calculate_jacobian(func_x, u, func_t);
    //         // x_dot = Ax + Bu
    //         // TODO: the correct formula is something dx_dot = A* dx + B du
    //         // since the jacobian is derivative relative to the state and inputs.
    //         // Would keeping track of previous states help?
    //         // Integrating this would obtain dx and then add that to the current x?
    //         linear_model.0 * *func_x + linear_model.1 * *u
    //     };
    //     integrator(&func, x, t, t + dt)
    // }

    /// Derivative function x_dot = f(x, u, t) which is a function of
    /// the system's current state, control input, and time
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state
    /// * `u` - System's current control input
    /// * `t` - Current timestamp
    /// * Returns rate of change of the system's states
    fn get_derivatives(
        &self,
        x: &na::SVector<T, N>,
        u: &na::SVector<T, M>,
        t: T,
    ) -> na::SVector<T, N>;

    /// Calculate system's jacobian with respect to state and input partial derivatives
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state
    /// * `u` - System's current control input
    /// * `t` - Current timestamp
    /// * Returns a pair of matrices where the first is the partial derivative of f(x, u, t)
    /// with respect to each state (x), and the second is the partial derivative of f(x, u, t)
    /// with respect to each control input (u). [(NxN), (NxM)] since there are n states and
    /// M inputs. These are the A and B matrices for linear systems
    fn calculate_jacobian(
        &self,
        x: &na::SVector<T, N>,
        u: &na::SVector<T, M>,
        t: T,
    ) -> (na::SMatrix<T, N, N>, na::SMatrix<T, N, M>);

    /// Calculate system's feedforward control inputs given the system's current states
    /// to produce a desired rate of change.
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `x` - System's current state
    /// * `x_dot_desired` - System's desired rate of change
    /// * `t` - Current timestamp
    /// * Returns feedforward control inputs required to obtain the desired rate of change
    fn calculate_input(
        &self,
        x: &na::SVector<T, N>,
        x_dot_desired: &na::SVector<T, N>,
        t: T,
    ) -> na::SVector<T, M>;

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
pub trait SystemH<T, const N: usize, const M: usize, const D: u8> {
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
    fn calculate_feasible_state_initial(&self, angles_desired: &[T]) -> na::SVector<T, N>;

    /// Inverse Kinematics calculates the angle differences from the current state
    /// to the desired position
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, functions, and values
    /// * `position_desired` - Desired position for the end effector in order (x,y) or (x,y,z)
    /// * `x` - Current state which should have the positions of each joint
    /// * Returns the angle error for each joint
    fn inverse_kinematics(&self, position_desired: Vec<f64>, x: &na::SVector<T, N>) -> Vec<T>;
}

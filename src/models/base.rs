extern crate nalgebra as na;
use crate::num_methods::defs;

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
    ///
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
    ///
    fn propagate_linear(
        &self,
        x: &na::SVector<T, N>,
        u: &na::SVector<T, M>,
        t: T,
        dt: T,
        integrator: impl defs::IntegrationFn<T, N>,
    ) -> na::SVector<T, N>
    where
        T: std::ops::Add<Output = T> + Copy,
        na::SMatrix<T, N, N>: std::ops::Mul<na::SVector<T, N>, Output = na::SVector<T, N>>,
        na::SMatrix<T, N, M>: std::ops::Mul<na::SVector<T, M>, Output = na::SVector<T, N>>,
        na::SVector<T, N>: std::ops::Add<Output = na::SVector<T, N>>,
    {
        let func = |func_x: &na::SVector<T, N>, func_t: T| -> na::SVector<T, N> {
            let linear_model = self.calculate_jacobian(func_x, u, func_t);
            // x_dot = Ax + Bu
            linear_model.0 * *func_x + linear_model.1 * *u
        };
        integrator(&func, x, t, t + dt)
    }

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
    ///
    fn get_derivatives(
        &self,
        x: &na::SVector<T, N>,
        u: &na::SVector<T, M>,
        t: T,
    ) -> na::SVector<T, N>;

    /// Get system's jacobian with respect to state and input partial derivatives
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
    ///
    fn calculate_jacobian(
        &self,
        x: &na::SVector<T, N>,
        u: &na::SVector<T, M>,
        t: T,
    ) -> (na::SMatrix<T, N, N>, na::SMatrix<T, N, M>);

    /// Read a toml file to change the model's parameters if any
    ///
    /// # Arguments
    ///
    /// * `self` - Model's parameters, and values that could be changed from the toml file
    /// * `filename` - File name to read values
    ///
    fn read(&mut self, filename: &str) -> ();
}

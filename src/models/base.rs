extern crate nalgebra as na;
use crate::num_methods::runge_kutta;

#[allow(dead_code)]

pub trait System<T, const N: usize, const M: usize> {
    fn propagate(
        &self,
        x: &na::SVector<T, N>,
        u: &na::SVector<T, M>,
        t: T,
        dt: T,
        integrator: impl runge_kutta::IntegrationFn<T, N>,
    ) -> na::SVector<T, N>
    where
        T: std::ops::Add<Output = T> + Copy,
    {
        let func = |func_x: &na::SVector<T, N>, func_t: T| -> na::SVector<T, N> {
            self.get_derivatives(func_x, u, func_t)
        };
        integrator(&func, x, t, t + dt)
    }

    fn get_derivatives(
        &self,
        x: &na::SVector<T, N>,
        u: &na::SVector<T, M>,
        t: T,
    ) -> na::SVector<T, N>;

    fn get_jacobian(
        &self,
        x: &na::SVector<T, N>,
        u: &na::SVector<T, M>,
        t: T,
    ) -> (na::SMatrix<T, N, N>, na::SMatrix<T, M, M>);
}

// use crate::utils::files;
use nalgebra::{self as na};
pub trait VectorFnO<T, R>: Fn(&na::OVector<T, R>, T) -> na::OVector<T, R>
where
    T: std::fmt::Debug + std::cmp::PartialEq + std::clone::Clone + 'static,
    R: na::Dim,
    na::DefaultAllocator: na::allocator::Allocator<R>,
{
}

impl<Func: Fn(&na::OVector<T, R>, T) -> na::OVector<T, R>, T, R> VectorFnO<T, R> for Func
where
    T: std::fmt::Debug + std::cmp::PartialEq + std::clone::Clone + 'static,
    R: na::Dim,
    na::DefaultAllocator: na::allocator::Allocator<R>,
{
}

/// Type alias hack for integrator to use
pub trait IntegrationFnO<T, R>:
    Fn(&dyn VectorFnO<T, R>, &na::OVector<T, R>, T, T) -> na::OVector<T, R>
where
    T: std::fmt::Debug + std::cmp::PartialEq + std::clone::Clone + 'static,
    R: na::Dim,
    na::DefaultAllocator: na::allocator::Allocator<R>,
{
}
impl<Func: Fn(&dyn VectorFnO<T, R>, &na::OVector<T, R>, T, T) -> na::OVector<T, R>, T, R>
    IntegrationFnO<T, R> for Func
where
    T: std::fmt::Debug + std::cmp::PartialEq + std::clone::Clone + 'static,
    R: na::Dim,
    na::DefaultAllocator: na::allocator::Allocator<R>,
{
}

pub fn rk1o<T, R>(
    func: &dyn VectorFnO<T, R>,
    x0: &na::OVector<T, R>,
    t0: T,
    tf: T,
) -> na::OVector<T, R>
where
    T: std::ops::Sub<Output = T> // T - T = T
        + std::ops::Mul<na::OVector<T, R>, Output = na::OVector<T, R>> // T * Vec = Vec
        + Copy // T is copyable
        + std::fmt::Debug
        + std::cmp::PartialEq
        + std::clone::Clone
        + 'static,
    R: na::Dim,
    na::DefaultAllocator: na::allocator::Allocator<R>,
    na::storage::Owned<T, R>: Copy,
    na::OVector<T, R>: std::ops::Add<na::OVector<T, R>, Output = na::OVector<T, R>>, // Vec + Vec = Vec
{
    let dt: T = tf - t0;
    *x0 + dt * func(x0, t0)
}

pub trait VectorFnD<T>: Fn(&na::DVector<T>, T) -> na::DVector<T> {}

impl<Func: Fn(&na::DVector<T>, T) -> na::DVector<T>, T> VectorFnD<T> for Func {}

pub trait IntegrationFnD<T>:
    Fn(&dyn VectorFnD<T>, &na::DVector<T>, T, T) -> na::DVector<T>
{
}
impl<Func: Fn(&dyn VectorFnD<T>, &na::DVector<T>, T, T) -> na::DVector<T>, T> IntegrationFnD<T>
    for Func
{
}

pub fn rk1d<T>(func: &dyn VectorFnD<T>, x0: &na::DVector<T>, t0: T, tf: T) -> na::DVector<T>
where
    T: std::ops::Sub<Output = T> // T - T = T
        + std::ops::Mul<na::DVector<T>, Output = na::DVector<T>> // T * Vec = Vec
        + Copy, // T is copyable
    na::DVector<T>: std::ops::Add<Output = na::DVector<T>>,
{
    let dt: T = tf - t0;
    x0.clone() + dt * func(x0, t0)
}

pub fn rk2d<T>(func: &dyn VectorFnD<T>, x0: &na::DVector<T>, t0: T, tf: T) -> na::DVector<T>
where
    T: num_traits::Float + std::ops::Mul<na::DVector<T>, Output = na::DVector<T>>, // T * Vec = Vec,
    na::DVector<T>: std::ops::Add<na::DVector<T>, Output = na::DVector<T>>, // Vec + Vec = Vec
{
    let dt: T = tf - t0;
    let k1: na::DVector<T> = func(x0, t0);
    let k2: na::DVector<T> = func(&(x0.clone() + dt * k1.clone()), t0 + dt);
    x0.clone() + T::from(0.5).unwrap() * dt * (k1 + k2)
}

pub fn rk3d<T>(func: &dyn VectorFnD<T>, x0: &na::DVector<T>, t0: T, tf: T) -> na::DVector<T>
where
    T: num_traits::Float + std::ops::Mul<na::DVector<T>, Output = na::DVector<T>>, // T * Vec = Vec
    na::DVector<T>: std::ops::Add<na::DVector<T>, Output = na::DVector<T>>, // Vec + Vec = Vec
{
    let dt: T = tf - t0;
    let k1: na::DVector<T> = func(x0, t0);
    let k2: na::DVector<T> = func(&(x0.clone() + dt * k1.clone()), t0 + dt);
    let k3: na::DVector<T> = func(
        &(x0.clone() + (dt / T::from(4).unwrap()) * (k1.clone() + k2.clone())),
        t0 + dt / T::from(2).unwrap(),
    );

    x0.clone() + (dt / T::from(6).unwrap()) * (k1 + k2 + T::from(4).unwrap() * k3)
}

pub fn rk4d<T>(func: &dyn VectorFnD<T>, x0: &na::DVector<T>, t0: T, tf: T) -> na::DVector<T>
where
    T: num_traits::Float + std::ops::Mul<na::DVector<T>, Output = na::DVector<T>>, // T * Vec = Vec
    na::DVector<T>: std::ops::Add<na::DVector<T>, Output = na::DVector<T>>, // Vec + Vec = Vec
{
    let two: T = T::from(2).unwrap();
    let dt: T = tf - t0;
    let k1: na::DVector<T> = func(x0, t0);
    let k2: na::DVector<T> = func(&(x0.clone() + (dt / two) * k1.clone()), t0 + dt / two);
    let k3: na::DVector<T> = func(&(x0.clone() + (dt / two) * k2.clone()), t0 + dt / two);
    let k4: na::DVector<T> = func(&(x0.clone() + dt * k3.clone()), tf);

    x0.clone() + (dt / T::from(6).unwrap()) * (k1 + two * k2 + two * k3 + k4)
}

mod tests {
    use nalgebra::{dmatrix, dvector};

    use super::*;

    fn test_integration_o<R>(
        integrator: impl IntegrationFnO<f64, R>,
        func: &dyn VectorFnO<f64, R>,
        x0: &na::OVector<f64, R>,
        start: f64,
        end: f64,
        step: f64,
        expected_result: &na::OVector<f64, R>,
        max_error: &na::OVector<f64, R>,
    ) -> na::OVector<f64, R>
    where
        R: na::Dim,
        na::DefaultAllocator: na::allocator::Allocator<R>,
        na::storage::Owned<f64, R>: Copy,
    {
        let mut t0: f64 = start;
        let mut tf: f64 = start + step;
        let mut result: na::OVector<f64, R> = *x0;

        while tf <= end {
            result = integrator(func, &result, t0, tf);
            t0 = tf;
            tf += step;
        }
        // Testing total error is less than the expected max error from
        // the integrator
        let total_error: na::OVector<f64, R> = expected_result - result;

        for i in 0..R::try_to_usize().unwrap() {
            println!(
                "i: {},  total error: {},  max error: {}\n",
                i,
                total_error[i].abs(),
                max_error[i].abs()
            );
            assert!(total_error[i].abs() < max_error[i].abs());
        }
        // return
        result
    }

    fn test_cos_o(
        integrator: impl IntegrationFnO<f64, na::U1>,
        start: f64,
        end: f64,
        step: f64,
        max_error: f64,
    ) -> () {
        let func =
            |_: &na::OVector<f64, na::U1>, t: f64| na::OVector::<f64, na::U1>::new(f64::cos(t));

        let expected = f64::sin(end) - f64::sin(start);
        test_integration_o(
            integrator,
            &func,
            &na::SVector::<f64, 1>::new(0.),
            start,
            end,
            step,
            &na::SVector::<f64, 1>::new(expected),
            &na::SVector::<f64, 1>::new(max_error),
        );
    }

    fn test_integration_d(
        integrator: impl IntegrationFnD<f64>,
        func: &dyn VectorFnD<f64>,
        x0: &na::DVector<f64>,
        start: f64,
        end: f64,
        step: f64,
        expected_result: &na::DVector<f64>,
        max_error: &na::DVector<f64>,
    ) -> na::DVector<f64> {
        let mut t0: f64 = start;
        let mut tf: f64 = start + step;
        let mut result: na::DVector<f64> = x0.clone();

        while tf <= end {
            result = integrator(func, &result, t0, tf);
            t0 = tf;
            tf += step;
        }
        // Testing total error is less than the expected max error from
        // the integrator
        let total_error: na::DVector<f64> = expected_result - result.clone();

        for i in 0..x0.nrows() {
            println!(
                "i: {},  total error: {},  max error: {}\n",
                i,
                total_error[i].abs(),
                max_error[i].abs()
            );
            assert!(total_error[i].abs() < max_error[i].abs());
        }
        // return
        result
    }

    fn test_cos_d(
        integrator: impl IntegrationFnD<f64>,
        start: f64,
        end: f64,
        step: f64,
        max_error: f64,
    ) -> () {
        let func = |_: &na::DVector<f64>, t: f64| dvector![f64::cos(t)];

        let expected = f64::sin(end) - f64::sin(start);
        test_integration_d(
            integrator,
            &func,
            &na::DVector::<f64>::zeros(1),
            start,
            end,
            step,
            &dvector![expected],
            &dvector![max_error],
        );
    }

    #[test]
    fn rk1_cos_o() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 2e-2;
        test_cos_o(rk1o, START, END, STEP, MAX_ERROR);
    }

    #[test]
    fn rk1_cos_d() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 2e-2;
        test_cos_d(rk1d, START, END, STEP, MAX_ERROR);
    }
}

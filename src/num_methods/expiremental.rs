// use crate::utils::files;
use nalgebra as na;
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

pub trait VectorFnD<T>: Fn(&na::DVector<T>, T) -> na::DVector<T> {}

impl<Func: Fn(&na::DVector<T>, T) -> na::DVector<T>, T> VectorFnD<T> for Func {}

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

// pub fn rk1d<T>(func: &dyn VectorFnD<T>, x0: &na::DVector<T>, t0: T, tf: T) -> na::DVector<T>
// where
//     T: std::ops::Sub<Output = T> // T - T = T
//         + std::ops::Mul<na::OVector<T, R>, Output = na::OVector<T, R>> // T * Vec = Vec
//         + Copy, // T is copyable
//     na::OVector<T, R>: std::ops::Add<na::OVector<T, R>, Output = na::OVector<T, R>>, // Vec + Vec = Vec
//     T: std::fmt::Debug + std::cmp::PartialEq + std::clone::Clone + 'static,
//     R: na::Dim,
//     na::DefaultAllocator: na::allocator::Allocator<R>,
//     na::storage::Owned<T, R>: Copy,
// {
//     let dt: T = tf - t0;
//     *x0 + dt * func(x0, t0)
// }

mod tests {
    use super::*;

    fn test_integration<R>(
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

    fn test_cos(
        integrator: impl IntegrationFnO<f64, na::U1>,
        start: f64,
        end: f64,
        step: f64,
        max_error: f64,
    ) -> () {
        let func =
            |_: &na::OVector<f64, na::U1>, t: f64| na::OVector::<f64, na::U1>::new(f64::cos(t));

        let expected = f64::sin(end) - f64::sin(start);
        test_integration(
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

    #[test]
    fn rk1_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 2e-2;
        test_cos(rk1o, START, END, STEP, MAX_ERROR);
    }
}

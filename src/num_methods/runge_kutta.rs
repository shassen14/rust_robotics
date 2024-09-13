use crate::num_methods::defs::VectorFn;
use crate::num_methods::defs::VectorFnD;

use nalgebra as na;
use num_traits;

#[allow(dead_code)]
/// Integrate a function, dx/dt = func(x, t), using Runge-Kutta 1st order (Euler) for a single timestep
///
/// # Arguments
///
/// * `func` - Function to integrate
/// * `x0` - Initital states x(t0)
/// * `t0` - Initial time
/// * `tf` - Final time
/// * Returns Final State x(tf)
///
pub fn rk1<T, const R: usize>(
    func: &dyn VectorFn<T, R>,
    x0: &na::SVector<T, R>,
    t0: T,
    tf: T,
) -> na::SVector<T, R>
where
    T: num_traits::Float + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>>, // T * Vec = Vec
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>, // Vec + Vec = Vec
{
    let dt: T = tf - t0;
    *x0 + dt * func(x0, t0)
}

#[allow(dead_code)]
/// Integrate a function, dx/dt = func(x, t), using Runge-Kutta 2nd order (midpoint) for a single timestep
///
/// # Arguments
///
/// * `func` - Function to integrate
/// * `x0` - Initital states x(t0)
/// * `t0` - Initial time
/// * `tf` - Final time
/// * Returns Final State x(tf)
///
pub fn rk2<T, const R: usize>(
    func: &dyn VectorFn<T, R>,
    x0: &na::SVector<T, R>,
    t0: T,
    tf: T,
) -> na::SVector<T, R>
where
    T: num_traits::Float + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>>, // T * Vec = Vec,
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>, // Vec + Vec = Vec
{
    let dt: T = tf - t0;
    let k1: na::SVector<T, R> = func(x0, t0);
    let k2: na::SVector<T, R> = func(&(*x0 + dt * k1), t0 + dt);
    *x0 + T::from(0.5).unwrap() * dt * (k1 + k2)
}

#[allow(dead_code)]
/// Integrate a function, dx/dt = func(x, t), using Runge-Kutta 3rd order for a single timestep
///
/// # Arguments
///
/// * `func` - Function to integrate
/// * `x0` - Initital states x(t0)
/// * `t0` - Initial time
/// * `tf` - Final time
/// * Returns Final State x(tf)
///
pub fn rk3<T, const R: usize>(
    func: &dyn VectorFn<T, R>,
    x0: &na::SVector<T, R>,
    t0: T,
    tf: T,
) -> na::SVector<T, R>
where
    T: num_traits::Float + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>>, // T * Vec = Vec
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>, // Vec + Vec = Vec
{
    let dt: T = tf - t0;
    let k1: na::SVector<T, R> = func(x0, t0);
    let k2: na::SVector<T, R> = func(&(*x0 + dt * k1), t0 + dt);
    let k3: na::SVector<T, R> = func(
        &(*x0 + (dt / T::from(4).unwrap()) * (k1 + k2)),
        t0 + dt / T::from(2).unwrap(),
    );

    *x0 + (dt / T::from(6).unwrap()) * (k1 + k2 + T::from(4).unwrap() * k3)
}

#[allow(dead_code)]
/// Integrate a function, dx/dt = func(x, t), using Runge-Kutta 4th order for a single timestep
///
/// # Arguments
///
/// * `func` - Function to integrate
/// * `x0` - Initital states x(t0)
/// * `t0` - Initial time
/// * `tf` - Final time
/// * Returns Final State x(tf)
///
pub fn rk4<T, const R: usize>(
    func: &dyn VectorFn<T, R>,
    x0: &na::SVector<T, R>,
    t0: T,
    tf: T,
) -> na::SVector<T, R>
where
    T: num_traits::Float + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>>, // T * Vec = Vec
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>, // Vec + Vec = Vec
{
    let two: T = T::from(2).unwrap();
    let dt: T = tf - t0;
    let k1: na::SVector<T, R> = func(x0, t0);
    let k2: na::SVector<T, R> = func(&(*x0 + (dt / two) * k1), t0 + dt / two);
    let k3: na::SVector<T, R> = func(&(*x0 + (dt / two) * k2), t0 + dt / two);
    let k4: na::SVector<T, R> = func(&(*x0 + dt * k3), tf);

    *x0 + (dt / T::from(6).unwrap()) * (k1 + two * k2 + two * k3 + k4)
}

// TODO: write descriptions.
// TODO: check matrix size???
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

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*;
    use crate::num_methods::defs::IntegrationFn;
    use crate::num_methods::defs::IntegrationFnD;

    /// Helper function to est integrating a function over a time interval [start, end]
    ///
    /// # Arguments
    ///
    /// * `integrator` - Integrator to use
    /// * `func` - Function to integrate
    /// * `start` -  Start time for integration
    /// * `end` - Final time for integration
    /// * `step` - Step size for integration
    /// * `expected_result` - Expected result of integration [start, final]
    /// * `max_error` - Maximum allowed error between final result and the expected result
    /// * Returns Final State x(tf)
    ///
    fn test_integration<const R: usize>(
        integrator: impl IntegrationFn<f64, R>,
        func: &dyn VectorFn<f64, R>,
        x0: &na::SVector<f64, R>,
        start: f64,
        end: f64,
        step: f64,
        expected_result: &na::SVector<f64, R>,
        max_error: &na::SVector<f64, R>,
    ) -> na::SVector<f64, R> {
        let mut t0: f64 = start;
        let mut tf: f64 = start + step;
        let mut result: na::SVector<f64, R> = *x0;

        while tf <= end {
            result = integrator(func, &result, t0, tf);
            t0 = tf;
            tf += step;
        }
        // Testing total error is less than the expected max error from
        // the integrator
        let total_error: na::SVector<f64, R> = expected_result - result;

        for i in 0..R {
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

    /// Test integrating a cos(t) over a time interval [start, end]
    ///
    /// # Arguments
    ///
    /// * `integrator` - Integrator to use
    /// * `start` -  Start time for integration
    /// * `end` - Final time for integration
    /// * `step` - Step size for integration
    /// * `max_error` - Maximum allowed error between final result and the expected result
    ///
    fn test_cos(
        integrator: impl IntegrationFn<f64, 1>,
        start: f64,
        end: f64,
        step: f64,
        max_error: f64,
    ) -> () {
        let func = |_: &na::SVector<f64, 1>, t: f64| na::SVector::<f64, 1>::new(f64::cos(t));

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

    /// Test integrating a sin(t) over a time interval [start, end]
    ///
    /// # Arguments
    ///
    /// * `integrator` - Integrator to use
    /// * `start` -  Start time for integration
    /// * `end` - Final time for integration
    /// * `step` - Step size for integration
    /// * `max_error` - Maximum allowed error between final result and the expected result
    ///
    fn test_sin(
        integrator: impl IntegrationFn<f64, 1>,
        start: f64,
        end: f64,
        step: f64,
        max_error: f64,
    ) -> () {
        let func = |_: &na::SVector<f64, 1>, t: f64| na::SVector::<f64, 1>::new(f64::sin(t));

        let expected = -f64::cos(end) - -f64::cos(start);
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

    /// Test integrating a constant velocity system over a time interval [start, end]
    ///
    /// # Arguments
    ///
    /// * `integrator` - Integrator to use
    /// * `start` -  Start time for integration
    /// * `end` - Final time for integration
    /// * `step` - Step size for integration
    /// * `max_error` - Maximum allowed error between final result and the expected result
    ///
    fn test_cv(
        integrator: impl IntegrationFn<f64, 2>,
        x0: f64,
        vel0: f64,
        start: f64,
        end: f64,
        step: f64,
        max_error: &na::SVector<f64, 2>,
    ) -> () {
        let func = |x: &na::SVector<f64, 2>, _: f64| {
            #[allow(non_snake_case)]
            let A: na::SMatrix<f64, 2, 2> = na::SMatrix::<f64, 2, 2>::new(0., 1., 0., 0.);
            A * x
        };

        let total_time: f64 = end - start;
        test_integration(
            integrator,
            &func,
            &na::SVector::<f64, 2>::new(x0, vel0),
            start,
            end,
            step,
            &na::SVector::<f64, 2>::new(x0 + vel0 * total_time, vel0),
            max_error,
        );
    }

    /// Test integrating a constant acceleration system over a time interval [start, end]
    ///
    /// # Arguments
    ///
    /// * `integrator` - Integrator to use
    /// * `start` -  Start time for integration
    /// * `end` - Final time for integration
    /// * `step` - Step size for integration
    /// * `max_error` - Maximum allowed error between final result and the expected result
    ///
    fn test_ca(
        integrator: impl IntegrationFn<f64, 3>,
        x0: f64,
        vel0: f64,
        accel0: f64,
        start: f64,
        end: f64,
        step: f64,
        max_error: &na::SVector<f64, 3>,
    ) -> () {
        let func = |x: &na::SVector<f64, 3>, _: f64| {
            #[allow(non_snake_case)]
            let A: na::SMatrix<f64, 3, 3> =
                na::SMatrix::<f64, 3, 3>::new(0., 1., 0., 0., 0., 1., 0., 0., 0.);
            A * x
        };

        let total_time: f64 = end - start;
        test_integration(
            integrator,
            &func,
            &na::SVector::<f64, 3>::new(x0, vel0, accel0),
            start,
            end,
            step,
            &na::SVector::<f64, 3>::new(
                x0 + vel0 * total_time + 0.5 * accel0 * total_time * total_time,
                vel0 + accel0 * total_time,
                accel0,
            ),
            max_error,
        );
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk1_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 2e-2;
        test_cos(rk1, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk1_sin() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-4;
        test_sin(rk1, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk1_cv() {
        const X0: f64 = 0.;
        const VEL0: f64 = 2.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;
        const MAX_ERROR: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(1e-12, 1e-12);
        test_cv(rk1, X0, VEL0, START, END, STEP, &MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk1_ca() {
        const X0: f64 = 0.;
        const VEL0: f64 = 1.;
        const ACCEL0: f64 = 1.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;

        const MAX_ERROR: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(1e-1, 1e-12, 1e-12);
        test_ca(rk1, X0, VEL0, ACCEL0, START, END, STEP, &MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk2_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-2;
        test_cos(rk2, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk2_sin() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-4;
        test_sin(rk2, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk2_cv() {
        const X0: f64 = 0.;
        const VEL0: f64 = 2.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;
        const MAX_ERROR: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(1e-12, 1e-12);
        test_cv(rk2, X0, VEL0, START, END, STEP, &MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk2_ca() {
        const X0: f64 = 0.;
        const VEL0: f64 = 1.;
        const ACCEL0: f64 = 1.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;

        const MAX_ERROR: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(1e-11, 1e-12, 1e-12);
        test_ca(rk2, X0, VEL0, ACCEL0, START, END, STEP, &MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk3_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-2;
        test_cos(rk3, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk3_sin() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-5;
        test_sin(rk3, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk3_cv() {
        const X0: f64 = 0.;
        const VEL0: f64 = 2.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;
        const MAX_ERROR: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(1e-12, 1e-12);
        test_cv(rk3, X0, VEL0, START, END, STEP, &MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk3_ca() {
        const X0: f64 = 0.;
        const VEL0: f64 = 1.;
        const ACCEL0: f64 = 1.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;

        const MAX_ERROR: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(1e-11, 1e-12, 1e-12);
        test_ca(rk3, X0, VEL0, ACCEL0, START, END, STEP, &MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk4_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-2;
        test_cos(rk4, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk4_sin() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-5;
        test_sin(rk4, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk4_cv() {
        const X0: f64 = 0.;
        const VEL0: f64 = 2.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;
        const MAX_ERROR: na::SVector<f64, 2> = na::SVector::<f64, 2>::new(1e-12, 1e-12);
        test_cv(rk4, X0, VEL0, START, END, STEP, &MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk4_ca() {
        const X0: f64 = 0.;
        const VEL0: f64 = 1.;
        const ACCEL0: f64 = 1.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;

        const MAX_ERROR: na::SVector<f64, 3> = na::SVector::<f64, 3>::new(1e-11, 1e-12, 1e-12);
        test_ca(rk4, X0, VEL0, ACCEL0, START, END, STEP, &MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Dynamic Matrix / Vector Implementations
    ///////////////////////////////////////////////////////////////////////////
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
        let func = |_: &na::DVector<f64>, t: f64| na::dvector![f64::cos(t)];

        let expected = f64::sin(end) - f64::sin(start);
        test_integration_d(
            integrator,
            &func,
            &na::DVector::<f64>::zeros(1),
            start,
            end,
            step,
            &na::dvector![expected],
            &na::dvector![max_error],
        );
    }

    /// Test integrating a sin(t) over a time interval [start, end]
    ///
    /// # Arguments
    ///
    /// * `integrator` - Integrator to use
    /// * `start` -  Start time for integration
    /// * `end` - Final time for integration
    /// * `step` - Step size for integration
    /// * `max_error` - Maximum allowed error between final result and the expected result
    ///
    fn test_sin_d(
        integrator: impl IntegrationFnD<f64>,
        start: f64,
        end: f64,
        step: f64,
        max_error: f64,
    ) -> () {
        let func = |_: &na::DVector<f64>, t: f64| na::dvector![f64::sin(t)];

        let expected = -f64::cos(end) - -f64::cos(start);
        test_integration_d(
            integrator,
            &func,
            &na::DVector::<f64>::zeros(1),
            start,
            end,
            step,
            &na::dvector![expected],
            &na::dvector![max_error],
        );
    }

    /// Test integrating a constant velocity system over a time interval [start, end]
    ///
    /// # Arguments
    ///
    /// * `integrator` - Integrator to use
    /// * `start` -  Start time for integration
    /// * `end` - Final time for integration
    /// * `step` - Step size for integration
    /// * `max_error` - Maximum allowed error between final result and the expected result
    ///
    fn test_cv_d(
        integrator: impl IntegrationFnD<f64>,
        x0: f64,
        vel0: f64,
        start: f64,
        end: f64,
        step: f64,
        max_error: &na::DVector<f64>,
    ) -> () {
        let func = |x: &na::DVector<f64>, _: f64| {
            #[allow(non_snake_case)]
            let A: na::DMatrix<f64> = na::dmatrix![0., 1.; 0., 0.];
            A * x
        };

        let total_time: f64 = end - start;
        test_integration_d(
            integrator,
            &func,
            &na::dvector![x0, vel0],
            start,
            end,
            step,
            &na::dvector![x0 + vel0 * total_time, vel0],
            max_error,
        );
    }

    /// Test integrating a constant acceleration system over a time interval [start, end]
    ///
    /// # Arguments
    ///
    /// * `integrator` - Integrator to use
    /// * `start` -  Start time for integration
    /// * `end` - Final time for integration
    /// * `step` - Step size for integration
    /// * `max_error` - Maximum allowed error between final result and the expected result
    ///
    fn test_ca_d(
        integrator: impl IntegrationFnD<f64>,
        x0: f64,
        vel0: f64,
        accel0: f64,
        start: f64,
        end: f64,
        step: f64,
        max_error: &na::DVector<f64>,
    ) -> () {
        let func = |x: &na::DVector<f64>, _: f64| {
            #[allow(non_snake_case)]
            let A: na::DMatrix<f64> = na::dmatrix![0., 1., 0.; 0., 0., 1.; 0., 0., 0.];
            A * x
        };

        let total_time: f64 = end - start;
        test_integration_d(
            integrator,
            &func,
            &na::dvector![x0, vel0, accel0],
            start,
            end,
            step,
            &na::dvector![
                x0 + vel0 * total_time + 0.5 * accel0 * total_time * total_time,
                vel0 + accel0 * total_time,
                accel0,
            ],
            max_error,
        );
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk1d_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 2e-2;
        test_cos_d(rk1d, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk1d_sin() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-4;
        test_sin_d(rk1d, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk1d_cv() {
        const X0: f64 = 0.;
        const VEL0: f64 = 2.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;
        let max_error: na::DVector<f64> = na::dvector![1e-12, 1e-12];
        test_cv_d(rk1d, X0, VEL0, START, END, STEP, &max_error);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk1d_ca() {
        const X0: f64 = 0.;
        const VEL0: f64 = 1.;
        const ACCEL0: f64 = 1.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;

        let max_error: na::DVector<f64> = na::dvector![1e-1, 1e-12, 1e-12];
        test_ca_d(rk1d, X0, VEL0, ACCEL0, START, END, STEP, &max_error);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk2d_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-2;
        test_cos_d(rk2d, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk2d_sin() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-4;
        test_sin_d(rk2d, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk2d_cv() {
        const X0: f64 = 0.;
        const VEL0: f64 = 2.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;
        let max_error: na::DVector<f64> = na::dvector![1e-12, 1e-12];
        test_cv_d(rk2d, X0, VEL0, START, END, STEP, &max_error);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk2d_ca() {
        const X0: f64 = 0.;
        const VEL0: f64 = 1.;
        const ACCEL0: f64 = 1.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;

        let max_error: na::DVector<f64> = na::dvector![1e-11, 1e-12, 1e-12];
        test_ca_d(rk2d, X0, VEL0, ACCEL0, START, END, STEP, &max_error);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk3d_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-2;
        test_cos_d(rk3d, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk3d_sin() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-5;
        test_sin_d(rk3d, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk3d_cv() {
        const X0: f64 = 0.;
        const VEL0: f64 = 2.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;
        let max_error: na::DVector<f64> = na::dvector![1e-12, 1e-12];
        test_cv_d(rk3d, X0, VEL0, START, END, STEP, &max_error);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk3d_ca() {
        const X0: f64 = 0.;
        const VEL0: f64 = 1.;
        const ACCEL0: f64 = 1.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;
        let max_error: na::DVector<f64> = na::dvector![1e-11, 1e-12, 1e-12];
        test_ca_d(rk3d, X0, VEL0, ACCEL0, START, END, STEP, &max_error);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk4d_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-2;
        test_cos_d(rk4d, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk4d_sin() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 1e-5;
        test_sin_d(rk4d, START, END, STEP, MAX_ERROR);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk4d_cv() {
        const X0: f64 = 0.;
        const VEL0: f64 = 2.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;
        let max_error: na::DVector<f64> = na::dvector![1e-12, 1e-12];
        test_cv_d(rk4d, X0, VEL0, START, END, STEP, &max_error);
    }

    ///////////////////////////////////////////////////////////////////////////

    #[test]
    fn rk4d_ca() {
        const X0: f64 = 0.;
        const VEL0: f64 = 1.;
        const ACCEL0: f64 = 1.;
        const START: f64 = 0.;
        const END: f64 = 10.;
        const STEP: f64 = 0.01;

        let max_error: na::DVector<f64> = na::dvector![1e-11, 1e-12, 1e-12];
        test_ca_d(rk4d, X0, VEL0, ACCEL0, START, END, STEP, &max_error);
    }
}

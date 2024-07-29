extern crate nalgebra as na;

/// Generic function pointer alias which represents dx/dt = f(x,t)
// Old way but this did not take in any outside variables
// Praying for trait alias or type alias for impl
// pub type VectorFn<T, const R: usize> = fn(&na::SVector<T, R>, T) -> na::SVector<T, R>;
pub trait VectorFn<T, const R: usize>: Fn(&na::SVector<T, R>, T) -> na::SVector<T, R> {}

impl<Func: Fn(&na::SVector<T, R>, T) -> na::SVector<T, R>, T, const R: usize> VectorFn<T, R>
    for Func
{
}

/// Type alias hack for integrator to use
// Old way but this did not take in any outside variables
// Praying for trait alias or type alias for impl
// pub type IntegrationFn<T, const R: usize> =
//     fn(impl VectorFn<T, R>, &na::SVector<T, R>, T, T) -> na::SVector<T, R>;
pub trait IntegrationFn<T, const R: usize>:
    Fn(&dyn VectorFn<T, R>, &na::SVector<T, R>, T, T) -> na::SVector<T, R>
{
}
impl<
        Func: Fn(&dyn VectorFn<T, R>, &na::SVector<T, R>, T, T) -> na::SVector<T, R>,
        T,
        const R: usize,
    > IntegrationFn<T, R> for Func
{
}

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
    T: std::ops::Sub<Output = T> // T - T = T
        + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>> // T * Vec = Vec
        + Copy, // T is copyable
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
    T: std::ops::Sub<Output = T> // T - T = T
        + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>> // T * Vec = Vec
        + std::ops::Add<Output = T> // T + T = T
        + Copy, // T is copyable
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>, // Vec + Vec = Vec
    f64: std::ops::Mul<T, Output = T>,                                               // f64 * T = T
{
    let dt: T = tf - t0;
    let k1: na::SVector<T, R> = func(x0, t0);
    let k2: na::SVector<T, R> = func(&(*x0 + dt * k1), t0 + dt);
    *x0 + 0.5 * dt * (k1 + k2)
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
    T: std::ops::Sub<Output = T> // T - T = T
        + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>> // T * Vec = Vec
        + std::ops::Add<Output = T> // T + T = T
        + std::ops::Div<f64, Output = T> // T / f64 = T
        + Copy, // T is copyable
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>, // Vec + Vec = Vec
    f64: std::ops::Mul<T, Output = T> // f64 * T = T
        + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>>, // f64 * Vec = Vec
{
    let dt: T = tf - t0;
    let k1: na::SVector<T, R> = func(x0, t0);
    let k2: na::SVector<T, R> = func(&(*x0 + dt * k1), t0 + dt);
    let k3: na::SVector<T, R> = func(&(*x0 + (dt / 4.) * (k1 + k2)), t0 + dt / 2.);

    *x0 + (dt / 6.) * (k1 + k2 + 4. * k3)
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
    T: std::ops::Sub<Output = T> // T - T = T
        + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>> // T * Vec = Vec
        + std::ops::Div<f64, Output = T> // T / f64 = T
        + std::ops::Mul<f64, Output = T> // T * f64 = T
        + std::ops::Add<Output = T> // T + T = T
        + Copy, // T is copyable
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>, // Vec + Vec = Vec
    f64: std::ops::Mul<T, Output = T> // f64 * T = T
        + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>>, // f64 * Vec = Vec
{
    let dt: T = tf - t0;
    let k1: na::SVector<T, R> = func(x0, t0);
    let k2: na::SVector<T, R> = func(&(*x0 + (dt / 2.) * k1), t0 + dt / 2.);
    let k3: na::SVector<T, R> = func(&(*x0 + (dt / 2.) * k2), t0 + dt / 2.);
    let k4: na::SVector<T, R> = func(&(*x0 + dt * k3), tf);

    *x0 + (dt / 6.) * (k1 + 2. * k2 + 2. * k3 + k4)
}

///////////////////////////////////////////////////////////////////////////////
// Tests
///////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests {
    use super::*;

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

    /////////////////////////////////////////////////////////////////////////

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
        const N: usize = 3;
        let func = |x: &na::SVector<f64, N>, _: f64| {
            #[allow(non_snake_case)]
            let A: na::SMatrix<f64, N, N> =
                na::SMatrix::<f64, N, N>::new(0., 1., 0., 0., 0., 1., 0., 0., 0.);
            A * x
        };

        let total_time: f64 = end - start;
        test_integration(
            integrator,
            &func,
            &na::SVector::<f64, N>::new(x0, vel0, accel0),
            start,
            end,
            step,
            &na::SVector::<f64, N>::new(
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
}

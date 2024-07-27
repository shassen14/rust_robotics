extern crate nalgebra as na;

#[allow(dead_code)]
// Generic function pointer alias which represents dx/dt = f(x,t)
type VectorFn<T, const R: usize> = fn(na::SVector<T, R>, T) -> na::SVector<T, R>;

#[allow(dead_code)]
// Euler method
// This is stating for Type T: Subtracting another T will give an output of T
// Type T: Multiplying with na::SVector will return a SVector
// Type T: Allowing copying in order to input into func
// where SVector: added with another SVector will output SVector
fn rk1<T, const R: usize>(
    func: VectorFn<T, R>,
    x0: na::SVector<T, R>,
    t0: T,
    tf: T,
) -> na::SVector<T, R>
where
    T: std::ops::Sub<Output = T>
        + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>>
        + Copy,
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>,
{
    let dt: T = tf - t0;
    x0 + dt * func(x0, t0)
}

#[allow(dead_code)]
fn rk4<T, const R: usize>(
    func: VectorFn<T, R>,
    x0: na::SVector<T, R>,
    t0: T,
    tf: T,
) -> na::SVector<T, R>
where
    T: std::ops::Sub<Output = T>
        + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>>
        + std::ops::Div<f64, Output = T>
        + std::ops::Mul<f64, Output = T>
        + std::ops::Add<Output = T>
        + Copy,
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>,
    f64:
        std::ops::Mul<T, Output = T> + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>>,
{
    let dt: T = tf - t0;
    let k1: na::SVector<T, R> = func(x0, t0);
    let k2: na::SVector<T, R> = func(x0 + (dt / 2.) * k1, t0 + dt / 2.);
    let k3: na::SVector<T, R> = func(x0 + (dt / 2.) * k2, t0 + dt / 2.);
    let k4: na::SVector<T, R> = func(x0 + dt * k3, tf);

    x0 + (1. / 6.) * dt * (k1 + 2. * k2 + 2. * k3 + k4)
}
#[cfg(test)]
mod tests {
    use super::*;

    // Alias for integrator to use
    type IntegrationFn<const R: usize> =
        fn(VectorFn<f64, R>, na::SVector<f64, R>, f64, f64) -> na::SVector<f64, R>;

    // Helper function to check the results from integrations
    fn test_integration<const R: usize>(
        integrator: IntegrationFn<R>,
        func: VectorFn<f64, R>,
        x0: na::SVector<f64, R>,
        start: f64,
        end: f64,
        step: f64,
        expected_result: na::SVector<f64, R>,
        max_error: na::SVector<f64, R>,
    ) -> na::SVector<f64, R> {
        let mut t0: f64 = start;
        let mut tf: f64 = start + step;
        let mut result: na::SVector<f64, R> = x0;

        while tf <= end {
            result = integrator(func, result, t0, tf);
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

    ///////////////////////////////////////////////////////////////////////////

    // cos test
    fn test_cos(
        integrator: IntegrationFn<1>,
        start: f64,
        end: f64,
        step: f64,
        max_error: f64,
    ) -> () {
        let func = |_: na::SVector<f64, 1>, t: f64| na::SVector::<f64, 1>::new(f64::cos(t));

        let expected = f64::sin(end) - f64::sin(start);
        test_integration(
            integrator,
            func,
            na::SVector::<f64, 1>::new(0.),
            start,
            end,
            step,
            na::SVector::<f64, 1>::new(expected),
            na::SVector::<f64, 1>::new(max_error),
        );
    }

    ///////////////////////////////////////////////////////////////////////////

    // sin test
    fn test_sin(
        integrator: IntegrationFn<1>,
        start: f64,
        end: f64,
        step: f64,
        max_error: f64,
    ) -> () {
        let func = |_: na::SVector<f64, 1>, t: f64| na::SVector::<f64, 1>::new(f64::sin(t));

        let expected = -f64::cos(end) - -f64::cos(start);
        test_integration(
            integrator,
            func,
            na::SVector::<f64, 1>::new(0.),
            start,
            end,
            step,
            na::SVector::<f64, 1>::new(expected),
            na::SVector::<f64, 1>::new(max_error),
        );
    }

    ///////////////////////////////////////////////////////////////////////////

    fn test_cv(
        integrator: IntegrationFn<2>,
        x0: f64,
        vel0: f64,
        start: f64,
        end: f64,
        step: f64,
        max_error: na::SVector<f64, 2>,
    ) -> () {
        let func = |x: na::SVector<f64, 2>, _: f64| {
            #[allow(non_snake_case)]
            let A: na::SMatrix<f64, 2, 2> = na::SMatrix::<f64, 2, 2>::new(0., 1., 0., 0.);
            A * x
        };

        let total_time: f64 = end - start;
        test_integration(
            integrator,
            func,
            na::SVector::<f64, 2>::new(x0, vel0),
            start,
            end,
            step,
            na::SVector::<f64, 2>::new(x0 + vel0 * total_time, vel0),
            max_error,
        );
    }

    ///////////////////////////////////////////////////////////////////////////

    fn test_ca(
        integrator: IntegrationFn<3>,
        x0: f64,
        vel0: f64,
        accel0: f64,
        start: f64,
        end: f64,
        step: f64,
        max_error: na::SVector<f64, 3>,
    ) -> () {
        const N: usize = 3;
        let func = |x: na::SVector<f64, N>, _: f64| {
            #[allow(non_snake_case)]
            let A: na::SMatrix<f64, N, N> =
                na::SMatrix::<f64, N, N>::new(0., 1., 0., 0., 0., 1., 0., 0., 0.);
            A * x
        };

        let total_time: f64 = end - start;
        test_integration(
            integrator,
            func,
            na::SVector::<f64, N>::new(x0, vel0, accel0),
            start,
            end,
            step,
            na::SVector::<f64, N>::new(
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
        test_cv(rk1, X0, VEL0, START, END, STEP, MAX_ERROR);
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
        test_ca(rk1, X0, VEL0, ACCEL0, START, END, STEP, MAX_ERROR);
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
        test_cv(rk4, X0, VEL0, START, END, STEP, MAX_ERROR);
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
        test_ca(rk4, X0, VEL0, ACCEL0, START, END, STEP, MAX_ERROR);
    }
}

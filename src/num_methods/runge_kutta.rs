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
fn rk1<
    T: std::ops::Sub<Output = T> + std::ops::Mul<na::SVector<T, R>, Output = na::SVector<T, R>> + Copy,
    const R: usize,
>(
    func: VectorFn<T, R>,
    x0: na::SVector<T, R>,
    t0: T,
    tf: T,
) -> na::SVector<T, R>
where
    na::SVector<T, R>: std::ops::Add<na::SVector<T, R>, Output = na::SVector<T, R>>,
{
    let dt: T = tf - t0;
    return x0 + dt * func(x0, t0);
}
#[cfg(test)]
mod tests {
    use super::*;

    type IntegrationFn<const R: usize> =
        fn(VectorFn<f64, R>, na::SVector<f64, R>, f64, f64) -> na::SVector<f64, R>;

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
                "total error: {},  max error: {}\n",
                total_error[i].abs(),
                max_error[i].abs()
            );
            assert!(total_error[i].abs() < max_error[i].abs());
        }
        // return
        result
    }

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

    #[test]
    fn rk1_cos() {
        const START: f64 = 0.;
        const END: f64 = std::f64::consts::PI;
        const STEP: f64 = 0.01;
        const MAX_ERROR: f64 = 2e-2;
        test_cos(rk1, START, END, STEP, MAX_ERROR);
    }
}

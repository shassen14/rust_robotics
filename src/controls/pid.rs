// initial/experimental. not sure how I want to program it yet
use crate::utils::math;
use nalgebra as na;

pub struct Controller<const R: usize> {
    kp: na::SVector<f64, R>,
    ki: na::SVector<f64, R>,
    kd: na::SVector<f64, R>,
    error_previous: na::SVector<f64, R>,
    error_current: na::SVector<f64, R>,
    error_total: na::SVector<f64, R>,
    error_total_lower_bound: na::SVector<f64, R>,
    error_total_upper_bound: na::SVector<f64, R>,
}

impl<const R: usize> Controller<R> {
    pub fn new(
        kp: na::SVector<f64, R>,
        ki: na::SVector<f64, R>,
        kd: na::SVector<f64, R>,
        lower_bound: na::SVector<f64, R>,
        upper_bound: na::SVector<f64, R>,
    ) -> Self {
        Controller {
            kp: kp,
            ki: ki,
            kd: kd,
            error_previous: na::SVector::<f64, R>::zeros(),
            error_current: na::SVector::<f64, R>::zeros(),
            error_total: na::SVector::<f64, R>::zeros(),
            error_total_lower_bound: lower_bound,
            error_total_upper_bound: upper_bound,
        }
    }

    pub fn update_gains(
        &mut self,
        kp: &na::SVector<f64, R>,
        ki: &na::SVector<f64, R>,
        kd: &na::SVector<f64, R>,
    ) {
        self.kp = *kp;
        self.ki = *ki;
        self.kd = *kd;
    }

    pub fn compute(&mut self, error: &na::SVector<f64, R>) -> na::SVector<f64, R> {
        self.error_previous = self.error_current;
        self.error_current = *error;
        self.error_total += self.error_current;
        self.error_total = math::bound_value(
            self.error_total,
            self.error_total_lower_bound,
            self.error_total_upper_bound,
        );

        let u = self.kp.component_mul(&self.error_current)
            + self.ki.component_mul(&self.error_total)
            + self
                .kd
                .component_mul(&(self.error_current - self.error_previous));

        u
    }
}
